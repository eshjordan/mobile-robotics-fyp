import numpy as np

class ShapeError(Exception):
    pass

def remove_duplicate_vertices(vertices):
    mask = [np.True_ if i==0 else np.linalg.norm(vertices[i,:]-vertices[i-1,:]) > 1E-10 for i in range(len(vertices))]
    return vertices[mask]

class Vertex:
    def __init__(self, xy, nbrs: list["Vertex"] = []):
        self.xy = xy
        self.nbrs = nbrs
        self.nbrs_backup = None
        self.visited = False

    def __str__(self):
        return f"({self.xy[0]}, {self.xy[1]})"
    
    def vtx_str(self):
        return str(self) + " -> " + ", ".join([str(vtx) for vtx in self.nbrs])
        
    
    def traverse_find_edges(self, edges):
       
        self.visited = True

        for nbr in self.nbrs:
            if not nbr.visited:
                edges.append((self, nbr))
        
        for nbr in self.nbrs:
            if not nbr.visited:
                nbr.traverse_find_edges(edges)

        return edges      


    def traverse_find_outside(self, merged_poly: "Shape", prev_vtx: "Vertex", starting_vtx: "Vertex"):

        # print(f"Traversing from vtx: {str(self)}")

        if self == starting_vtx:
            # The start vertex will have been added twice, so need to remove it from the end
            merged_poly.vertices.pop()
            self.nbrs = [prev_vtx, self.nbrs[0]]
            return

        # If only two neighbours, traverse the other one
        if len(self.nbrs) == 2:
            if self.nbrs[0] == prev_vtx:
                next_vtx = self.nbrs[1]
            elif self.nbrs[1] == prev_vtx:
                next_vtx = self.nbrs[0]
            else:
                # In this case, there is most likely a single vertex joining the two shapes. Need to make a duplicate vertex.
                # if self.nbrs_backup is None:
                raise ShapeError(f"In vertex - ({str(self)}) - Neighther of vertex's neighbours are the previous vertex in the traversal AND no backup nbrs")
                # merged_poly.vertices.pop()  # get rid of this vertex

                # new_vtx = Vertex(xy = self.xy, nbrs = self.nbrs_backup) # new vertex
                # merged_poly.vertices.append(new_vtx)                    # insert into graph
                # prev_vtx.nbrs[1] = new_vtx                              # ensure prev vertex is linked to new vertex
                # for nbr in new_vtx.nbrs:
                #     nbr.nbrs[nbr.nbrs.index(self)] = new_vtx
                # new_vtx.traverse_find_outside(merged_poly, prev_vtx, starting_vtx)
                # return
                
            # First 2 cases ...
            merged_poly.vertices.append(next_vtx)
            self.nbrs = [prev_vtx, next_vtx]
            next_vtx.traverse_find_outside(merged_poly, self, starting_vtx)
            return
            

        # Find neighbour with largest interior angle
        prev_mag = np.linalg.norm(prev_vtx.xy)
        nbrs = [nbr for nbr in self.nbrs if nbr != prev_vtx]
        crosses = [np.cross(prev_vtx.xy - self.xy, nbr.xy - self.xy) for nbr in nbrs]
        scalars = [np.dot(prev_vtx.xy - self.xy, nbr.xy - self.xy) for nbr in nbrs]
        magnitudes = [np.linalg.norm(nbr.xy - self.xy) for nbr in nbrs]
        cosines = [(dot / (mag*prev_mag)) for dot,mag in zip(scalars, magnitudes)]
        # cross > 0 means 0 < angle < 180
        cosines_scores = [(1 - cos) if cross >= 0 else (3 + cos) for cos, cross in zip(cosines, crosses)]

        vtx_max_interior_angle: Vertex = max( [(vtx, score) for vtx,score in zip(nbrs, cosines_scores)] , key = lambda x:x[1]) [0]

        # for nbr, score in zip(nbrs, cosines_scores):
        #     print(f"  {str(nbr)} : {score}")
        # print(f"Max -> {str(vtx_max_interior_angle)}")
        # print(f"")

        merged_poly.vertices.append(vtx_max_interior_angle)
        self.nbrs_backup = self.nbrs
        self.nbrs = [prev_vtx, vtx_max_interior_angle]  # need to delete vertices as we go (so we only have two for each vertex)
        vtx_max_interior_angle.traverse_find_outside(merged_poly, self, starting_vtx)




class Shape:
    def __init__(self, vertices: list[Vertex]):
        self.vertices: list[Vertex] = vertices
    
    @staticmethod
    def from_points(vertices):
        n = len(vertices)
        vertex_lst = [Vertex(vtx) for vtx in vertices]
        for i,vtx in enumerate(vertex_lst):
            vtx.nbrs = [
                vertex_lst[(i-1)%n],
                vertex_lst[(i+1)%n]
            ]
        return Shape(vertex_lst)
    
    def is_simple_polygon(self):
        for vtx in self.vertices:
            if len(vtx.nbrs) != 2:
                return False
        return True
    
    def to_points(self) -> np.ndarray:
        if not self.is_simple_polygon():
            raise ShapeError(f"Cannot convert Shape to points when some vertices don't have exactly 2 edges")
        
        points = []
        vtx = self.vertices[0]
        for i in range(len(self.vertices)):
            points.append(vtx.xy)
            # By convention, nbrs[0] is backwards, nbrs[1] is forwards
            vtx = vtx.nbrs[1]
        return np.array(points)

    @staticmethod
    def edge_to_abc(vtx_1: Vertex, vtx_2: Vertex):
        x1,y1 = vtx_1
        x2,y2 = vtx_2
        a = y1-y2
        b = x2-x1
        c = x1*(y2-y1)-y1*(x2-x1)
        return (a,b,c)
    
    @staticmethod
    def line_intersection(abc1, abc2) -> np.ndarray | None:
        """
        Compute intersection point of two lines a1x+b1y+c1=0 and a2x+b2y+c2=0
        
        Args:
            abc1 (tuple): Coefficients (a1,b1,c1) of first line 
            abc2 (tuple): Coefficients (a2,b2,c2) of second line
            
        Returns:
            ndarray or None: Intersection point [x,y] if lines intersect, None if parallel
        """
        a1,b1,c1 = abc1
        a2,b2,c2 = abc2
        
        det = a1*b2 - a2*b1
        if abs(det) < 1e-10:
            return None
            
        x = (b1*c2 - b2*c1)/det
        y = (a2*c1 - a1*c2)/det
        
        return np.array([x,y])

    @staticmethod
    def edge_intersection(vtx_a_1: Vertex, vtx_a_2: Vertex, vtx_b_1: Vertex, vtx_b_2: Vertex) -> np.ndarray | None:
        """
        Unlink ray test, we don't want to return exact vertex matches
        """
        
        itx = Shape.line_intersection(Shape.edge_to_abc(vtx_a_1,vtx_a_2),Shape.edge_to_abc(vtx_b_1,vtx_b_2))

        if itx is None:
            return None
        
        if not Shape.point_lies_inside_edge_bounds(itx, (vtx_a_1,vtx_a_2)):
            return None
        
        if not Shape.point_lies_inside_edge_bounds(itx, (vtx_b_1,vtx_b_2)):
            return None
        
        # Check for exact vertex intersection
        for vtx in [vtx_a_1, vtx_a_2, vtx_b_1, vtx_b_2]:
            if np.linalg.norm(itx - vtx) < 1E-10:
                return vtx

        # print(f"edge_intersection: {str(Vertex(itx))} not outside of edges ({str(Vertex(vtx_a_1))}, {str(Vertex(vtx_a_2))}) or ({str(Vertex(vtx_b_1))}, {str(Vertex(vtx_b_2))})")
    
        return itx
    

    @staticmethod
    def ray_intersect_edge(vtx_1: Vertex, vtx_2: Vertex, ray_abc: tuple):
        """
        For ray tests, we want to be able to detect vertices
        """

        itx = Shape.line_intersection(Shape.edge_to_abc(vtx_1,vtx_2), ray_abc)

        if itx is None:
            return None

        # Check for exact vertex intersection
        for vtx in [vtx_1, vtx_2]:
            if np.linalg.norm(itx - vtx) < 1E-10:
                return vtx

        if not Shape.point_lies_inside_edge_bounds(itx, (vtx_1,vtx_2)):
            return None
    
        return itx
    
    
    def get_simple_edges(self):
        points = self.to_points()
        n = len(points)
        edges = [(points[i], points[(i+1)%n]) for i in range(n)]
        return edges

    def ray_test(self, point):
        # Find m so that the ray is not parallel to any edges
        edges = self.get_simple_edges()
        edge_lines = [Shape.edge_to_abc(*edge) for edge in edges]
        gradients = sorted([-a/b for a,b,c in edge_lines if b != 0])    # m must not be infinity
        n = len(gradients)
        for i in range(n):
            if abs(gradients[i] - gradients[(i+1)%n]) > 1E-10:
                m = 0.5 * (gradients[i] + gradients[(i+1)%n])
                break
        else:
            print("WARN: All edges have the same gradients")
            m = max(gradients) + 1

        
        # a, b, c = m, -1, -m*point[0] + point[1]
        a = m
        b = -1
        c = -m*point[0] + point[1]
        intersections = []
        for i,(vtx_1, vtx_2) in enumerate(edges):
            itx = Shape.ray_intersect_edge(vtx_1, vtx_2, (a, b, c))

            # Check for intersection
            if itx is None:
                continue

            # Check for itx with first vertex
            if np.linalg.norm(itx - vtx_1) < 1E-10:
                # Intersection with FIRST vertex. Must check if the vertex is a "turning point" on the ray
                next_vtx_val = a*vtx_2[0] + b*vtx_2[1] + c
                vtx_0, _ = edges[(i-1)%len(edges)]
                prev_vtx_val = a*vtx_0[0] + b*vtx_0[1] + c
                
                if next_vtx_val*prev_vtx_val > 1E-10:
                    # next and prev vertices are on same side of the ray. do not count this itx
                    continue

            # Check for itx with second vertex
            if np.linalg.norm(itx - vtx_2) < 1E-10:
                # Intersection with SECOND vertex. Must check if the vertex is a "turning point" on the ray
                _, vtx_3 = edges[(i+1)%len(edges)]
                next_vtx_val = a*vtx_3[0] + b*vtx_3[1] + c
                prev_vtx_val = a*vtx_1[0] + b*vtx_1[1] + c
                
                if next_vtx_val*prev_vtx_val > 1E-10:
                    # next and prev vertices are on same side of the ray. do not count this itx
                    continue

                
            
            intersections.append(itx)
        

        unique_intersections = list(remove_duplicate_vertices(np.array(intersections)))
        return unique_intersections


    @staticmethod
    def point_lies_on_line(point, edge):
        vtx_1, vtx_2 = edge
        px, py = point
        a,b,c = Shape.edge_to_abc(vtx_1, vtx_2)
        return abs(a*px + b*py + c) < 1E-10
    
    @staticmethod
    def point_lies_on_edge(point, edge):

        if not Shape.point_lies_on_line(point, edge):
            return False
        
        if not Shape.point_lies_inside_edge_bounds(point, edge):
            return False
        
        return True
        
    @staticmethod
    def point_lies_inside_edge_bounds(point, edge):

        # greater then both x coords
        if point[0] > edge[0][0] and point[0] > edge[1][0]:
            return None
        # greater then both y coords
        if point[1] > edge[0][1] and point[1] > edge[1][1]:
            return None
        # less then both x coords
        if point[0] < edge[0][0] and point[0] < edge[1][0]:
            return None
        # less then both y coords
        if point[1] < edge[0][1] and point[1] < edge[1][1]:
            return None
        
        return True


    def point_lies_inside(self, point):
        
        if sum([self.point_lies_on_edge(point, (vtx_1, vtx_2)) for vtx_1, vtx_2 in self.get_simple_edges()]) > 0:
            # TODO handle this case - point lies on an edge
            return False

        # check hi and lo
        # if theres only 1 intersectiom, then it was a vertex. TODO handle this case
        # can simply check along the x axis because m is never infinity

        intersections = self.ray_test(point)
        lo = [itx for itx in intersections if itx[0] < point[0]]
        hi = [itx for itx in intersections if itx[0] > point[0]]

        if len(lo)%2 == 1 and len(hi)%2 == 1:
            return True
        if len(lo)%2 == 0 and len(hi)%2 == 0:
            return False
        
        raise ShapeError("lo and hi are not the same")


    def is_fully_inside_of(self, other):

        # Find point in shape1 which is outside of shape2
        for vtx in self.vertices:
            if not other.point_lies_inside(vtx.xy):
                return False
        return True
    

    @staticmethod
    def build_vertex_graph(shape1: "Shape", shape2: "Shape"):

        """
        Assumes the shapes intersect at least once

        Assumes shape.vertices is ordered and forms a polygon

        Mutates both shapes

        Returns: (graph, overlap)
            * graph - the Shape object describing the vertex graph
            * overlap - True only if the two polygons overlap or intersect in any way
        """

        new_shape = Shape(shape1.vertices + shape2.vertices)


        # STAGE 1 - Merge duplicate vertices from both shapes

        # Assume no duplicates within shape1 and shape2. Therefore, we can terminate inner for loop when we've found a duplicate
        # All vertices in shape 1
        some_vertices_merged = False
        some_edges_intersected = False

        for vtx1 in shape1.vertices:

            # Search for duplicates with shape 2
            for vtx2 in shape2.vertices:
                if np.linalg.norm(vtx1.xy - vtx2.xy) < 1E-10:

                    # Found one. Go through shape 2's vtx neighbours. 
                    for nbr in vtx2.nbrs:

                        # Point nbr to shape 1's vtx. Delete any references to shape 2's vtx
                        idx = nbr.nbrs.index(vtx2)
                        if vtx1 in nbr.nbrs:
                            del nbr.nbrs[idx]
                        else:
                            nbr.nbrs[idx] = vtx1

                        # Add shape 2's nbrs to shape 1's
                        if nbr not in vtx1.nbrs:
                            vtx1.nbrs.append(nbr)
                            
                    # For visualisation, disconnect from vtx2 as well
                    vtx2.nbrs = []
                    some_vertices_merged = True
        
        # Filter out disconnected vertices        
        new_shape.vertices = [vtx for vtx in new_shape.vertices if len(vtx.nbrs)>0]




        # STAGE 2 - Merge duplicate vertices from both shapes

        edges: tuple[Vertex, Vertex] = []
        if some_vertices_merged:
            new_shape.vertices[0].traverse_find_edges(edges)
        else:
            shape1.vertices[0].traverse_find_edges(edges)
            shape2.vertices[0].traverse_find_edges(edges)

        edge_broken = [False for _ in edges]


        i = 0
        while i < len(edges):
            j = i+1
            while j < len(edges):

                if edge_broken[i]:
                    break
                if edge_broken[j]:
                    j += 1
                    continue

                # STAGE 2a - Verify edges don't touch vertices

                # Check if any vertices are the same
                for vtx1, vtx2 in [(edges[i][0], edges[j][0]), (edges[i][0], edges[j][1]), (edges[i][1], edges[j][0]), (edges[i][1], edges[j][1])]:
                    if np.linalg.norm(vtx1.xy - vtx2.xy) < 1E-10:
                        break
                else:


                    
                    # STAGE 2b - Check for edge intersecting vertex exactly

                    # Check for vertex-line itx 
                    for point, edge, edge_idx in [
                        (edges[i][0], edges[j], j),
                        (edges[i][1], edges[j], j),
                        (edges[j][0], edges[i], i),
                        (edges[j][1], edges[i], i)
                    ]:
                        if Shape.point_lies_on_edge(point.xy, (edge[0].xy, edge[1].xy)):

                            vtx_0_idx = edge[0].nbrs.index(edge[1])
                            vtx_1_idx = edge[1].nbrs.index(edge[0])

                            # Redirect edges to point
                            edge[0].nbrs[vtx_0_idx] = point
                            edge[1].nbrs[vtx_1_idx] = point
                            edge_broken[edge_idx] = True

                            # Connect point to other points
                            point.nbrs.append(edge[0])
                            point.nbrs.append(edge[1])

                            # Add new edges to check
                            edges.append((edge[0], point))
                            edges.append((edge[1], point))
                            edge_broken.extend([False]*2)

                            some_edges_intersected = True

                            break
                    
                    # Continue only if the above checks yielded nothing (no break)
                    else:


                        # STAGE 2c - Check for edge intersecting edge

                        itx = Shape.edge_intersection(edges[i][0].xy, edges[i][1].xy, edges[j][0].xy, edges[j][1].xy)
                        if itx is None:
                            j += 1
                            continue
                        # # TODO: do we need this for loop check?
                        # for xy in [edges[i][0].xy, edges[i][1].xy, edges[j][0].xy, edges[j][1].xy]:
                        #     if np.linalg.norm(itx-xy) < 1E-10:
                        #         break
                        # Continue only after checking it doesn't lie on any existing vertices
                        # else:

                        new_vtx = Vertex(
                            xy = itx,
                            nbrs = [edges[i][0], edges[i][1], edges[j][0], edges[j][1]]
                        )
                        new_shape.vertices.append(new_vtx)  # keep track of new vertex

                        i_0_idx = edges[i][0].nbrs.index(edges[i][1])
                        i_1_idx = edges[i][1].nbrs.index(edges[i][0])
                        j_0_idx = edges[j][0].nbrs.index(edges[j][1])
                        j_1_idx = edges[j][1].nbrs.index(edges[j][0])

                        edges[i][0].nbrs[i_0_idx] = new_vtx
                        edges[i][1].nbrs[i_1_idx] = new_vtx

                        edges[j][0].nbrs[j_0_idx] = new_vtx
                        edges[j][1].nbrs[j_1_idx] = new_vtx

                        edge_broken[i] = True
                        edge_broken[j] = True

                        # Add new edges to check
                        edges.append((edges[i][0], new_vtx))
                        edges.append((edges[i][1], new_vtx))
                        edges.append((edges[j][0], new_vtx))
                        edges.append((edges[j][1], new_vtx))
                        edge_broken.extend([False]*4)

                        some_edges_intersected = True
            
                j += 1
            i += 1
        return new_shape, (some_vertices_merged or some_edges_intersected)



    def merge(self, other: "Shape") -> "Shape":
        
        # Make copies
        
        # Get graph of vertices (and intersections)
        graph, overlap = Shape.build_vertex_graph(self, other)

        # Check for fully inside
        if not overlap:
            if self.is_fully_inside_of(other):
                return other
            if other.is_fully_inside_of(self):
                return self
            raise ShapeError("No overlap or intersection between the two shapes")
        
        # Traverse v_graph starting from left bottom vertex. At each vertex, take left-most edge
        # TODO: (should be fine actually...) Fix - special case where shape 1 is exactly the same as shape 2, in which case the below won't return anything

        simple_vtxs = [vtx for vtx in graph.vertices if len(vtx.nbrs)==2]
        min_y = min([vtx.xy[1] for vtx in simple_vtxs])
        bottom_vtxs = [vtx for vtx in simple_vtxs if vtx.xy[1] == min_y]
        if len(bottom_vtxs) == 1:
            starting_vtx = bottom_vtxs[0]
        elif len(bottom_vtxs) == 0:
            raise ShapeError("No vertices with minimum y (???)")
        else:
            # Get bottom left vertex
            starting_vtx = min(bottom_vtxs, key = lambda vtx: vtx.xy[0])

        # Get neighbour that is most clockwise
        vector0 = starting_vtx.nbrs[0].xy - starting_vtx.xy
        vector0 = vector0 / np.linalg.norm(vector0)
        vector1 = starting_vtx.nbrs[1].xy - starting_vtx.xy
        vector1 = vector1 / np.linalg.norm(vector1)

        scalar_0 = np.dot( np.array([-1,0]), vector0 )
        scalar_1 = np.dot( np.array([-1,0]), vector1 )

        if scalar_0 > scalar_1:
            first_nbr = starting_vtx.nbrs[0]
        else:
            first_nbr = starting_vtx.nbrs[1]

        starting_vtx.nbrs = [first_nbr] # need to delete neighbours as we go
        merged_poly = Shape([starting_vtx, first_nbr])
        first_nbr.traverse_find_outside(merged_poly, starting_vtx, starting_vtx)

        return merged_poly


