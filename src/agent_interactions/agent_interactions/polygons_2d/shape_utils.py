from shapely.geometry import Polygon as ShapelyPolygon
from scipy.spatial import ConvexHull
import numpy as np
from agent_interactions.polygons_2d.merging import Shape as MergingShape


def merge_polygons(vertices1, vertices2):
    return MergingShape.from_points(vertices1).merge(MergingShape.from_points(vertices2)).to_points()

def remove_duplicate_vertices(vertices):
    mask = [np.True_ if i==0 else np.linalg.norm(vertices[i,:]-vertices[i-1,:]) > 1E-10 for i in range(len(vertices))]
    return vertices[mask]


def centre_of_polygon(vertices):
    """
    Calculate the centroid of a polygon defined by its vertices. Uses the convex hull of the polygon.
    
    Args:
        vertices (array (n,2)): Vertices of polygon

    Returns:
        center (array (2,)): Centroid of convex hull of polygon
    """

    # Compute the convex hull of the points
    hull = ConvexHull(vertices)

    # Extract the vertices of the convex hull
    hull_vertices = vertices[hull.vertices]

    # Calculate area of polygon and center using weighted average
    area = 0
    cx = 0
    cy = 0
    n = len(hull_vertices)

    for i in range(n):
        p1 = hull_vertices[i]
        p2 = hull_vertices[(i + 1) % n]
        
        # Calculate cross product
        cross = p1[0] * p2[1] - p2[0] * p1[1]
        
        # Update area
        area += cross
        
        # Update center coordinates
        cx += (p1[0] + p2[0]) * cross
        cy += (p1[1] + p2[1]) * cross

    area = area / 2.0
    center = np.array([cx / (6.0 * area), cy / (6.0 * area)])

    return center

    # return np.mean(vertices, axis=0)
    # return polylabel([vertices])


def polygon_area(vertices):
    """
    Calculate area of a polygon using shoelace formula.
    Works for both convex and non-convex polygons.
    
    Parameters: vertices: array (n,2) containing vertices of polygon
    Returns: float, area of polygon
    """
    # Handle empty or degenerate polygons
    if len(vertices) < 3:
        return 0.0
        
    # Initialize area
    area = 0.0
    
    # Number of vertices
    n = len(vertices)
    
    # Calculate area using shoelace formula
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i][0] * vertices[j][1]
        area -= vertices[j][0] * vertices[i][1]
    
    # Take absolute value and divide by 2
    return abs(area) / 2.0


def clean_polygon(points):
    """
    Clean up vertices with bad interior angles.
    """

    if len(points) < 3:
        return points
        
    cleaned = []
    n = len(points)
    
    for i in range(n):
        prev = points[(i-1)%n] 
        curr = points[i]
        next = points[(i+1)%n]
        
        # Calculate vectors
        v1 = curr - prev
        v2 = next - curr
        
        # Calculate angle between vectors
        dot = np.dot(v1, v2)
        norm = np.linalg.norm(v1) * np.linalg.norm(v2)
        
        if norm < 1e-10:  # Points too close together
            continue
            
        angle = np.arccos(np.clip(dot/norm, -1.0, 1.0))
        angle_deg = np.degrees(angle)
        
        # Keep vertex if angle is not 0°, 180° or 360°
        if abs(angle_deg) > 1e-10 and abs(angle_deg-180) > 1e-10 and abs(angle_deg-360) > 1e-10:
            cleaned.append(curr)
            
    return np.array(cleaned)


class PolygonError(Exception):
    """
    Custom error to catch specifically polygon related errors
    """
    pass


# def merge_polygons_shapely(vertices1, vertices2):
#     """
#     Merges two potentially concave polygons into their union using Shapely.
#     Assumes vertices are ordered counter-clockwise.
    
#     Args:
#         vertices1 (np.array): Vertices of first polygon
#         vertices2 (np.array): Vertices of second polygon
    
#     Returns:
#         merged_vertices (np.array): Vertices of merged polygon
#     """
#     # from shapely.geometry import Polygon
#     # from shapely import buffer
    
#     # Convert vertex arrays to Shapely polygons

#     # Buffer to avoid multiple geometries from neighbouring polygons
#     poly1 = ShapelyPolygon(vertices1)
#     poly2 = ShapelyPolygon(vertices2)
#     # Compute union
#     union = poly1.union(poly2)

#     # Handle potential multi-polygon result
#     if union.geom_type == 'MultiPolygon':

#         # Buffer to avoid multiple geometries from neighbouring polygons
#         poly1 = ShapelyPolygon(vertices1).buffer(distance=1E-10, join_style=2)
#         poly2 = ShapelyPolygon(vertices2).buffer(distance=1E-10, join_style=2)
#         # Compute union
#         union = poly1.union(poly2)

#         if union.geom_type == 'MultiPolygon':
#             raise PolygonError("Multiple polygons returned")

#         # Take the polygon with largest area if multiple are returned
#         # union = max(union.geoms, key=lambda x: x.area)
#         # plt.show()

#         # fig, ax = plt.subplots(1,1,figsize=(18, 18))
#         # polygon1_coords = union.geoms[0].buffer(distance=1E-10, join_style=2).exterior.coords[:-1]
#         # polygon2_coords = union.geoms[1].buffer(distance=1E-10, join_style=2).exterior.coords[:-1]
#         # ax.fill(*zip(*polygon1_coords), color='red', alpha=0.5, edgecolor='black')
#         # ax.fill(*zip(*polygon2_coords), color='blue', alpha=0.5, edgecolor='black')
#         # ax.set_aspect('equal')
#         # ax.scatter(*zip(*polygon1_coords), color='red', marker='o')
#         # ax.scatter(*zip(*polygon2_coords), color='blue', marker='o')
#         # plt.show()
        
    
#     # Extract vertices from the union
#     merged_vertices = np.array(union.exterior.coords[:-1])  # Remove duplicate last point

#     return merged_vertices
    
#     # Order vertices counter-clockwise
#     # center = np.mean(merged_vertices, axis=0)
#     # angles = np.arctan2(merged_vertices[:,1] - center[1],
#     #                    merged_vertices[:,0] - center[0])
    
#     # return merged_vertices[np.argsort(angles)]


def intersect_polygons_shapely(vertices1, vertices2):
    """
    Compute intersection of two polygons using Shapely.
    
    Args:
        vertices1: np.array of shape (n,2) containing vertices of first polygon
        vertices2: np.array of shape (m,2) containing vertices of second polygon
    
    Returns:
        itx: np.array of shape (k,2) containing vertices of intersection polygon
    """

    
    # Convert vertex arrays to Shapely polygons
    # poly1 = ShapelyPolygon(vertices1).buffer(distance=1E-10, join_style=2)
    # poly2 = ShapelyPolygon(vertices2).buffer(distance=1E-10, join_style=2)
    
    # Compute intersection
    intersection = ShapelyPolygon(vertices1).intersection(ShapelyPolygon(vertices2))
    
    # Handle empty intersection
    if intersection.is_empty:
        return np.array([])
    
    # Handle multiple polygons in result
    if intersection.geom_type == 'MultiPolygon':
        # Take polygon with largest area
        # intersection = max(intersection.geoms, key=lambda x: x.area)
        raise PolygonError("Multiple polygons returned")
    
    # Extract vertices
    itx = np.array(intersection.exterior.coords[:-1])

    return itx

    # Remove duplicates
    # return np.unique(itx, axis=0)

def subtract_polygons_shapely(vertices1, vertices2):
    """
    Subtract polygon2 from polygon1 using Shapely (set difference A-B).
    
    Parameters:
    vertices1: np.array of shape (n,2) containing vertices of first polygon (A)
    vertices2: np.array of shape (m,2) containing vertices of second polygon (B)
    
    Returns:
    np.array of shape (k,2) containing vertices of resulting polygon (A-B)
    """
    
    # Convert vertex arrays to Shapely polygons
    poly1 = ShapelyPolygon(vertices1)
    poly2 = ShapelyPolygon(vertices2)
    
    # Compute difference
    difference = poly1.difference(poly2)
    
    # Handle empty result
    if difference.is_empty:
        return np.array([])
    
    # Handle multiple polygons in result
    if difference.geom_type == 'MultiPolygon':
        raise PolygonError("Multiple polygons returned")
    
    # Extract vertices
    diff_vertices = np.array(difference.exterior.coords[:-1])
    
    return diff_vertices



def split_polygon_by_line(vertices, abc):
    """Split a polygon by a line ax + by + c = 0 into two polygons
    
    This function takes a polygon defined by vertices and splits it into two polygons
    along a line defined by the equation ax + by + c = 0. The algorithm:
    1. Finds the two intersection points between the line and polygon edges
    2. Starting from first intersection, builds first polygon by following original vertices
        until reaching second intersection
    3. Starting from second intersection, builds second polygon by following remaining vertices
        back to first intersection
    4. Returns both polygons as numpy arrays of vertices
    
    Args:
        vertices: numpy array of polygon vertices
        a, b, c: coefficients defining the splitting line ax + by + c = 0
        
    Returns:
        Two numpy arrays containing vertices of the split polygons,
        or (None, None) if line does not properly intersect polygon
    """
    # Find intersection points with polygon edges
    a,b,c = abc
    intersections = []
    n = len(vertices)
    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i+1)%n]
        
        # Convert edge to line equation
        a2 = p2[1] - p1[1]
        b2 = p1[0] - p2[0]
        c2 = p1[1]*p2[0] - p1[0]*p2[1]
        
        # Find intersection
        det = a*b2 - a2*b
        if abs(det) > 1e-10:
            x = (b*c2 - b2*c)/det
            y = (a2*c - a*c2)/det
            
            # Check if intersection lies on edge
            if (min(p1[0],p2[0])-1e-10 <= x <= max(p1[0],p2[0])+1e-10 and
                min(p1[1],p2[1])-1e-10 <= y <= max(p1[1],p2[1])+1e-10):

                # if its the same as the last one, then its a vertex. update the previous index to this one
                if len(intersections)>0 and abs(x-intersections[-1][0][0]) < 1E-10 and abs(y-intersections[-1][0][1]) < 1E-10:
                    intersections[-1][1] = i
                    intersections[-1][2] = True # mark as vertex
                else:
                    intersections.append([np.array([x,y]), i, False])
    
    # Check last itx for duplicate
    if abs(intersections[0][0][0]-intersections[-1][0][0]) < 1E-10 and abs(intersections[0][0][1]-intersections[-1][0][1]) < 1E-10:
        intersections.pop() # remove last one
        intersections[0][2] = True # mark as vertex
    


    if len(intersections) > 2 and (len(intersections) % 2 == 1):  # odd number of itx (3)

        not_vertices = [x for x in intersections if not x[2]]
        if len(not_vertices) % 2 == 0:
            intersections = not_vertices
        else:
            raise PolygonError("Something SERIOUSLY wrong for line {:.2}x + {:.2}y + {:.2} = 0 through polygon\n{}".format(a,b,c,vertices))

    # if len(intersections) > 2:

    #     # Split only along the pair with the longest distance
    #     if a > b:
    #         intersections.sort(key=lambda itx: itx[0][1])
    #     else:
    #         intersections.sort(key=lambda itx: itx[0][0])
        
    #     max_pair = None
    #     max_dist = -1
    #     for i in range(2,len(intersections),2):
    #         dist = np.linalg.norm(np.array(intersections[i][0]) - np.array(intersections[i+1][0]))
    #         if dist > max_dist:
    #             max_pair = intersections[i:i+2]

    elif len(intersections) != 2:
        # import matplotlib.pyplot as plt
        # from agent_interactions.polygons_2d.interactions import visualise_regions
        # fig, ax = plt.subplots(1,1)
        # visualise_regions([vertices], vertices, ax=ax)
        # # Draw itxs
        # ax.scatter([x[0][0] for x in intersections], [x[0][1] for x in intersections], s=20)
        # # Draw line
        # xlim = ax.get_xlim()
        # x = np.array(xlim)
        # y = (-a * x - c) / b
        # ax.plot(x, y, '--', color='black', alpha=0.5, zorder=1)
        # # Show plot
        # plt.show()
        raise PolygonError("Not 2 intersections for line {:.2}x + {:.2}y + {:.2} = 0 through polygon\nIntersections:\n{}".format(a,b,c,"\n".join([f"({round(float(x[0][0]),1),round(float(x[0][1]),1)})" for x in intersections])))

        
    # Sort vertices into two polygons
    points1 = []
    points2 = []
    
    # Start at first intersection
    curr_point = intersections[0][0]
    curr_idx = intersections[0][1]
    points1.append(curr_point)
    
    # Add vertices until reaching second intersection
    while curr_idx != intersections[1][1]:
        next_idx = (curr_idx + 1) % n
        points1.append(vertices[next_idx])
        curr_idx = next_idx
        
    # Check for vtx intersection
    if np.linalg.norm(np.array(points1[-1]) - intersections[1][0]) > 1E-10:
        points1.append(intersections[1][0])
    
    # Complete second polygon
    curr_point = intersections[1][0]
    curr_idx = intersections[1][1]
    points2.append(curr_point)
    
    while curr_idx != intersections[0][1]:
        next_idx = (curr_idx + 1) % n
        points2.append(vertices[next_idx])
        curr_idx = next_idx
        
    # Check for vtx intersection
    if np.linalg.norm(np.array(points2[-1]) - intersections[0][0]) > 1E-10:
        points2.append(intersections[0][0])
    
    return np.array(points1), np.array(points2)


# def split_polygon_by_line(vertices, abc):

#     """
#     1. Find all intersections. Sort by increase x and y
#     2. There will always be multiple of 2 intersections, where each pair enters and exits the polygon. So, find the pair with the longest distance. Only split the polygon on the line joining this pair.
#     3. 

#     """



def split_polygon_exactly_50_50(merged_poly, agent_centre_1, agent_centre_2):
    """
    Split a polygon along a line whose slope and offset ensure the polygons are split exactly 50:50 (with some small error)
    Maintains original agent polygon local regions (i.e. they don't just swap positions randomly)

    Args:
        merged_poly (ndarray): The polygon to split 
        agent_centre_1 (tuple | ndarray): (x,y) coordinates of first agent's center
        agent_centre_2 (tuple | ndarray): (x,y) coordinates of second agent's center

    Returns:
        tuple:
            - new_poly1 (ndarray): Agent 1's new polygon vertices
            - new_poly2 (ndarray): Agent 2's new polygon vertices
            - line (tuple): Line coefficients (a,b,c) that was used to split agents where ax + by + c = 0
    """

    MAX_ERR = 1E-4

    x1,y1 = agent_centre_1
    x2,y2 = agent_centre_2

    # Determine slope from original agent centers
    a = x2-x1
    b = y2-y1

    # c_vals = [(-a*x -b*y, (x,y)) for x,y in merged_poly]
    # c_min, (x_min, y_min) = min(c_vals, key=lambda x:x[0])
    # c_max, (x_max, y_max) = max(c_vals, key=lambda x:x[0])


    # c1_o = -a*x1 -b*y1
    # c2_o = -a*x2 -b*y2



    # c_vals = [(-a*x -b*y) for x,y in merged_poly]
    # c_min = min(c_vals)
    # c_max = max(c_vals)



    c_vals = [(-a*x -b*y, (x,y)) for x,y in merged_poly]
    c_min, (x_min, y_min) = min(c_vals, key=lambda x:x[0])
    c_max, (x_max, y_max) = max(c_vals, key=lambda x:x[0])

    dist_1 = a*x1 + b*y1 + c_min
    dist_2 = a*x2 + b*y2 + c_min

    if abs(dist_1) < abs(dist_2):
        c1, x1, y1 = c_min, x_min, y_min
        c2, x2, y2 = c_max, x_max, y_max
    else:
        c2, x2, y2 = c_min, x_min, y_min
        c1, x1, y1 = c_max, x_max, y_max
        

    while True:

        try:
            c = 0.5 * (c1 + c2)
            new_poly1, new_poly2 = split_polygon_by_line(merged_poly, (a,b,c))
            if new_poly1 is None or new_poly2 is None:
                raise PolygonError("Can't split merged polygon")
            
        except PolygonError as e:
            raise e
            # # Convert to convex hull and redo
            # print("WARNING: CONVERTING TO CONVEX HULL")
            # return split_polygon_exactly_50_50(merged_poly[ConvexHull(merged_poly).vertices], agent_centre_1, agent_centre_2)
        
        # Ensure polys are assigned to the correct agent
        agent1_side = a*x1+b*y1+c
        cx1,cy1 = centre_of_polygon(new_poly1)
        new_poly1_side = a*cx1+b*cy1+c
        if new_poly1_side*agent1_side < 0 and abs(new_poly1_side*agent1_side) > 1e-10:
            new_poly1, new_poly2 = new_poly2, new_poly1
    
        new_area_1 = polygon_area(new_poly1)
        new_area_2 = polygon_area(new_poly2)

        if np.abs(new_area_1 - new_area_2) < MAX_ERR:
            break

        if new_area_1 > new_area_2:
            c2 = c
            continue

        if new_area_1 < new_area_2:
            c1 = c
            continue
    
    return new_poly1, new_poly2, (a,b,c)

