import numpy as np
from agent_interactions.polygons_2d.shape_utils import subtract_polygons_shapely, intersect_polygons_shapely

# not an actual wedge, needs to be intersected with domain bounds
def get_wedge(centroid, p, N, centre=[2,2], r=8):
    """
    Produces a wedge shape for an agent, as in the 1D case, but in 2D

    :/

    Args:
        centroid (float): Agent centroid
        p (int): Agent index
        N (int): Total number of agents
        centre (list): Centre coordinates [x,y] of the formation.
        r (float): Radius of wedge.

    Returns:
        wedge (np.array): Vertices of the wedge shape
    """

    lower_bound = (centroid - (np.pi/min(N,p+1))) % (2*np.pi)
    upper_bound = (centroid + (np.pi/min(N,p+1))) % (2*np.pi)
    cx, cy = centre
    wedge = np.array([
        centre,
        [cx + r*np.cos(lower_bound), cy + r*np.sin(lower_bound)],
        [cx + r*np.cos(centroid),    cy + r*np.sin(centroid)],
        [cx + r*np.cos(upper_bound), cy + r*np.sin(upper_bound)],
    ])
    return wedge


def generate_circular_insertion(N, centre=[2,2], rad_from_ctr=1, rad_shape=1.1):
    """
    Inserts agents with cirlce-like polygons, each arranged in a circle

    Args:
        N (int): Number of agents
        centre (list): Centre coordinates [x,y] of the formation.
        rad_from_ctr (float): Radial distance from the centre to each agent.
        rad_shape (float): Size scale of the circle-like shapes.

    Returns:
        agent_vertices (list): N circular-like shapes, arranged in a circle

    """
    cx,cy = centre
    agent_vertices = [
        np.array([
            [cx + rad_from_ctr*np.cos(th) + rad_shape,          cy + rad_from_ctr*np.sin(th) + rad_shape],
            [cx + rad_from_ctr*np.cos(th) + 1.4*rad_shape,      cy + rad_from_ctr*np.sin(th) + 0],
            [cx + rad_from_ctr*np.cos(th) + rad_shape,          cy + rad_from_ctr*np.sin(th) - rad_shape],
            [cx + rad_from_ctr*np.cos(th) + 0,                  cy + rad_from_ctr*np.sin(th) - 1.4*rad_shape],
            [cx + rad_from_ctr*np.cos(th) - rad_shape,          cy + rad_from_ctr*np.sin(th) - rad_shape],
            [cx + rad_from_ctr*np.cos(th) - 1.4*rad_shape,      cy + rad_from_ctr*np.sin(th) + 0],
            [cx + rad_from_ctr*np.cos(th) - rad_shape,          cy + rad_from_ctr*np.sin(th) + rad_shape],
            [cx + rad_from_ctr*np.cos(th) + 0,                  cy + rad_from_ctr*np.sin(th) + 1.4*rad_shape]
        ])
        for th in list(np.linspace(0, 2*np.pi, N+1))[:-1]
    ]
    return agent_vertices


def generate_radial_insertion(N, centre=[2,2], r=8):
    """
    Inserts agents into circular wedges, similar to 1D case

    Args:
        N (int): Number of agents
        centre (list): Centre coordinates [x,y] of the formation
        r (float): Radius of wedge (expected to be intersected with domain vertices later)

    Returns:
        agent_wedges (np.array): Agent wedges
    """

    agent_wedges = []
    for p in range(1,N+1):
        if p==1:
            centroid = 0
        elif p==2:
            centroid = 2*np.pi/3
        elif p==N:
            centroid = np.pi + sum([np.pi/m for m in range(3,N+1)])
            centroid = centroid % (2*np.pi)
        else:
            centroid = np.pi + sum([np.pi/m for m in range(3,p+1)]) - (np.pi/(p+1))
            centroid = centroid % (2*np.pi)

        agent_wedges.append(get_wedge(centroid, p, N, centre, r))

    return np.array(agent_wedges)


def generate_annulus_insertion(N, r1=8, r2=1.5, centre=[2,2], circle_resolution=33):
    """
    Similar to `generate_radial_insertion` with the addition of a hole in the center
    """

    agent_vertices = remove_duplicate_vertices(generate_radial_insertion(N, centre, r=r1))

    # Create hole in centre
    centre_hole_vertices = generate_circle(centre, radius=r2, circle_resolution=circle_resolution)
    agent_vertices = [subtract_polygons_shapely(verts, centre_hole_vertices) for verts in agent_vertices]
    agent_vertices = [remove_duplicate_vertices(verts) for verts in agent_vertices]

    return agent_vertices


def remove_duplicate_vertices(vertices):
    mask = [np.True_ if i==0 else np.linalg.norm(vertices[i,:]-vertices[i-1,:]) > 1E-10 for i in range(len(vertices))]
    return vertices[mask]


def generate_circle(centre=[2,2], radius=2, circle_resolution = 33):
    """
    Produces vertices in a circle shape

    Args:
        centre (list): Centre coordinates [x,y] of the circle
        radius (float): Radius of the circle
        circle_resolution (int): Number of vertices to use for the circle approximation

    Returns:
        vertices (np.array): Vertices approximating a circle
    """
    cx, cy = centre
    vertices = np.array([
        np.array([cx + radius*np.cos(th), cy + radius*np.sin(th)]) for th in list(np.linspace(0, 2*np.pi, circle_resolution))[:-1]
    ])
    return vertices

def intersect_with_domain_boundaries(agent_vertices, domain_vertices):
    """
    Clips the agent vertices along the domain lines.

    Args:
        agent_vertices (list): Agent vertices to clip against domain
        domain_vertices (np.array): Domain boundaries to clip against

    Returns:
        agent_vertices (list): Clipped agent vertices
    """
    agent_vertices = [remove_duplicate_vertices(intersect_polygons_shapely(verts, domain_vertices)) for verts in agent_vertices]
    return agent_vertices
