from agent_interactions.polygons_2d.shape_utils import centre_of_polygon, merge_polygons_shapely, clean_polygon, split_polygon_exactly_50_50, PolygonError, polygon_area
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon


def test_neighbourhood(vertices1, vertices2):
    """
    Test neighbourhood by merging and testing if the result is two separate geometries
    """
    try:
        result = merge_polygons_shapely(vertices1, vertices2)
        return True
    except PolygonError:
        return False


def interact(agent_vertices, agent1, agent2):
    """
    Agents interact (with state change)
    """

    new_poly1, new_poly2 = interact_polygons(agent_vertices[agent1], agent_vertices[agent2])
    agent_vertices[agent1] = new_poly1
    agent_vertices[agent2] = new_poly2


def interact_polygons(agent_vertices_1, agent_vertices_2):
    """
    Calculate result of two agents interacting.

     - Merge the two areas
     - Split them along a line whose slope and offset ensure the polygons are split exactly 50:50 (with some small error)

    """
    # Centroids of original polygons
    agent_centre_1 = centre_of_polygon(agent_vertices_1)
    agent_centre_2 = centre_of_polygon(agent_vertices_2)

    # Get union of new polygon
    merged_polys = merge_polygons_shapely(agent_vertices_1, agent_vertices_2)
    merged_polys_cleaned = clean_polygon(merged_polys)

    new_poly1, new_poly2, (a,b,c) = split_polygon_exactly_50_50(merged_polys_cleaned, agent_centre_1, agent_centre_2)

    return new_poly1, new_poly2



def visualise_regions(agent_vertices, domain_vertices, interaction=None, ax=None):
    """
    visualise points and their enclosed regions

    """

    if ax is None:
        fig, ax = plt.subplots(figsize=(7, 7))

    # Generate colors for agents
    colors = plt.cm.rainbow(np.linspace(0, 1, len(agent_vertices)))

    # Plot each point and its region
    for vertices, color in zip(agent_vertices, colors):
        # Combine domain lines with point-specific lines

        if len(vertices) >= 3:
            polygon = Polygon(vertices, alpha=0.3, facecolor=color, edgecolor='black')
            ax.add_patch(polygon)

            for vertex in vertices:
                ax.scatter(vertex[0], vertex[1], marker='x', color=color, s=10, zorder=5, alpha=1)

        # # Plot the agent
        # ax.scatter(point[0], point[1], color=color, s=100, zorder=5)

    # Draw union
    if interaction is not None:

        agent1, agent2 = interaction

        merged_polys = clean_polygon(merge_polygons_shapely(agent_vertices[agent1], agent_vertices[agent2]))

        polygon = Polygon(merged_polys, alpha=0.4, facecolor='none', edgecolor='black',
                    hatch='///', fill=True, linewidth=2)
        ax.add_patch(polygon)
        cx, cy = centre_of_polygon(merged_polys)
        ax.scatter(cx, cy, marker='x', color='black', s=150, zorder=5, alpha=0.8)
        # Plot vertices
        for vertex in merged_polys:
            ax.scatter(vertex[0], vertex[1], marker='.', color='black', s=80, zorder=5, alpha=0.5)

        # Draw line between two agents
        cx1, cy1 = centre_of_polygon(agent_vertices[agent1])
        cx2, cy2 = centre_of_polygon(agent_vertices[agent2])
        ax.scatter(cx1, cy1, marker='x', color='black', s=50, zorder=5, alpha=0.5)
        ax.scatter(cx2, cy2, marker='x', color='black', s=50, zorder=5, alpha=0.5)

        # a = cx2-cx1
        # b = cy2-cy1
        # c = 0.5*(cx1**2+cy1**2-cx2**2-cy2**2)

        # Draw line between two agents (shift towards the larger polygon)

        agent_area_1 = polygon_area(agent_vertices[agent1])
        agent_area_2 = polygon_area(agent_vertices[agent2])
        alpha = agent_area_2 / (agent_area_2 + agent_area_1)
        a = cx2-cx1
        b = cy2-cy1
        c = (cy2-cy1)*(-cy1*(1-alpha)-alpha*cy2) + (cx2-cx1)*(-cx1*(1-alpha)-alpha*cx2)

        new_poly1, new_poly2, (a,b,c) = split_polygon_exactly_50_50(merged_polys, [cx1, cy1], [cx2, cy2])

        xlim = ax.get_xlim()
        x = np.array(xlim)
        y = (-a * x - c) / b
        ax.plot(x, y, '--', color='black', alpha=0.5, zorder=1)


    # Draw domain boundary as hollow polygon
    domain_polygon = Polygon(domain_vertices, alpha=1, facecolor='none', edgecolor='black', linewidth=2)
    ax.add_patch(domain_polygon)


    # Set equal aspect ratio and add grid
    ax.set_aspect('equal')
    ax.grid(True)

    # Set limits with some padding
    padding = 0.1 * (np.max(domain_vertices) - np.min(domain_vertices))
    ax.set_xlim(np.min(domain_vertices[:, 0]) - padding,
                np.max(domain_vertices[:, 0]) + padding)
    ax.set_ylim(np.min(domain_vertices[:, 1]) - padding,
                np.max(domain_vertices[:, 1]) + padding)

    # # ax.set_xlim(1.4,1.5)
    # # ax.set_ylim(0.9,1)

    # if ax is None:
    #     plt.show()



def visualise_interaction(agent_vertices, domain_vertices, interaction):
    """
    Visualizes an interaction between agents in a 3-panel plot showing before, during and after states.

    Args:
        agent_vertices (list): Vertices for each agent area.
        domain_vertices (list): Vertices defining the domain boundaries.
        interaction (tuple): Agent indices (agent1, agent2).

    """
    fig, ax = plt.subplots(1,3,figsize=(24, 8))
    ax[0].set_title('Initial')
    ax[1].set_title(f'Interaction {interaction}')
    ax[2].set_title('After Interaction')
    visualise_regions(agent_vertices, domain_vertices, ax=ax[0])
    visualise_regions(agent_vertices, domain_vertices, interaction=interaction, ax=ax[1])
    interact(agent_vertices, interaction[0], interaction[1])
    visualise_regions(agent_vertices, domain_vertices, ax=ax[2])
    plt.show()