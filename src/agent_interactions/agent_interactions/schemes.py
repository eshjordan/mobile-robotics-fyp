from abc import ABC, abstractmethod
from copy import deepcopy
from lazy_agent_sim_interfaces.msg import EpuckInteraction, EpuckKnowledgePacket, EpuckKnowledgeRecord, Boundary, Centroid
import numpy as np
from typing import Any, Tuple


# Base class
class InteractionScheme(ABC):

    @abstractmethod
    def interact(self, agent1: Any, agent2: Any) -> Tuple[Any,Any]:
        """
        Implement this to compute the interaction between
        two agents and return the new agent states
        """
        pass

    @abstractmethod
    def test_neighbourhood(self, agent1, agent2):
        """
        Implement this to test whether two agents are neighbours
        """
        pass

    @abstractmethod
    def agent_from_record(self, record: EpuckKnowledgeRecord, n: int) -> Any:
        """
        Implement this to convert an EpuckKnowledgeRecord msg
        to an agent data struture to be used in other functions
        """
        pass

    @abstractmethod
    def new_centroid_boundary_n(self, agent: Any) -> Tuple[Centroid, Boundary, int]:
        """
        Implement this to convert an agent data structure
        to a Centroid, Boundary, and N (knowledge)
        """
        pass

    @abstractmethod
    def get_agent_initial_state(self, agent_idx: int, N: int) -> Tuple[Centroid, Boundary, int]:
        """
        Implement this to compute the initial state assigned to an agent once
        the agent comms manager makes first contact to the agent.
        """
        pass





import agent_interactions.polygons_2d.interactions as p2d
from agent_interactions.polygons_2d.insertions import generate_circle
class Scheme2dPolygons(InteractionScheme):

    def interact(self, agent1, agent2):
        vertices1, vertices2 = p2d.interact_polygons(agent1["vertices"], agent2["vertices"])
        new_n = max(agent1["n"], agent2["n"], 2) # should always be at least 2 after an interaction
        return {"vertices": vertices1, "n": new_n}, {"vertices": vertices2, "n": new_n}

    def test_neighbourhood(self, agent1, agent2):
        return p2d.test_neighbourhood(agent1["vertices"], agent2["vertices"])
    
    def agent_from_record(self, record, n) -> np.ndarray:

        agent = {
            "vertices": np.array([
                [x,y] for x,y in zip(record.boundary.x_points, record.boundary.y_points)
            ]),
            "n": n
        }
        return agent
    
    def new_centroid_boundary_n(self, agent: dict):
        
        centroid = Centroid( x = p2d.centre_of_polygon(agent["vertices"]) )

        boundary = Boundary(
            x_points = list(agent["vertices"][:,0]), 
            y_points = list(agent["vertices"][:,1])
        )
        n = agent["n"]

        return centroid, boundary, n
    
    def get_agent_initial_state(self, agent_idx, N):
        # inititalise with entire domain bounds
        agent = {
            "vertices": generate_circle(centre=[0,0], radius=1, circle_resolution=50),
            "n": 1
        }
        return self.new_centroid_boundary_n(agent)
    




import agent_interactions.modified_vickery_1d.interactions as mv1d
class Scheme1dModifiedVickery(InteractionScheme):

    def interact(self, agent1, agent2, forward=True):
        mv1d.interact(agent1, agent2, forward)
        return agent1, agent2

    def test_neighbourhood(self, agent1, agent2):
        return mv1d.test_neighbourhood(agent1, agent2)
    
    def agent_from_record(self, record, n) -> mv1d.Agent:

        agent = mv1d.Agent(
            theta_l = record.boundary.x_points[0],
            theta_u = record.boundary.x_points[1],
            n = n
        )
        agent.epsilon = ((agent.theta_u - agent.theta_l) % (2*np.pi)) - (2*np.pi / agent.n)

        return agent
    
    def new_centroid_boundary_n(self, agent: mv1d.Agent):
        
        centroid = Centroid( x = agent.theta_c() )
        boundary = Boundary(x_points = [agent.theta_l, agent.theta_u])
        n = agent.n

        return centroid, boundary, n

class Scheme1dModifiedVickerySimple(InteractionScheme):

    """
    No epsilon, no minimum area, no anchoring.
    Simply repartition inside the union of the areas.
    """

    def interact(self, agent1, agent2):
        mv1d.interact_simple(agent1, agent2)
        return agent1, agent2

    def test_neighbourhood(self, agent1, agent2):
        return mv1d.test_neighbourhood(agent1, agent2)
    
    def agent_from_record(self, record, n) -> mv1d.Agent:

        agent = mv1d.Agent(
            theta_l = record.boundary.x_points[0],
            theta_u = record.boundary.x_points[1],
            n = n,
            epsilon = 0
        )

        return agent
    
    def new_centroid_boundary_n(self, agent: mv1d.Agent):
        
        centroid = Centroid( x = agent.theta_c(), y = 0, z = 0 )
        boundary = Boundary(x_points = [agent.theta_l, agent.theta_u], y_points = [], z_points = [])
        n = max(agent.n,2)  # should always be at least 2 after an interaction

        return centroid, boundary, n
    
    def get_agent_initial_state(self, agent_idx: int, N: int):
        """
        agent_idx is the index, not the actual robot id
        """

        agent = mv1d.insert_simple(agent_idx, N)
        return self.new_centroid_boundary_n(agent)







# TODO
import agent_interactions.vickery_1d.interactions as v1d
class Scheme1dVickery(InteractionScheme):

    def interact(self, agent1, agent2, forward=True):

        ag1 = v1d.Agent(0,1, n=agent1.n)
        ag2 = v1d.Agent(0,1, n=agent2.n)
        ag1.move_to(agent1.centroid)
        ag2.move_to(agent2.centroid)

        if forward:
            v1d.interact_1(ag1, ag2)
        else:
            v1d.interact_2(ag1, ag2)


    def test_neighbourhood(self, agent1, agent2):

        ag1 = v1d.Agent(0,1, n=agent1.n)
        ag2 = v1d.Agent(0,1, n=agent2.n)
        ag1.move_to(agent1.centroid)
        ag2.move_to(agent2.centroid)

        return v1d.test_neighbourhood(ag1, ag2)



# # Dummy class for testing
# class Agent:
#     def __init__(self, vertices):
#         self.vertices = vertices


import matplotlib.pyplot as plt
if __name__ == "__main__":
    pass
    
    # scheme = Scheme1dModifiedVickerySimple()

    # N = 5
    # for i in range(1, N+1):
    #     centroid, boundary, n = scheme.get_agent_initial_state(i, N)
    #     print("Bounds: {:.3f}pi, {:.3f}pi".format(boundary.x_points[0] / np.pi, boundary.x_points[1] / np.pi))



    # scheme = Scheme2dPolygons()
    # a1 = Agent(vertices=np.array([[0,0],[0,2],[2,2],[2,0]]))
    # a2 = Agent(vertices=np.array([[1,1],[1,3],[3,3],[3,1]]))
    # domain_vertices = np.array([[-1,-1],[-1,4],[4,4],[4,-1]])

    # print(f"Neighbours? {scheme.test_neighbourhood(a1,a2)}")

    # a1_new, a2_new = scheme.interact(a1,a2)
    # p2d.visualise_regions([a1_new, a2_new], domain_vertices=domain_vertices, interaction=(0,1))
    # plt.show()

    # print(f"Still neighbours? {scheme.test_neighbourhood(a1,a2)}")
