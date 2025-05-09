from abc import ABC, abstractmethod
from copy import deepcopy
from lazy_agent_sim_interfaces.msg import EpuckInteraction, EpuckKnowledgePacket, EpuckKnowledgeRecord, Boundary, Centroid
import numpy as np

# Base class
class InteractionScheme(ABC):

    @abstractmethod
    def interact(self, agent1, agent2):
        pass

    @abstractmethod
    def test_neighbourhood(self, agent1, agent2):
        pass

    @abstractmethod
    def agent_from_record(self, record, n):
        pass

    @abstractmethod
    def new_centroid_boundary_n(self, agent):
        pass


import agent_interactions.polygons_2d.interactions as p2d
class Scheme2dPolygons(InteractionScheme):

    def interact(self, agent1, agent2):
        vertices1, vertices2 = p2d.interact_polygons(agent1["vertices"], agent2["vertices"])
        return {"vertices": vertices1}, {"vertices": vertices2}

    def test_neighbourhood(self, agent1, agent2):
        return p2d.test_neighbourhood(agent1["vertices"], agent2["vertices"])
    
    def agent_from_record(self, record, n) -> np.ndarray:

        agent = {
            "vertices": np.array([
                [x,y] for x,y in zip(record.boundary.x_points, record.boundary.y_points)
            ])
        }
        return agent
    
    def new_centroid_boundary_n(self, agent):
        
        centroid = Centroid( x = p2d.centre_of_polygon(agent["vertices"]) ) # not necessary for this scheme

        boundary = Boundary(
            x_points = list(agent["vertices"][:,0]), 
            y_points = list(agent["vertices"][:,1])
        )
        n = 0       # not necessary for this scheme

        return centroid, boundary, n
    

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



# Dummy class for testing
class Agent:
    def __init__(self, vertices):
        self.vertices = vertices

import matplotlib.pyplot as plt
if __name__ == "__main__":
    pass    
    # scheme = Scheme2dPolygons()
    # a1 = Agent(vertices=np.array([[0,0],[0,2],[2,2],[2,0]]))
    # a2 = Agent(vertices=np.array([[1,1],[1,3],[3,3],[3,1]]))
    # domain_vertices = np.array([[-1,-1],[-1,4],[4,4],[4,-1]])

    # print(f"Neighbours? {scheme.test_neighbourhood(a1,a2)}")

    # a1_new, a2_new = scheme.interact(a1,a2)
    # p2d.visualise_regions([a1_new, a2_new], domain_vertices=domain_vertices, interaction=(0,1))
    # plt.show()

    # print(f"Still neighbours? {scheme.test_neighbourhood(a1,a2)}")
