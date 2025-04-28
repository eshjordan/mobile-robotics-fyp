from abc import ABC, abstractmethod

# Base class
class InteractionScheme(ABC):

    @abstractmethod
    def interact(self, agent1, agent2):
        pass

    @abstractmethod
    def test_neighbourhood(self, agent1, agent2):
        pass

    def get_all_neighbours(self, agents):
        """ TODO """
        pass


"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - 'vertices'
"""
import polygons_2d.interactions as p2d
class Scheme2dPolygons(InteractionScheme):
    
    def interact(self, agent1, agent2):
        return p2d.interact_polygons(agent1.vertices, agent2.vertices)

    def test_neighbourhood(self, agent1, agent2):
        return p2d.test_neighbourhood(agent1.vertices, agent2.vertices)



"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - theta_l
 - theta_u
 - n
 - eps 
"""
import modified_vickery_1d.interactions as mv1d
class Scheme1dModifiedVickery(InteractionScheme):
    
    def interact(self, agent1, agent2, forward):
        ag1 = mv1d.Agent(agent1.theta_l, agent1.theta_u, agent1.n, agent1.epsilon)
        ag2 = mv1d.Agent(agent2.theta_l, agent2.theta_u, agent2.n, agent2.epsilon)
        mv1d.interact(ag1, ag2, forward)
        return ag1, ag2

    def test_neighbourhood(self, agent1, agent2):
        ag1 = mv1d.Agent(agent1.theta_l, agent1.theta_u, agent1.n, agent1.epsilon)
        ag2 = mv1d.Agent(agent2.theta_l, agent2.theta_u, agent2.n, agent2.epsilon)
        return mv1d.test_neighbourhood(ag1, ag2)



"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - n
 - centroid
"""
import vickery_1d.interactions as v1d
class Scheme1dVickery(InteractionScheme):
    
    def interact(self, agent1, agent2, forward):

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

import numpy as np
import matplotlib.pyplot as plt
if __name__ == "__main__":
    scheme = Scheme2dPolygons()
    a1 = Agent(vertices=np.array([[0,0],[0,2],[2,2],[2,0]]))
    a2 = Agent(vertices=np.array([[1,1],[1,3],[3,3],[3,1]]))
    domain_vertices = np.array([[-1,-1],[-1,4],[4,4],[4,-1]])

    print(f"Neighbours? {scheme.test_neighbourhood(a1,a2)}")

    a1_new, a2_new = scheme.interact(a1,a2)
    p2d.visualise_regions([a1_new, a2_new], domain_vertices=domain_vertices, interaction=(0,1))
    plt.show()

    print(f"Still neighbours? {scheme.test_neighbourhood(a1,a2)}")
