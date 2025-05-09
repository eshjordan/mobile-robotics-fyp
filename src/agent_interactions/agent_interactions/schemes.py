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

    # def get_all_neighbours(self, agents):
    #     """ TODO """
    #     pass
    
    # @abstractmethod
    # def agents_info_from_request_msg(self, msg):
    #     pass

    # @abstractmethod
    # def new_knowledge_packet(self, original_knowledge_packet, agent1, agent2):
    #     pass

    @abstractmethod
    def agent_from_record(self, record, n):
        pass

    @abstractmethod
    def new_centroid_boundary_n(self, agent):
        pass


"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - 'vertices'
"""
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
    

    
    # def agents_info_from_request_msg(self, msg):
    #     # Get my record (TODO can we access this simply using msg.known_ids[robot_id] ?? )
    #     my_record = [record for record in msg.knowledge_packet.known_ids if record.robot_id == msg.knowledge_packet.robot_id][0]
    #     other_record = [record for record in msg.knowledge_packet.known_ids if record.robot_id == msg.other_agent_id][0]
    #     agent1 = {
    #         "vertices": np.array([
    #             [x,y] for x,y in zip(my_record.boundary.x_points, my_record.boundary.y_points)
    #         ])
    #     }
    #     agent2 = {
    #         "vertices": np.array([
    #             [x,y] for x,y in zip(other_record.boundary.x_points, other_record.boundary.y_points)
    #         ])
    #     }
    #     return agent1, agent2


"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - theta_l
 - theta_u
 - n
 - eps
"""
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


    # def agents_info_from_request_msg(self, msg):
    #     # Get my record (TODO can we access this simply using msg.known_ids[robot_id] ?? )
        
    #     my_record = [record for record in msg.knowledge_packet.known_ids if record.robot_id == msg.knowledge_packet.robot_id][0]
    #     other_record = [record for record in msg.knowledge_packet.known_ids if record.robot_id == msg.other_agent_id][0]
        
        # agent1 = {
        #     "theta_l":  my_record.boundary.x_points[0],
        #     "theta_u":  my_record.boundary.x_points[1],
        #     "n":        msg.knowledge_packet.n,
        # }
        # agent1["epsilon"] = (agent1["theta_u"] - agent1["theta_l"]) % (2*np.pi) - (2*np.pi / agent1["n"])

        # agent2 = {
        #     "theta_l":  other_record.boundary.x_points[0],
        #     "theta_u":  other_record.boundary.x_points[1],
        #     "n":        msg.knowledge_packet.n,
        # }
        # agent2["epsilon"] = (agent2["theta_u"] - agent2["theta_l"]) % (2*np.pi) - (2*np.pi / agent2["n"])

    #     return agent1, agent2
    
    # def new_knowledge_packet(self, original_request, new_bounds_1, new_bounds_2):
    #     new_packet = deepcopy(original_request.knowledge_packet)

    #     idx_1 = [record.robot_id for record in new_packet.known_ids].index(new_packet.robot_id)
    #     idx_2 = [record.robot_id for record in new_packet.known_ids].index(original_request.other_agent_id)

    #     new_packet.known_ids[idx_1].boundary.x_points = new_bounds_1
    #     new_packet.known_ids[idx_2].boundary.x_points = new_bounds_2

    #     # print(f"idx_1: {idx_1}\nidx_2: {idx_2}\nnew_bounds_1: {new_bounds_1}\nnew_bounds_2: {new_bounds_2}\n\n")
    #     # new_packet.n = agent1["n"]

    #     return new_packet


"""
TO FIX: Currenly assumes agent1 and agent 2 have attributes:
 - n
 - centroid
"""
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

import numpy as np
import matplotlib.pyplot as plt
if __name__ == "__main__":

    scheme = Scheme1dModifiedVickery()

    scheme.interact

    def interact(self, agent1, agent2, forward=True):
        ag1 = mv1d.Agent(theta_l=agent1["theta_l"], theta_u=agent1["theta_u"], n=agent1["n"], epsilon=agent1["epsilon"])
        ag2 = mv1d.Agent(theta_l=agent2["theta_l"], theta_u=agent2["theta_u"], n=agent2["n"], epsilon=agent2["epsilon"])
        mv1d.interact(ag1, ag2, forward)
        return ag1, ag2




    
    # scheme = Scheme2dPolygons()
    # a1 = Agent(vertices=np.array([[0,0],[0,2],[2,2],[2,0]]))
    # a2 = Agent(vertices=np.array([[1,1],[1,3],[3,3],[3,1]]))
    # domain_vertices = np.array([[-1,-1],[-1,4],[4,4],[4,-1]])

    # print(f"Neighbours? {scheme.test_neighbourhood(a1,a2)}")

    # a1_new, a2_new = scheme.interact(a1,a2)
    # p2d.visualise_regions([a1_new, a2_new], domain_vertices=domain_vertices, interaction=(0,1))
    # plt.show()

    # print(f"Still neighbours? {scheme.test_neighbourhood(a1,a2)}")
