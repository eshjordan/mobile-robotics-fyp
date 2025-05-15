from math import pi
import numpy as np

import typing

class Agent:

    def __init__(self, theta_l=None, theta_u=None, n=None, epsilon=0):
        self.theta_u = theta_u
        self.theta_l = theta_l
        self.n = n
        self.epsilon = epsilon  # extra 0.5*epsilon on each side
    
    @staticmethod
    def default_1_2():
        # Factory to initialise first 2
        a1 = Agent(theta_l=3*pi/2, theta_u=pi/2, n=2)
        a2 = Agent(theta_l=pi/2, theta_u=3*pi/2, n=2)
        return [a1, a2]
     
    def __str__(self):
        # return f"Agent: 0_l = {round(self.theta_l,3)}, 0_u = {round(self.theta_u,3)}, 0_c = {round(self.theta_c(),3)}, n = {self.n}"
        return "Agent:   0_l = {:.3f} = {:.3f} π,    0_u = {:.3f} = {:.3f} π,    0_c = {:.3f} = {:.3f} π,    n = {}".format(self.theta_l, self.theta_l/pi, self.theta_u, self.theta_u/pi, self.theta_c(), self.theta_c()/pi, self.n)
    
    def mid(self, t1, t2):
        # t1, t2 in [0, 2pi], where t2 > t1 in positive direction
        if t2 < t1:
            t2 += 2*pi
        return (0.5*(t1 + t2)) % (2*pi)

    def theta_c(self):
        return self.mid(self.theta_l, self.theta_u)
    
    def update_knowledge(self, n):
        self.n = n
        self.move_to(self.theta_c())
    
    def move_to(self, c):
        self.theta_l = (c - pi/self.n - self.epsilon/2) % (2*pi)
        self.theta_u = (c + pi/self.n + self.epsilon/2) % (2*pi)
    
    def aod(self):
        return (self.theta_u - self.theta_l) % (2*pi)   # includes epsilon
    
    def rad(self):
        return self.aod()/2
    
    def base_aod(self): # no epsilon
        return 2*pi/self.n

    def base_rad(self):
        return self.base_aod()/2


def sequential_add_int(N):
    # assert N>2
    # Algorithm 3 to seuqentially add and interact
    agents = Agent.default_1_2()

    t = 2

    while t < N:
        t += 1

        i = agents[-1]
        j = Agent()

        j.n, i.n = i.n+1, i.n+1
        i.theta_u, j.theta_l = i.theta_c(), i.theta_c()
        i.theta_l = (i.theta_u - (2*pi/i.n)) % (2*pi)
        j.theta_u = (j.theta_l + (2*pi/j.n)) % (2*pi)

        agents.append(j)
    
    return agents


def angles_equal(t1, t2):
    t1,t2 = min(t1,t2), max(t1,t2)  # t2 > t1
    eps = 1e-6
    return ((t2-t1)%(2*pi)) < eps

def d(t1, t2):
    m = abs(t2-t1)
    if m < pi:
        return m
    else:
        return 2*pi - m

def angle_diff(t1,t2):
    # For data collection
    t1,t2 = min(t1,t2), max(t1,t2)  # t2 > t1
    m = t2-t1
    if m > pi:
        m -= 2*pi
    return m
    

def compute_overlap_eps(i: Agent, j: Agent):
    # i and j are agents
    return min(max(i.rad() + j.rad() - d(i.theta_c(), j.theta_c()), 0), i.aod(), j.aod())

# def pairwise_interaction(agents: list[Agent], update_draw_func, sim=False):
#     if not sim: print("Begin Algorithm 4")
#     # Algorithm 4
#     N = len(agents)
#     return N-1


#     k = 1
#     run_flag = 1
#     while run_flag==1 and k<=N-2:
#         agent_n_k : Agent = agents[N-k-1]
#         agent_n_k_1 : Agent = agents[N-k-2]
#         update_draw_func([N-k-1,N-k-2])
#         wait = input("Iteration {} (N-k = {} ,  N-k-1 = {} ,  Overlap = {:5.3f} = {:5.3f} π : ".format(k,N-k,N-k-1,compute_overlap(agent_n_k,agent_n_k_1),compute_overlap(agent_n_k,agent_n_k_1)/pi))
#         # wait = input(f"Iteration {k} (N-k = {N-k} ,  N-k-1 = {N-k-1} ,  Overlap = {compute_overlap(agent_n_k,agent_n_k_1)} = {compute_overlap(agent_n_k,agent_n_k_1)/pi}): ")
#         S_overlap = compute_overlap(agent_n_k, agent_n_k_1)
#         if S_overlap>0 or angles_equal(agent_n_k_1.theta_u,agent_n_k.theta_l) or angles_equal(agent_n_k_1.theta_l,agent_n_k.theta_u):
            
#             # Update knowledge of agent[N-k-2]
#             # agent_n_k.n, agent_n_k_1.n = max(agent_n_k.n, agent_n_k_1.n), max(agent_n_k.n, agent_n_k_1.n)
#             # agent_n_k_1.theta_u = (agent_n_k.theta_u - 2*pi/N) % (2*pi)
#             # agent_n_k_1.theta_l = (agent_n_k.theta_l - 2*pi/N) % (2*pi)

#             lower_bound = agent_n_k_1.theta_l

#             agent_n_k.update_knowledge(max(agent_n_k.n, agent_n_k_1.n))
#             agent_n_k_1.update_knowledge(max(agent_n_k.n, agent_n_k_1.n))
#             # agent_n_k_1.move_to(agent_n_k.theta_c() - 2*pi/N)
#             agent_n_k_1.move_to(agent_n_k.theta_c() - 2*pi/N - 0.5*agent_n_k.epsilon - 0.5*agent_n_k_1.epsilon)

#             # if there is PREDICTED loss of coverage, move centroids and add epsilon
#             gap = (agent_n_k_1.theta_l - lower_bound) # % (2*pi)
#             if gap < -pi:
#                 gap += 2*pi
#             print(f"N-k-1 theta_l: {agent_n_k_1.theta_l}\t lower_bound: {lower_bound}\t Gap: {gap}")
#             update_draw_func([N-k-1,N-k-2])
#             wait = input("Fixing gap...")
#             if gap > 0:
#                 upper_bound = agent_n_k.theta_u
#                 area = (upper_bound-lower_bound) % (2*pi)
#                 agent_n_k_1.epsilon += gap/2
#                 agent_n_k.epsilon += gap/2
#                 agent_n_k_1.move_to( (lower_bound + 0.25*gap + 0.5*agent_n_k_1.aod()) % (2*pi) )
#                 agent_n_k.move_to(   (upper_bound - 0.25*gap - 0.5*agent_n_k.aod()  ) % (2*pi) )
        
#         else:
#             run_flag = 0
#             print("Run Flag = 0")
#             # Check for coverage
#         k = k+1
#     update_draw_func([])
#     return N-(k-1)-1

def compute_pc(agents : list[Agent], p):
    current_min = 2*pi
    current_j = -1
    agent_p: Agent = agents[p-1]
    for j,agent_j in enumerate(agents):
        if j == p-1:
            continue
        dist = (agent_p.theta_c() - agent_j.theta_c()) % (2*pi)
        overlapping = compute_overlap_eps(agent_p, agent_j)>0 or angles_equal(agent_j.theta_l,agent_p.theta_u) or angles_equal(agent_j.theta_u,agent_p.theta_l)
        if dist < pi and overlapping:
            if dist < current_min:
                current_min = dist
                current_j = j
    return current_j+1

def compute_pc_backwards(agents : list[Agent], p):
    current_min = 2*pi
    current_j = -1
    agent_p: Agent = agents[p-1]
    for j,agent_j in enumerate(agents):
        if j == p-1:
            continue
        dist = (agent_j.theta_c() - agent_p.theta_c()) % (2*pi)
        overlapping = compute_overlap_eps(agent_p, agent_j)>0 or angles_equal(agent_j.theta_l,agent_p.theta_u) or angles_equal(agent_j.theta_u,agent_p.theta_l)
        if dist < pi and overlapping:
            if dist < current_min:
                current_min = dist
                current_j = j
    return current_j+1


import random

def naive_extension(agents: list[Agent], update_draw_func, p, sim=False, kmax=2000, idx=0):
    # Need to first do algorithm 4, which terminates at agent p
    # Algorithm 6
    if not sim: print("Begin Algorithm 6")
    N = len(agents)
    k = 1
    int_num = 1
    run_flag = 1
    fname = f'./data/{N}_{idx}.csv'
    fname_eps = f'./data/{N}_eps.csv'
    fname_n = f'./data/{N}_n.csv'
    fname_c = f'./data/{N}_c.csv'
    last_epsilons = [a.epsilon/a.aod() for a in agents]
    last_n = [a.n for a in agents]
    last_c = [a.theta_c() for a in agents]

    # p = N
    # kmax = 2000

    # clear data file, write headings
    if sim:
        with open(fname, 'w') as f:
            f.write(f"Interaction," + ','.join([f"Agent_{i+1}_eps,Agent_{i+1}_n,Agent_{i+1}_c,Agent_{i+1}_aod" for i in range(N)]) + '\n')
            f.write(str(0) + ',' + ','.join([f"{0 if abs(a.epsilon)<1E-9 else (a.epsilon/a.aod())},{a.n},{a.theta_c()},{a.aod()}" for a in agents]) + '\n')


    while run_flag==1 and k<=kmax:

        # Randomise interactions
        p = random.randint(1,N)

        # pc = compute_pc(agents, p)
        forward = random.random() < 0.5

        # p = 5
        # forward = True

        # p = 4
        # forward = False

        # Get neighbours
        pc_forwards = compute_pc(agents, p)
        pc_backwards = compute_pc_backwards(agents, p)

        if pc_forwards==0 and pc_backwards==0:
            raise Exception("pc_fowards and pc_backwards invalid")
        
        # Swap pc if invalid
        if forward:
            # Forwards and valid
            if pc_forwards != 0:
                pc = pc_forwards
            # Switch to backward
            else:
                forward = False
                pc = pc_backwards
                
        else:
            # Backwards and valid
            if pc_backwards != 0:
                pc = pc_backwards
            # Switch to backward
            else:
                forward = True
                pc = pc_forwards
        
        agent_p = agents[p-1]
        agent_pc = agents[pc-1]

        # INTERACTION
        interact(agent_p, agent_pc, forward)
    

        k = k+1
        p = pc

        if k>N*2 and max([a.epsilon/a.aod() for a in agents])<1E-4:
            run_flag = 0

    if k>=kmax:
        print("            Reached kmax")
    update_draw_func([],None)



def test_neighbourhood(agent1: Agent, agent2: Agent):
    overlapping = (compute_overlap_eps(agent1, agent2)>0 or
                   angles_equal(agent2.theta_l,agent1.theta_u) or
                   angles_equal(agent2.theta_u,agent1.theta_l))
    return overlapping



def interact_simple(agent_p: Agent, agent_pc: Agent):

    """
    Modifies the Agent objects in-place
    """

    bounds = [
        (agent_p.theta_l,   agent_p.theta_u,    agent_p,    agent_pc),
        (agent_p.theta_l,   agent_pc.theta_u,   agent_p,    agent_pc),
        (agent_pc.theta_l,  agent_pc.theta_u,   agent_pc,   agent_p),
        (agent_pc.theta_l,  agent_p.theta_u,    agent_pc,   agent_p),
    ]

    def compute_bound(x):
        if angles_equal(x[0],x[1]):
            return 0
        return (x[1]-x[0])%(2*pi)

    biggest_bound = max(bounds, key=compute_bound)
    lower_bound, upper_bound, agent_lower, agent_upper = biggest_bound

    # move normally
    new_n = max(agent_p.n, agent_pc.n)
    agent_p.update_knowledge(new_n)
    agent_pc.update_knowledge(new_n)

    middle = agent_p.mid(lower_bound, upper_bound)
    agent_lower.theta_l = lower_bound
    agent_lower.theta_u = middle
    agent_upper.theta_l = middle
    agent_upper.theta_u = upper_bound



def interact(agent_p: Agent, agent_pc: Agent, forward: bool):

    """
    Modifies the Agent objects in-place
    """

    bounds = [
        (agent_p.theta_l, agent_p.theta_u),
        (agent_p.theta_l, agent_pc.theta_u),
        (agent_pc.theta_l, agent_pc.theta_u),
        (agent_pc.theta_l, agent_p.theta_u),
    ]

    def compute_bound(x):
        if angles_equal(x[0],x[1]):
            return 0
        return (x[1]-x[0])%(2*pi)

    biggest_bound = max(bounds, key=compute_bound)
    lower_bound, upper_bound = biggest_bound

    # move normally
    new_n = max(agent_p.n, agent_pc.n)
    agent_p.update_knowledge(new_n)
    agent_pc.update_knowledge(new_n)

    if forward:
        agent_pc.move_to(agent_p.theta_c() - agent_p.rad() - agent_pc.rad())
    else:
        agent_pc.move_to(agent_p.theta_c() + agent_p.rad() + agent_pc.rad())

    gap = ((upper_bound - lower_bound) % (2*pi)) - agent_p.base_aod() - agent_pc.base_aod()
    gap = max(0,gap)
    agent_pc.epsilon = gap/2
    agent_p.epsilon = gap/2

    # re-partition inside bounds, using agent_p as an anchor if their base aod's extend past the bounds
    if forward:
        agent_p.move_to(  (upper_bound - 0.25*gap - 0.5*agent_p.base_aod() ) % (2*pi) )
        agent_pc.move_to(  (agent_p.theta_l - 0.25*gap - 0.5*agent_pc.base_aod() ) % (2*pi) )
    else:
        agent_p.move_to(  (lower_bound + 0.25*gap + 0.5*agent_p.base_aod() ) % (2*pi) )
        agent_pc.move_to(  (agent_p.theta_u + 0.25*gap + 0.5*agent_pc.base_aod() ) % (2*pi) )