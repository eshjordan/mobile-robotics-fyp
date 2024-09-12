from math import pi

import typing

class Agent:

    def __init__(self, theta_l=None, theta_u=None, n=None):
        self.theta_u = theta_u
        self.theta_l = theta_l
        self.n = n
    
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
        self.theta_l = (c - pi/self.n) % (2*pi)
        self.theta_u = (c + pi/self.n) % (2*pi)




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
    if t1<t2:
        t1 += 2*pi
    eps = 1e-6
    return abs(t1-t2) < eps

def d(t1, t2):
    m = abs(t2-t1)
    if m < pi:
        return m
    else:
        return 2*pi - m

def compute_overlap(i, j):
    # i and j are agents
    return min(max(pi/i.n + pi/j.n - d(i.theta_c(), j.theta_c()), 0), 2*pi/i.n, 2*pi/j.n)

def pairwise_interaction(agents: list[Agent], update_draw_func):
    print("Begin Algorithm 4")
    # Algorithm 4
    N = len(agents)
    k = 1
    run_flag = 1
    while run_flag==1 and k<=N-2:
        agent_n_k : Agent = agents[N-k-1]
        agent_n_k_1 : Agent = agents[N-k-2]
        update_draw_func([N-k-1,N-k-2])
        wait = input("Iteration {} (N-k = {} ,  N-k-1 = {} ,  Overlap = {:5.3f} = {:5.3f} π : ".format(k,N-k,N-k-1,compute_overlap(agent_n_k,agent_n_k_1),compute_overlap(agent_n_k,agent_n_k_1)/pi))
        # wait = input(f"Iteration {k} (N-k = {N-k} ,  N-k-1 = {N-k-1} ,  Overlap = {compute_overlap(agent_n_k,agent_n_k_1)} = {compute_overlap(agent_n_k,agent_n_k_1)/pi}): ")
        S_overlap = compute_overlap(agent_n_k, agent_n_k_1)
        if S_overlap>0 or angles_equal(agent_n_k_1.theta_l,agent_n_k.theta_u) or angles_equal(agent_n_k_1.theta_u,agent_n_k.theta_l):
            
            # Update knowledge of agent[N-k-2]
            # agent_n_k.n, agent_n_k_1.n = max(agent_n_k.n, agent_n_k_1.n), max(agent_n_k.n, agent_n_k_1.n)
            # agent_n_k_1.theta_u = (agent_n_k.theta_u - 2*pi/N) % (2*pi)
            # agent_n_k_1.theta_l = (agent_n_k.theta_l - 2*pi/N) % (2*pi)

            agent_n_k.update_knowledge(max(agent_n_k.n, agent_n_k_1.n))
            agent_n_k_1.update_knowledge(max(agent_n_k.n, agent_n_k_1.n))
            agent_n_k_1.move_to(agent_n_k.theta_c() - 2*pi/N)
        
        else:
            run_flag = 0
            print("Run Flag = 0")
            # Check for coverage
        k = k+1
    update_draw_func([])
    return N-(k-1)-1


def pairwise_interaction_2(agents: list[Agent], update_draw_func):
    # Algorithm 5
    print("Begin Algorithm 5")
    N = len(agents)
    k = 1
    run_flag = 1
    while run_flag==1 and k<=N-2:
        wait = input(f"Iteration {k}: ")
        agent_k : Agent = agents[k-1]
        agent_k_1 : Agent = agents[k-2]
        S_overlap = compute_overlap(agent_k, agent_k_1)
        if S_overlap>0 or angles_equal(agent_k_1.theta_l,agent_k.theta_u) or angles_equal(agent_k_1.theta_u,agent_k.theta_l):

            # # Update knowledge of agent[k-1]
            # agent_k.n, agent_k_1.n = max(agent_k.n, agent_k_1.n), max(agent_k.n, agent_k_1.n)
            # # Update theta_c ???
            # agent_k.theta_u = (agent_k_1.theta_u + 2*pi/N) % (2*pi)
            # agent_k.theta_l = (agent_k_1.theta_l + 2*pi/N) % (2*pi)

            agent_k.update_knowledge(max(agent_k.n, agent_k_1.n))
            agent_k_1.update_knowledge(max(agent_k.n, agent_k_1.n))
            agent_k.move_to(agent_k_1.theta_c() - 2*pi/N)
        
        else:
            run_flag = 0
            print("Run Flag = 0")
            # Check for coverage
        k = k+1
        update_draw_func([])

def compute_pc(agents : list[Agent], p):
    current_min = 2*pi
    current_j = -1
    agent_p = agents[p-1]
    for j,agent_j in enumerate(agents):
        if j == p-1:
            continue
        dist = (agent_p.theta_c() - agent_j.theta_c()) % (2*pi)
        if dist < pi:
            if dist < current_min:
                current_min = dist
                current_j = j
    return current_j+1
        


def naive_extension(agents: list[Agent], update_draw_func, p):
    # Need to first do algorithm 4, which terminates at agent p
    # Algorithm 6
    print("Begin Algorithm 6")
    N = len(agents)
    k = 1
    run_flag = 1
    # p = N
    kmax = 50
    while run_flag==1 and k<=kmax:
        pc = compute_pc(agents, p)
        agent_p = agents[p-1]
        agent_pc = agents[pc-1]
        update_draw_func([p-1,pc-1])
        wait = input(f"Iteration {k} (p = {p},  pc = {pc})")
        S_overlap = compute_overlap(agent_p, agent_pc)
        if S_overlap>0 or angles_equal(agent_p.theta_l,agent_pc.theta_u) or angles_equal(agent_p.theta_u,agent_pc.theta_l):

            # agent_p.n, agent_pc.n = max(agent_p.n, agent_pc.n), max(agent_p.n, agent_pc.n)
            # agent_pc.theta_u = (agent_p.theta_u - 2*pi/N) % (2*pi)
            # agent_pc.theta_l = (agent_p.theta_l - 2*pi/N) % (2*pi)

            agent_p.update_knowledge(max(agent_p.n, agent_pc.n))
            agent_pc.update_knowledge(max(agent_p.n, agent_pc.n))
            agent_pc.move_to(agent_p.theta_c() - 2*pi/N)
    
        else:
            run_flag = 0
            print("Run Flag = 0")
        print("Is Q equipartitioned ??")
        k = k+1
        p = pc
    update_draw_func([])

