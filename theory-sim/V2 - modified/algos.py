from math import pi

import typing

class Agent:

    def __init__(self, theta_l=None, theta_u=None, n=None, eps=0):
        self.theta_u = theta_u
        self.theta_l = theta_l
        self.n = n
        self.eps = eps
    
    @staticmethod
    def default_1_2():
        # Factory to initialise first 2
        a1 = Agent(theta_l=3*pi/2, theta_u=pi/2, n=2)
        a2 = Agent(theta_l=pi/2, theta_u=3*pi/2, n=2)
        return [a1, a2]
     
    def __str__(self):
        # return f"Agent: 0_l = {round(self.theta_l,3)}, 0_u = {round(self.theta_u,3)}, 0_c = {round(self.theta_c(),3)}, n = {self.n}"
        return "Agent:   ε = {:.3f} = {:.3f} π,    0_l = {:.3f} = {:.3f} π,    0_u = {:.3f} = {:.3f} π,    0_c = {:.3f} = {:.3f} π,    n = {}".format(self.eps,self.eps/pi,self.theta_l, self.theta_l/pi, self.theta_u, self.theta_u/pi, self.theta_c(), self.theta_c()/pi, self.n)
    
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
    
    # Assign epsilon
    k = 1
    while N-k >= 1:
        ind = N-k-1 # python indexing
        # agents[ind].eps = max(0, calc_D(N,k), calc_D(N,k-1))
        k += 1
    
    return agents

def get_bound_angles(self):
    return ((self.theta_l - self.eps/2)%(2*pi), (self.theta_u + self.eps/2)%(2*pi))

def get_upper_altruism(self):
    return (self.theta_u, (self.theta_u + self.eps/2)%(2*pi))

def get_lower_altruism(self):
    return ((self.theta_l - self.eps/2)%(2*pi), self.theta_l)

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
    # return min(max(pi/i.n + pi/j.n - d(i.theta_c(), j.theta_c()), 0), 2*pi/i.n, 2*pi/j.n)
    return min(max(pi/i.n + pi/j.n + i.eps/2 + j.eps/2 - d(i.theta_c(), j.theta_c()), 0), 2*pi/i.n, 2*pi/j.n)

def compute_overlap_neg(i, j):
    # i and j are agents
    # return min(max(pi/i.n + pi/j.n - d(i.theta_c(), j.theta_c()), 0), 2*pi/i.n, 2*pi/j.n)
    return min(pi/i.n + pi/j.n + i.eps/2 + j.eps/2 - d(i.theta_c(), j.theta_c()), 2*pi/i.n, 2*pi/j.n)

import random

def pairwise_interaction(agents: list[Agent], update_draw_func, krange_fixed=None, max_iters=-1, step=True, hush=False):
    # print("Begin Algorithm 4")
    # Algorithm 4
    # SHUFFLED ORDERING
    # Basically Algorithm 6 now

    # krange = [0, 1, 5, 4, 3, 2]

    N = len(agents)
    run_flag = 1
    iters = 0
    max_gap = 0

    while run_flag==1 and iters!=max_iters:

        i = 0

        krange = list(range(N))
        random.shuffle(krange)
        if krange_fixed is not None:
            krange = krange_fixed

        if not hush:
            print(krange) # python indexing

        while run_flag==1 and i<len(krange):
            k = krange[i]
            agent_k : Agent = agents[k]  # python indexing
            nn = compute_nn(agents, k)
            agent_k_nn : Agent = agents[nn]   # python indexing
            update_draw_func([k, nn])  # 1-based indexing
            # S_overlap = compute_overlap(agent_k,agent_k_nn)
            overlap_neg = compute_overlap_neg(agent_k,agent_k_nn)
            if (-overlap_neg) > max_gap:
                max_gap = -overlap_neg
            
            if hush and step:
                wait = input("")
            if not hush:
                print("Iteration {} (k = {} ,  k_nn = {} ,  Overlap = {:5.3f} = {:5.3f} π".format(i+1, k+1, nn+1, overlap_neg, overlap_neg/pi), end="")
                if step:
                    wait = input(" : ")
            # else:
            #     print("")
            
            # if S_overlap>0 or angles_equal(agent_n_k_1.theta_l,agent_n_k.theta_u) or angles_equal(agent_n_k_1.theta_u,agent_n_k.theta_l):
            if True:
                
                # Update knowledge of agent[N-k-2]
                new_n = max(agent_k.n, agent_k_nn.n)
                agent_k.update_knowledge(new_n)
                agent_k_nn.update_knowledge(new_n)
                # Move
                agent_k.move_to(agent_k_nn.theta_c() + 2*pi/new_n)
                # agent_k_nn.move_to(agent_k.theta_c() - 2*pi/new_n)
            
            else:
                run_flag = 0
                print("Run Flag = 0")
                # Check for coverage

            i = i+1

        update_draw_func([])
        iters += 1

    return max_gap
    # return N-(k-1)-1


def compute_nn(agents : list[Agent], k):
    current_min = 2*pi
    current_j = -1
    for j in range(len(agents)):
        if j == k:
            continue
        dist = (agents[k].theta_c() - agents[j].theta_c()) % (2*pi)
        # if dist < pi:
        if dist < current_min:
            current_min = dist
            current_j = j
    return current_j



# def pairwise_interaction(agents: list[Agent], update_draw_func):
#     print("Begin Algorithm 4")
#     # Algorithm 4
#     # SHUFFLED ORDERING
#     N = len(agents)
#     krange = list(range(1,N-2 + 1))
#     random.shuffle(krange)
#     print(krange)
#     # k = 1
#     i = 0
#     run_flag = 1
#     # while run_flag==1 and k<=N-2:
#     while run_flag==1 and i<len(krange):
#         k = krange[i]
#         agent_n_k : Agent = agents[(N-k)-1]
#         agent_n_k_1 : Agent = agents[(N-k-2)]
#         update_draw_func([N-k-1,N-k-2])
#         S_overlap = compute_overlap(agent_n_k,agent_n_k_1)
#         wait = input("Iteration {} (N-k = {} ,  N-k-1 = {} ,  Overlap = {:5.3f} = {:5.3f} π : ".format(k,N-k,N-k-1,S_overlap,S_overlap/pi))
#         # wait = input(f"Iteration {k} (N-k = {N-k} ,  N-k-1 = {N-k-1} ,  Overlap = {compute_overlap(agent_n_k,agent_n_k_1)} = {compute_overlap(agent_n_k,agent_n_k_1)/pi}): ")
        
#         # if S_overlap>0 or angles_equal(agent_n_k_1.theta_l,agent_n_k.theta_u) or angles_equal(agent_n_k_1.theta_u,agent_n_k.theta_l):
#         if True:
            
#             # Update knowledge of agent[N-k-2]
#             new_n = max(agent_n_k.n, agent_n_k_1.n)
#             agent_n_k.update_knowledge(new_n)
#             agent_n_k_1.update_knowledge(new_n)
#             agent_n_k_1.move_to(agent_n_k.theta_c() - 2*pi/new_n)
        
#         else:
#             run_flag = 0
#             print("Run Flag = 0")
#             # Check for coverage

#         # k = k+1
#         i = i+1

#     update_draw_func([])
#     return N-(k-1)-1



"""
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
        
        
        # if S_overlap>0 or angles_equal(agent_n_k_1.theta_l,agent_n_k.theta_u) or angles_equal(agent_n_k_1.theta_u,agent_n_k.theta_l):
        if True:
            
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
"""


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
        # if True:

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
        # if S_overlap>0 or angles_equal(agent_p.theta_l,agent_pc.theta_u) or angles_equal(agent_p.theta_u,agent_pc.theta_l):
        if True:
            # agent_p.n, agent_pc.n = max(agent_p.n, agent_pc.n), max(agent_p.n, agent_pc.n)
            # agent_pc.theta_u = (agent_p.theta_u - 2*pi/N) % (2*pi)
            # agent_pc.theta_l = (agent_p.theta_l - 2*pi/N) % (2*pi)

            new_n = max(agent_p.n, agent_pc.n)
            agent_p.update_knowledge(new_n)
            agent_pc.update_knowledge(new_n)
            agent_pc.move_to(agent_p.theta_c() - 2*pi/new_n)
    
        else:
            run_flag = 0
            print("Run Flag = 0")
        print("Is Q equipartitioned ??")
        k = k+1
        p = pc
    update_draw_func([])


def calc_D(N,k):
    D_n_k = None
    if N-k-1 == 2:
        D_n_k = sum([pi/m for m in range(3,N)]) - 2*pi*k/N
    elif N-k-1 == 1:
        D_n_k = sum([pi/m for m in range(3,N)]) - 2*pi*k/N + pi/2
    elif N-k-1 == 0:
        D_n_k = 0
    else:
        D_n_k = sum([pi/m for m in range(N-k,N)]) - 2*pi*k/N
    return D_n_k