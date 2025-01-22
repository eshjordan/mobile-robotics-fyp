import multiprocessing.pool

from sympy.utilities.iterables import multiset_permutations

from algos import *

# Define constants
WIDTH, HEIGHT = 960, 960
WHITE = (255, 255, 255, 255)
BLACK = (0, 0, 0, 255)
GREEN = (50, 255, 0, 255)
GREEN_TRANS = (50, 255, 0, 100)
BLUE_TRANS = (50, 0, 255, 100)
RED = (255, 0, 0, 255)
RADIUS = 380  # Radius of the circle
ALPHA = 0.5

import sys
import multiprocessing
from tqdm import tqdm

def krange_process(arg):

    krange, num_iters = arg

    if num_iters == 1:

        max_gap = 0
        max_krange = []

        agents = sequential_add_int(NUM_AGENTS)

        gap = pairwise_interaction(agents=agents, update_draw_func=lambda x:x, krange_fixed=krange, max_iters=1, step=False, hush=True)
        if gap > max_gap:
            max_gap = gap
            max_krange = krange
    
        return (max_gap, max_krange)
    
    else:

        results = [ krange_process( (krange + krange2, num_iters-1) ) for krange2 in multiset_permutations(list(range(NUM_AGENTS)))  ]

        return max(results, key=lambda x:x[0])
            

def main():

    global NUM_AGENTS, NUM_ITERS
    NUM_AGENTS = 7
    NUM_ITERS = 1

    if len(sys.argv) >= 2:
        NUM_AGENTS = int(sys.argv[1])
    
    if len(sys.argv) >= 3:
        NUM_ITERS = int(sys.argv[2])

    NUM_AGENTS_RANGE = [NUM_AGENTS]
    MAX_GAPS = []
    MAX_KRANGES = []

    if NUM_AGENTS < 0:
        NUM_AGENTS_RANGE = list(range(3,abs(NUM_AGENTS)+1))
    
    for NUM_AGENTS in NUM_AGENTS_RANGE:
        print(f"NUM_AGENTS = {NUM_AGENTS}")
        max_gap = 0
        max_krange = []

        results = []

        with multiprocessing.Pool(processes=16) as pool: # MULTIPROCESSING
            input = [(krange, NUM_ITERS) for krange in list(multiset_permutations(list(range(NUM_AGENTS))))]
            for result in tqdm(pool.imap_unordered(krange_process, input), total=len(input)):
                results.append(result)
            pool.close()
            pool.join()
        
        (max_gap, max_krange) = max(results, key=lambda x:x[0])
        
        MAX_GAPS.append(max_gap)
        MAX_KRANGES.append(max_krange)


    print("\nNUM_ITERS =",NUM_ITERS,end="\n\n")
    print("{:10} {:7} {}".format("NUM_AGENTS","MAX_GAP","K_RANGE"))
    for i in range(len(MAX_GAPS)):
        print("{:10} {:7.4f} {}".format(i+3,MAX_GAPS[i],str(MAX_KRANGES[i])))


if __name__ == "__main__":
    main()