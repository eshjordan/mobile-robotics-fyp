/*
Full CUDA translation of algos.py for GPU
 */

#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "math.h"
#include "stdio.h"

#include "limits.h"

// #include "perms.cpp"

#define PI 3.14159265358979323846f

#define min(X, Y) (((X) > (Y)) ? (Y) : (X))
#define max(X, Y) (((X) < (Y)) ? (Y) : (X))

#define MIN_NUM_AGENTS 5
#define MAX_NUM_AGENTS 8
#define NUM_ITERS 3

#define THREAD_PER_BLOCK 1024
#define SPLIT_SIZE (2048*1024)

__host__ __device__
float mod(float x, float y) {
    float res = fmodf(x,y);
    if (res < 0) {
        res += y;
    }
    return res;
}

// #define NUM_ITERS 1




// #define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
// inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
// {
//    if (code != cudaSuccess) 
//    {
//       fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
//       if (abort) exit(code);
//    }
// }




class Agent {
public:
    float theta_u;
    float theta_l;
    int n;
    float eps;

    __host__ __device__
        Agent(float theta_l = NULL, float theta_u = NULL, int n = NULL, float eps = 0)
        : theta_l(theta_l), theta_u(theta_u), n(n), eps(eps) {}

    __host__ __device__
        float mid(float t1, float t2) const {
        if (t2 < t1) t2 += 2 * PI;
        return mod(0.5f * (t1 + t2), 2 * PI);
    }

    __host__ __device__
        float theta_c() const {
        return mid(theta_l, theta_u);
    }

    __host__ __device__
        void update_knowledge(int new_n) {
        n = new_n;
        move_to(theta_c()); // update bounds
    }

    __host__ __device__
        void move_to(float c) {
        theta_l = mod(c - PI / n, 2 * PI);
        theta_u = mod(c + PI / n, 2 * PI);
    }
};



__device__
float d(float t1, float t2) {
    float m = fabsf(t2 - t1);
    return (m < PI) ? m : (2 * PI - m);
}

__device__
float compute_overlap_neg(Agent* i, Agent* j) {
    // allow negative overlap, to compute max gap
    return fminf(PI / i->n + PI / j->n + i->eps / 2 + j->eps / 2 - d(i->theta_c(), j->theta_c()), fminf(2 * PI / i->n, 2 * PI / j->n));
}

__host__ __device__
unsigned long long factorial(int N) {
    int fact = 1, i;
    for (i = 1; i <= N; i++) { fact *= i; }
    return fact;
}

__host__ __device__
unsigned long long power(unsigned long long X, int Y) {
    // compute X^Y
    unsigned long long pwr = 1, i;
    for (i = 0; i < Y; i++) { pwr *= X; }
    return pwr;
}

// Function to generate the nth permutation of the array
__host__ __device__
void getNthPermutation(unsigned long long n, int result[], int num_agents, bool* taken) {
    // Adjust n to be zero-based
    // n -= 1;

    // clear arrays
    for (int i = 0; i < num_agents; i++) {
        result[i] = -1;
        taken[i] = false;
    }

    // Compute the permutation
    for (int i = 0; i < num_agents; i++) {
        int fact = factorial(num_agents - 1 - i);
        int index = n / fact;
        int count = 0;

        // Find the (index + 1)th available number
        for (int j = 0; j < num_agents; j++) {
            if (!taken[j]) {
                if (count == index) {
                    //result[i] = nums[j];
                    result[i] = j;
                    taken[j] = true;
                    break;
                }
                count++;
            }
        }

        n %= fact;
    }
}


__host__ __device__
void getNthPermutationMultIters(int num_iters, unsigned long long n, int result[], int num_agents, bool* taken) {


    unsigned long long NUM_TASKS = factorial(num_agents); // for a single iter

    for (int i = 0; i < num_iters; i++) {
        unsigned long long base = power(NUM_TASKS, i);
        getNthPermutation((n / base) % NUM_TASKS, &(result[i * num_agents]), num_agents, taken);
    }
}


__device__
int compute_nn(const Agent* agents, int k, int NUM_AGENTS) {
    float current_min = 2 * PI;
    int current_j = -1;
    for (int j = 0; j < NUM_AGENTS; ++j) {
        if (j == k) continue;
        float dist = mod(agents[k].theta_c() - agents[j].theta_c(), 2 * PI);
        if (dist < current_min) {
            current_min = dist;
            current_j = j;
        }
    }
    return current_j;
}


__host__
void sequential_add_in(int num_agents, Agent* agents) {

    /*
     * In: num_agents
     * Out: agents (space already allocated)
    */

    Agent new_agent;
    new_agent.update_knowledge(2);
    new_agent.move_to(0);

    agents[0] = Agent();
    agents[0].update_knowledge(2);
    agents[0].move_to(0);

    agents[1] = Agent();
    agents[1].update_knowledge(3);
    agents[1].move_to(2 * PI / 3);

    for (int p = 2; p < num_agents - 1; p++) {

        float theta_c = PI;
        for (int m = 3; m <= p + 1; m++) {
            theta_c += PI / m;
        }
        theta_c -= PI / (p + 2);

        agents[p] = Agent();
        agents[p].update_knowledge(p + 2);
        agents[p].move_to(theta_c);
    }

    // last one
    float theta_c = PI;
    for (int m = 3; m <= num_agents; m++) {
        theta_c += PI / m;
    }
    agents[num_agents - 1] = Agent();
    agents[num_agents - 1].update_knowledge(num_agents);
    agents[num_agents - 1].move_to(theta_c);
}


__global__
void pairwise_interaction(int NUM_AGENTS, float* max_gap_arr, Agent* d_agents, Agent* d_agents_init, int* d_krange, bool* d_taken, int split, int NUM_TASKS) {

    //unsigned long long NUM_TASKS, TASK_IDX;
    //NUM_TASKS = factorial(NUM_AGENTS);
    unsigned long long TASK_IDX;
    TASK_IDX = THREAD_PER_BLOCK * blockIdx.x + threadIdx.x;

    // don't compute if outside task range
    if (TASK_IDX < NUM_TASKS) {

        int KRANGE_SIZE = NUM_AGENTS * NUM_ITERS;

        Agent* agents = &(d_agents[TASK_IDX * NUM_AGENTS]);
        int* krange = &(d_krange[TASK_IDX * NUM_AGENTS * NUM_ITERS]);
        bool* taken = &(d_taken[TASK_IDX * NUM_AGENTS]);

        float max_gap = 0, overlap_neg;
        int i = 0, k, nn, new_n;
        Agent* agent_k, * agent_k_nn; // *agents;

        // init agents
        //cudaMemcpy(agents, d_agents_init, NUM_AGENTS * sizeof(Agent), cudaMemcpyDevicetoDevice);
        for (int i = 0; i < NUM_AGENTS; i++) {
            agents[i] = d_agents_init[i];
        }

        // sequential_add_in(NUM_AGENTS, agents);
        //getNthPermutation(TASK_IDX, krange, NUM_AGENTS, taken);
        getNthPermutationMultIters(NUM_ITERS, (split*SPLIT_SIZE) + TASK_IDX, krange, NUM_AGENTS, taken);

        while (i < KRANGE_SIZE) {
            k = krange[i];
            agent_k = &agents[k];
            nn = compute_nn(agents, k, NUM_AGENTS);
            agent_k_nn = &agents[nn];
            overlap_neg = compute_overlap_neg(agent_k, agent_k_nn);
            if (-overlap_neg > max_gap) {
                max_gap = -overlap_neg;
            }

            // Assume run_flag not unset

            new_n = max(agent_k->n, agent_k_nn->n);
            agent_k->update_knowledge(new_n);
            agent_k_nn->update_knowledge(new_n);
            agent_k->move_to(agent_k_nn->theta_c() + 2 * PI / new_n);
            i += 1;
        }
    
        max_gap_arr[TASK_IDX] = max_gap;
        //max_gap_arr[TASK_IDX] = TASK_IDX;
    }

}




int main() {

    printf("Hello\n\n");

    float* max_gap_arr;
    float max_gap;
    int NUM_BLOCKS, NUM_THREADS, split;
    unsigned long long NUM_TASKS, TOT_NUM_TASKS, arg_max_gap;

    Agent* agents_init, * d_agents_init, * d_agents;
    int* d_krange;
    bool* d_taken;

    for (int NUM_AGENTS = MIN_NUM_AGENTS; NUM_AGENTS <= MAX_NUM_AGENTS; NUM_AGENTS++) {


        //TOT_NUM_TASKS = factorial(NUM_AGENTS);
        NUM_TASKS = factorial(NUM_AGENTS);
        if (NUM_TASKS > pow(ULLONG_MAX, 1.0 / NUM_ITERS)) {
            printf("%i agents with %i iterations is too large. Stopping.\n", NUM_AGENTS, NUM_ITERS);
            break;
        }
        TOT_NUM_TASKS = power(NUM_TASKS, NUM_ITERS);

        printf("Computing max gap for %i agents with %i iterations: %llu tasks\n", NUM_AGENTS, NUM_ITERS, TOT_NUM_TASKS);


        // Allocate memory

        // Max gap array
        cudaMallocManaged(&max_gap_arr, SPLIT_SIZE * sizeof(float));
        // Agents init array
        agents_init = (Agent*)malloc(NUM_AGENTS * sizeof(Agent));
        sequential_add_in(NUM_AGENTS, agents_init);
        cudaMalloc(&d_agents_init, NUM_AGENTS * sizeof(Agent));
        cudaMemcpy(d_agents_init, agents_init, NUM_AGENTS * sizeof(Agent), cudaMemcpyHostToDevice);
        // Agents array
        cudaMalloc(&d_agents, SPLIT_SIZE * NUM_AGENTS * sizeof(Agent));
        // d_krange
        cudaMalloc(&d_krange, SPLIT_SIZE * NUM_AGENTS * NUM_ITERS * sizeof(int)); // agents will compute their own krange (???????)
        // taken
        cudaMalloc(&d_taken, SPLIT_SIZE * NUM_AGENTS * sizeof(bool)); // each task only needs 1xNUM_AGENTS array of bools for 'taken'


        // Split up into SPLIT_SIZE
        split = 0;
        max_gap = 0;
        arg_max_gap = 0;
        while (TOT_NUM_TASKS > 0)
        {
            NUM_TASKS = min(TOT_NUM_TASKS, SPLIT_SIZE);

            //// Reset stuff
            //for (int i = 0; i < NUM_TASKS; i++) {
            //    cudaMemcpy(&(d_agents[i * NUM_AGENTS]), agents_init, NUM_AGENTS * sizeof(Agent), cudaMemcpyHostToDevice);
            //}


            //printf("Max gap array initialised\n");

            NUM_BLOCKS = 1 + ((NUM_TASKS-1) / THREAD_PER_BLOCK);
            //NUM_THREADS = min(THREAD_PER_BLOCK, NUM_TASKS);
            NUM_THREADS = THREAD_PER_BLOCK;

            printf("\rUsing %i blocks and %i threads per block...", NUM_BLOCKS, NUM_THREADS);

            pairwise_interaction << <NUM_BLOCKS, NUM_THREADS >> > (NUM_AGENTS, max_gap_arr, d_agents, d_agents_init, d_krange, d_taken, split, NUM_TASKS);
            cudaDeviceSynchronize();



            //printf("Kernel completed\n");

            // find max
            //printf("Computing max gap\n");
            for (int i = 0; i < NUM_TASKS; i++) {
                //printf("%.3f ", max_gap_arr[i]);
                if (max_gap_arr[i] > max_gap) {
                    max_gap = max_gap_arr[i];
                    arg_max_gap = (split*SPLIT_SIZE) + i;
                    //printf("Max gap is now %.3f\n", max_gap);
                }
            }

            printf("%llu done", (split * SPLIT_SIZE) + NUM_TASKS);

            TOT_NUM_TASKS -= NUM_TASKS;
            split += 1;

            
        }


        //printf("\nComputed max gap\n");

        printf("\nMax gap for %i agents over %i iterations is %.4f\n", NUM_AGENTS, NUM_ITERS, max_gap);
        int* arg_max_gap_perm = (int*)malloc(NUM_AGENTS * NUM_ITERS * sizeof(int));
        bool* taken = (bool*)malloc(NUM_AGENTS * sizeof(bool));
        for (int i = 0; i < NUM_AGENTS; i++) {
            taken[i] = false;
        }
        getNthPermutationMultIters(NUM_ITERS, arg_max_gap, arg_max_gap_perm, NUM_AGENTS, taken);
        printf("[");
        for (int i = 0; i < NUM_AGENTS * NUM_ITERS; i++) {
            printf(" %i,", arg_max_gap_perm[i]);
        }
        printf("]\n\n");
        free(arg_max_gap_perm);
        free(taken);


        cudaFree(max_gap_arr);
        cudaFree(d_agents);
        cudaFree(d_krange);
        cudaFree(d_taken);

        free(agents_init);
    }

    //cudaFree(&d_init_arr);

    return 0;
}

