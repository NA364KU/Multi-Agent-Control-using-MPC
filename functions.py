import numpy as np

gamma = 1 #discount function
w = 0.5 #weight
N = 15 #number of time steps
agent_num = 4 # number of agents
some_distance = 0.5 #safety dist [m]
sampling_time = 0.01 #[s]
n_true = 4
n_required = 4
n_position = 2
n_vel = 2
nx = n_true + n_required*N + (agent_num-1)*2 #n_true*agent_num*N   # number of states
nu = 2 # number of inputs

def initial_x(x,u):
    x_result = [x[0],x[1],x[2],x[3]]

    u_t = 2
    for i in range(N-1):
        x_next = dyn2(x_result,u[u_t:u_t+2])
        x_result.extend(x_next)
        u_t += 2

    x_result.extend(x_next)
    x_result.extend(x[n_true + N*n_required:]) #n_true + N*n_required
    return x_result

def dyn2(x,u):
    sampling_time = 0.1
    x_next = [x[0] + sampling_time * x[2] + 0.5 * u[0] * sampling_time ** 2,
                x[1] + sampling_time * x[3] + 0.5 * u[1] * sampling_time ** 2,
                x[2] + u[0] * sampling_time,
                x[3] + u[1] * sampling_time]
    return x_next

def neighbor_set(x,X,time_step):

    sigma_final = []
    j=0
    d = 10 #neighbour dist
    sig_c = 0
    for i in range(N):
        count = 1
        sig = [x[sig_c], x[sig_c+1], x[sig_c+2], x[sig_c+3]]
        for agent in X:
            if np.sqrt((x[sig_c] - agent[0][time_step])**2 + (x[sig_c+1] - agent[1][time_step])**2) < d:
                sig[0] += agent[0][time_step]
                sig[1] += agent[1][time_step]
                sig[2] += agent[2][time_step]
                sig[3] += agent[3][time_step]
                count += 1
            j += 1
        sig_c+=4
        sig1 = np.array(sig)
        time_step += 1
        j = 0
        sigma_final.extend(sig1/count)
    c = 0

    #adds next states of neighbor agents in order to compute future sigma
    for i in range(n_true + N*n_required,nx,n_position):
        x[i] = X[c][0][time_step]
        sigma_final.append(x[i])
        x[i+1] = X[c][1][time_step]
        sigma_final.append(x[i+1])
        c += 1

    return sigma_final

def dynamics_dt(x, u): #X: set of other agents
    sampling_time = 0.1
    x_lower = []
    for i in range(n_true,nx): x_lower.append(x[i])

    x_higher = [x[0] + sampling_time * x[2] + 0.5 * u[0] * sampling_time ** 2,
            x[1] + sampling_time * x[3] + 0.5 * u[1] * sampling_time ** 2,
            x[2] + u[0] * sampling_time,
            x[3] + u[1] * sampling_time]
    x_higher.extend(x_lower)
    return x_higher

def stage_cost(x,u_t,sig_t): #set of all other agents:  x = current state , sig_t= index of dummy states, used to compute sig values
    
    # cost function
    cost = gamma*((1-w)*(((x[0] + x[2] * sampling_time + 0.5 * u_t[0] * sampling_time ** 2) - x[sig_t]) ** 2 +
                         ((x[1] + x[3] * sampling_time + 0.5 * u_t[1] * sampling_time ** 2) - x[sig_t + 1]) ** 2) +
                      w*(((x[2] + u_t[0] * sampling_time) - x[sig_t + 2]) ** 2 +
                         ((x[3] + u_t[1] * sampling_time) - x[sig_t + 3]) ** 2))

    return cost
