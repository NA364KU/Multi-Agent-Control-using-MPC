import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import functions as f
import math
import datetime
import os
# p = position , v = velocity, u = acceleration/input
# 4 states: x, y, vx, vy

# x = [ x; y; vx; vy]
# dx = [vx; vy; ux; uy]
# u = [ux; uy]
gamma = 0.9 #discount function
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

########################################################################################################################
#make predetermined position and velocity of 3 agents, they have all same inital velocity
#they all move in a straight line
agent_vel = 1 #constant

#generate x axis positions in increments of 0.1
pos = []
pos_sin = []
vx_sin = []
vy_sin = []
pos_val = 0
increment = 0.05
for i in range(1000):
    pos_val += increment
    pos.append(pos_val)
    pos_sin.append(1 + 0.5*math.sin(pos_val))
    vx_sin.append(0.5*math.cos(pos_val)*math.cos(math.atan(0.5*math.cos(pos_val))))
    vy_sin.append(0.5*math.cos(pos_val)*math.sin(math.atan(0.5*math.cos(pos_val))))
agent_1 = pos
agent_2 = pos
agent_3 = pos

#Set trajecotry for other agents, index corresponds to y axis value (does not change with time)
agents = [[agent_1,[0]*1000,[1]*1000,[0]*1000],[pos,pos_sin,vx_sin,vy_sin],[agent_3,[2]*1000,[1]*1000,[0]*1000]]
########################################################################################################################

########################################################################################################################
#Build Problem

#u = cs.SX.sym('u', nu) #inputs vector 2 x 1 Ux Uy // ax ay
#z0 = cs.SX.sym('z0', nx) #makes a 4 by 1 matrix for px,py, Vx, Vy

u_seq = cs.SX.sym("u", nu*N)  # sequence of all u's
x0 = cs.SX.sym("x0", nx)   # initial state

x_t = x0
o_a = (agent_num-1)*2 #other_agent state indexes
total_cost = 0
ut = 0
sig_t = 4
for t in range(0, N):
    total_cost += f.stage_cost(x_t,u_seq[ut:ut+2],sig_t)  # update cost
    x_t = f.dynamics_dt(x_t, u_seq[ut:ut+2])         # update state
    if t==-1:
        f1 = cs.vertcat((x_t[0] - x_t[-(o_a - 0)]) ** 2 + (x_t[1] - x_t[-(o_a - 1)]) ** 2 - some_distance ** 2,
                        (x_t[0] - x_t[-(o_a - 2)]) ** 2 + (x_t[1] - x_t[-(o_a - 3)]) ** 2 - some_distance ** 2,
                        (x_t[0] - x_t[-(o_a - 4)]) ** 2 + (x_t[1] - x_t[-(o_a - 5)]) ** 2 - some_distance ** 2)
    ut += 2
    sig_t += 4

#optional terminal cost
#total_cost += terminal_cost(x_t)  # terminal cost

#add constraints here

f1 = cs.vertcat((x_t[0] - x_t[-(o_a - 0)]) ** 2 + (x_t[1] - x_t[-(o_a - 1)]) ** 2 - some_distance ** 2,
                        (x_t[0] - x_t[-(o_a - 2)]) ** 2 + (x_t[1] - x_t[-(o_a - 3)]) ** 2 - some_distance ** 2,
                        (x_t[0] - x_t[-(o_a - 4)]) ** 2 + (x_t[1] - x_t[-(o_a - 5)]) ** 2 - some_distance ** 2)

u_mag = 0.5
umin = [-u_mag] * (nu*N)
umax = [u_mag] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

fmin = []
fmax = []

fmin.extend([0]*(agent_num-1))
fmax.extend([math.inf]*(agent_num-1))

set_c = og.constraints.Rectangle(fmin,fmax)

problem = og.builder.Problem(u_seq, x0, total_cost)  \
    .with_aug_lagrangian_constraints(f1,set_c)        \
    .with_constraints(bounds)

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_build")      \
    .with_build_python_bindings()

meta = og.config.OptimizerMeta().with_optimizer_name("navigation")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-6)\
    .with_initial_tolerance(1e-6)

builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()
########################################################################################################################

#Run simulation loop

#import generated module
import sys
path = os.getcwd() + r"/python_build/navigation"
sys.path.insert(1,path)
import navigation
solver = navigation.solver()

# Run simulations
x_state_0 = [0.1, 0.95, 0, 0] #inital true/real states
x_sig_init = [-0.75,0.25,0.75,0]*N  #add inital values here for sigma
x_state_0.extend(x_sig_init)

for X in agents: x_state_0.extend([X[0][0],X[1][0]])

simulation_steps = 100
state_sequence = x_state_0
input_sequence = []

#use parametric variable
x = x_state_0

#Main loop
for k in range(simulation_steps):
    a = datetime.datetime.now()
    if k!=0:
        sig = f.neighbor_set(x,agents,k)
        for i in range(n_true,nx): x[i] = sig[i-4]
    result = solver.run(x)
    us = result.solution
    u = us[0:nu]
    x_next = f.dynamics_dt(x, u)
    state_sequence += x_next
    input_sequence += [u]
    x = x_next
    x_next_next = f.initial_x(x, us)
    x = x_next_next
    b = datetime.datetime.now()
    delta = b-a
    print("Computation time for Iteration ",k," : ", delta, )

time = np.arange(0, sampling_time*simulation_steps, sampling_time)

#Print output trajectory of agents


plt.plot(state_sequence[0:nx*simulation_steps:nx],state_sequence[1:nx*simulation_steps:nx], 'o', label="Autonomous agent's trajectory")
plt.plot(agents[0][0][:simulation_steps],agents[0][1][:simulation_steps],'o',label = "Agent 1")
plt.plot(agents[1][0][:simulation_steps],agents[1][1][:simulation_steps],'o',label = "Agent 2")
plt.plot(agents[2][0][:simulation_steps],agents[2][1][:simulation_steps],'o',label = "Agent 3")

plt.grid()
plt.ylabel('Y')
plt.xlabel('X')
plt.legend(bbox_to_anchor=(0.7, 0.85), loc='upper left', borderaxespad=0.)

plt.show()
