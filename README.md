# Multi-Agent-Control-using-MPC
An early prototype which showcases the use of an efficient nonlinear MPC solver known as PANOC. Here it is used to solve optimization problems very quickly which allows for real-time applications such as the navigation of an autonomous agent. More information about PANOC can be found here: https://arxiv.org/abs/1709.06487

## What is Model Predictive Control (MPC) ?

MPC is an advanced online (realtime) control method where an objective function is solved at every timestep. It uses existing state information as an input, along with known model dynamics to iteratively solve/minimize the function. This is repeated until a certain number of time steps into the future. The length of the prediction is known as the prediction horizon. 

Advantages:
1. Flexible parameters such as prediction horizon can be adjusted as needed
2. Robust to noise and modelling error due to re-solving the problem at every timestep

Disadvanatges:
1. Requires a known model
2. Until recently, difficult to use in low latency applications due to the time/computation required to solve the optimization problem

This is where the MPC solver, PANOC, comes into play. It is an efficient solver which can work in real-time, allowing for such low latency applications.

![MPC](https://miro.medium.com/max/1000/0*qOIUY20YJ-dhPeVB.png)
(Source: https://miro.medium.com/max/1000/0*qOIUY20YJ-dhPeVB.png)

# How to Use
Prerequisites:  Python, OpEn, clang, Rust

An easy way to use the solver is via the open source package, Optimization Engine, or OpEn for short. It can be found here: https://alphaville.github.io/optimization-engine/

Further information about dependant softwares/packages are well documented here: https://alphaville.github.io/optimization-engine/docs/installation

A linux environment is recommended.

Run the main file. The optimizer will be built first which can take up to a few minutes, after which the optimizer can be called in real-time.

The output generates a trajectory graph of the agent. Here 3 agent's paths are predetermined, while the autonomous agent (blue) navigates to the other agents by sharing state information while avoiding collision with each other. 

The path planning method/optimization problem is based on a modification of boids rules. More information about the modified boids rules can be found here: https://github.com/NA364KU/Multi-Agent-Leader-Follower-Control

## Generated Trajectory 

<img width="961" alt="MPC_navigation" src="https://user-images.githubusercontent.com/95622570/163326251-b0fbf96d-81ee-4f95-8026-e38980bfb472.png">

In this simple demonstration, we have 3 agents (For e.g mobile robots) that move in a predetermined path. 2 move in a straight line, while one moves in a sinusoidal path. 

The autonomous agent is placed near the middle agent deliberately. We can see that it tries to avoid the other agents while maintaing a formation and moving in the general direction of the multi-agent system. 

At every timestep, an optimization problem is solved using the MPC solver and optimal control inputs are applied to the autonomous agent. The problem is formulated using shared state information of neaby agents to avoid collision and to move together towards some goal. 

#### Smaller Details
This was tested on Linux Ubuntu 20.04 LTS using an i7 10th gen mobile processor. 

Each optimization problem takes roughly 1-10 ms to solve, which is very fast. 



