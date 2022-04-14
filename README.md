# Multi-Agent-Control-using-MPC
An early prototype which showcases the use of an efficient nonlinear MPC solver known as PANOC. Here it is used to solve optimization problems very quickly which allows for real-time applications such as the navigation of an autonomous agent. More information about PANOC can be found here: https://arxiv.org/abs/1709.06487

An easy way to use the solver is via the open source package, Optimization Engine, or OpEn for short. It can be found here: https://alphaville.github.io/optimization-engine/

## What is MPC ?

# How to Use
Prerequisites:  Python, OpEn, clang, Rust

Further information about dependant softwares/packages are well documented here: https://alphaville.github.io/optimization-engine/docs/installation

A linux environment is recommended  

Run the main file. The optimizer will be built first which can take up to a few minutes, after which the optimizer can be called in realtime to solve the optimization problem to compute the control inputs for the system. 

The output is a trajectory graph of the agent. Here 3 agent's paths are predetermined, while the autonomous agent (blue) navigates to the other agents by sharing state information while avoiding collision with each other. 

The path planning method/optimization problem is based on a modification of boids rules. More information about the modified boids rules can be found here: https://github.com/NA364KU/Multi-Agent-Leader-Follower-Control

<img width="961" alt="MPC_navigation" src="https://user-images.githubusercontent.com/95622570/163326251-b0fbf96d-81ee-4f95-8026-e38980bfb472.png">
