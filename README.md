# [TTK4115-Linear Systems Theory](http://www.ntnu.edu/studies/courses/TTK4115#tab=omEmnet)
Projects from the course TTK4115 - Linear Systems Theory @ NTNU

**Course content:** Theory for linear multivariable systems, state space models, discretization, canonical forms and realizations, Lyapunov stability, controllability and observability, state feedback, LQR control, state estimation, the Kalman filter, descriptions of stochastic processes and random signals. 

The course had two projects that counted 50% towards the final grade, a helicopter project (30%) and a ship autopilot project (20%).

## Helicopter laboratory assignement
The point of the helicopter project was to:
*  Derive a model for a given system (helicopter) to be used for control purposes
*  Derive and apply PD and multivariable controllers in a real-time environment
*  Demonstrate how states that are not directly measured can be estimated
*  Give an introduction to optimal control by developing a linear quadratic regulator (LQR) for the helicopter.

The project is divded into four parts:

* **PART I:** Mathematical Modeling
* **PART II:** Monovariable Control
* **PART III:** Multivariable Control
* **PART IV:** State Estimation


We used a [quanser helicopter](http://www.quanser.com/products/3dof_helicopter) with 3 degrees of freedom.
We were given an MATLAB code for initialization of the system, and a Simulink-model to start with. 


## Ship autopilot using discrete Kalman filtering
The purpose of the ship assignement was to:

*  Partly model and simulate a continuous system influenced by stochstic signals
*  Use basic identification techniques on parameters that are not explicitly given
*  Use basic control theory to design a simple autopilot
*  Implement a discrete Kalman filter for wave filtering and estimation of disturbances using MATLAB and Simulink

This project used a given ship model in Simulink as a starting point, a .mat file containing wave data and a Kalman filter function shell i MATLAB (we instead implemented one separate). The report is divided into five sections:

* **PART I:** Identification of the boat parameters
* **PART II:** Identification of the wave spectrum model
* **PART III:** Control system design
* **PART IV:** System observability
* **PART V:** Discrete Kalman filter
