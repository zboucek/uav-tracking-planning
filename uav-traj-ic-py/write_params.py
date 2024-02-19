#!/usr/bin/env python
"""
Generates data with parameters for quadrotor Unmanned Aerial Vehicle (UAV).

The UAV is modelled as simple flat dynamics with linear constraints
for state, control, and disturbance.
"""
import numpy as np
import polytope as pc
import pickle

# parameters of system x(k+1) = A*x + B*u + w
dt = 0.01 # sampling period
A = np.array([[1, dt], [0, 1]])
B = np.array([[0], [dt]])

# linear constraints of system
xlim = pc.box2poly([[-10,10],[-5,5]]) #state x box constraints
F = np.array([[1.0],[-1.0]])
g = np.array([[50],[50]])
ulim = pc.box2poly([[-50,50]]) #control u box constraints
wlim = pc.box2poly([[-0.05,0.05],[-0.02,0.02]]) #disturbance w box constraints

# weights of cost function J(x,u) = x'*Q*x + u'*R*u
Q1 = np.array([[5000, 0], [0, 800]])
R1 = np.array(500)
Q2 = np.array(np.diag(np.power(xlim.b[:(xlim.b).size//2],-2)))
R2 = np.array(10*np.diag(np.power(ulim.b[:(ulim.b).size//2],-2)))

# save data to files "dynamics.pkl" and "sets.pkl"
f_params = 'dynamics.pkl'
f_sets = 'sets.pkl'
with open(f_params, 'wb') as outfile:
    pickle.dump([dt, A, B, Q1, Q2, R1, R2], outfile, pickle.HIGHEST_PROTOCOL)
with open(f_sets, 'wb') as outfile:
    pickle.dump([xlim,ulim,wlim], outfile, pickle.HIGHEST_PROTOCOL)