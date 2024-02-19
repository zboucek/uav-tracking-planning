#!/usr/bin/env python
# coding: utf-8

# # Interpolating Control

# The Interpolating Control (IC) is implemented in this notebook using CVXPY package. In IC, there is solved simple Linear Program (LP) in each time step. CVXPY serves as a universal Python interface (similarly to well known Yalmip for Matlab).
# 
# As the LP solver, Gurobi is employed  because of results in [1]. Nevertherless, thanks to CVXPY, solver can be switched simply by changing one parameter.

# [1] Z. Bouček and M. Flídr, "Interpolating Control Based Trajectory Tracking*," 2020 16th International Conference on Control, Automation, Robotics and Vision (ICARCV), Shenzhen, China, 2020, pp. 701-706, doi: 10.1109/ICARCV50220.2020.9305511.
# 
# [2] T. Baca et al., “Autonomous landing on a moving vehicle with an unmanned aerial vehicle,” J. F. Robot., no. January, 2019.)

# ## Imports

import cvxpy as cp
import numpy as np
from scipy.io import loadmat
import polytope as pc
import pickle

# ## Loading data
# 
# Parameters of UAV model are based on [2].

# Load parameters of dynamics and cost function.
with open('dynamics.pkl', 'rb') as infile:
    result = pickle.load(infile)
dt, A, B, Q1, Q2, R1, R2 = result

# Load linear constraints as polytopes.
with open('sets.pkl', 'rb') as infile:
    result = pickle.load(infile)
xlim, ulim, wlim = result

annots = loadmat('params/ic_params_py.mat')
L1 = annots['ctrl1'][0,0]['L']
K1 = annots['ctrl1'][0,0]['K']
invSet1 = pc.Polytope(annots['ctrl1'][0,0]['F'],
                    annots['ctrl1'][0,0]['g'])
L2 = annots['ctrl2'][0,0]['L']
K2 = annots['ctrl2'][0,0]['K']
invSet2 = pc.Polytope(annots['ctrl2'][0,0]['F'],
                    annots['ctrl2'][0,0]['g'])

# ## Set up problem

# Dimensionality
[nx,nu] = np.shape(B)
# Problem variables and parameters.
setpoint = cp.Parameter(nx)
x = cp.Parameter(nx)
c = cp.Variable(1)
rv = cp.Variable(nx)

# Initialization and data for saving the solution.
rk = np.zeros((nx,5000))
rk[0,100:] = 1
x0 = np.zeros(nx)
Nsims = 1
N = 800
xk_ic = np.zeros((nx,N+1))
xk_ic[:,0] = x0
uk_ic = np.zeros((nu,N))
times_ic = np.zeros((N,Nsims))
ck = np.zeros(N+1)
reftraj = np.reshape(rk, (1,-1), order='F')
traj = np.zeros(N*nx)

# ## Run simulation

# Objective function and constraints.
objective = cp.Minimize(c)
constraints = [invSet2.A@rv <= c*invSet2.b,
               invSet1.A@(x-rv) <= (1-c)*(invSet1.b + invSet1.A@setpoint),
               0 <= c, c <= 1]
prob_ic = cp.Problem(objective, constraints)
solver = cp.GUROBI

for j in range(Nsims):
    x.value = x0
    for i in range(1,N+1):
        # Set current trajectory and setpoint.
        traj = reftraj[:,(i-1)*nx:(i-1)*nx+N*nx]
        setpoint.value = traj[:,:nx].flatten()
        
        # Find interpolating coeficient.
        result = prob_ic.solve(solver)
        
        # Calc control input.
        r0 = x.value - rv.value
        uvk = K2@rv.value + c.value*L2@traj.T
        u0k = K1@r0 + (1-c.value)*L1@traj.T
        
        # Save data.
        uk_ic[:,i-1] = u0k + uvk
        #uk[:,i-1] = K1@x.value + L2@traj.T
        ck[i-1] = c.value
        # Run system dynamics.
        x.value = A@x.value + B@uk_ic[:,i-1]
        xk_ic[:,i] = x.value
        times_ic[i-1,j] = prob_ic.solver_stats.solve_time

# ## Save using Pandas to CSV

dict_ic = {'Timestamp': np.arange(0,N*dt,dt),
            'State1': xk_ic[0,1:N+1], 'State2': xk_ic[1,1:N+1],
            'Control': uk_ic[0,:N],
            'Ref1': rk[0,:N], 'Ref2': rk[1,:N],
            'Err1': xk_ic[0,1:N+1] - rk[0,:N],
            'Err2': xk_ic[1,1:N+1] - rk[1,:N],
            'Time Comp.': times_ic[:N,0]}
df_ic = pd.DataFrame(dict_ic)
df_ic.to_csv('sim_ic.csv',sep='\t',index=False,header=True)


# # Model Predictive Control
# 
# Model Predictive Control (MPC) will be covered for in two distinct implementations. First, the standard MPC will be implemented. Afterwards, a computationally more efficient alternative using the move blocking scheme. For the sake of efficiency, varying control input is considered only for small numer of time instants and for later for longer time periods it is constant.

xk_mpc = np.zeros((nx,N+1))
xk_mpc[:,0] = x0
uk_mpc = np.zeros((nu,N))
times_mpc = np.zeros((N,Nsims))
u = cp.Variable((nu,N))
x = cp.Variable((nx,N+1))
r = cp.Parameter((nx,N+1))
x_init = cp.Parameter(nx)

# Objective function and constraints.
cost_mpc = 0
constraints_mpc = [x[:,0] == x_init]
for k in range(N):
    cost_mpc += cp.quad_form(x[:,k]-r[:,k], Q1) 
    cost_mpc += cp.quad_form(u[:,k], np.array([[R1]]))
    constraints_mpc += [x[:,k+1] == A@x[:,k] + B@u[:,k]]
    constraints_mpc += [xlim.A@x[:,k] <= xlim.b]
    constraints_mpc += [ulim.A@u[:,k] <= ulim.b]
    
objective_mpc = cp.Minimize(cost_mpc)
prob_mpc = cp.Problem(objective_mpc, constraints_mpc)

for j in range(Nsims):
    x_init.value = x0
    for i in range(1,N+1):
        # Set current trajectory.
        r.value = rk[:,i:i+N+1]
        
        # Find control strategy.
        result = prob_mpc.solve(solver)
        
        # Save data.
        uk_mpc[:,i-1] = u[:,0].value
        # Run system dynamics.
        x_init.value = A@x_init.value + B@uk_mpc[:,i-1]
        xk_mpc[:,i] = x_init.value
        times_mpc[i-1,j] = prob_mpc.solver_stats.solve_time

dict_mpc = {'Timestamp': np.arange(0,N*dt,dt),
            'State1': xk_mpc[0,1:N+1], 'State2': xk_mpc[1,1:N+1],
            'Control': uk_mpc[0,:N],
            'Ref1': rk[0,:N], 'Ref2': rk[1,:N],
            'Err1': xk_mpc[0,1:N+1] - rk[0,:N],
            'Err2': xk_mpc[1,1:N+1] - rk[1,:N],
            'Time Comp.': times_mpc[:N,0]}
df_mpc = pd.DataFrame(dict_mpc)
df_mpc.to_csv('sim_mpc.csv',sep='\t',index=False,header=True)


# # MPC with move blocking
# 
# In the first step, the dynamics is conscidered as fast as in the standard model. However, in latter steps the discrete system has much slower period from $T_s = 0.01$ to $T_s = 0.2$. Because of the change, the weights in criterion must be altered accordingly. Therefore, weights are multipled by 10 for i=2:N horizon. The change is not reflected in the evaluation of controller performance, the same criterion is employed.

xk_mpc_block = np.zeros((nx,N+1))
xk_mpc_block[:,0] = x0
Ublock = [1,1,1,1,1,5,5,5,5,5,10]
Nblock = np.sum(Ublock)
Uindex = np.zeros(np.shape(Ublock))
for k in range(len(Ublock)):
    Uindex[k] = np.sum(Ublock[:k])
uk_mpc_block = np.zeros((nu,N))
times_mpc_block = np.zeros((N,Nsims))
u = cp.Variable((nu,np.size(Ublock)))
x = cp.Variable((nx,Nblock+1))
r = cp.Parameter((nx,Nblock+1))
x_init = cp.Parameter(nx)
Ad = np.copy(A)
Bd = np.copy(B)
Qd = np.copy(Q1)
Rd = np.copy(R1)
dtd = dt

# Objective function and constraints.
cost_mpc_block = 0
constraints_mpc_block = [x[:,0] == x_init]
for k in range(Nblock):
    j = np.where(Uindex == k)
    if np.size(j) != 0:
        u_idx = j[0][0]
    cost_mpc_block += cp.quad_form(x[:,k]-r[:,k], Qd)
    cost_mpc_block += cp.quad_form(u[:,u_idx], np.array([[Rd]]))
    constraints_mpc_block += [x[:,k+1] == Ad@x[:,k] + Bd@u[:,u_idx]]
    constraints_mpc_block += [xlim.A@x[:,k] <= xlim.b]
    constraints_mpc_block += [ulim.A@u[:,u_idx] <= ulim.b]
    if k == 0:
        dtd = 0.2
        Ad = np.array([[1,dtd],[0,1]])
        Bd = np.array([[0],[dtd]])
        Qd = 10*np.copy(Q1)
        Rd = 10*np.copy(R1)
    
objective_mpc_block = cp.Minimize(cost_mpc_block)
prob_mpc_block = cp.Problem(objective_mpc_block, constraints_mpc_block)

for j in range(Nsims):
    x_init.value = x0
    for i in range(1,N+1):
        # Set current trajectory.
        index = np.concatenate(([i],list(range(i+1,i+N,20))))
        r.value = rk[:,index]
        
        # Find control strategy.
        result = prob_mpc_block.solve(solver)
        
        # Save data.
        uk_mpc_block[:,i-1] = u[:,0].value
        # Run system dynamics.
        x_init.value = A@x_init.value + B@uk_mpc_block[:,i-1]
        xk_mpc_block[:,i] = x_init.value
        times_mpc_block[i-1,j] = prob_mpc_block.solver_stats.solve_time

dict_mpc_block = {'Timestamp': np.arange(0,N*dt,dt),
                  'State1': xk_mpc_block[0,1:N+1], 'State2': xk_mpc_block[1,1:N+1],
                  'Control': uk_mpc_block[0,:N],
                  'Ref1': rk[0,:N], 'Ref2': rk[1,:N],
                  'Err1': xk_mpc_block[0,1:N+1] - rk[0,:N],
                  'Err2': xk_mpc_block[1,1:N+1] - rk[1,:N],
                  'Time Comp.': times_mpc_block[:N,0]}
df_mpc_block = pd.DataFrame(dict_mpc_block)
df_mpc_block.to_csv('sim_mpc_block.csv',sep='\t',index=False,header=True)