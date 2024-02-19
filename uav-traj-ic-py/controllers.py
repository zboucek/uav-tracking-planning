#!/usr/bin/env python
# coding: utf-8

# Interpolating Control

# The initial functions for Interpolating Control (IC), standard Model Predictive Control (MPC),
# and MPC with move-blocking scheme is implemented using CVXPY package. Move Blocking scheme is implemented accoridng to [2].
#
# [1] Z. Bouček and M. Flídr, "Interpolating Control Based Trajectory Tracking*," 2020 16th International Conference on Control, Automation, Robotics and Vision (ICARCV), Shenzhen, China, 2020, pp. 701-706, doi: 10.1109/ICARCV50220.2020.9305511.
#
# [2] T. Baca et al., “Autonomous landing on a moving vehicle with an unmanned aerial vehicle,” J. F. Robot., no. January, 2019.)

import cvxpy as cp
import numpy as np
import polytope as pc

from dlqr import dlqtraj


class ControllerLQ(object):
    """ Linear-Quadratic Regulator (LQR) with setpoint control ability
    """

    def __init__(self, gain, compensation=None):
        """ Set parameters of LQR

        Input:
          gain: gain matrix
          comp: matrix for compensation to setpoint
        """
        self.gain = gain
        if compensation is not None:
            self.compensation = compensation
        else:
            self.compensation = np.zeros(self.gain.shape)

    def spin(self, state, setpoint=None):
        """ Get control action based on current state and setpoint
        """
        if setpoint is None:
            return self.gain@state
        else:
            return self.gain@state + self.compensation@setpoint


class ControllerPID(object):
    """ Simple PID controller
    """

    def __init__(self, Kp=0, Kd=0, Ki=0, dt=1e-3, t_now=0):
        """ Set parameters of PID

        Input:
          Kp: gain parameter
          Kd: derivative coefficient
          Ki: integral part coefficient
          t_now: current time in [s]
        """
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.t_last = t_now
        self.dt = dt
        self.last_control_error = 0.0

    def spin(self, state, setpoint=None, t_now=None):
        """ Get control action based on current state, setpoint, and current time
        """

        if hasattr(state, "__len__"):
            if len(state) > 1:
                state = state[0]
        if setpoint is None:
            control_error = - state
        else:
            if hasattr(setpoint, "__len__"):
                if len(setpoint) > 1:
                    setpoint = setpoint[0]
            control_error = setpoint - state

        if t_now is None:
            delta_time = self.dt
        else:
            delta_time = t_now - self.t_last

        proportional = self.Kp * control_error
        integral = self.Ki * control_error
        if delta_time > 0:
            derivative = self.Kd * \
                ((control_error - self.last_control_error) / (delta_time))
        else:
            derivative = 0

        desired_process = proportional + integral + derivative
        self.t_last = t_now
        self.last_control_error = control_error

        return desired_process


class ControllerIC(object):
    """ Interpolating Controller for constrained optimal control of linear discrete-time system
    """

    def __init__(self, invSet_h, invSet_l, K_h, L_h, K_l, L_l, solver):
        """Init Linear Program for solution of Interpolating Control.

        Input:
          invSet_h:  invariant set for high-gain controller
          invSet_l:  invariant set for low-gain controller
          K_h:       high-gain controller gain
          K_l:       low-gain controller gain
          L_h:       compensation of high-gain controller to given trajectory
          L_l:       compensation of low-gain controller to given trajectory
          solver:   solve the problem with 
        """

        # Dimensionality
        self.nx = invSet_h.dim
        # Problem variables and parameters.
        self.setpoint = cp.Parameter(self.nx)
        self.x = cp.Parameter(self.nx)
        self.c = cp.Variable(1)
        self.rv = cp.Variable(self.nx)
        # Set up LP
        objective = cp.Minimize(self.c)
        constraints = [invSet_l.A@self.rv <= self.c*invSet_l.b,
                       invSet_h.A@(self.x-self.rv) <= (1-self.c) *
                       (invSet_h.b + invSet_h.A@self.setpoint),
                       0 <= self.c, self.c <= 1]
        self.problem = cp.Problem(objective, constraints)
        self.solver = solver

        # Paramters of controller
        self.K_h = K_h
        self.K_l = K_l
        self.L_h = L_h
        self.L_l = L_l

    def spin(self, trajectory, state):
        """Run the Interpolating Controller.

        Return control input for desired trajectory and current state.

        Input:
          trajectory: desired trajectory N-D (state at time k in k-th column)
          state:      current state of controlled system
        Output:
          control value
        """
        # Assign parameter values
        if state.ndim > 1:
            self.x.value = state[:, 0]
        else:
            self.x.value = state
        self.setpoint.value = trajectory[:, 0]
        self.r = trajectory.flatten('F').T

        # Find interpolating coefficient
        self.problem.solve(self.solver, reoptimize = True)

        if self.rv.value is None:
            return None, None, None

        # Interpolate control input
        r0 = self.x.value - self.rv.value
        uv = self.K_l@self.rv.value + self.c.value*self.L_l@self.r
        u0 = self.K_h@r0 + (1-self.c.value)*self.L_h@self.r
        u = u0 + uv

        return u[0], self.c.value[0], self.problem.solver_stats.solve_time

class ControllerICExtended(object):
    
    def __init__(self, invSet_h, invSet_m, invSet_l, K_h, L_h, K_m, L_m, K_l, L_l, solver):
        """Init Linear Programs for solution of extended Interpolating Control.

        Input:
          invSet_h:  invariant set for high-gain controller
          invSet_h:  invariant set for medium-gain controller
          invSet_l:  invariant set for low-gain controller
          K_h:       high-gain controller gain
          K_h:       medium-gain controller gain
          K_l:       low-gain controller gain
          L_h:       compensation of high-gain controller to given trajectory
          L_h:       compensation of medium-gain controller to given trajectory
          L_l:       compensation of low-gain controller to given trajectory
          solver:   solve the problem with 
        """

        # Dimensionality
        self.nx = invSet_h.dim
        # Problem variables and parameters.
        self.setpoint = cp.Parameter(self.nx)
        self.x = cp.Parameter(self.nx)
        self.c1 = cp.Variable(1)
        self.c2 = cp.Variable(1)
        self.rv = cp.Variable(self.nx)
        self.rs = cp.Variable(self.nx)
        # Set up LP
        objective_l = cp.Minimize(self.c1)
        objective_h = cp.Minimize(self.c2)
        constraints_l = [invSet_l.A@self.rv <= self.c1*invSet_l.b,
                       invSet_m.A@(self.x-self.rv) <= (1-self.c1) *
                       (invSet_m.b + invSet_m.A@self.setpoint),
                       0 <= self.c1, self.c1 <= 1]
        constraints_h = [invSet_m.A@self.rs <= self.c2*(invSet_m.b+invSet_m.A@self.setpoint),
                       invSet_h.A@(self.x-self.rs) <= (1-self.c2) *
                       (invSet_h.b + invSet_h.A@self.setpoint),
                       0 <= self.c2, self.c2 <= 1]
        self.problem_l = cp.Problem(objective_l, constraints_l)
        self.problem_h = cp.Problem(objective_h, constraints_h)
        self.solver = solver
        self.invSet_m = invSet_m

        # Paramters of controller
        self.K_h = K_h
        self.K_m = K_m
        self.K_l = K_l
        self.L_h = L_h
        self.L_m = L_m
        self.L_l = L_l
    
    def spin(self, trajectory, state):
        """Run the extended Interpolating Controller.

        Return control input for desired trajectory and current state.

        Input:
          trajectory: desired trajectory N-D (state at time k in k-th column)
          state:      current state of controlled system
        Output:
          control value
        """
        # Assign parameter values
        if state.ndim > 1:
            self.x.value = state[:, 0]
        else:
            self.x.value = state
        self.setpoint.value = trajectory[:, 0]
        self.r = trajectory.flatten('F').T

        # Find interpolating coefficient
        c = [0,0]
        stats = []
        if [self.x.value-self.setpoint.value] in self.invSet_m:
            self.problem_h.solve(self.solver, reoptimize = True)
            if self.rs.value is None:
                return None, None, None
             # Interpolate control input
            r0 = self.x.value - self.rs.value
            uv = self.K_m@self.rs.value + self.c2.value*self.L_m@self.setpoint.value
            u0 = self.K_h@r0 + (1-self.c2.value)*self.L_h@self.r
            u = u0 + uv
            c[1] = self.c2.value[0]
            stats = self.problem_h.solver_stats.solve_time
        else:
            self.problem_l.solve(self.solver, reoptimize = True)
            if self.rv.value is None:
                return None, None, None
            # Interpolate control input
            r0 = self.x.value - self.rv.value
            uv = self.K_l@self.rv.value + self.c1.value*self.L_l@self.setpoint.value
            u0 = self.K_m@r0 + (1-self.c1.value)*self.L_m@self.setpoint.value
            u = u0 + uv
            c[0] = self.c1.value[0]
            stats = self.problem_l.solver_stats.solve_time

        return u[0], c, stats

class ControllerMPC(object):
    """ Model Predictive Controller for constrained optimal control of linear discrete-time system
    """

    def __init__(self, xlim, ulim, A, B, R, Q, N, solver):
        """Init Quadratic Program for solution of standard Model Predictive Control.

        Input:
          xlim: polytope with limits of state vector
          ulim: polytope with limits of control vector
          A:    matrix of system dynamics
          B:    matrix of control mapping
          Q:    weight matrix of state
          R:    weight matrix of control action
          N:    length of predictive horizon
        """

        # Dimensionality
        self.nx = xlim.dim
        self.nu = ulim.dim
        # Problem variables and parameters.
        self.u = cp.Variable((self.nu, N))
        self.x = cp.Variable((self.nx, N+1))
        self.r = cp.Parameter((self.nx, N))
        self.x_init = cp.Parameter(self.nx)

        # Objective function and constraints.
        cost = 0
        constraints = [self.x[:, 0] == self.x_init]
        for k in range(N):
            cost += cp.quad_form(self.x[:, k]-self.r[:, k], Q)
            cost += cp.quad_form(self.u[:, k], np.array([[R]]))
            constraints += [self.x[:, k+1] == A@self.x[:, k] + B@self.u[:, k]]
            constraints += [xlim.A@self.x[:, k] <= xlim.b]
            constraints += [ulim.A@self.u[:, k] <= ulim.b]

        objective = cp.Minimize(cost)
        self.problem = cp.Problem(objective, constraints)
        self.solver = solver

    def spin(self, trajectory, state):
        """Run the Model Predictive Controller.

        Return control input for desired trajectory and current state.

        Input:
          trajectory: desired trajectory N-D (state at time k in k-th column)
          state:      current state of controlled system
        Output:
          control value
        """
        # Assign parameter values
        if state.ndim > 1:
            self.x_init.value = state[:, 0]
        else:
            self.x_init.value = state
        self.r.value = trajectory

        # Find control strategy
        self.problem.solve(self.solver, reoptimize = True)

        return self.u[0, 0].value, self.problem.objective.value, self.problem.solver_stats.solve_time


class ControllerMPCMB(object):
    """ Model Predictive Controller with move-blocking scheme for constrained optimal control of linear discrete-time system
    """

    def __init__(self, xlim, ulim, A, B, R, Q, dt, dt2, Ublock, A2, B2, Q2, R2, N, solver=cp.GUROBI):
        """Init Quadratic Program for solution of Model Predictive Control 
        with move-blocking scheme.

        Input:
          xlim: polytope with limits of state vector
          ulim: polytope with limits of control vector
          A:    matrix of system dynamics
          B:    matrix of control mapping
          dt:   first step sampling period
          dt2:  subsequent steps sampling period
          Ublock: matrix with move-blocking scheme 
          Q:    weight matrix of state
          R:    weight matrix of control action
          N:    length of predictive horizon
        """

        # Dimensionality
        self.nx = xlim.dim
        self.nu = ulim.dim
        # Auxiliary variables for move-blocking scheme.
        Nblock = np.sum(Ublock)
        Uindex = np.zeros(np.shape(Ublock))
        for k in range(len(Ublock)):
            Uindex[k] = np.sum(Ublock[:k])
        self.N = N
        self.r_index = np.concatenate(([0], list(range(1, 1+N, 20))))
        # Problem variables and parameters.
        self.u = cp.Variable((self.nu, np.size(Ublock)))
        self.x = cp.Variable((self.nx, Nblock+1))
        self.r = cp.Parameter((self.nx, Nblock+1))
        self.x_init = cp.Parameter(self.nx)

        Ad = np.copy(A)
        Bd = np.copy(B)
        Qd = np.copy(Q)
        Rd = np.copy(R)

        # Objective function and constraints.
        cost = 0
        constraints = [self.x[:, 0] == self.x_init]
        for k in range(Nblock):
            j = np.where(Uindex == k)
            if np.size(j) != 0:
                u_idx = j[0][0]
            cost += cp.quad_form(self.x[:, k]-self.r[:, k], Qd)
            cost += cp.quad_form(self.u[:, u_idx], np.array([[Rd]]))
            constraints += [self.x[:, k+1] == Ad @
                            self.x[:, k] + Bd@self.u[:, u_idx]]
            constraints += [xlim.A@self.x[:, k] <= xlim.b]
            constraints += [ulim.A@self.u[:, u_idx] <= ulim.b]
            if k == 0:
                # Switch dynamics.
                Ad = A2
                Bd = B2
                Qd = Q2
                Rd = R2

        objective = cp.Minimize(cost)
        self.problem = cp.Problem(objective, constraints)
        self.solver = solver

    def spin(self, trajectory, state):
        """Run the Model Predictive Controller with move-blocking scheme.

        Return control input for desired trajectory and current state.

        Input:
          trajectory: desired trajectory N-D (state at time k in k-th column)
          state:      current state of controlled system
        Output:
          control value
        """
        # Assign parameter values
        if state.ndim > 1:
            self.x_init.value = state[:, 0]
        else:
            self.x_init.value = state
        self.r.value = trajectory[:, self.r_index]

        # Find control strategy
        self.problem.solve(self.solver, reoptimize = True)

        return self.u[0, 0].value, self.problem.objective.value, self.problem.solver_stats.solve_time


def test_lqr():
    """ Test of LQR using 2D discrete time linear system
    """
    from dlqr import dlqsp

    # system parameters
    A = np.array([[1, 0.5], [0, 1]])
    B = np.array([[0], [0.7]])
    Q = np.diag([100, 10])
    R = np.array([1e-3])

    # LQR acquisition
    K, L = dlqsp(A, B, Q, R)
    lqr = ControllerLQ(K, L)

    # set variables
    setpoint = np.array([[1], [0]])
    x0 = np.zeros((2, 1))
    x0[0, 0] = -1
    N = 10
    xk = np.zeros((2, N+1))
    uk = np.zeros((1, N))
    xk[:, 0] = x0[:, 0]

    # fixing compansation matrix
    L[0, 0] = -K[0, 0]
    L[0, 1] = 0

    for k in range(N):
        uk[:, k] = lqr.spin(xk[:, k], setpoint)
        xk[:, k+1] = A@xk[:, k] + B@uk[:, k]

    print(xk[0, :])


def test_pid():
    """ Test of PID using 2D discrete time linear system
    """

    # system parameters
    A = np.array([[1, 1e-3], [0, 1]])
    B = np.array([[0.00008], [0.15625]])

    Kp = 0.15
    Kd = 0.003
    Ki = 0.05
    t0 = 0.0

    # PID acquisition
    pid = ControllerPID(Kp, Kd, Ki, t0)

    # set variables
    dt = 1e-3
    setpoint = np.array([[0.1], [0]])
    x0 = np.zeros((2, 1))
    x0[0, 0] = -0.2
    N = 50000
    xk = np.zeros((2, N+1))
    uk = np.zeros((1, N))
    xk[:, 0] = x0[:, 0]
    t = np.linspace(t0+dt, N*dt, N)

    for k in range(N):
        uk[:, k] = pid.spin(xk[:, k], setpoint, t[k])
        xk[:, k+1] = A@xk[:, k] + B@uk[:, k]

    print(xk[0, :])


def test():
    """Test of constrained controllers.

    Files with system and linear-quadratic regulator parameters needed!!!
    Test is using GUROBI solver (can be changed on the line #268).

    Printed output is the control value

    Files to run example:
      dynamics.pkl
      params/ic_params_py.mat
    """
    import pickle
    from multiprocessing import Pool
    import time

    # load parameters of dynamics and cost function
    with open('dynamics.pkl', 'rb') as infile:
        result = pickle.load(infile)
    dt, A, B, Q1, Q2, R1, R2 = result

    # load linear constraints as polytopes
    with open('sets.pkl', 'rb') as infile:
        result = pickle.load(infile)
    xlim, ulim, wlim = result

    from scipy.io import loadmat
    annots = loadmat('params/ic_params_py.mat')
    L_h = annots['ctrl1'][0, 0]['L']
    K_h = annots['ctrl1'][0, 0]['K']
    invSet_h = pc.Polytope(annots['ctrl1'][0, 0]['F'],
                           annots['ctrl1'][0, 0]['g'])
    L_l = annots['ctrl2'][0, 0]['L']
    K_l = annots['ctrl2'][0, 0]['K']
    invSet_l = pc.Polytope(annots['ctrl2'][0, 0]['F'],
                           annots['ctrl2'][0, 0]['g'])

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    solver = cp.GUROBI

    # Parameters
    N = 800
    nx = 2
    x0 = np.zeros((nx, 1))
    x0[0] = 0.9

    # Init desired trajectory
    trajectory = np.zeros((nx, N))
    A2 = np.array([[1, dt2], [0, 1]])
    B2 = np.array([[0], [dt2]])
    # Init controllers
    N_test = 20
    ic_x = ControllerIC(invSet_h, invSet_l, K_h, L_h, K_l, L_l, solver)
    mpc_mb_x = ControllerMPCMB(
        xlim, ulim, A, B, R1, Q1, dt, dt2, Ublock, A2, B2, 10*Q1, 10*R1, N, solver)
    mpc_x = ControllerMPC(xlim, ulim, A, B, R1, Q1, N, solver)

    # Test IC
    for i in range(N_test):
        ctrl, interp_ctrl, comp_time = ic_x.spin(trajectory, x0[:, 0])
        print(ctrl)
        x0 = A@x0 + B*ctrl

    # Test move-blocking MPC
    x0[0] = 0.9
    x0[1] = 0
    for i in range(N_test):
        ctrl, crit, comp_time = mpc_mb_x.spin(trajectory, x0)
        print(ctrl)
        x0 = A@x0 + B*ctrl

    # Test MPC
    x0[0] = 0.9
    x0[1] = 0
    for i in range(200):
        ctrl, crit, comp_time = mpc_x.spin(trajectory, x0)
        print(ctrl)
        x0 = A@x0 + B*ctrl

    # Test of IC with parallelism
    t_start = time.time()
    N_test = 3000
    x0 = np.random.rand(2, 3*N_test)

    for i in range(N_test):
        ctrl = ic_x.spin(trajectory, x0[:, i])
        ctrl = ic_x.spin(trajectory, x0[:, N_test+i])
        ctrl = ic_x.spin(trajectory, x0[:, 2*N_test+i])

    ic_x2 = ControllerIC(invSet_h, invSet_l, K_h, L_h, K_l, L_l, solver)
    t_execution = time.time()-t_start
    print(f'Execution time: {t_execution}s')

    with Pool(processes=4) as pool:
        t_start = time.time()
        # Test IC
        for i in range(N_test):
            ctrl, crit, comp_time = pool.starmap(ic_x2.spin, [(trajectory, x0[:, i]),
                                                              (trajectory,
                                                               x0[:, N_test+i]),
                                                              (trajectory, x0[:, 2*N_test+i])])

    t_execution = time.time()-t_start
    print(f'Execution time (parallel): {t_execution}s')


if __name__ == "__main__":
    test()
    # test_lqr()
    # test_pid()
