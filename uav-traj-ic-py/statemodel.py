import numpy as np
import control as ctrl
from invset import calcInvSet
import polytope as pc
from dlqr import dlqsp, dlqtraj


class StateModel:
    def __init__(self, A, B, C, D, xlim, ulim, Q_h, R_h, Q_l, R_l, dt, N, set_h, set_l, control, dt2=None, Ublock=None, Q_m=None, R_m=None, set_m=None):
        # system params
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.nx = np.shape(A)[0]
        self.nu = np.shape(B)[1]
        self.xlim = xlim
        self.Q_h = Q_h
        self.Q_m = Q_m
        self.Q_l = Q_l
        self.ulim = ulim
        self.R_h = R_h
        self.R_m = R_m
        self.R_l = R_l
        self.dt = dt
        self.dt2 = dt2
        self.Ublock = Ublock
        self.N = N
        self.set_h = set_h
        self.set_m = set_m
        self.set_l = set_l
        # system params for moveblocking
        self.Q_h2 = 10*np.copy(Q_h)
        self.R_h2 = 10*np.copy(R_h)

        # discretization, invariant sets construction and controller desing
        self.sys = ctrl.c2d(
            ctrl.ss(self.A, self.B, self.C, self.D), self.dt, 'zoh')
        self.K_h = control['LQ1']['K'][0,0]
        self.L_h = control['LQ1']['L'][0,0]
        if dt2 is not None:
            if self.set_m is not None:
                self.K_l = control['LQ3']['K'][0,0]
                self.L_l = control['LQ3']['L'][0,0]
                self.K_m = control['LQ2']['K'][0,0]
                self.L_m = control['LQ2']['L'][0,0]
            else:
                self.K_l = control['LQ2']['K'][0,0]
                self.L_l = control['LQ2']['L'][0,0]
            self.sys2 = ctrl.c2d(
                ctrl.ss(self.A, self.B, self.C, self.D), self.dt2, 'zoh')

    def save(self, file_name="system", folder_name=""):
        import cloudpickle

        # save data to file
        with open(folder_name+"/"+file_name+".pkl", 'wb') as outfile:
            cloudpickle.dump(self, outfile)

    def plot_sets(self):
        import matplotlib.pyplot as plt
        if self.set_m is None:
            r = pc.Region([self.xlim, self.set_l, self.set_h])
        else:
            r = pc.Region([self.xlim, self.set_l, self.set_m, self.set_h])
        r.plot()
        xmin, ymin = 1.1*np.min(self.xlim.vertices, 0)
        xmax, ymax = 1.1*np.max(self.xlim.vertices, 0)
        plt.xlim([xmin, xmax])
        plt.ylim([ymin, ymax])
        plt.show()


def flat_dynamics():
    dt = 0.01
    A = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    B = np.array([[0], [0], [dt]])
    C = np.eye(np.shape(A)[0])
    D = np.zeros((np.shape(A)[1], 1))

    Q = np.array([[5000, 0, 0], [0, 800, 0], [0, 0, 800]])
    R = np.array(500)

    pc.box2poly([[-0.05, 0.05], [-0.02, 0.02], [-0.01, 0.01]])
    model2 = ctrl.StateSpace(A, B, C, D, dt)
    print(model2)
    xmin = pc.box2poly([[-10, 10], [-5, 5], [-2, 2]])
    print(xmin)
    F = np.array([[1.0], [-1.0]])
    g = np.array([[50], [50]])
    umin = pc.Polytope(F, g)
    print(umin)
    K = ctrl.lqr(np.matrix(A), np.matrix(B), np.matrix(Q), np.matrix(R))


def plannar_uav(save_params=False, show_plots=False):
    from uav_params import UAVParams
    from scipy.io import loadmat
    # load invariant sets from Matlab file (constructing using MPT3)
    set_params = loadmat('params/sets_0.01_room')

    # general parameters
    # sampling
    time_horizon = 8.0
    dt_phi = 1e-3
    dt = 1e-2
    N_phi = 1  # int(time_horizon/dt_phi)
    N = int(time_horizon/dt)
    
    # attitude control
    control_phi = loadmat('params/lqr_phi_0.01_room_params.mat')
    # set matrix of dynamics and input
    A_phi = np.zeros([2, 2])
    A_phi[0, 1] = 1
    B_phi = np.zeros([2, 1])
    B_phi[1, 0] = 1/UAVParams.Ix
    C_phi = np.eye(np.shape(A_phi)[0])
    D_phi = np.zeros((np.shape(A_phi)[1], 1))

    # weight matrices
    Q_phi = np.diag([1/np.deg2rad(10)**2, 1/np.deg2rad(100)**2])
    R_phi = np.array(1/UAVParams.tau_max**2)

    phi_x_lim = pc.box2poly([[UAVParams.angle_min, UAVParams.angle_max], [
                            UAVParams.arate_min, UAVParams.arate_max]])
    phi_u_lim = pc.box2poly([[UAVParams.tau_min, UAVParams.tau_max]])

    Q_phi2 = np.diag(np.array([UAVParams.angle_max, UAVParams.arate_max])**-2)
    R_phi2 = np.diag(10*np.array([UAVParams.tau_max])**-2)

    phi_set_h = pc.Polytope(set_params['set_h_f_A'], set_params['set_h_f_b'])
    phi_model = StateModel(A_phi, B_phi, C_phi, D_phi, phi_x_lim, phi_u_lim,
                           Q_phi, R_phi, Q_phi2, R_phi2, dt_phi, N_phi, phi_set_h, None, control_phi)

    # lateral control
    control_y = loadmat('params/lqr_y_0.01_room_params.mat')
    # set matrix of dynamics and input
    A_y = np.zeros([2, 2])
    A_y[0, 1] = 1
    B_y = np.zeros([2, 1])
    B_y[1, 0] = -UAVParams.g
    C_y = np.eye(np.shape(A_y)[0])
    D_y = np.zeros((np.shape(A_y)[1], 1))

    # weight matrices
    Q_y = np.diag([1/0.5**2, 1/(0.5*UAVParams.vy_max)**2])
    R_y = np.array(1/np.deg2rad(2)**2)

    y_x_lim = pc.box2poly([[UAVParams.y_min, UAVParams.y_max], [
                          UAVParams.vy_min, UAVParams.vy_max]])
    y_u_lim = pc.box2poly([[UAVParams.angle_min, UAVParams.angle_max]])

    Q_y2 = np.linalg.pinv(
        np.diag(np.array([UAVParams.y_max, UAVParams.vy_max])**2))
    R_y2 = np.linalg.pinv(np.diag(10*np.array([UAVParams.angle_max])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    y_set_h = pc.Polytope(set_params['set_h_y_A'], set_params['set_h_y_b'])
    y_set_l = pc.Polytope(set_params['set_l_y_A'], set_params['set_l_y_b'])
    y_model = StateModel(A_y, B_y, C_y, D_y, y_x_lim, y_u_lim,
                         Q_y, R_y, Q_y2, R_y2, dt, N, y_set_h, y_set_l, control_y, dt2, Ublock)

    # altitude control
    control_z = loadmat('params/lqr_z_0.01_room_params.mat')
    # set matrix of dynamics and input
    A_z = np.zeros([2, 2])
    A_z[0, 1] = 1
    B_z = np.zeros([2, 1])
    B_z[1, 0] = 1/UAVParams.m
    C_z = np.eye(np.shape(A_z)[0])
    D_z = np.zeros((np.shape(A_z)[1], 1))

    # weight matrices
    Q_z = np.diag([1/0.5**2, 1/(0.5*UAVParams.vz_max)**2])
    R_z = np.array(1/(0.01*UAVParams.f_max)**2)

    z_x_lim = pc.box2poly([[UAVParams.z_min, UAVParams.z_max], [
                          UAVParams.vz_min, UAVParams.vz_max]])
    z_u_lim = pc.box2poly(
        [[-UAVParams.m*UAVParams.g + UAVParams.f_min, UAVParams.f_max-UAVParams.m*UAVParams.g]])

    Q_z2 = np.linalg.pinv(
        np.diag(np.array([UAVParams.z_max, UAVParams.vz_max])**2))
    R_z2 = np.linalg.pinv(
        np.diag(100*np.array([UAVParams.f_max-UAVParams.m*UAVParams.g])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    z_set_h = pc.Polytope(set_params['set_h_z_A'], set_params['set_h_z_b'])
    z_set_l = pc.Polytope(set_params['set_l_z_A'], set_params['set_l_z_b'])
    z_model = StateModel(A_z, B_z, C_z, D_z, z_x_lim, z_u_lim,
                         Q_z, R_z, Q_z2, R_z2, dt, N, z_set_h, z_set_l, control_z, dt2, Ublock)

    if save_params:
        phi_model.save("ctrl_attitude", "params")
        y_model.save("ctrl_lateral", "params")
        z_model.save("ctrl_altitude", "params")

    if show_plots:
        y_model.plot_sets()
        z_model.plot_sets()

def plannar_hummingbird(save_params=False, show_plots=False):
    from uav_params import UAVParams
    from scipy.io import loadmat
    # load invariant sets from Matlab file (constructing using MPT3)
    set_params = loadmat('params/sets_0.01_room_hummingbird.mat')
    
    # general parameters
    # sampling
    time_horizon = 8.0
    dt_phi = 1e-3
    dt = 1e-2
    N_phi = 1  # int(time_horizon/dt_phi)
    N = int(time_horizon/dt)
    
    # attitude control
    control_phi = loadmat('params/lqr_phi_0.01_room_hummingbird_params.mat')
    # set matrix of dynamics and input
    A_phi = np.zeros([2, 2])
    A_phi[0, 1] = 1
    B_phi = np.zeros([2, 1])
    B_phi[1, 0] = 1/UAVParams.Ix
    C_phi = np.eye(np.shape(A_phi)[0])
    D_phi = np.zeros((np.shape(A_phi)[1], 1))

    # weight matrices
    Q_phi = np.diag([1/np.deg2rad(30)**2, 1/np.deg2rad(100)**2])
    R_phi = np.array(1/UAVParams.tau_max**2)

    phi_x_lim = pc.box2poly([[UAVParams.angle_min, UAVParams.angle_max], [
                            UAVParams.arate_min, UAVParams.arate_max]])
    phi_u_lim = pc.box2poly([[UAVParams.tau_min, UAVParams.tau_max]])

    Q_phi2 = np.diag(np.array([UAVParams.angle_max, UAVParams.arate_max])**-2)
    R_phi2 = np.diag(10*np.array([UAVParams.tau_max])**-2)

    phi_set_h = pc.Polytope(set_params['set_h_f_A'], set_params['set_h_f_b'])
    phi_model = StateModel(A_phi, B_phi, C_phi, D_phi, phi_x_lim, phi_u_lim,
                           Q_phi, R_phi, Q_phi2, R_phi2, dt_phi, N_phi, phi_set_h, None, control_phi)

    # lateral control
    control_y = loadmat('params/lqr_y_0.01_room_hummingbird_params.mat')
    # set matrix of dynamics and input
    A_y = np.zeros([2, 2])
    A_y[0, 1] = 1
    B_y = np.zeros([2, 1])
    B_y[1, 0] = -UAVParams.g
    C_y = np.eye(np.shape(A_y)[0])
    D_y = np.zeros((np.shape(A_y)[1], 1))

    # weight matrices
    Q_y = np.diag([1/0.5**2, 1/(0.5*UAVParams.vy_max)**2])
    R_y = np.array(1/np.deg2rad(30)**2)

    y_x_lim = pc.box2poly([[UAVParams.y_min, UAVParams.y_max], [
                          UAVParams.vy_min, UAVParams.vy_max]])
    y_u_lim = pc.box2poly([[UAVParams.angle_min, UAVParams.angle_max]])

    Q_y2 = np.linalg.pinv(
        np.diag(np.array([UAVParams.y_max, UAVParams.vy_max])**2))
    R_y2 = np.linalg.pinv(np.diag(10*np.array([UAVParams.angle_max])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    y_set_h = pc.Polytope(set_params['set_h_y_A'], set_params['set_h_y_b'])
    y_set_l = pc.Polytope(set_params['set_l_y_A'], set_params['set_l_y_b'])
    y_model = StateModel(A_y, B_y, C_y, D_y, y_x_lim, y_u_lim,
                         Q_y, R_y, Q_y2, R_y2, dt, N, y_set_h, y_set_l, control_y, dt2, Ublock)

    # altitude control
    control_z = loadmat('params/lqr_z_0.01_room_hummingbird_params.mat')
    # set matrix of dynamics and input
    A_z = np.zeros([2, 2])
    A_z[0, 1] = 1
    B_z = np.zeros([2, 1])
    B_z[1, 0] = 1/UAVParams.m
    C_z = np.eye(np.shape(A_z)[0])
    D_z = np.zeros((np.shape(A_z)[1], 1))

    # weight matrices
    Q_z = np.diag([1/0.5**2, 1/(0.5*UAVParams.vz_max)**2])
    R_z = np.array(1/(0.01*UAVParams.f_max)**2)

    z_x_lim = pc.box2poly([[UAVParams.z_min, UAVParams.z_max], [
                          UAVParams.vz_min, UAVParams.vz_max]])
    z_u_lim = pc.box2poly(
        [[-UAVParams.m*UAVParams.g + UAVParams.f_min, UAVParams.f_max-UAVParams.m*UAVParams.g]])

    Q_z2 = np.linalg.pinv(
        np.diag(np.array([UAVParams.z_max, UAVParams.vz_max])**2))
    R_z2 = np.linalg.pinv(
        np.diag(100*np.array([UAVParams.f_max-UAVParams.m*UAVParams.g])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    z_set_h = pc.Polytope(set_params['set_h_z_A'], set_params['set_h_z_b'])
    z_set_l = pc.Polytope(set_params['set_l_z_A'], set_params['set_l_z_b'])
    z_model = StateModel(A_z, B_z, C_z, D_z, z_x_lim, z_u_lim,
                         Q_z, R_z, Q_z2, R_z2, dt, N, z_set_h, z_set_l, control_z, dt2, Ublock)

    if save_params:
        phi_model.save("ctrl_attitude_hummingbird", "params")
        y_model.save("ctrl_lateral_hummingbird", "params")
        z_model.save("ctrl_altitude_hummingbird", "params")

    if show_plots:
        y_model.plot_sets()
        z_model.plot_sets()

def plannar_crazyflie(save_params=False, show_plots=False):
    from uav_params import UAVParamsCF
    from scipy.io import loadmat
    # load invariant sets from Matlab file (constructing using MPT3)
    set_params = loadmat('params/sets_0.01_room_crazyflie.mat')
    
    # general parameters
    # sampling
    time_horizon = 8.0
    dt_phi = 1e-3
    dt = 1e-2
    N_phi = 1  # int(time_horizon/dt_phi)
    N = int(time_horizon/dt)
    
    # attitude control
    control_phi = loadmat('params/lqr_phi_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_phi = np.zeros([2, 2])
    A_phi[0, 1] = 1
    B_phi = np.zeros([2, 1])
    B_phi[1, 0] = 1/UAVParamsCF.Ix
    C_phi = np.eye(np.shape(A_phi)[0])
    D_phi = np.zeros((np.shape(A_phi)[1], 1))

    # weight matrices
    Q_phi = np.diag([1/np.deg2rad(30)**2, 1/np.deg2rad(100)**2])
    R_phi = np.array(1/UAVParamsCF.tau_max**2)

    phi_x_lim = pc.box2poly([[UAVParamsCF.angle_min, UAVParamsCF.angle_max], [
                            UAVParamsCF.arate_min, UAVParamsCF.arate_max]])
    phi_u_lim = pc.box2poly([[UAVParamsCF.tau_min, UAVParamsCF.tau_max]])

    Q_phi2 = np.diag(np.array([UAVParamsCF.angle_max, UAVParamsCF.arate_max])**-2)
    R_phi2 = np.diag(10*np.array([UAVParamsCF.tau_max])**-2)

    phi_set_h = pc.Polytope(set_params['set_h_f_A'], set_params['set_h_f_b'])
    phi_model = StateModel(A_phi, B_phi, C_phi, D_phi, phi_x_lim, phi_u_lim,
                           Q_phi, R_phi, Q_phi2, R_phi2, dt_phi, N_phi, phi_set_h, None, control_phi)

    # lateral control
    control_y = loadmat('params/lqr_y_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_y = np.zeros([2, 2])
    A_y[0, 1] = 1
    B_y = np.zeros([2, 1])
    B_y[1, 0] = 1
    C_y = np.eye(np.shape(A_y)[0])
    D_y = np.zeros((np.shape(A_y)[1], 1))

    # weight matrices
    Q_y = np.diag([1/(0.5*UAVParamsCF.y_max)**2, 1/(0.5*UAVParamsCF.vy_max)**2])
    R_y = np.array(1/(UAVParamsCF.acc_max)**2)

    y_x_lim = pc.box2poly([[UAVParamsCF.y_min, UAVParamsCF.y_max], [
                          UAVParamsCF.vy_min, UAVParamsCF.vy_max]])
    y_u_lim = pc.box2poly([[UAVParamsCF.acc_min, UAVParamsCF.acc_max]])

    Q_y2 = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.y_max, UAVParamsCF.vy_max])**2))
    R_y2 = np.linalg.pinv(np.diag(10*np.array([UAVParamsCF.acc_max])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    y_set_h = pc.Polytope(set_params['set_h_y_A'], set_params['set_h_y_b'])
    y_set_l = pc.Polytope(set_params['set_l_y_A'], set_params['set_l_y_b'])
    y_model = StateModel(A_y, B_y, C_y, D_y, y_x_lim, y_u_lim,
                         Q_y, R_y, Q_y2, R_y2, dt, N, y_set_h, y_set_l, control_y, dt2, Ublock)

    # altitude control
    control_z = loadmat('params/lqr_z_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_z = np.zeros([2, 2])
    A_z[0, 1] = 1
    B_z = np.zeros([2, 1])
    B_z[1, 0] = 1
    C_z = np.eye(np.shape(A_z)[0])
    D_z = np.zeros((np.shape(A_z)[1], 1))
    
    # weight matrices
    Q_z = np.diag([1/(0.1*UAVParamsCF.z_max)**2, 1/(0.5*UAVParamsCF.vz_max)**2])
    R_z = np.array(1/(UAVParamsCF.accz_max)**2)

    z_x_lim = pc.box2poly([[UAVParamsCF.z_min, UAVParamsCF.z_max], [
                          UAVParamsCF.vz_min, UAVParamsCF.vz_max]])
    z_u_lim = pc.box2poly(
        [[UAVParamsCF.accz_min, UAVParamsCF.accz_max]])

    Q_z2 = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.z_max, UAVParamsCF.vz_max])**2))
    R_z2 = np.linalg.pinv(
        np.diag(10*np.array([UAVParamsCF.accz_max])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    z_set_h = pc.Polytope(set_params['set_h_z_A'], set_params['set_h_z_b'])
    z_set_l = pc.Polytope(set_params['set_l_z_A'], set_params['set_l_z_b'])
    z_model = StateModel(A_z, B_z, C_z, D_z, z_x_lim, z_u_lim,
                         Q_z, R_z, Q_z2, R_z2, dt, N, z_set_h, z_set_l, control_z, dt2, Ublock)

    if save_params:
        phi_model.save("ctrl_attitude_crazyflie", "params")
        y_model.save("ctrl_lateral_crazyflie", "params")
        z_model.save("ctrl_altitude_crazyflie", "params")

    if show_plots:
        y_model.plot_sets()
        z_model.plot_sets()
        
def plannar_crazyflie_with_eic(save_params=False, show_plots=False):
    from uav_params import UAVParamsCF
    from scipy.io import loadmat
    # load invariant sets from Matlab file (constructing using MPT3)
    set_params = loadmat('params/eic_sets_0.01_room_crazyflie.mat')
    
    # general parameters
    # sampling
    time_horizon = 8.0
    dt_phi = 1e-3
    dt = 1e-2
    N_phi = 1  # int(time_horizon/dt_phi)
    N = int(time_horizon/dt)
    
    # attitude control
    control_phi = loadmat('params/eic_lqr_phi_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_phi = np.zeros([2, 2])
    A_phi[0, 1] = 1
    B_phi = np.zeros([2, 1])
    B_phi[1, 0] = 1/UAVParamsCF.Ix
    C_phi = np.eye(np.shape(A_phi)[0])
    D_phi = np.zeros((np.shape(A_phi)[1], 1))

    # weight matrices
    Q_phi = np.diag([1/np.deg2rad(30)**2, 1/np.deg2rad(100)**2])
    R_phi = np.array(1/UAVParamsCF.tau_max**2)

    phi_x_lim = pc.box2poly([[UAVParamsCF.angle_min, UAVParamsCF.angle_max], [
                            UAVParamsCF.arate_min, UAVParamsCF.arate_max]])
    phi_u_lim = pc.box2poly([[UAVParamsCF.tau_min, UAVParamsCF.tau_max]])

    Q_phi2 = np.diag(np.array([UAVParamsCF.angle_max, UAVParamsCF.arate_max])**-2)
    R_phi2 = np.diag(10*np.array([UAVParamsCF.tau_max])**-2)

    phi_set_h = pc.Polytope(set_params['set_h_f_A'], set_params['set_h_f_b'])
    phi_model = StateModel(A_phi, B_phi, C_phi, D_phi, phi_x_lim, phi_u_lim,
                           Q_phi, R_phi, Q_phi2, R_phi2, dt_phi, N_phi, phi_set_h, None, control_phi)

    # lateral control
    control_y = loadmat('params/eic_lqr_y_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_y = np.zeros([2, 2])
    A_y[0, 1] = 1
    B_y = np.zeros([2, 1])
    B_y[1, 0] = 1
    C_y = np.eye(np.shape(A_y)[0])
    D_y = np.zeros((np.shape(A_y)[1], 1))

    # weight matrices
    Q_y = np.diag([1/(0.5*UAVParamsCF.y_max)**2, 1/(0.5*UAVParamsCF.vy_max)**2])
    R_y = np.array(1/(UAVParamsCF.acc_max)**2)

    y_x_lim = pc.box2poly([[UAVParamsCF.y_min, UAVParamsCF.y_max], [
                          UAVParamsCF.vy_min, UAVParamsCF.vy_max]])
    y_u_lim = pc.box2poly([[UAVParamsCF.acc_min, UAVParamsCF.acc_max]])

    Q_y2 = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.y_max, UAVParamsCF.vy_max])**2))
    R_y2 = np.linalg.pinv(np.diag(10*np.array([UAVParamsCF.acc_max])**2))
    
    Q_ym = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.y_max, UAVParamsCF.vy_max])**2))
    R_ym = np.linalg.pinv(np.diag(np.array([UAVParamsCF.acc_max])**2))

    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    y_set_h = pc.Polytope(set_params['set_h_y_A'], set_params['set_h_y_b'])
    y_set_m = pc.Polytope(set_params['set_m_y_A'], set_params['set_m_y_b'])
    y_set_l = pc.Polytope(set_params['set_l_y_A'], set_params['set_l_y_b'])
    y_model = StateModel(A_y, B_y, C_y, D_y, y_x_lim, y_u_lim,
                         Q_y, R_y, Q_y2, R_y2, dt, N, y_set_h, y_set_l, control_y, dt2, Ublock, Q_ym, R_ym, y_set_m)

    # altitude control
    control_z = loadmat('params/eic_lqr_z_0.01_room_crazyflie_params.mat')
    # set matrix of dynamics and input
    A_z = np.zeros([2, 2])
    A_z[0, 1] = 1
    B_z = np.zeros([2, 1])
    B_z[1, 0] = 1
    C_z = np.eye(np.shape(A_z)[0])
    D_z = np.zeros((np.shape(A_z)[1], 1))
    
    # weight matrices
    Q_z = np.diag([1/(0.1*UAVParamsCF.z_max)**2, 1/(0.5*UAVParamsCF.vz_max)**2])
    R_z = np.array(1/(UAVParamsCF.accz_max)**2)

    z_x_lim = pc.box2poly([[UAVParamsCF.z_min, UAVParamsCF.z_max], [
                          UAVParamsCF.vz_min, UAVParamsCF.vz_max]])
    z_u_lim = pc.box2poly(
        [[UAVParamsCF.accz_min, UAVParamsCF.accz_max]])

    Q_z2 = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.z_max, UAVParamsCF.vz_max])**2))
    R_z2 = np.linalg.pinv(
        np.diag(10*np.array([UAVParamsCF.accz_max])**2))

    Q_zm = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.z_max, UAVParamsCF.vz_max])**2))
    R_zm = np.linalg.pinv(
        np.diag(np.array([UAVParamsCF.accz_max])**2))
    
    # Move-blocking MPC parameters
    dt2 = 0.2   # 2nd period for MPC

    # Number of steps when the control command is considered constant
    Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

    z_set_h = pc.Polytope(set_params['set_h_z_A'], set_params['set_h_z_b'])
    z_set_m = pc.Polytope(set_params['set_m_z_A'], set_params['set_m_z_b'])
    z_set_l = pc.Polytope(set_params['set_l_z_A'], set_params['set_l_z_b'])
    z_model = StateModel(A_z, B_z, C_z, D_z, z_x_lim, z_u_lim,
                         Q_z, R_z, Q_z2, R_z2, dt, N, z_set_h, z_set_l, control_z, dt2, Ublock, Q_zm, R_zm, z_set_m)

    if save_params:
        phi_model.save("eic_ctrl_attitude_crazyflie_eic", "params")
        y_model.save("eic_ctrl_lateral_crazyflie_eic", "params")
        z_model.save("eic_ctrl_altitude_crazyflie_eic", "params")

    if show_plots:
        y_model.plot_sets()
        z_model.plot_sets()

if __name__ == "__main__":
    # flat_dynamics()
    # plannar_hummingbird(save_params=True, show_plots=True)
    # plannar_crazyflie(save_params=True, show_plots=True)
    plannar_crazyflie_with_eic(save_params=True, show_plots=True)