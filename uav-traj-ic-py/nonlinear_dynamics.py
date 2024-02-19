import numpy as np


def uavPlanarNonlin(state, control):
    """ Nonlinear dynamic planar model of quadrotor UAV with parameters for AscTec Hummingbird

        continuos dynamics:
        dx/dt = [x4, x5, x6, -u1/m*sin(x3), -g + u1/m*cos(x3), u2/Ix]'

        state variable:
        x = [y, z, phi, dy, dz, dphi]'

    Input:
        state - state vector
        control - control vector
    Output:
        dx - derivative of state
    """

    g = 9.81   # gravitational acceleration
    m = 0.5    # mass of quadrotor
    Ix = 6.4*1e-3  # inertia of quadrotor aroud x-axis

    f_dyn = np.zeros(len(state))
    f_dyn[0] = state[3]
    f_dyn[1] = state[4]
    f_dyn[2] = state[5]
    f_dyn[3] = - (1/m)*control[0]*np.sin(state[2])
    f_dyn[4] = - g + (1/m)*control[0]*np.cos(state[2])
    f_dyn[5] = control[1]/Ix

    return f_dyn

def uavPlanarNonlinCF(state, control):
    """ Nonlinear dynamic planar model of quadrotor UAV with parameters for Bitcraze Crazyflie

        continuos dynamics:
        dx/dt = [x4, x5, x6, -u1/m*sin(x3), -g + u1/m*cos(x3), u2/Ix]'

        state variable:
        x = [y, z, phi, dy, dz, dphi]'

    Input:
        state - state vector
        control - control vector
    Output:
        dx - derivative of state
    """

    g = 9.81   # gravitational acceleration
    m = 0.03  # mass of quadrotor
    Ix = 2.3951e-5  # inertia of quadrotor aroud x-axis

    f_dyn = np.zeros(len(state))
    f_dyn[0] = state[3]
    f_dyn[1] = state[4]
    f_dyn[2] = state[5]
    f_dyn[3] = - (1/m)*control[0]*np.sin(state[2])
    f_dyn[4] = - g + (1/m)*control[0]*np.cos(state[2])
    f_dyn[5] = control[1]/Ix

    return f_dyn

def discreteUavCF(state, control, dt):
    """ Simple simulator with nonlinear discrete model of UAV
    """

    dx = uavPlanarNonlinCF(state, control)

    return state+dx*dt

def discreteUavNoiseCF(state, control, dt):
    """ Simple simulator with nonlinear discrete model of UAV 
    """

    dx = uavPlanarNonlinCF(state, control)
    var_pos = 0.00001**2
    var_vel = 0.000005**2
    var_angle = np.deg2rad(0.001)**2
    var_arate = np.deg2rad(0.0001)**2
    Qdiag_process = np.array(
        [var_pos, var_pos, var_angle, var_vel, var_vel, var_arate])
    process_noise = np.random.normal(np.zeros(Qdiag_process.shape), np.sqrt(
        (Qdiag_process)), Qdiag_process.shape)
    var_mpos = 0.005**2
    var_mvel = 0.0001**2
    var_mangle = np.deg2rad(0.005)**2
    var_marate = np.deg2rad(0.0001)**2
    Rdiag_measurement = np.array(
        [var_mpos, var_mpos, var_mangle, var_mvel, var_mvel, var_marate])
    measurement_noise = np.random.normal(np.zeros(Rdiag_measurement.shape), np.sqrt(
        (Rdiag_measurement)), Rdiag_measurement.shape)

    return state+(dx + process_noise)*dt + measurement_noise


def discreteUav(state, control, dt):
    """ Simple simulator with nonlinear discrete model of UAV
    """

    dx = uavPlanarNonlin(state, control)

    return state+dx*dt


def discreteUavNoise(state, control, dt):
    """ Simple simulator with nonlinear discrete model of UAV 
    """

    dx = uavPlanarNonlin(state, control)
    var_pos = 0.00001**2
    var_vel = 0.000005**2
    var_angle = np.deg2rad(0.001)**2
    var_arate = np.deg2rad(0.0001)**2
    Qdiag_process = np.array(
        [var_pos, var_pos, var_angle, var_vel, var_vel, var_arate])
    process_noise = np.random.normal(np.zeros(Qdiag_process.shape), np.sqrt(
        (Qdiag_process)), Qdiag_process.shape)
    var_mpos = 0.005**2
    var_mvel = 0.0001**2
    var_mangle = np.deg2rad(0.005)**2
    var_marate = np.deg2rad(0.0001)**2
    Rdiag_measurement = np.array(
        [var_mpos, var_mpos, var_mangle, var_mvel, var_mvel, var_marate])
    measurement_noise = np.random.normal(np.zeros(Rdiag_measurement.shape), np.sqrt(
        (Rdiag_measurement)), Rdiag_measurement.shape)

    return state+(dx + process_noise)*dt + measurement_noise


def example():
    """ Short simulation of flight with randomly generated control input 
    """

    dt = 0.001
    state = np.zeros(6)
    N = 500
    log_state = np.zeros((6, N))
    log_control = np.zeros((2, N))
    for i in range(N):
        control = np.random.randn(2, 1)
        state = discreteUav(state, control, dt)
        log_state[:, i] = state
        log_control[:, i] = control.T

    return log_state, log_control


def example_noise():
    """ Short simulation of flight with randomly generated control input 
    """

    dt = 0.001
    state = np.zeros(6)
    N = 500
    log_state = np.zeros((6, N))
    log_control = np.zeros((2, N))
    for i in range(N):
        control = np.random.randn(2, 1)
        state = discreteUavNoise(state, control, dt)
        log_state[:, i] = state
        log_control[:, i] = control.T

    return log_state, log_control


def printLog(logs_x, logs_u):
    import matplotlib.pyplot as plt

    dt = 1e-3
    t = np.linspace(0, logs_x.shape[1]*dt, logs_x.shape[1])
    plt.figure()
    for log in logs_x:
        plt.step(t, log)
    plt.title("UAV State")

    plt.figure()
    for log in logs_u:
        plt.step(t, log)
    plt.title("UAV Control")
    plt.show()


if __name__ == "__main__":
    log_x, log_u = example_noise()
    # log_x, log_u = example()
    printLog(log_x, log_u)
