import numpy as np


class UAVParams_original:
    """ Parameters of AscTec Hummingbird quadrotor used in paper
        Boucek, Z., Flidr, M. (2019). Explicit Interpolating Control of Unmanned Aerial Vehicle. 
        MMAR2019, 384–389. https://doi.org/10.1109/MMAR.2019.8864719
    """
    kT = 3.2032e-06  # rotor thrust coefficient
    ktau = 1.33191e-7  # rotor torque coefficient
    rad_max = (2*np.pi*64*200)/60  # maximal angular rate of rotor
    rad_min = (2*np.pi*64*18)/60  # minimal angular rate of rotor

    g = 9.81  # gravitational acceleration
    m = 0.5  # mass of quadrotor
    Ix = 6.4*1e-3  # inertia of quadrotor aroud x-axis
    Ts = 1e-3  # sampling period

    # limit thrust
    f_max = 0.8*4*kT*rad_max**2
    f_min = 1.1*4*kT*rad_min**2
    # limit torque
    tau_max = 0.5*kT*(-rad_min**2+rad_max**2)
    tau_min = -tau_max

    # limit angle
    angle_max = np.deg2rad(52)
    angle_min = -angle_max
    # unlimited angular rate
    arate_max = np.Inf
    arate_min = -arate_max

    # limit area of operation with floor
    y_max = 100
    y_min = -y_max
    z_max = 100
    z_min = -z_max

    # limit speed
    vy_max = 15
    vy_min = -vy_max
    Dz = 0.5*1.29*0.2*0.1  # drag coefficient in z
    vz_max = np.sqrt(1/Dz*(-m*g + f_max))  # calculated according to drag
    vz_min = -np.sqrt(-1/Dz*(-m*g + f_min))


class UAVParams():
    """ Paramters of AscTec Hummingbird quadrotor from UAVParams_original 
        tuned for flying in Motion Capture Lab at the University of West Bohemia.
    """
    kT = 3.2032e-06  # rotor thrust coefficient
    ktau = 1.33191e-7  # rotor torque coefficient
    rad_max = (2*np.pi*64*200)/60  # maximal angular rate of rotor
    rad_min = (2*np.pi*64*18)/60  # minimal angular rate of rotor

    g = 9.81  # gravitational acceleration
    m = 0.5  # mass of quadrotor
    Ix = 6.4*1e-3  # inertia of quadrotor aroud x-axis
    Ts = 1e-3  # sampling period

    # limit thrust
    f_max = 0.7*4*kT*rad_max**2
    f_min = 1.2*4*kT*rad_min**2
    # limit torque
    tau_max = 0.5*kT*(-rad_min**2+rad_max**2)
    tau_min = -tau_max

    # limit angle
    angle_max = np.deg2rad(30)
    angle_min = -angle_max
    # unlimited angular rate
    arate_max = np.Inf
    arate_min = -arate_max

    # limit area of operation with floor
    y_max = 2
    y_min = -y_max
    z_max = 2.5/2
    z_min = -z_max

    # limit speed
    vy_max = 5
    vy_min = -vy_max
    vz_max = 5
    vz_min = -vz_max

    def rotmat_yaw(yaw):
        """Rotation matrix for rotation along z-axis. 
        Input:
            yaw: angle in [rad]
        Output:
            3x3 matrix
        """

        return np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0, 0, 1]])

    def rotvec_yaw(vec, yaw):
        """ Return vector rotated along z-axis. 
        Input:
            vec: 3dim vec
            yaw: angle in [rad]
        """

        return UAVParams.rotmat_yaw(yaw)@vec

class UAVParamsCF():
    """ Paramters of Bitcraze Crazyflie quadrotor
        tuned for flying in Motion Capture Lab at the University of West Bohemia.
    """
    kT = 3.16e-10  # rotor thrust coefficient
    ktau = 7.94e-12  # rotor torque coefficient
    rad_max = (2*np.pi*64*200)/60  # maximal angular rate of rotor
    rad_min = (2*np.pi*64*18)/60  # minimal angular rate of rotor
    arm_length = 0.0397
    
    g = 9.81  # gravitational acceleration
    m = 0.03  # mass of quadrotor
    m_lh = 0.032 # mass of quadrotor with lighthouse deck
    Ix = 2.3951e-5  # inertia of quadrotor aroud x-axis
    Ts = 1e-3  # sampling period

    # limit thrust
    f_max = 0.15*4
    f_min = 0.0
    # limit torque
    tau_max = f_max*arm_length - f_min*arm_length
    tau_min = -tau_max

    # limit angle
    angle_max = np.deg2rad(30)
    angle_min = -angle_max
    # unlimited angular rate
    arate_max = np.Inf
    arate_min = -arate_max

    # limit area of operation with floor
    y_max = 2
    y_min = -y_max
    z_max = 2.5/2
    z_min = -z_max

    # limit speed
    vy_max = 5
    vy_min = -vy_max
    vz_max = 5
    vz_min = -vz_max
    
    # limit acceleration
    acc_max = 0.5*f_max/m*np.sin(angle_max)
    acc_min = -acc_max
    
    # accz_max = f_max/m-g
    # accz_min = -g+f_min/m
    accz_max = 5
    accz_min = -5

    def rotmat_yaw(yaw):
        """Rotation matrix for rotation along z-axis. 
        Input:
            yaw: angle in [rad]
        Output:
            3x3 matrix
        """

        return np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0, 0, 1]])

    def rotvec_yaw(vec, yaw):
        """ Return vector rotated along z-axis. 
        Input:
            vec: 3dim vec
            yaw: angle in [rad]
        """

        return UAVParamsCF.rotmat_yaw(yaw)@vec