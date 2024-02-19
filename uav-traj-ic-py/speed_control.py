"""This script enables control of CrazyFlie with speed commands 
by Interpolating or Model Predictive Control (IC & MPC).

Simple example that connects to the crazyflie at `URI` and runs a
sequence from file. This script requires some kind of location system, 
it is intended to use with Lighthouse system.

The Commander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import math
import logging
import time
from threading import Timer

# Controllers
from controllers import ControllerIC
from controllers import ControllerMPCMB

# Loading and math
import pickle
import cvxpy as cp
import numpy as np
from scipy.io import loadmat
import polytope as pc

# Crazylib functions
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

# Parameters
URI = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E702')
IC = 1
N = 800
solver = cp.GUROBI

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Load parameters of dynamics and cost function.
with open('dynamics.pkl', 'rb') as infile:
    result = pickle.load(infile)
dt, A, B, Q1, Q2, R1, R2 = result

# Load linear constraints as polytopes.
with open('sets.pkl', 'rb') as infile:
    result = pickle.load(infile)
xlim, ulim, wlim = result
nx = xlim.dim
nu = ulim.dim

# Load paramters of IC
annots = loadmat('params/ic_params_py.mat')
L1 = annots['ctrl1'][0, 0]['L']
K1 = annots['ctrl1'][0, 0]['K']
invSet1 = pc.Polytope(annots['ctrl1'][0, 0]['F'],
                      annots['ctrl1'][0, 0]['g'])
L2 = annots['ctrl2'][0, 0]['L']
K2 = annots['ctrl2'][0, 0]['K']
invSet2 = pc.Polytope(annots['ctrl2'][0, 0]['F'],
                      annots['ctrl2'][0, 0]['g'])

# Move-blocking MPC parameters
dt2 = 0.2   # 2nd period for MPC

# Number of steps when the control command is considered constant
Ublock = [1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 10]

# Init variables for saving
current_position = np.zeros(3)
current_speed = np.zeros(3)
x_history = [0]
y_history = [0]
z_history = [0]
vx_history = [0]
vy_history = [0]
vz_history = [0]
control_x_history = [0]
control_y_history = [0]
control_z_history = [0]
t_history = [0]

# Init controllers for x, y and z axis
if IC:
    controller_x = ControllerIC(invSet1, invSet2, K1, L1, K2, L2, solver)
    controller_y = ControllerIC(invSet1, invSet2, K1, L1, K2, L2, solver)
    controller_z = ControllerIC(invSet1, invSet2, K1, L1, K2, L2, solver)
else:
    controller_x = ControllerMPCMB(
        xlim, ulim, A, B, R1, Q1, dt, dt2, Ublock, N, solver)
    controller_y = ControllerMPCMB(
        xlim, ulim, A, B, R1, Q1, dt, dt2, Ublock, N, solver)
    controller_z = ControllerMPCMB(
        xlim, ulim, A, B, R1, Q1, dt, dt2, Ublock, N, solver)


class LoggingExample:
    """
    Simple logging example class that logs the State Estimate from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(
            name='State Estimate', period_in_ms=np.floor(dt*1000))

        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stateEstimate.vx', 'float')
        self._lg_stab.add_variable('stateEstimate.vy', 'float')
        self._lg_stab.add_variable('stateEstimate.vz', 'float')
        # # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # t = Timer(10, self._cf.close_link)
        # # Start a timer to disconnect in 10s
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""

        # Get position and velocity
        x_history.append(data['stateEstimate.x'])
        y_history.append(data['stateEstimate.y'])
        z_history.append(data['stateEstimate.z'])
        vx_history.append(data['stateEstimate.vx'])
        vy_history.append(data['stateEstimate.vy'])
        vz_history.append(data['stateEstimate.vz'])

        current_position[0] = x_history[-1]
        current_position[1] = y_history[-1]
        current_position[2] = z_history[-1]
        current_speed[0] = vx_history[-1]
        current_speed[1] = vy_history[-1]
        current_speed[2] = vz_history[-1]

        # Get timestamp
        t_history.append(timestamp)

        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def preflight():
    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 0  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    time.sleep(1)

    initial_x = np.mean(x_history[:50])
    initial_y = np.mean(y_history[:50])
    initial_z = np.mean(z_history[:50])

    return initial_x, initial_y, initial_z, initial_yaw


def distance(current, goal):
    """Return distance to goal"""
    return np.linalg.norm(current-goal, ord=2)
    #return np.sqrt(np.sum((current-goal)**2))


def on_position(com, goal, yaw):
    d = distance(current_position, goal)
    eps = 0.1
    while d > eps:
        com.send_position_setpoint(goal[0], goal[1], goal[2], yaw)
        time.sleep(0.01)
        d = distance(current_position, goal)


def is_est_frozen():
    """Return True if estimator is frozen"""
    if (x_history[-1] == x_history[-2] == x_history[-3]
        and y_history[-1] == y_history[-2] == y_history[-3]
        and z_history[-1] == z_history[-2] == z_history[-3]):
        # Safety catch if the state estimation is frozen
        return True
    else:
        return False


if __name__ == '__main__':

    into_air = True    # True if you want to fly the drone

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(URI)

    # set initial points
    initial_x, initial_y, initial_z, initial_yaw = preflight()
    initial = np.array([initial_x, initial_y, initial_z])
    initial_air = np.array([initial_x, initial_y, initial_z+0.5])

    # whole reference trajectory
    # TODO: prepare trajectories in NP files and load for flight
    reference_trajectory = np.zeros((3*nx, 30*1000))
    reference_trajectory[0:-1:2, :] = 1

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        # reset_estimator(scf)
        N = 800
        dyaw = 0.0

        current_x = np.zeros(nx)
        current_y = np.zeros(nx)
        current_z = np.zeros(nx)

        # We take off when the commander is created
        com = scf.cf.commander
        time.sleep(1)

        print("Commander operational, taking off!")

        if into_air:
            # Take off to inital air position
            on_position(com, initial_air, 0)
            # To the start of trajectory
            print("Move to start.")
            on_position(com, reference_trajectory[0:-1:nx, 0], 0)
            print("At start.")

        for i in np.arange(100):
            # Interpolating Control
            current_x[0] = x_history[-1]
            current_x[1] = vx_history[-1]
            current_y[0] = y_history[-1]
            current_y[1] = vy_history[-1]
            current_z[0] = z_history[-1]
            current_z[1] = vz_history[-1]

            if not is_est_frozen() and into_air:
                # Safety catch if the state estimation is frozen
                print("ERR: Estimator is broken, go for emergency landing!")
                break

            # Set current trajectory and setpoint.
            current_trajectory = reference_trajectory[:, i:i+N]

            # Interpolate control input
            dvx = controller_x.spin(current_trajectory[:nx, :N], current_x)
            dvy = controller_y.spin(current_trajectory[nx:2*nx, :N], current_y)
            dvz = controller_y.spin(
                current_trajectory[2*nx:3*nx, :N], current_z)

            print(f"desired speed x,y,z: {dvx}, {dvy}, {dvz} m/s")

            # Save data
            control_x_history.append(dvx)
            control_y_history.append(dvy)
            control_z_history.append(dvz)

            if into_air:
                speed_check = np.linalg.norm(current_speed, ord=2)
                if (speed_check > 3):
                    # Safety catch if the speed is high
                    print("ERR: Too fast, sending ZERO speed command.")
                    com.send_velocity_world_setpoint(0, 0, 0, dyaw)
                # else:
                #     # Comment while testing
                #     com.send_velocity_world_setpoint(dvx, dvy, dvz, dyaw)
                #     # time.sleep(dt)

        print("End of mission.")
        # End of mission, going for landing!
        if into_air:
            # RTH
            print("RTH.")
            on_position(com, initial_air, 0)
            # land
            print("Landing.")
            com.send_velocity_world_setpoint(0, 0, -0.05, dyaw) # landing 0.05 m/s
            time.sleep(5)
            # Stop motors
            print("Stopping motors.")
            com.send_stop_setpoint()

        print("Saving data.")
        np.savez("flight_data",
                 x_history=x_history, vx_history=vx_history,
                 y_history=y_history, vy_history=vy_history,
                 z_history=z_history, vz_history=vz_history,
                 control_x_history=control_x_history,
                 control_y_history=control_y_history,
                 control_z_history=control_z_history,
                 t_history=t_history,
                 initial=initial, initial_yaw=initial_yaw,
                 initial_air=initial_air)

        print("Closing connection.")
        le._cf.close_link()
