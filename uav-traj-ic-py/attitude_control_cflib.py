"""This script enables control of CrazyFlie with attitude commands
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
import datetime

# Controllers, reference and parameters
from control_cascade import ControlCascade
from attitude_control import ReferenceTrajectory
from uav_params import UAVParamsCF

# Loading and math
import pickle
import cvxpy as cp
import numpy as np

# Crazylib functions
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

# Parameters
DRONE_NO = 1
PARALLEL = True
URIS = ['radio://0/100/2M/E7E7E7E701','radio://0/100/2M/E7E7E7E702','radio://0/100/2M/E7E7E7E703','radio://0/100/2M/E7E7E7E704']
URI = uri_helper.uri_from_env(default=URIS[DRONE_NO-1])
THRUST_COEFS = [0.7, 0.62, 0.6, 0.64]
THRUST_COEF = THRUST_COEFS[DRONE_NO-1]
SAVE_RESULTS = True
SAVE_FOLDER = "results/cf_flight/data/"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Init variables for saving
current_position = np.zeros(3)
current_speed = np.zeros(3)
current_az = 0.0
x_history = []
y_history = []
z_history = []
vx_history = []
vy_history = []
vz_history = []
az_history = []
control_x_history = []
control_y_history = []
control_z_history = []
t_history = []

class Logging:
    """
    Simple logging example class that logs the State Estimate from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, cf = None):
        """ Initialize and run the example with the specified link_uri """
        if cf is not None:
            self._cf = cf
        else:
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
            name='State Estimate', period_in_ms=10)

        self._lg_stab.add_variable('stateEstimate.x', 'FP16')
        self._lg_stab.add_variable('stateEstimate.y', 'FP16')
        self._lg_stab.add_variable('stateEstimate.z', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vx', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vy', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vz', 'FP16')
        self._lg_stab.add_variable('stateEstimate.az', 'FP16')
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
        az_history.append(data['stateEstimate.az'])

        current_position[0] = x_history[-1]
        current_position[1] = y_history[-1]
        current_position[2] = z_history[-1]
        current_speed[0] = vx_history[-1]
        current_speed[1] = vy_history[-1]
        current_speed[2] = vz_history[-1]
        current_az = az_history[-1]

        # Get timestamp
        t_history.append(timestamp)

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
    
    def __wait_for_position_estimator(self, scf):
        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        # log_config.add_variable('kalman.varPX', 'float')
        # log_config.add_variable('kalman.varPY', 'float')
        # log_config.add_variable('kalman.varPZ', 'float')
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        N = 5
        temp_x_history = [0] * N
        temp_y_history = [0] * N
        # var_y_history = [N] * 10
        # var_x_history = [N] * 10
        # var_z_history = [N] * 10

        threshold = 0.01

        with SyncLogger(scf, log_config) as logger:
            for i, log_entry in enumerate(logger):
                data = log_entry[1]

                temp_x_history.append(data['stateEstimate.x'])
                temp_x_history.pop(0)
                temp_y_history.append(data['stateEstimate.y'])
                temp_y_history.pop(0)
                # var_x_history.append(data['kalman.varPX'])
                # var_x_history.pop(0)
                # var_y_history.append(data['kalman.varPY'])
                # var_y_history.pop(0)
                # var_z_history.append(data['kalman.varPZ'])
                # var_z_history.pop(0)

                # min_x = min(var_x_history)
                # max_x = max(var_x_history)
                # min_y = min(var_y_history)
                # max_y = max(var_y_history)
                # min_z = min(var_z_history)
                # max_z = max(var_z_history)
                # if (max_x - min_x) < threshold and (
                #         max_y - min_y) < threshold and (
                #         max_z - min_z) < threshold and i>N:
                if  i>len(temp_x_history):
                    x = np.mean(temp_x_history)
                    y = np.mean(temp_y_history)
                    return x,y
    
    def reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        self.__wait_for_position_estimator(scf)

    def preflight(self, scf):
        # Set these to the position and yaw based on how your Crazyflie is placed
        # on the floor
        initial_x = 0.0
        initial_y = 0.0
        initial_z = 0.0
        initial_yaw = 0  # In degrees

        no_of_samples = 50
        # initial_x = np.mean(x_history[-no_of_samples:-1])
        # initial_y = np.mean(y_history[-no_of_samples:-1])
        # initial_z = np.mean(z_history[-no_of_samples:-1])
        while len(x_history) < no_of_samples:
            time.sleep(1)
            
        initial_x = np.mean(x_history)
        initial_y = np.mean(y_history)
        initial_z = np.mean(z_history)
        # self.__set_initial_position(scf = scf, x=initial_x, y=initial_y, z=initial_z, yaw_radians=initial_yaw)

        return initial_x, initial_y, initial_z, initial_yaw


def set_initial_position(scf, x=0.0, y=0.0, z=0.0, yaw_radians=0.0):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def distance(current, goal):
    """Return distance to goal"""
    return np.linalg.norm(current-goal, ord=2)


def on_position(com, goal, yaw):
    # t_pos_start = time.time()
    # t_to_pos = 5.0
    # while t_pos_start+t_to_pos < time.time():
    #     com.send_position_setpoint(goal[0], goal[1], goal[2], yaw)
    #     time.sleep(0.01)
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

def land(scf):
    t_land = 3.0
    scf.cf.high_level_commander.land(0.00,t_land)
    time.sleep(t_land)

def emergency_landing(scf):
    if scf is not None:
        print("Mission aborted!")
        print("Emergency landing!")
        land(scf)
        # Stop motors
        print("Stopping motors.")
        scf.cf.commander.send_stop_setpoint()

def fly(ctrl = None, REFERENCE = None, t_mission = 5.0, into_air = False, control_enabled = False, thrust_control = False):
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    z_offset = 0.3 # [m]
    unlocked_motors = False

    scf = SyncCrazyflie(URI, cf=Crazyflie(ro_cache='./cache', rw_cache='./cache')) 
    le = Logging(URI, scf.cf)
    # set initial points
    initial_x, initial_y, initial_z, initial_yaw = le.preflight(scf)
    set_initial_position(scf = scf, x=initial_x, y=initial_y, z=initial_z, yaw_radians=initial_yaw)
    initial = np.array([initial_x, initial_y, initial_z])
    initial_air = np.array([initial_x, initial_y, initial_z+z_offset])
    time_cascade = []

    # We take off when the commander is created
    com = scf.cf.commander
    t_flight = []
    
    if into_air:
        print("Commander operational, taking off!")
        # Take off to inital air position
        on_position(com, initial_air, 0)
        # To the start of trajectory
        print("Move to start.")
        r0 = [0.0,0.0,0.0]
        # r0[0] = REFERENCE.x[0]
        # r0[1] = REFERENCE.y[0]
        r0[2] = REFERENCE.z[0]
        print(r0)
        on_position(com, r0, 0)
        print("Start.")
        euler = [0.0,0.0,0.0]

    t_flight.append(time.time())
    euler = [0.0,0.0,0.0]
    u_acc_x_ref_temp, u_acc_y_ref_temp, u_acc_z_ref_temp = None, None, None
    u_out = [None]*len(models)

    while t_flight[-1]-t_flight[0] < t_mission:
        # Control input
        if t_flight[-1] + ctrl.models[0].dt <= time.time():
            t_flight.append(time.time())
            if not thrust_control:
                state_remapped = np.zeros((8,1))
                state_remapped[0,0] = x_history[-1]
                state_remapped[1,0] = y_history[-1]
                state_remapped[4,0] = vx_history[-1]
                state_remapped[5,0] = vy_history[-1]
                rk = np.zeros((8,ctrl.models[0].N))
                k0 = int((t_flight[-1]-t_flight[0])//ctrl.models[0].dt)
                kN = int(k0 + ctrl.models[0].N)
                rk[0,:] = REFERENCE.x[k0:kN]
                rk[1,:] = REFERENCE.y[k0:kN]
            else:
                state_remapped = np.zeros((12,1))
                state_remapped[0,0] = x_history[-1]
                state_remapped[1,0] = y_history[-1]
                state_remapped[2,0] = z_history[-1] + UAVParamsCF.z_min
                state_remapped[6,0] = vx_history[-1]
                state_remapped[7,0] = vy_history[-1]
                state_remapped[8,0] = vz_history[-1]
                rk = np.zeros((12,ctrl.models[0].N))
                k0 = int((t_flight[-1]-t_flight[0])//ctrl.models[0].dt)
                kN = int(k0 + ctrl.models[0].N)
                rk[0,:] = REFERENCE.x[k0:kN]
                rk[1,:] = REFERENCE.y[k0:kN]
                rk[2,:] = REFERENCE.z[k0:kN] + UAVParamsCF.z_min
                # print(f"err: {rk[:3,0]-current_position-[0,0,UAVParamsCF.z_min]}")
            t_start = time.time()
            # if not thrust_control or CTRL_TYPE =="mpcmb":
            #     u_out = ctrl.spin(rk, state_remapped)
            # else:
            #     u_out = ctrl.spin_par(rk, state_remapped)
            if PARALLEL:
                u_out = ctrl.spin_par(rk, state_remapped)
            else: 
                u_out = ctrl.spin(rk, state_remapped)
            if thrust_control:
                u_acc_x_ref_temp, u_acc_y_ref_temp, u_acc_z_ref_temp = u_out
            else:
                u_acc_x_ref_temp, u_acc_y_ref_temp = u_out
            time_cascade.append(time.time() - t_start)

        # Save data
        control_x_history.append(u_acc_x_ref_temp)
        control_y_history.append(u_acc_y_ref_temp)
        if thrust_control:
            control_z_history.append(u_acc_z_ref_temp)

        if into_air:
            speed_check = np.linalg.norm(current_speed, ord=2)
            if (speed_check < 50.0):
                if (not control_enabled) or (None in u_out):
                    print("Safety control active.")
                    k = int((t_flight[-1]-t_flight[0])//ctrl.models[0].dt)
                    com.send_position_setpoint(REFERENCE.x[k], REFERENCE.y[k], REFERENCE.z[k], 0.0)
                    time.sleep(0.01)
                else:
                    roll_ref = 1/UAVParamsCF.g*(u_acc_x_ref_temp * np.sin(euler[2]) - u_acc_y_ref_temp * np.cos(euler[2]))
                    pitch_ref = 1/UAVParamsCF.g*(u_acc_x_ref_temp * np.cos(euler[2]) + u_acc_y_ref_temp * np.sin(euler[2]))
                    if thrust_control:
                        thrust_ref = int((UAVParamsCF.m_lh*(u_acc_z_ref_temp)/(UAVParamsCF.f_max)+THRUST_COEF)*50000+10000) # f_max in cf lib is 60000
                        thrust_ref = np.clip(thrust_ref,10001, 60000)
                        if not unlocked_motors:
                            print('unlock')
                            com.send_setpoint(0.0, 0.0, 0, 0)
                            unlocked_motors = True
                            com.set_client_xmode(False)
                            time.sleep(0.001)
                        com.send_setpoint(np.rad2deg(roll_ref), np.rad2deg(pitch_ref), 0.0, thrust_ref)
                        # print(f"Thrust: {thrust_ref}")
                    else:
                        k = int((t_flight[-1]-t_flight[0])//ctrl.models[0].dt)
                        com.send_zdistance_setpoint(np.rad2deg(roll_ref), -np.rad2deg(pitch_ref), 0.0, REFERENCE.z[k])
                        # com.send_position_setpoint(REFERENCE.x[k], REFERENCE.y[k], REFERENCE.z[k], 0.0)
            else: 
                # Safety catch if the constraints are violated
                emergency_landing(scf)
                into_air = False
                break


    #### Save the simulation results ###########################
    if SAVE_RESULTS:
        t_save = datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        print("Saving data.")
        path_np = SAVE_FOLDER + "cflib_log_"+REFERENCE.curve+"_" + CTRL_TYPE
        path = SAVE_FOLDER + "log_controller_"+REFERENCE.curve+"_"
        path_time = SAVE_FOLDER + "time_"+REFERENCE.curve+"_"
        path_time_traj = SAVE_FOLDER + "time_traj_"+REFERENCE.curve+"_"
        if thrust_control:
            path_np = path_np+"_thrust"
            path = path+"thrust_"
            path_time = path_time +"thrust_"
            path_time_traj = path_time_traj+"thrust_"
        if PARALLEL:
            path_np = path_np+"_par"
            path = path+"par_"
            path_time = path_time +"par_"
            path_time_traj = path_time_traj+"par_"
        np.savez(path_np + "_" +t_save,
                x_history=x_history, vx_history=vx_history,
                y_history=y_history, vy_history=vy_history,
                z_history=z_history, vz_history=vz_history,
                control_x_history=control_x_history,
                control_y_history=control_y_history,
                control_z_history=control_z_history,
                t_history=t_history,
                initial=initial, initial_yaw=initial_yaw,
                initial_air=initial_air)

        ctrl.save(path,"_"+t_save)
        np.save(path_time+CTRL_TYPE + "_" +t_save, np.array(time_cascade))
        np.save(path_time_traj+CTRL_TYPE + "_" +t_save, np.array(t_flight))

    print("End of mission.")
    # End of mission, going for landing!
    if into_air:
        # RTH
        print("RTH.")
        on_position(com, initial_air, 0)
        # land
        print("Landing.")
        land(scf)
        # Stop motors
        print("Stopping motors.")
        com.send_stop_setpoint()
    print("Closing connection.")
    le._cf.close_link()

if __name__ == '__main__':
    ref_types = ["spiral", "figure8"]
    ref_type = ref_types[1]
    speed = [0.6, 0.6, 0.3] #FINAL: 0.6
    # space_spiral = [1.8, 1.0, 1.0] # old
    space_spiral = [1.5, 1.0, 0.7]
    space_figure8 = [1.5, 1.5, 0.7]
    if ref_type == "spiral":
        reference = ReferenceTrajectory(curve=ref_type, space = space_spiral, tscale = speed)
    else:
        reference = ReferenceTrajectory(curve=ref_type, space = space_figure8, tscale = speed)

    # CTRL_TYPE = "mpcmb"
    CTRL_TYPE = "ic"
    # CTRL_TYPE = "eic"

    t_mission = 30.0
    control_enabled = True # True if you want to employ the commands of controller
    into_the_air = True   # True if you want to fly the drone
    thrust_control = True # True if zcontrol is active

    model_data_folder = "params"
    if CTRL_TYPE == "eic":
        models_file = ["lateral_crazyflie_eic", "lateral_crazyflie_eic", "altitude_crazyflie_eic"]
    else:
        models_file = ["lateral_crazyflie", "lateral_crazyflie", "altitude_crazyflie"]
    if not thrust_control:
        models_file = models_file[:-1]
    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))
            
    # Init control cascade with x, y and z axis control
    ctrl = ControlCascade(CTRL_TYPE, models, log_data=True)
    fly(ctrl, reference, t_mission, into_the_air, control_enabled, thrust_control)