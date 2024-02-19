"""Script demonstrating the joint use of simulation and control.
The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.
Example
-------
In a terminal, run as:
    $ python fly.py
Notes
-----
The drones move, at different altitudes, along cicular trajectories 
in the X-Y plane, around point (0, -.3).
"""
import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

from control_cascade import ControlCascade
from attitude_control import ReferenceTrajectory
import pickle
from uav_params import UAVParams

# DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_DRONES = DroneModel("hb")
DEFAULT_NUM_DRONES = 1
# DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_PHYSICS = Physics("pyb_gnd_drag_dw")
DEFAULT_VISION = False
DEFAULT_GUI = False
DEFAULT_RECORD_VISION = True
DEFAULT_PLOT = False
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_AGGREGATE = True
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 5000
DEFAULT_CONTROL_FREQ_HZ = 1000
# DEFAULT_SIMULATION_FREQ_HZ = 240
# DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 100
DEFAULT_OUTPUT_FOLDER = 'results/3d_pybullet/data/'
DEFAULT_COLAB = False
SAVE_RESULTS = True
CTRL_TYPE = "ic"
SAVE_FOLDER = "results/3d_pybullet/data/"
# REF_TYPE = "spiral", "figure8"
# CTRL_TYPE = "mpc"
# CTRL_TYPE = "mpcmb"


def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        vision=DEFAULT_VISION,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        aggregate=DEFAULT_AGGREGATE,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
):
    #### Initialize the simulation #############################
    H = 1.0
    H_STEP = .05
    R = .3
    # reference = ReferenceTrajectory(curve="figure8", space = [1.2, 1.2, 1.0], tscale = 0.5)
    # reference = ReferenceTrajectory(curve=ref_type, space = [1.5, 1.0, 1.0], tscale = 0.4)
    # reference = ReferenceTrajectory(curve="circle", space = [1.0, 1.0, 1.0], tscale = 0.3)
    # INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)
    #                      * 2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
    # INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones]
    #                      for i in range(num_drones)])
    INIT_XYZS = np.array([[REFERENCE.x[0], REFERENCE.y[0], REFERENCE.z[0]] for i in range(num_drones)])
    # INIT_XYZS = np.array([[0, 0, H] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, 0]
                         for i in range(num_drones)])
    
    AGGR_PHY_STEPS = int(simulation_freq_hz /
                         control_freq_hz) if aggregate else 1

    #### Initialize a circular trajectory ######################
    PERIOD = 10
    NUM_WP = control_freq_hz*PERIOD
    TARGET_POS = np.zeros((NUM_WP, 3))
    for i in range(NUM_WP):
        TARGET_POS[i, :] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0,
                                                                            0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0, 1], 0
    wp_counters = np.array([int((i*NUM_WP/6) % NUM_WP)
                           for i in range(num_drones)])

    #### Debug trajectory ######################################
    # Uncomment alt. target_pos in .computeControlFromState()
    # INIT_XYZS = np.array([[.3 * i, 0, .1] for i in range(num_drones)])
    # INIT_RPYS = np.array([[0, 0,  i * (np.pi/3)/num_drones] for i in range(num_drones)])
    # NUM_WP = control_freq_hz*15
    # TARGET_POS = np.zeros((NUM_WP,3))
    # for i in range(NUM_WP):
    #     if i < NUM_WP/6:
    #         TARGET_POS[i, :] = (i*6)/NUM_WP, 0, 0.5*(i*6)/NUM_WP
    #     elif i < 2 * NUM_WP/6:
    #         TARGET_POS[i, :] = 1 - ((i-NUM_WP/6)*6)/NUM_WP, 0, 0.5 - 0.5*((i-NUM_WP/6)*6)/NUM_WP
    #     elif i < 3 * NUM_WP/6:
    #         TARGET_POS[i, :] = 0, ((i-2*NUM_WP/6)*6)/NUM_WP, 0.5*((i-2*NUM_WP/6)*6)/NUM_WP
    #     elif i < 4 * NUM_WP/6:
    #         TARGET_POS[i, :] = 0, 1 - ((i-3*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-3*NUM_WP/6)*6)/NUM_WP
    #     elif i < 5 * NUM_WP/6:
    #         TARGET_POS[i, :] = ((i-4*NUM_WP/6)*6)/NUM_WP, ((i-4*NUM_WP/6)*6)/NUM_WP, 0.5*((i-4*NUM_WP/6)*6)/NUM_WP
    #     elif i < 6 * NUM_WP/6:
    #         TARGET_POS[i, :] = 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-5*NUM_WP/6)*6)/NUM_WP
    # wp_counters = np.array([0 for i in range(num_drones)])

    #### Create the environment with or without video capture ##
    if vision:
        env = VisionAviary(drone_model=drone,
                           num_drones=num_drones,
                           initial_xyzs=INIT_XYZS,
                           initial_rpys=INIT_RPYS,
                           physics=physics,
                           neighbourhood_radius=10,
                           freq=simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=gui,
                           record=record_video,
                           obstacles=obstacles
                           )
    else:
        env = CtrlAviary(drone_model=drone,
                         num_drones=num_drones,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=physics,
                         neighbourhood_radius=10,
                         freq=simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    # load model data
    model_data_folder = "params"
    # models_file = ["lateral", "lateral", "altitude"]
    models_file = ["lateral_hummingbird", "lateral_hummingbird", "altitude"]

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))
    POS_CTRL_FREQ = 1/models[0].dt

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]
        # ctrl = [ControlCascade("ic", models, True) for i in range(num_drones)]
    elif drone in [DroneModel.HB]:
        ctrl = [ControlCascade(CTRL_TYPE, models, True) for i in range(num_drones)]
        # ctrl = [ControlCascade("mpcmb", models, True) for i in range(num_drones)]
        # ctrl = [SimplePIDControl(drone_model=drone) for i in range(num_drones)]
        ctrl_original = [SimplePIDControl(
            drone_model=drone) for i in range(num_drones)]
        
    time_cascade = []

    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/control_freq_hz))
    CTRL_POS_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/POS_CTRL_FREQ))
    action = {str(i): np.array([0, 0, 0, 0]) for i in range(num_drones)}
    START = time.time()
    for i in range(0, int(duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):
        
        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        #### Compute control at the desired frequency ##############
        if i % CTRL_EVERY_N_STEPS == 0:

            #### Compute control for the current way point #############
            for j in range(num_drones):
                # u_roll_ref_temp, u_pitch_ref_temp, u_thrust_temp = [0,0,0]
                state = obs[str(j)]["state"]
                #### Observation vector ### X,Y,Z,Q1,Q2,Q3,Q4,R,P,Y,VX,VY,VZ,WX,WY,WZ,P0,P1,P2,P3
                cur_quat = state[3:7]
                if i % CTRL_POS_EVERY_N_STEPS == 0:
                    state_remapped = np.zeros((12,1))
                    state_remapped[0,0] = state[0]
                    state_remapped[1,0] = state[1]
                    state_remapped[2,0] = state[2] + UAVParams.z_min
                    state_remapped[6,0] = state[10]
                    state_remapped[7,0] = state[11]
                    state_remapped[8,0] = state[12]
                    euler = p.getEulerFromQuaternion(cur_quat)
                    
                    rk = np.zeros((12,ctrl[0].models[0].N))
                    rk[0,:] = REFERENCE.x[i//CTRL_POS_EVERY_N_STEPS:i//CTRL_POS_EVERY_N_STEPS+ctrl[0].models[0].N]
                    rk[1,:] = REFERENCE.y[i//CTRL_POS_EVERY_N_STEPS:i//CTRL_POS_EVERY_N_STEPS+ctrl[0].models[0].N]
                    rk[2,:] = REFERENCE.z[i//CTRL_POS_EVERY_N_STEPS:i//CTRL_POS_EVERY_N_STEPS+ctrl[0].models[0].N] + UAVParams.z_min
                    
                    t_start = time.time()
                    # u_pitch_ref_temp,u_roll_ref_temp, u_thrust_temp = ctrl[j].spin_par(rk, state_remapped)
                    u_pitch_ref_temp,u_roll_ref_temp, u_thrust_temp = ctrl[j].spin(rk, state_remapped)
                    u_roll_ref_temp, u_pitch_ref_temp, __ = UAVParams.rotvec_yaw([u_roll_ref_temp, u_pitch_ref_temp,0],euler[2])
                    time_cascade.append(time.time() - t_start)
                    # print(u_roll_ref_temp, u_pitch_ref_temp, u_thrust_temp)
                    if u_thrust_temp == None:
                        u_thrust_temp = 0
                        print("Z: Solution not found.")
                    # u_thrust_01 = (u_thrust_temp+UAVParams.m*UAVParams.g)/(UAVParams.f_max)
                
                action[str(j)] = ctrl_original[j]._simplePIDAttitudeControl(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                        #    thrust=(3-np.cos(euler[0])-np.cos(euler[1]))* u_thrust_temp+UAVParams.m*UAVParams.g,
                                                           thrust= u_thrust_temp+UAVParams.m*UAVParams.g,
                                                           cur_quat=cur_quat,
                                                           target_rpy=np.array(
                                                               [u_roll_ref_temp, -u_pitch_ref_temp , 0.0])#-np.pi/2])
                                                           )

            #### Go to the next way point and loop #####################
            for j in range(num_drones):
                wp_counters[j] = wp_counters[j] + \
                    1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.SIM_FREQ,
                       state=obs[str(j)]["state"],
                    #    control=np.hstack(
                    #        [TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                       # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        if i % env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if vision:
                for j in range(num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(
                              obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(
                              obs[str(j)]["seg"])
                          )

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    if SAVE_RESULTS:
        path = SAVE_FOLDER + "log_controller_"+REFERENCE.curve+"_"
        path_time = SAVE_FOLDER + "time_"+REFERENCE.curve+"_"
        ctrl[j].save(path)
        np.save(path_time+ctrl_type, np.array(time_cascade))
    logger.save()
    logger.save_as_csv(ctrl[j].ctrl_type+"_"+REFERENCE.curve+"_"+"pid")  # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()


if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(
        description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,
                        help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,
                        type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,
                        help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=DEFAULT_VISION,      type=str2bool,
                        help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,
                        help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,
                        type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,
                        help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,
                        help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=DEFAULT_AGGREGATE,       type=str2bool,
                        help='Whether to aggregate physics steps (default: True)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,
                        help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,
                        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,
                        type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,
                        help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,
                        help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    # ctrl_types = ["ic", "mpcmb", "mpc"]    
    ctrl_types = ["mpc"]
    # ctrl_types = ["mpcmb", "ic", "mpc"]
    # ref_types = ["spiral", "figure8"]
    ref_types = ["figure8"]
    for ref_type in ref_types:
        # REFERENCE = ReferenceTrajectory(curve=ref_type, space = [1.5, 1.5, 1.0], tscale = 0.4) # for spiral
        REFERENCE = ReferenceTrajectory(curve=ref_type, space = [1.5, 1.0, 1.0], tscale = 0.35) #for figure8
        for ctrl_type in ctrl_types:
            CTRL_TYPE = ctrl_type
            run(**vars(ARGS))
