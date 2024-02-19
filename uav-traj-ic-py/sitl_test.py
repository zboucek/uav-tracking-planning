#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.offboard import (Attitude, OffboardError)

from control_cascade import ControlCascade
from attitude_control import ReferenceTrajectory
import numpy as np
import pickle
from uav_params import UAVParams

async def run():

    # load model data
    model_data_folder = "params"
    models_file = ["lateral", "lateral", "altitude"]

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))
    reference = ReferenceTrajectory(curve="spiral")
    
    ctrl = ControlCascade("ic", models, True)
    # ctrl = ControlCascade("mpcmb", models, True)

    drone = System()
    print("Init connection...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {drone._sysid}")
            break
        
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()
    # await drone.action.takeoff()
    # await asyncio.sleep(5)
    
    drone.u_roll_ref = 0
    drone.u_pitch_ref = 0
    drone.u_thrust = 0
    
    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    # await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    # await asyncio.sleep(5)
    print("-- Test started")
    drone.trajectory_finished = False
    drone.rotor_scale = 1.0
    asyncio.ensure_future(fly_along_trajectory(drone, ctrl, reference))
    
    # while not drone.trajectory_finished:
    #     await asyncio.sleep(1)

    # print_fly_to_dronport_task = asyncio.ensure_future(fly_along_trajectory(drone))
# 
    # running_tasks = [fly_along_trajectory]
    # termination_task = asyncio.ensure_future(
    #     observe_is_in_air(drone, running_tasks))


    # go to Droneport location
    # print("-- Trajectory ended")
    # await drone.action.goto_location(49.7268269, 13.3535106, 400, 0)

    # await termination_task


# def distance_to_droneport(drone_position):
#     """ Return the distance to Droneport """
#     drone_coords = (drone_position.latitude_deg, drone_position.longitude_deg)
#     droneport_coords = (49.7268269, 13.3535106)
#     distance = geopy.distance.distance(drone_coords, droneport_coords).km
#     print(f"Distance: {distance}km")
#     return geopy.distance.distance(drone_coords, droneport_coords).km


async def fly_along_trajectory(drone, ctrl, reference):
    """ Starts the landing procedure above the Droneport """
    # async for position in drone.telemetry.position():
    async for position_velocity in drone.telemetry.position_velocity_ned():
        
        # print(position_velocity.position)
        xk = np.zeros((12,1))
        xk[0,0] = position_velocity.position.north_m
        xk[1,0] = position_velocity.position.east_m
        xk[2,0] = -position_velocity.position.down_m +UAVParams.z_min
        xk[6,0] = position_velocity.velocity.north_m_s
        xk[7,0] = position_velocity.velocity.east_m_s
        xk[8,0] = -position_velocity.velocity.down_m_s
        rk = np.zeros((12,ctrl.models[0].N))
        rk[0,:] = 1.0
        rk[1,:] = 0.5
        rk[2,:] = 1.0+UAVParams.z_min
        
        drone.u_pitch_ref_temp, drone.u_roll_ref_temp, drone.u_thrust_temp = ctrl.spin(
            rk, xk)
        if drone.u_roll_ref_temp is not None:
            drone.u_roll_ref = drone.u_roll_ref_temp
        else:
            print("Roll solution not found!")
        if drone.u_pitch_ref_temp is not None:
            drone.u_pitch_ref = drone.u_pitch_ref_temp
        else:
            print("Pitch solution not found!")
        if drone.u_thrust_temp is not None:
            drone.u_thrust = drone.u_thrust_temp
        else:
            print("Thrust solution not found!")
        u_thrust_01 = (drone.rotor_scale*drone.u_thrust+2.665*UAVParams.m*UAVParams.g)/(UAVParams.f_max)
        if u_thrust_01 > 1:
            u_thrust_01 = 1
        elif u_thrust_01 < 0:
            u_thrust_01 = 0
            
        u_roll_ref_deg = np.rad2deg(drone.u_roll_ref)
        u_pitch_ref_deg = np.rad2deg(drone.u_pitch_ref)
        await drone.offboard.set_attitude(Attitude(-u_roll_ref_deg, u_pitch_ref_deg, 0.0, u_thrust_01))

        print(u_thrust_01,u_roll_ref_deg, u_pitch_ref_deg)
        await asyncio.sleep(0.005)
        # await asyncio.sleep(1)
        # print(velocity)
        # # flight mode 6 -> LANDING
        # if distance < 1e-4 and drone.telemetry.flight_mode != 6:
        #     print("-- Landing on Droneport")
        #     await drone.action.land()


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    await asyncio.sleep(1)

    if drone.trajectory_finished:    
        for task in running_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await asyncio.get_event_loop().shutdown_asyncgens()
        
        return

    # was_in_air = False
    # async for is_in_air in drone.telemetry.in_air():
    #     if is_in_air:
    #         was_in_air = is_in_air

    #     if was_in_air and not is_in_air:
    #         for task in running_tasks:
    #             task.cancel()
    #             try:
    #                 await task
    #             except asyncio.CancelledError:
    #                 pass
    #         await asyncio.get_event_loop().shutdown_asyncgens()

    #         return


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
    # loop = asyncio.get_event_loop()
    # loop.run_until_complete(run())
