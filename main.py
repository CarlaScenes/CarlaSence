import sys
import subprocess
import glob
import shutil
import os
try:
    sys.path.append(glob.glob('./carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation
import argparse
import logging
from npc_spawning import spawnWalkers, spawnVehicles
from configuration import attachSensorsToVehicle, SimulationParams, setupTrafficManager, setupWorld, setupWorldWeather, createOutputDirectories, CarlaSyncMode
import save_sensors
import random
import json
import time
import queue
import os
from os import path
from ego_vehicle import EgoVehicle
from utils.arg_parser import CommandLineArgsParser
from utils.weather import weather_presets


def main():

    args_parser = CommandLineArgsParser()
    args = args_parser.parse_args()
    print(args)

    # Create CARLA client
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    # Setup simulation parameters
    SimulationParams.town_map = args.map
    SimulationParams.num_of_walkers = args.number_of_walkers
    SimulationParams.num_of_vehicles = args.number_of_vehicles
    SimulationParams.delta_seconds = args.delta_seconds
    SimulationParams.ignore_first_n_ticks = args.ignore_first_n_ticks
    # TODO: Is > 1 ego vehicle really required?
    SimulationParams.number_of_ego_vehicles = args.number_of_ego_vehicles
    SimulationParams.PHASE = SimulationParams.town_map + \
        "_" + SimulationParams.dt_string
    SimulationParams.data_output_subfolder = os.path.join(
        "out/", SimulationParams.PHASE)
    SimulationParams.manual_control = args.manual_control
    SimulationParams.res = args.res
    SimulationParams.autopilot = args.autopilot
    SimulationParams.debug = args.debug

    world = client.get_world()
    avail_maps = client.get_available_maps()
    world = client.load_world(SimulationParams.town_map)
    blueprint_library = world.get_blueprint_library()

    for name, value in weather_presets:
        if name == args.weather:
            SimulationParams.weather = value
            break

    # Setup
    setupWorld(world)
    if SimulationParams.weather is not None:
        setupWorldWeather(world, SimulationParams.weather)
    setupTrafficManager(client)

    # Get all required blueprints
    blueprintsVehicles = blueprint_library.filter('vehicle.*')
    vehicles_spawn_points = world.get_map().get_spawn_points()
    blueprintsWalkers = blueprint_library.filter('walker.pedestrian.*')
    walker_controller_bp = blueprint_library.find('controller.ai.walker')
    walkers_spawn_points = world.get_random_location_from_navigation()
    lidar_segment_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')

    egos = []
    for i in range(SimulationParams.number_of_ego_vehicles):
        egos.append(EgoVehicle(
            SimulationParams.sensor_json_filepath[i], None, world, args))

    # Spawn npc actors
    w_all_actors, w_all_id = spawnWalkers(
        client, world, blueprintsWalkers, SimulationParams.num_of_walkers)
    v_all_actors, v_all_id = spawnVehicles(
        client, world, vehicles_spawn_points, blueprintsVehicles, SimulationParams.num_of_vehicles)
    world.tick()

    spectator = world.get_spectator()
    if (SimulationParams.number_of_ego_vehicles > 0):
        transform = egos[0].ego.get_transform()
        spectator.set_transform(carla.Transform(
            transform.location + carla.Location(z=100), carla.Rotation(pitch=-90)))

    print("Starting simulation...")

    k = 0
    run_intersection = False

    try:
        with CarlaSyncMode(world, []) as sync_mode:
            while True:
                frame_id = sync_mode.tick(timeout=5.0)
                if (k < SimulationParams.ignore_first_n_ticks):
                    k = k + 1
                    continue

                # if run_intersection == False:
                #     run_intersection = True
                #     timeout_seconds = 120  # Change this to the desired timeout
                #     try:
                #         # Run the Python script with a timeout
                #         subprocess.run(
                #             ["python", "intersection-scenario.py"], check=True, timeout=timeout_seconds)
                #     except subprocess.TimeoutExpired:
                #         print(
                #             f"Script exceeded the timeout of {timeout_seconds} seconds and was terminated.")
                #     except subprocess.CalledProcessError as e:
                #         print(f"Error running the script: {e}")

                for i in range(len(egos)):
                    data = egos[i].getSensorData(frame_id)

                    output_folder = os.path.join(
                        SimulationParams.data_output_subfolder, "ego" + str(i))

                    # save_sensors.saveAllSensors(output_folder, data, egos[i].sensor_types)
                    save_sensors.saveAllSensors(
                        output_folder, data, egos[i].sensor_names)

                    control = egos[i].ego.get_control()
                    angle = control.steer
                    save_sensors.saveSteeringAngle(angle, output_folder)

                    # TODO: move output data to desired folder. Is this really necessary?
                    # if (frame_id > 1000):
                    #     destination_folder = None
                    #     if len(sys.argv[1:]) == 1:
                    #         assert path.exists(str(sys.argv[1:][0])), "Path does not exist"
                    #         destination_folder = sys.argv[1:][0]
                    #
                    #     # move generated data to other folder
                    #     if (destination_folder is not None):
                    #         shutil.move(SimulationParams.data_output_subfolder, destination_folder)
                    #     return

                print("new frame!")
    finally:
        # stop pedestrians (list is [controller, actor, controller, actor ...])
        for i in range(0, len(w_all_actors)):
            try:
                w_all_actors[i].stop()
            except:
                pass
        # destroy pedestrian (actor and controller)
        client.apply_batch([carla.command.DestroyActor(x) for x in w_all_id])
        client.apply_batch([carla.command.DestroyActor(x) for x in v_all_id])

        for ego in egos:
            ego.destroy()

        # This is to prevent Unreal from crashing from waiting the client.
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)


if __name__ == '__main__':
    try:
        # assert len(sys.argv) > 1, "no path for destination folder given.."
        main()
    except KeyboardInterrupt:
        pass
