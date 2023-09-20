import sys
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
from npc_spawning import spawnWalkers, spawnVehicles
from configuration import attachSensorsToVehicle, SimulationParams, setupTrafficManager, setupWorld, createOutputDirectories, CarlaSyncMode
import save_sensors
import random
import json
import time
import queue
import os
from os import path
from EgoVehicle import EgoVehicle


def main():
    assert (len(SimulationParams.ego_vehicle_spawn_point)
            == len(SimulationParams.sensor_json_filepath))

    # Connect and load map
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    avail_maps = client.get_available_maps()
    world = client.load_world(SimulationParams.town_map)
    blueprint_library = world.get_blueprint_library()

    # Setup
    setupWorld(world)
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
            SimulationParams.sensor_json_filepath[i], SimulationParams.ego_vehicle_spawn_point[i], world))

    # Spawn npc actors
    w_all_actors, w_all_id = spawnWalkers(
        client, world, blueprintsWalkers, SimulationParams.num_of_walkers)
    v_all_actors, v_all_id = spawnVehicles(
        client, world, vehicles_spawn_points, blueprintsVehicles, SimulationParams.num_of_vehicles)
    world.tick()

    spectator = world.get_spectator()
    transform = egos[0].ego.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(z=100), carla.Rotation(pitch=-90)))

    print("Starting simulation...")

    k = 0
    try:
        with CarlaSyncMode(world, []) as sync_mode:
            while True:
                frame_id = sync_mode.tick(timeout=5.0)
                if (k < SimulationParams.ignore_first_n_ticks):
                    k = k + 1
                    continue
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

                    # move output data to desired folder
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
        assert len(sys.argv) > 1, "no path for destination folder given.."
        main()
    except KeyboardInterrupt:
        pass
    finally:
        destination_folder = None
        if len(sys.argv[1:]) == 1:
            assert path.exists(str(sys.argv[1:][0])), "Path does not exist"
            destination_folder = sys.argv[1:][0]

        # move generated data to other folder
        if (destination_folder is not None and path.exists(str(SimulationParams.data_output_subfolder))):
            shutil.move(SimulationParams.data_output_subfolder,
                        destination_folder)
        print('\ndone.')
