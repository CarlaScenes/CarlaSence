import carla
from carla import Transform, Location, Rotation
from npc_spawning import spawnWalkers, spawnVehicles
from configuration import attachSensorsToVehicle, SimulationParams, setupTrafficManager, setupWorld, createOutputDirectories, CarlaSyncMode
from utils import g29_steering_wheel
import save_sensors
import random
import json
import time
import queue
import threading


def findClosestSpawnPoint(spawn_points, target):
    dist = [(target.location.x-spawn_points[i].location.x)**2 + (target.location.y-spawn_points[i].location.y)
            ** 2 + (target.location.z-spawn_points[i].location.z)**2 for i in range(len(spawn_points))]
    return spawn_points[dist.index(min(dist))]


class EgoVehicle:

    def __init__(self, config_filepath, position, world, args):
        self.world = world

        f = open(config_filepath)
        data = json.load(f)
        createOutputDirectories(data)

        # Get all required blueprints
        blueprint_library = world.get_blueprint_library()
        blueprintsVehicles = blueprint_library.filter('vehicle.*')
        vehicles_spawn_points = world.get_map().get_spawn_points()

        # Spawn and configure Ego vehicle
        self.ego_bp = random.choice(
            blueprint_library.filter('vehicle.citroen.*'))
        print("************** ", self.ego_bp)
        self.ego_bp.set_attribute('role_name', 'ego')
        random_spawn_point = random.choice(vehicles_spawn_points)
        self.ego = world.spawn_actor(self.ego_bp, random_spawn_point)

        # TODO: Revise this. I am spawning at a random location. Is this correct?
        # self.ego = world.spawn_actor(self.ego_bp, findClosestSpawnPoint(
        #     spawn_points=vehicles_spawn_points, target=position))

        self.sensors_ref, self.sensor_types, self.sensor_names = attachSensorsToVehicle(
            world, data, self.ego)  # attachSensorsToVehicle should be a member function

        self.queues = []
        q = queue.Queue()
        world.on_tick(q.put)
        self.queues.append(q)
        self.sensor_q_map = {}
        self.sensor_q_map[q] = None
        for sensor in self.sensors_ref:
            q = queue.Queue()
            sensor.listen(q.put)
            self.sensor_q_map[q] = sensor
            self.queues.append(q)

        # TODO: Is threading necessary?
        print("Entering the manual control block...")
        if SimulationParams.manual_control == True:
            g29_thread = threading.Thread(
                target=g29_steering_wheel.main,
                args=(args, world, self.ego)
            )
            g29_thread.start()
        else:
            self.ego.set_autopilot(True)

    def getSensorData(self, frame_id):
        data = [self._retrieve_data(q, frame_id) for q in self.queues]
        return data

    def _retrieve_data(self, sensor_queue, frame_id):
        while True:
            data = sensor_queue.get(timeout=60.0)
            sensor = None
            if sensor_queue in self.sensor_q_map:
                sensor = self.sensor_q_map[sensor_queue]
            if data.frame == frame_id:
                return (data,sensor, self.ego)

    def destroy(self):
        [s.destroy() for s in self.sensors_ref]
        self.ego.destroy()

        # This is to prevent Unreal from crashing from waiting the client.
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
