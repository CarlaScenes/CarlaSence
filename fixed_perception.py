import carla
from carla import Transform, Location, Rotation
from npc_spawning import spawnWalkers, spawnVehicles
from configuration import attachSensorsForFixedPerception, SimulationParams, setupTrafficManager, setupWorld, createOutputDirectoriesFixedPerception, CarlaSyncMode
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


class FixedPerception:

    def __init__(self, config_filepath, position, world, args, coordinate):
        self.world = world

        f = open(config_filepath)
        data = json.load(f)
        print(data)
        createOutputDirectoriesFixedPerception(data, coordinate['id'])

        self.sensors_ref, self.sensor_types, self.sensor_names = attachSensorsForFixedPerception(
            world, data, coordinate)  

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

    def getSensorData(self, frame_id):
        try:
            data = [self._retrieve_data(q, frame_id) for q in self.queues]
            print("yes")
            return data
        except Exception as error:
            print("An exception occurred:", error)

    def _retrieve_data(self, sensor_queue, frame_id):
        while True:
            data = sensor_queue.get(timeout=5.0)
            sensor = None
            if sensor_queue in self.sensor_q_map:
                sensor = self.sensor_q_map[sensor_queue]
            if data.frame == frame_id:
                return (data,sensor)

    def destroy(self):
        [s.destroy() for s in self.sensors_ref]
        # This is to prevent Unreal from crashing from waiting the client.
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
