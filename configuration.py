import carla
from datetime import datetime
import os
from carla import Transform, Location, Rotation
import queue
import time
import numpy as np


class SimulationParams:
    town_map = None
    weather = None
    num_of_walkers = None
    num_of_vehicles = None
    delta_seconds = None
    # At the very start at the simulation nothing happens, so skip the first n ticks
    ignore_first_n_ticks = None
    sensor_json_filepath = "config/sensors.json"
    fixed_perception_sensor_json_filepath = "config/sensors-fixed-perception.json"
    fixed_perception_sensor_locations_json_filepath = "config/sensors-cordinates-fixed-perception.json"
    number_of_ego_vehicles = None
    manual_control = None
    dt_string = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    PHASE = None
    # town_map + "_" + dt_string
    data_output_subfolder = None
    # os.path.join("out/", PHASE)

    # town_map = "Town03"
    # num_of_walkers = 20
    # num_of_vehicles = 15
    # delta_seconds = 0.03332
    # # At the very start at the simulation nothing happens, so skip the first n ticks
    # ignore_first_n_ticks = 70

    # sensor_json_filepath = [
    #     "config/sensors.json"
    # ]
    # ego_vehicle_spawn_point = [
    #     Transform(Location(x=35.679951, y=80.979996, z=0.500000),
    #               Rotation(pitch=0.000000, yaw=-89.999817, roll=0.000000))
    # ]

    # number_of_ego_vehicles = len(ego_vehicle_spawn_point)
    # manual_control = False
    # dt_string = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    # PHASE = town_map + "_" + dt_string
    # data_output_subfolder = os.path.join("out/", PHASE)


def attachSensorsToVehicle(world, data, vehicle_actor):
    blueprint_library = world.get_blueprint_library()
    sensor_references = []
    sensor_types = []
    sensor_names = []
    for i in range(len(data['sensors'])):
        sensor = data['sensors'][i]
        bp = blueprint_library.find(sensor['type'])
        json_trans = sensor["transform"][0]

        relative_transf = Transform(Location(x=float(json_trans['x']), y=float(json_trans['y']), z=float(json_trans['z'])), Rotation(
            pitch=float(json_trans['pitch']), yaw=float(json_trans['yaw']), roll=float(json_trans['roll'])))

        # Get all the attributes EXCLUDING type and transform
        blacklist = ['type', 'transform']
        settable_attributes = [
            attribute for attribute in sensor if attribute not in blacklist]
        for attr in settable_attributes:
            try:
                bp.set_attribute(str(attr), str(sensor[attr]))
            except:
                print("attr = ", attr)
                print("sensor[attr] = ", sensor[attr])
                print("sensor['type'] = ", sensor['type'])
                print("Problem with setting " + attr + "to " +
                      sensor[attr] + " in sensor " + sensor['type'])

        # vehicle_transform = vehicle_actor.get_transform()
        # roof_front_position = vehicle_transform.location + vehicle_transform.rotation.rotate(vehicle_actor.bounding_box.extent / 2 * carla.Vector3D(0.5, 0, 0.5))
        # print(roof_front_position)
        camera_init_trans = carla.Transform(carla.Location(z=2))
        sensor_actor = world.spawn_actor(
            bp, relative_transf, attach_to=vehicle_actor)

        sensor_types.append(sensor['type'])
        sensor_names.append(sensor['role_name'])
        sensor_references.append(sensor_actor)

        # PRINT CALIBRATION MATRICES
        if sensor["type"] == "sensor.lidar.ray_cast_semantic":
            lidar_2_world = sensor_actor.get_transform().get_matrix()
            print("LIDAR INFO")
            print("=================================================")
            print(
                "TRANSFORM the points from lidar space to world space = ", lidar_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.other.gnss":
            gnss_2_world = sensor_actor.get_transform().get_matrix()
            print("GNSS INFO")
            print("=================================================")
            print("TRANSFORM the points from gnss space to world space = ", gnss_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.other.imu":
            imu_2_world = sensor_actor.get_transform().get_matrix()
            print("IMU INFO")
            print("=================================================")
            print("TRANSFORM the points from imu space to world space = ", imu_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.camera.rgb":
            # Build the K projection matrix:
            # K = [[Fx,  0, image_w/2],
            #      [ 0, Fy, image_h/2],
            #      [ 0,  0,         1]]

            # This (4, 4) matrix transforms the points from world to sensor coordinates.
            world_2_camera = np.array(
                sensor_actor.get_transform().get_inverse_matrix())
            image_w = bp.get_attribute("image_size_x").as_int()
            image_h = bp.get_attribute("image_size_y").as_int()
            fov = bp.get_attribute("fov").as_float()
            focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

            # In this case Fx and Fy are the same since the pixel aspect
            # ratio is 1
            K = np.identity(3)
            K[0, 0] = K[1, 1] = focal
            K[0, 2] = image_w / 2.0
            K[1, 2] = image_h / 2.0

            print("CAMERA INFO")
            print("=================================================")
            print("image_w = ", image_w)
            print("image_h = ", image_h)
            print("fov = ", fov)
            print("focal = ", focal)
            print("TRANSFORM from world to sensor coordinates = ", world_2_camera)
            print("PROJECT from sensor to pixel coordinates = ", K)
            print("=================================================")

    return sensor_references, sensor_types, sensor_names


def attachSensorsForFixedPerception(world, data, coordinate):
    location_data = coordinate['location']
    rotation_data = coordinate['rotation']
    location = carla.Location(
                x=location_data['x'], y=location_data['y'], z=location_data['z'])
    rotation = carla.Rotation(
                pitch=rotation_data['pitch'], yaw=rotation_data['yaw'], roll=rotation_data['roll'])
    transform = carla.Transform(location, rotation)
    blueprint_library = world.get_blueprint_library()
    sensor_references = []
    sensor_types = []
    sensor_names = []
    for i in range(len(data['sensors'])):
        sensor = data['sensors'][i]
        bp = blueprint_library.find(sensor['type'])

        # Get all the attributes EXCLUDING type and transform
        blacklist = ['type', 'transform']
        settable_attributes = [
            attribute for attribute in sensor if attribute not in blacklist]
        for attr in settable_attributes:
            try:
                bp.set_attribute(str(attr), str(sensor[attr]))
            except:
                print("attr = ", attr)
                print("sensor[attr] = ", sensor[attr])
                print("sensor['type'] = ", sensor['type'])
                print("Problem with setting " + attr + "to " +
                      sensor[attr] + " in sensor " + sensor['type'])


        sensor_actor =  world.spawn_actor(bp, transform)
        sensor_types.append(sensor['type'])
        sensor_names.append(sensor['role_name'])
        sensor_references.append(sensor_actor)

        # PRINT CALIBRATION MATRICES
        if sensor["type"] == "sensor.lidar.ray_cast_semantic":
            lidar_2_world = sensor_actor.get_transform().get_matrix()
            print("LIDAR INFO")
            print("=================================================")
            print(
                "TRANSFORM the points from lidar space to world space = ", lidar_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.other.gnss":
            gnss_2_world = sensor_actor.get_transform().get_matrix()
            print("GNSS INFO")
            print("=================================================")
            print("TRANSFORM the points from gnss space to world space = ", gnss_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.other.imu":
            imu_2_world = sensor_actor.get_transform().get_matrix()
            print("IMU INFO")
            print("=================================================")
            print("TRANSFORM the points from imu space to world space = ", imu_2_world)
            print("=================================================")

        if sensor["type"] == "sensor.camera.rgb":
            # Build the K projection matrix:
            # K = [[Fx,  0, image_w/2],
            #      [ 0, Fy, image_h/2],
            #      [ 0,  0,         1]]

            # This (4, 4) matrix transforms the points from world to sensor coordinates.
            world_2_camera = np.array(
                sensor_actor.get_transform().get_inverse_matrix())
            image_w = bp.get_attribute("image_size_x").as_int()
            image_h = bp.get_attribute("image_size_y").as_int()
            fov = bp.get_attribute("fov").as_float()
            focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

            # In this case Fx and Fy are the same since the pixel aspect
            # ratio is 1
            K = np.identity(3)
            K[0, 0] = K[1, 1] = focal
            K[0, 2] = image_w / 2.0
            K[1, 2] = image_h / 2.0

            print("CAMERA INFO")
            print("=================================================")
            print("image_w = ", image_w)
            print("image_h = ", image_h)
            print("fov = ", fov)
            print("focal = ", focal)
            print("TRANSFORM from world to sensor coordinates = ", world_2_camera)
            print("PROJECT from sensor to pixel coordinates = ", K)
            print("=================================================")

    return sensor_references, sensor_types, sensor_names

class CarlaSyncMode(object):
    def __init__(self, world, sensors):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self._queues = []
        # self._settings = None
        self.first_time = True

    def __enter__(self):
        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        # data = [self._retrieve_data(q, timeout) for q in self._queues]
        # assert all(x.frame == self.frame for x in data)
        return self.frame

    def __exit__(self, *args, **kwargs):
        print("Got exit function!")

        self.world.apply_settings(self._settings)
        return

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data
            else:
                print("frame mismatch")
                print(data.frame)
                print(self.frame)


def setupTrafficManager(client):
    print("Settting up traffic manager...")
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)


def setupWorld(world):
    print("Settting up world...")
    settings = world.get_settings()
    settings.fixed_delta_seconds = SimulationParams.delta_seconds
    settings.synchronous_mode = True
    world.set_pedestrians_cross_factor(0.0)
    world.apply_settings(settings)


def setupWorldWeather(world, weather):
    world.set_weather(weather)

# This will create the whole file system structure. It will create a separate folder for each sensor


def createOutputDirectories(data):
    # output_sensor_folders = [ data['sensors'][i]['type'] for i in range(len(data['sensors'])) ]
    output_sensor_folders = [data['sensors'][i]['role_name']
                             for i in range(len(data['sensors']))]
    try:
        os.mkdir("out/")
    except OSError:
        pass
        # print("Folder " + "out/" + " already exists!")
    try:
        os.mkdir(SimulationParams.data_output_subfolder)
    except OSError:
        pass
        # print("Folder " + SimulationParams.data_output_subfolder + " already exists!")

    for i in range(SimulationParams.number_of_ego_vehicles):
        ego_name = "ego" + str(i) + "/"
        ego_folder = os.path.join(
            SimulationParams.data_output_subfolder, ego_name)
        try:
            os.mkdir(ego_folder)
        except:
            pass
            # print("Ego folder " + ego_folder + " already exists!")
        for sensor in output_sensor_folders:
            try:
                os.mkdir(os.path.join(ego_folder, sensor))
            except OSError:
                pass
                # print("Creation of " + os.path.join(ego_folder, sensor) + " failed")

def createOutputDirectoriesFixedPerception(data, id):
    # output_sensor_folders = [ data['sensors'][i]['type'] for i in range(len(data['sensors'])) ]
    output_sensor_folders = [data['sensors'][i]['role_name']
                             for i in range(len(data['sensors']))]
    try:
        os.mkdir("out/")
    except OSError:
        pass
        # print("Folder " + "out/" + " already exists!")
    try:
        os.mkdir(SimulationParams.data_output_subfolder)
    except OSError:
        pass
        # print("Folder " + SimulationParams.data_output_subfolder + " already exists!")

    fixed_name = "fixed-" + str(id) + "/"
    fixed_folder = os.path.join(
        SimulationParams.data_output_subfolder, fixed_name)
    try:
        os.mkdir(fixed_folder)
    except:
        pass
            # print("Ego folder " + fixed_folder + " already exists!")
    for sensor in output_sensor_folders:
        try:
            os.mkdir(os.path.join(fixed_folder, sensor))
        except OSError:
            pass