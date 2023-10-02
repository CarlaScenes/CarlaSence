import carla
import json
import cv2
import numpy as np
import os
import open3d as o3d
import pygame
from npc_spawning import spawnWalkers, spawnVehicles

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10)

world = client.get_world()
map_name = world.get_map().name

with open('scenarios/intersection/spectator_cordinates.json', 'r') as json_file:
    config_data = json.load(json_file)

location_data = config_data['location']
rotation_data = config_data['rotation']

new_location = carla.Location(
    x=location_data['x'], y=location_data['y'], z=location_data['z'])
new_rotation = carla.Rotation(
    pitch=rotation_data['pitch'], yaw=rotation_data['yaw'], roll=rotation_data['roll'])
new_transform = carla.Transform(new_location, new_rotation)

spectator = world.get_spectator()
new_transform = carla.Transform(new_location, new_rotation)
spectator.set_transform(new_transform)
spectator_transform = spectator.get_transform()

blueprint_library = world.get_blueprint_library()
blueprintsVehicles = blueprint_library.filter('vehicle.*')
vehicles_spawn_points = world.get_map().get_spawn_points()
blueprintsWalkers = blueprint_library.filter('walker.pedestrian.*')

# Spawn npc actors
# TODO: Reverify this. It is not working as expected sometimes.
# w_all_actors, w_all_id = spawnWalkers(
#     client, world, blueprintsWalkers, 15)
# v_all_actors, v_all_id = spawnVehicles(
#     client, world, vehicles_spawn_points, blueprintsVehicles, 20)
# world.tick()

output_folder = 'out'
os.makedirs(output_folder, exist_ok=True)

# RGB Camera setup
rgb_sensor_bp = world.get_blueprint_library().find('sensor.camera.rgb')
rgb_sensor_bp.set_attribute('image_size_x', '800')
rgb_sensor_bp.set_attribute('image_size_y', '600')
rgb_sensor_bp.set_attribute('fov', '90')
rgb_sensor_bp.set_attribute('sensor_tick', '.10')
rgb_sensor = world.spawn_actor(rgb_sensor_bp, new_transform)

# LiDAR setup
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '120')  # Adjust range as needed
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('sensor_tick', '.10')
lidar_bp.set_attribute('points_per_second', '3000000')
lidar_bp.set_attribute('upper_fov', '10.0')
lidar_bp.set_attribute('lower_fov', '-30.0')
lidar_bp.set_attribute('rotation_frequency', '30')
lidar_bp.set_attribute('dropoff_general_rate', '0.0')
lidar_bp.set_attribute('dropoff_intensity_limit', '0.0')
lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
lidar_sensor = world.spawn_actor(lidar_bp, new_transform)

# RADAR setup
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_bp.set_attribute('sensor_tick', '.10')
radar_sensor = world.spawn_actor(radar_bp, new_transform)

# Event Camera setup
event_camera_bp = world.get_blueprint_library().find('sensor.camera.dvs')
event_camera_bp.set_attribute('image_size_x', '800')
event_camera_bp.set_attribute('image_size_y', '600')
event_camera_bp.set_attribute('fov', '90')
event_camera_bp.set_attribute('sensor_tick', '.10')  # Adjust tick as needed
event_camera_sensor = world.spawn_actor(event_camera_bp, new_transform)


def process_lidar(lidar_data):
    lidar_points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
    lidar_points = lidar_points.reshape(-1, 4)

    # Extract X, Y, and Z coordinates from LiDAR points
    x = lidar_points[:, 0]
    y = lidar_points[:, 1]
    z = lidar_points[:, 2]

    # You may need to adjust the scaling and parameters based on your LiDAR sensor
    min_depth = 0.0
    max_depth = 50.0  # Range set based on your LiDAR setup
    depth_image = ((z - min_depth) / (max_depth - min_depth)
                   * 255).astype(np.uint8)

    return depth_image


def process_radar(radar_data):
    # Convert RADAR data to a numpy array
    radar_points = np.frombuffer(radar_data.raw_data, dtype=np.float32)

    return radar_points  # Example: Return the raw RADAR data for visualization


# Create the output directory structure
output_dir = 'out'
os.makedirs(output_dir, exist_ok=True)
sensor_names = ['rgb', 'lidar', 'radar']


def rgb_callback(data):
    timestamp = data.timestamp
    image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
        (data.height, data.width, 4))
    output_file = os.path.join(
        output_dir, 'rgb', f'{timestamp}.png')
    cv2.imwrite(output_file, image)


def lidar_callback(data):
    lidar_data = np.frombuffer(data.raw_data, dtype=np.float32)
    lidar_data = lidar_data.reshape((-1, 4))
    timestamp = data.timestamp
    lidar_xyz = lidar_data[:, :3]
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_xyz)
    filename = os.path.join(output_dir, 'lidar', f'{timestamp}.ply')
    o3d.io.write_point_cloud(filename, point_cloud)


def radar_callback(data):
    timestamp = data.timestamp
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(data), 4))
    radar_data = np.array([(p[0], p[1], p[2], p[3])
                          for p in points], dtype=np.dtype('f4'))
    np.save(f'out/radar/{timestamp}.npy', radar_data)


def dvs_callback(data):
    timestamp = data.timestamp
    dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
        ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
    dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
    # Blue is positive, red is negative
    dvs_img[dvs_events[:]['y'], dvs_events[:]
            ['x'], dvs_events[:]['pol'] * 2] = 255
    surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
    output_dir = 'out/dvs'
    os.makedirs(output_dir, exist_ok=True)
    image_filename = os.path.join(output_dir, f'{timestamp}.png')
    pygame.image.save(surface, image_filename)


# Attach the callback functions to the sensors
rgb_sensor.listen(rgb_callback)
lidar_sensor.listen(lidar_callback)
radar_sensor.listen(radar_callback)
event_camera_sensor.listen(dvs_callback)

try:
    while True:
        world.tick()
except KeyboardInterrupt:
    pass

# Clean up the sensors
rgb_sensor.stop()
lidar_sensor.stop()
radar_sensor.stop()
event_camera_sensor.stop()

# Destroy the sensors
rgb_sensor.destroy()
lidar_sensor.destroy()
radar_sensor.destroy()
event_camera_sensor.destroy()
