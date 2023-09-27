import queue
import carla
import json
import cv2
import numpy as np
import os
import sys
import logging
import open3d as o3d
import matplotlib.pyplot as plt
import pygame

import csv

from npc_spawning import spawnWalkers, spawnVehicles
import save_sensors


client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10)

world = client.get_world()
map_name = world.get_map().name
print(map_name)

with open('scenarios/intersection/sensor.json', 'r') as json_file:
    config_data = json.load(json_file)

location_data = config_data['location']
rotation_data = config_data['rotation']

# Create a carla.Location object from the extracted location data
new_location = carla.Location(
    x=location_data['x'], y=location_data['y'], z=location_data['z'])

# Create a carla.Rotation object from the extracted rotation data
new_rotation = carla.Rotation(
    pitch=rotation_data['pitch'], yaw=rotation_data['yaw'], roll=rotation_data['roll'])

# Create a carla.Transform object from the location and rotation
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
w_all_actors, w_all_id = spawnWalkers(
    client, world, blueprintsWalkers, 50)
v_all_actors, v_all_id = spawnVehicles(
    client, world, vehicles_spawn_points, blueprintsVehicles, 15)
world.tick()

output_folder = 'out'
os.makedirs(output_folder, exist_ok=True)

# RGB Camera setup
rgb_sensor_bp = world.get_blueprint_library().find('sensor.camera.rgb')
rgb_sensor_bp.set_attribute('image_size_x', '800')
rgb_sensor_bp.set_attribute('image_size_y', '600')
rgb_sensor_bp.set_attribute('fov', '90')
rgb_sensor_bp.set_attribute('sensor_tick', '1.0')
rgb_sensor = world.spawn_actor(rgb_sensor_bp, new_transform)

# LiDAR setup
# lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
# lidar_bp.set_attribute('range', '50')  # Adjust range as needed
# lidar_bp.set_attribute('sensor_tick', '1.0')
# lidar_sensor = world.spawn_actor(lidar_bp, new_transform)

lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')

# Set the LiDAR attributes
lidar_bp.set_attribute('range', '120')  # Adjust range as needed
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('sensor_tick', '1.0')
lidar_bp.set_attribute('points_per_second', '3000000')
lidar_bp.set_attribute('upper_fov', '10.0')
lidar_bp.set_attribute('lower_fov', '-30.0')
lidar_bp.set_attribute('rotation_frequency', '30')
lidar_bp.set_attribute('dropoff_general_rate', '0.0')
lidar_bp.set_attribute('dropoff_intensity_limit', '0.0')
lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')

# Create a transform for the LiDAR sensor's location
lidar_sensor = world.spawn_actor(lidar_bp, new_transform)

# RADAR setup
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_sensor = world.spawn_actor(radar_bp, new_transform)

# Event Camera setup
event_camera_bp = world.get_blueprint_library().find('sensor.camera.dvs')
event_camera_bp.set_attribute('image_size_x', '800')
event_camera_bp.set_attribute('image_size_y', '600')
event_camera_bp.set_attribute('fov', '90')
event_camera_bp.set_attribute('sensor_tick', '1.0')  # Adjust tick as needed
event_camera_sensor = world.spawn_actor(event_camera_bp, new_transform)

# Create OpenCV windows for visualization
# cv2.namedWindow('RGB', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('RGB', 800, 600)
# cv2.namedWindow('LiDAR', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('LiDAR', 800, 600)
# cv2.namedWindow('RADAR', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('RADAR', 800, 600)

# Define callback functions for RGB, LiDAR, and RADAR


def process_lidar(lidar_data):
    # Sample code to create a depth image from LiDAR data (example only)

    # Convert LiDAR data to a numpy array of shape (num_points, 4)
    # Each row contains (x, y, z, intensity) for a LiDAR point
    lidar_points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
    lidar_points = lidar_points.reshape(-1, 4)

    # Extract X, Y, and Z coordinates from LiDAR points
    x = lidar_points[:, 0]
    y = lidar_points[:, 1]
    z = lidar_points[:, 2]

    # Create a depth image from the Z coordinates
    # You may need to adjust the scaling and parameters based on your LiDAR sensor
    min_depth = 0.0
    max_depth = 50.0  # Range set based on your LiDAR setup
    depth_image = ((z - min_depth) / (max_depth - min_depth)
                   * 255).astype(np.uint8)

    return depth_image


def process_radar(radar_data):
    # Sample code to visualize RADAR data (example only)

    # Convert RADAR data to a numpy array
    radar_points = np.frombuffer(radar_data.raw_data, dtype=np.float32)

    # You can perform custom processing and visualization here

    return radar_points  # Example: Return the raw RADAR data for visualization

# Create the output directory structure
output_dir = 'out'
os.makedirs(output_dir, exist_ok=True)
sensor_names = ['rgb', 'lidar', 'radar']

frame = 0
def rgb_callback(data):
    global frame
    # print(data.raw_data)
    # rgb_image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
    #     (data.height, data.width, 4))
    # print(rgb_image)
    # print("-----------------")
    # print(frame)
    frame += 1
    timestamp = data.timestamp
    timestamp_str = str(timestamp).replace(".", "_")
    image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
        (data.height, data.width, 4))
    output_file = os.path.join(
        output_dir, 'rgb', f'{timestamp_str}.png')
    print(output_file)
    cv2.imwrite(output_file, image)
    # cv2.imshow('RGB', rgb_image)


def lidar_callback(data):
    # processed_lidar = process_lidar(data)
    # print(processed_lidar)
    global frame
    # print("-----------------")
    # print(frame)
    frame+=1
    lidar_data = np.frombuffer(data.raw_data, dtype=np.float32)
    lidar_data = lidar_data.reshape((-1, 4))  # Assuming XYZ format
    # lidar_points = np.frombuffer(data.raw_data, dtype=np.float32)
    # lidar_points = lidar_points.reshape(-1, 4)
    timestamp = data.timestamp
    lidar_xyz = lidar_data[:, :3]

    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_xyz)

    # Set the filename with .ply extension
    filename = os.path.join(output_dir, 'lidar', f'{timestamp}.ply')
    print(filename)
    o3d.io.write_point_cloud(filename, point_cloud)

    # Create an Open3D PointCloud object
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(lidar_xyz)
    # filename = os.path.join(
    #     output_dir, 'lidar', f'{timestamp}.pcd')
    # print(filename)
    # # Save the point cloud as .pcd
    # o3d.io.write_point_cloud(filename, point_cloud)

    
    # # Convert the point cloud to a NumPy array for visualization
    # point_cloud_array = np.asarray(point_cloud.points)

    # # Define camera intrinsics (you may need to adjust these)
    # intrinsics = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])

    #         # Project the 3D points to 2D image coordinates
    # def project_to_image(points, intrinsics):
    #     projected_points = np.dot(intrinsics, points.T).T
    #     projected_points[:, :-1] /= projected_points[:, -1:]
    #     return projected_points[:, :-1]

    # projected_points = project_to_image(point_cloud_array, intrinsics)

    # # Create an empty image
    # image = np.zeros((480, 640, 3), dtype=np.uint8)

    # # Draw the projected points as pixels on the image
    # for x, y in projected_points.astype(int):
    #     if 0 <= x < 640 and 0 <= y < 480:
    #         # Set the pixel color to white
    #         image[y, x] = [255, 255, 255]
    # filename = os.path.join(
    #     output_dir, 'lidar', f'{timestamp}.png')
    # # filename = os.path.join(
    # # output_folder, f'/lidar/{timestamp}.png')
    # # Save the image as .png
    # cv2.imwrite(filename, image)
    # timestamp = data.timestamp
    # timestamp_str = str(timestamp).replace(".", "_")
    # #         # Convert LiDAR data to a numpy array of shape (num_points, 4)
    # lidar_points = np.frombuffer(data.raw_data, dtype=np.float32)
    # lidar_points = lidar_points.reshape(-1, 4)

    #     # Create a PCL PointCloud object
    # cloud = pcl.PointCloud()
    #     # Use only X, Y, and Z coordinates
    # cloud.from_array(lidar_points[:, :3])

    #     # Save the PointCloud as a PCD file
    # output_file = os.path.join(
    # output_dir, 'lidar', f'{timestamp_str}.pcd')
    # pcl.save(cloud, output_file)
    # cv2.imshow('LiDAR', processed_lidar)

def radar_callback(data):
    # Convert CARLA RadarDetection data to NumPy array
    # points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    # points = np.reshape(points, (len(data), 4))
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(data), 4))
    radar_data = np.array([(p[0], p[1], p[2], p[3]) for p in points], dtype=np.dtype('f4'))

    # Save the structured NumPy array to a file
    np.save(f'out/radar/{data.timestamp}data.npy', radar_data)
    # Save the NumPy array to a file
    # np.save(f'out/radar/{data.timestamp}data.npy', points)

def dvs_callback(data):
    print(data.timestamp)
    dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
    dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
    # Blue is positive, red is negative
    dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
    surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))

    # Save the image as a file (e.g., PNG) to the local filesystem
    output_dir = 'out/dvs'  # Specify the directory where you want to save the images
    os.makedirs(output_dir, exist_ok=True)
    image_filename = os.path.join(output_dir, f'{data.timestamp}.png')
    pygame.image.save(surface, image_filename)

# def radar_callback(data):
#     points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
#     points = np.reshape(points, (len(data), 4))
#     np.save(f'out/radar/{data.timestamp}data.npy', data)
#     radar_data = np.frombuffer(data.raw_data, dtype=np.float32)
#     timestamp = data.timestamp
# #         # Save the RADAR data as a CSV file
#     filename = os.path.join(output_dir, 'radar', f'{timestamp}.csv')
#     with open(filename, 'w', newline='') as csvfile:
#         csv_writer = csv.writer(csvfile)
#         csv_writer.writerow(radar_data)
    # processed_radar = process_radar(data)
    # cv2.imshow('RADAR', processed_radar)


# Attach the callback functions to the sensors
# rgb_sensor.listen(rgb_callback)
# lidar_sensor.listen(lidar_callback)
# radar_sensor.listen(radar_callback)
event_camera_sensor.listen(dvs_callback)

# Main loop to run the simulation and display sensor data
# while True:
#     world.tick()
#     if cv2.waitKey(1) & 0xFF == 27:  # Press Esc key to exit
#         break
try:
    while True:
        world.tick()
except KeyboardInterrupt:
    pass
# Clean up the sensors
# rgb_sensor.stop()
# lidar_sensor.stop()
# radar_sensor.stop()

# Create a single queue for all sensor data
sensor_queue = queue.Queue()

# # Define a callback function to handle data from all sensors


# def sensor_callback(data):
#     sensor_queue.put(data)


# rgb_sensor.listen(sensor_callback)
# lidar_sensor.listen(sensor_callback)
# radar_sensor.listen(sensor_callback)

# # Main simulation loop
# while True:
#     try:
#         # Get sensor data from the queue
#         data = sensor_queue.get(block=True, timeout=1.0)

#         # Determine the sensor type based on the data's source attribute
#         sensor_type = data.attributes.get('sensor_type')

#         if sensor_type == 'rgb':
#             image = data.convert(carla.ColorConverter.Raw)
#             timestamp = data.timestamp
#             filename = os.path.join(
#                 output_folder, f'/rgb/{timestamp}.png')
#             image.save_to_disk("path_to_save_image.png")
#             pass
#             # Process RGB data and store it on disk
#         elif sensor_type == 'lidar':
#             # Convert the data to a NumPy array
#             lidar_data = np.frombuffer(data.raw_data, dtype=np.float32)
#             lidar_data = lidar_data.reshape((-1, 3))  # Assuming XYZ format

#             # Create an Open3D PointCloud object
#             point_cloud = o3d.geometry.PointCloud()
#             point_cloud.points = o3d.utility.Vector3dVector(lidar_data)
#             filename = os.path.join(
#                 output_folder, f'/lidar/{timestamp}.pcd')
#             # Save the point cloud as .pcd
#             o3d.io.write_point_cloud(filename, point_cloud)

#             # Convert the point cloud to a NumPy array for visualization
#             point_cloud_array = np.asarray(point_cloud.points)

#             # Define camera intrinsics (you may need to adjust these)
#             intrinsics = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])

#             # Project the 3D points to 2D image coordinates
#             def project_to_image(points, intrinsics):
#                 projected_points = np.dot(intrinsics, points.T).T
#                 projected_points[:, :-1] /= projected_points[:, -1:]
#                 return projected_points[:, :-1]

#             projected_points = project_to_image(point_cloud_array, intrinsics)

#             # Create an empty image
#             image = np.zeros((480, 640, 3), dtype=np.uint8)

#             # Draw the projected points as pixels on the image
#             for x, y in projected_points.astype(int):
#                 if 0 <= x < 640 and 0 <= y < 480:
#                     # Set the pixel color to white
#                     image[y, x] = [255, 255, 255]
#             filename = os.path.join(
#                 output_folder, f'/lidar/{timestamp}.png')
#             # Save the image as .png
#             cv2.imwrite(filename + "_lidar.png", image)
#             pass
#             # Process LiDAR data and store it on disk
#         elif sensor_type == 'radar':
#             pass
#             # Process RADAR data and store it on disk

#     except KeyboardInterrupt:
#         break

# # Clean up the sensors when done
# rgb_sensor.stop()
# lidar_sensor.stop()
# radar_sensor.stop()

# ---------------------------------------

# Define callback functions for RGB, LiDAR, and RADAR
# def rgb_callback(data):
#     save_sensor_data(data, 'rgb')


# def lidar_callback(data):
#     save_sensor_data(data, 'lidar')


# def radar_callback(data):
#     save_sensor_data(data, 'radar')


# rgb_sensor.listen(rgb_callback)
# lidar_sensor.listen(lidar_callback)
# radar_sensor.listen(radar_callback)

# # Create the output directory structure
# output_dir = 'out'
# os.makedirs(output_dir, exist_ok=True)
# sensor_names = ['rgb', 'lidar', 'radar']


# def save_sensor_data(data, sensor_name):
#     # Get the timestamp
#     timestamp = data.timestamp
#     # Replace '.' with '_' in the timestamp
#     timestamp_str = str(timestamp).replace(".", "_")

#     if sensor_name == 'rgb':
#         # Save RGB data as an image
#         image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
#             (data.height, data.width, 4))
#         output_file = os.path.join(
#             output_dir, sensor_name, f'{timestamp_str}.png')
#         cv2.imwrite(output_file, image)

#     elif sensor_name == 'lidar':
#         # Convert LiDAR data to a numpy array of shape (num_points, 4)
#         lidar_points = np.frombuffer(data.raw_data, dtype=np.float32)
#         lidar_points = lidar_points.reshape(-1, 4)

#         # Create a PCL PointCloud object
#         cloud = pcl.PointCloud()
#         # Use only X, Y, and Z coordinates
#         cloud.from_array(lidar_points[:, :3])

#         # Save the PointCloud as a PCD file
#         output_file = os.path.join(
#             output_dir, sensor_name, f'{timestamp_str}.pcd')
#         pcl.save(cloud, output_file)

#     elif sensor_name == 'radar':
#         # Convert RADAR data to a numpy array
#         radar_data = np.frombuffer(data.raw_data, dtype=np.float32)

#         # Save the RADAR data as a CSV file
#         output_file = os.path.join(
#             output_dir, sensor_name, f'{timestamp_str}.csv')
#         with open(output_file, 'w', newline='') as csvfile:
#             csv_writer = csv.writer(csvfile)
#             csv_writer.writerow(radar_data)


# # Main loop to run the simulation and collect sensor data
# try:
#     while True:
#         world.tick()
# except KeyboardInterrupt:
#     pass

# # Clean up the sensors
# rgb_sensor.stop()
# lidar_sensor.stop()
# radar_sensor.stop()
