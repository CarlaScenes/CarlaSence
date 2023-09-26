
import carla
import json
import cv2
import numpy as np
import os
import sys
import logging

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10)

world = client.get_world()
map_name = world.get_map().name
print(map_name)

with open('sensor.json', 'r') as json_file:
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

current_dir = os.path.dirname(__file__)
target_dir = os.path.abspath(os.path.join(
    current_dir, '..', '..'))
sys.path.append(target_dir)

from npc_spawning import spawnWalkers, spawnVehicles
import save_sensors

# Spawn npc actors
# w_all_actors, w_all_id = spawnWalkers(
#     client, world, blueprintsWalkers, 100)
# v_all_actors, v_all_id = spawnVehicles(
#     client, world, vehicles_spawn_points, blueprintsVehicles, 40)
world.tick()

output_folder = 'out'
os.makedirs(output_folder, exist_ok=True)


# RGB Camera setup
rgb_sensor_bp = world.get_blueprint_library().find('sensor.camera.rgb')
rgb_sensor_bp.set_attribute('image_size_x', '800')
rgb_sensor_bp.set_attribute('image_size_y', '600')
rgb_sensor_bp.set_attribute('fov', '90')
rgb_sensor = world.spawn_actor(rgb_sensor_bp, new_transform)

# LiDAR setup
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '50')  # Adjust range as needed
lidar_sensor = world.spawn_actor(lidar_bp, new_transform)

# RADAR setup
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_sensor = world.spawn_actor(radar_bp, new_transform)

# Event Camera setup
event_camera_bp = world.get_blueprint_library().find('sensor.camera.dvs')
event_camera_bp.set_attribute('image_size_x', '800')
event_camera_bp.set_attribute('image_size_y', '600')
event_camera_bp.set_attribute('fov', '90')
event_camera_bp.set_attribute('sensor_tick', '0.1')  # Adjust tick as needed
event_camera_sensor = world.spawn_actor(event_camera_bp, new_transform)
# height = event_camera_sensor.get_attribute('image_size_y')

# print(height)

# return

try:
    # Subscribe to the sensors and define callback functions to record data
    def save_rgb_image(data):
        timestamp = data.timestamp
        filename = os.path.join(output_folder, f'rgb_image_{timestamp}.png')
        data.save_to_disk(filename)

    def save_lidar_data(data):
        
        pass

    def save_radar_data(data):
        
        pass

    def save_event_camera_image(data):
        timestamp = data.timestamp
        event_image = np.zeros((600, 800), dtype=np.uint8)

        # Iterate over the event data and set the pixels in the event image.
        for event in data:
            x, y = event.x, event.y
        print(event)
        if event.pol == carla.sensor.data.EventPolarity.ON:
            event_image[y, x] = 255

        # Save the event image to disk.
        cv2.imwrite(f'event_image_{timestamp}.png', event_image)

    # rgb_sensor.listen(lambda data: save_rgb_image(data))
    lidar_sensor.listen(lambda data: save_lidar_data(data))
    radar_sensor.listen(lambda data: save_radar_data(data))
    event_camera_sensor.listen(lambda data: save_event_camera_image(data))

    # Keep the script running to capture sensor data
    input("Press Enter to exit...")

finally:
    # Clean up and destroy the sensors
    rgb_sensor.destroy()
    lidar_sensor.destroy()
    radar_sensor.destroy()
    event_camera_sensor.destroy()