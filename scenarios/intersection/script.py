import carla
import json
import cv2
import numpy as np

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
print(spectator_transform)
