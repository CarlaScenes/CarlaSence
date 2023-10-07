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
blueprint_library = world.get_blueprint_library()
map_name = world.get_map().name


with open('scenarios/intersection/spectator_cordinates.json', 'r') as json_file:
    config_data = json.load(json_file)


def rgb_callback(id, data):
    timestamp = data.timestamp
    image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
        (data.height, data.width, 4))
    output_folder = os.path.join(
        'out', 'rgb')
    os.makedirs(output_folder, exist_ok=True)
    output_file = os.path.join(
        output_folder, f'{timestamp}.png')
    cv2.imwrite(output_file, image)

def dvs_callback(id, data):
    return
    timestamp = data.timestamp
    dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
        ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
    dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
    dvs_img[dvs_events[:]['y'], dvs_events[:]
            ['x'], dvs_events[:]['pol'] * 2] = 255
    surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
    output_folder = os.path.join(
        'out', 'dvs')
    os.makedirs(output_folder, exist_ok=True)
    output_file = os.path.join(output_folder, f'{timestamp}.png')
    pygame.image.save(surface, output_file)

def optical_camera_callback(sensor_uid, image):
    print(1)
    width = image.width
    height = image.height
    width = image.width
    height = image.height

    # Convert the raw data to a numpy array of 32-bit floats
    image_data = np.frombuffer(image.raw_data, dtype=np.float32)

    # Reshape the image data to the correct shape (height, width, 2)
    image_data = image_data.reshape((height, width, 2))

    # Extract the magnitude of the optical flow vectors
    magnitude = np.sqrt(np.sum(image_data ** 2, axis=2))

    # Normalize the magnitude values to the range [0, 255] for visualization
    normalized_magnitude = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)

    # Convert the magnitude values to 8-bit BGR format (for visualization)
    bgr_image = cv2.applyColorMap(normalized_magnitude.astype(np.uint8), cv2.COLORMAP_JET)

    output_folder = os.path.join(
        'out', 'optical')
    os.makedirs(output_folder, exist_ok=True)
    filename = os.path.join(output_folder, f"optical_camera_{image.timestamp}.png")
    cv2.imwrite(filename, bgr_image)

# def optical_camera_callback(sensor_uid, data):
#     timestamp = data.timestamp
#     image_array = np.frombuffer(data.raw_data, dtype=np.uint8)
#     image_rgb = image_array.reshape((data.height, data.width, 3))  # 3 channels for RGB
#     output_folder = os.path.join('out', 'optical_camera')
#     os.makedirs(output_folder, exist_ok=True)
#     output_file = os.path.join(output_folder, f'{timestamp}.png')
#     cv2.imwrite(output_file, cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB))


rgb_sensors = []
event_camera_sensors = []
optical_camera_sensors = []
for config_entry in config_data:
    if config_entry["town"] == map_name:
        for coordinate in config_entry["cordinates"]:
            location_data = coordinate['location']
            rotation_data = coordinate['rotation']
            location = carla.Location(
                x=location_data['x'], y=location_data['y'], z=location_data['z'])
            rotation = carla.Rotation(
                pitch=rotation_data['pitch'], yaw=rotation_data['yaw'], roll=rotation_data['roll'])
            transform = carla.Transform(location, rotation)
            rgb_sensor_bp = blueprint_library.find('sensor.camera.rgb')
            rgb_sensor_bp.set_attribute('image_size_x', '1920')
            rgb_sensor_bp.set_attribute('image_size_y', '1280')
            rgb_sensor_bp.set_attribute('fov', '110')
            rgb_sensor_bp.set_attribute('sensor_tick', '.1')

            rgb_sensor = world.spawn_actor(rgb_sensor_bp, transform)
            rgb_sensor.attributes["sensor_uid"] = coordinate["id"]
            rgb_sensor.listen(lambda image: rgb_callback(
                coordinate["id"], image))
            rgb_sensors.append(rgb_sensor)

            event_camera_bp = blueprint_library.find('sensor.camera.dvs')
            event_camera_bp.set_attribute('image_size_x', '1920')
            event_camera_bp.set_attribute('image_size_y', '1280')
            event_camera_bp.set_attribute('fov', '110')
            event_camera_bp.set_attribute('sensor_tick', '.1')
            event_camera_sensor = world.spawn_actor(event_camera_bp, transform)
            event_camera_sensor.attributes["sensor_uid"] = coordinate["id"]
            event_camera_sensor.listen(lambda image: dvs_callback(
                coordinate["id"], image))
            event_camera_sensors.append(event_camera_sensor)

            optical_camera_bp = blueprint_library.find('sensor.camera.optical_flow')
            optical_camera_bp.set_attribute('image_size_x', '1920')
            optical_camera_bp.set_attribute('image_size_y', '1280')
            optical_camera_bp.set_attribute('fov', '110')
            optical_camera_bp.set_attribute('sensor_tick', '.1')
            optical_camera_sensor = world.spawn_actor(optical_camera_bp, transform)
            optical_camera_sensor.attributes["sensor_uid"] = coordinate["id"]
            optical_camera_sensor.listen(lambda image: optical_camera_callback(
                coordinate["id"], image))
            optical_camera_sensors.append(optical_camera_sensor)


try:
    while True:
        world.tick()
except KeyboardInterrupt:
    pass


for sensor in rgb_sensors:
    sensor.stop()
    sensor.destroy()

for sensor in event_camera_sensors:
    sensor.stop()
    sensor.destroy()

for sensor in optical_camera_sensors:
    sensor.stop()
    sensor.destroy()
