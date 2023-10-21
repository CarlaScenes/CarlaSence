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

def is_callback(id, data):
    timestamp = data.timestamp
    image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
        (data.height, data.width, 4))
    output_folder = os.path.join(
        'out', 'is')
    os.makedirs(output_folder, exist_ok=True)
    output_file = os.path.join(
        output_folder, f'{timestamp}.png')
    cv2.imwrite(output_file, image)

def ss_callback(id, data):
    timestamp = data.timestamp
    output_folder = os.path.join(
        'out', 'ss')
    os.makedirs(output_folder, exist_ok=True)

    data.convert(carla.ColorConverter.CityScapesPalette)
    image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
        (data.height, data.width, 4))
    output_folder = os.path.join(
        'out', 'ss')
    os.makedirs(output_folder, exist_ok=True)
    output_file = os.path.join(
        output_folder, f'{timestamp}.png')
    cv2.imwrite(output_file, image)


def dvs_callback(id, data):
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
    width = image.width
    height = image.height

    # Convert the raw data to a numpy array of 32-bit floats
    image_data = np.frombuffer(image.raw_data, dtype=np.float32)

    # Reshape the image data to the correct shape (height, width, 2)
    image_data = image_data.reshape((height, width, 2))

    # Extract the magnitude of the optical flow vectors
    magnitude = np.sqrt(np.sum(image_data ** 2, axis=2))

    # Normalize the magnitude values to the range [0, 255] for visualization
    normalized_magnitude = cv2.normalize(
        magnitude, None, 0, 255, cv2.NORM_MINMAX)

    # Convert the magnitude values to 8-bit BGR format (for visualization)
    bgr_image = cv2.applyColorMap(
        normalized_magnitude.astype(np.uint8), cv2.COLORMAP_JET)

    output_folder = os.path.join(
        'out', 'optical')
    os.makedirs(output_folder, exist_ok=True)
    filename = os.path.join(output_folder, f"{image.timestamp}.png")
    cv2.imwrite(filename, bgr_image)


def lidar_callback(sensor_uid, data):
    lidar_data = np.frombuffer(data.raw_data, dtype=np.float32)
    lidar_data = lidar_data.reshape((-1, 4))
    timestamp = data.timestamp
    lidar_xyz = lidar_data[:, :3]
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_xyz)
    # filename = os.path.join(output_folder, f'{timestamp}.ply')
    # o3d.io.write_point_cloud(filename, point_cloud)

#     view_data = [
#     {
#         "boundingbox_max": [109.03180694580078, 88.123725891113281, 20.824602127075195],
#         "boundingbox_min": [2.8766894340515137, -73.388092041015625, -3.0116114616394043],
#         "field_of_view": 60.0,
#         "front": [-0.98697397365259365, -0.13138490724786892, -0.092846009498945087],
#         "lookat": [55.954248189926147, 7.3678169250488281, 8.9064953327178955],
#         "up": [-0.10795593810728604, 0.11298358359598086, 0.98771464769192618],
#         "zoom": 0.23999999999999957
#     }
# ]

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(point_cloud)
    # ctr = vis.get_view_control()
    # ctr.change_field_of_view(-30)
    # ctr.scale(-16)
    # output_folder = os.path.join('out', 'lidar_png')
    # os.makedirs(output_folder, exist_ok=True)
    # filename = os.path.join(output_folder, f'{timestamp}.png')
    # vis.capture_screen_image(filename, do_render=True)
    # vis.destroy_window()
    # Save the point cloud as a .ply file
    output_folder = os.path.join('out', 'lidar')
    os.makedirs(output_folder, exist_ok=True)
    filename = os.path.join(output_folder, f'{timestamp}.ply')
    o3d.io.write_point_cloud(filename, point_cloud)


rgb_sensors = []
event_camera_sensors = []
optical_camera_sensors = []
lidar_sensors = []
is_sensors = []
ss_sensors = []

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
            spectator = world.get_spectator()
            spectator.set_transform(transform)
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


            is_sensor_bp = blueprint_library.find('sensor.camera.instance_segmentation')
            is_sensor_bp.set_attribute('image_size_x', '1920')
            is_sensor_bp.set_attribute('image_size_y', '1280')
            is_sensor_bp.set_attribute('fov', '110')
            is_sensor_bp.set_attribute('sensor_tick', '.1')

            is_sensor = world.spawn_actor(is_sensor_bp, transform)
            is_sensor.attributes["sensor_uid"] = coordinate["id"]
            is_sensor.listen(lambda image: is_callback(
                coordinate["id"], image))
            is_sensors.append(is_sensor)

            ss_sensor_bp = blueprint_library.find('sensor.camera.instance_segmentation')
            ss_sensor_bp.set_attribute('image_size_x', '1920')
            ss_sensor_bp.set_attribute('image_size_y', '1280')
            ss_sensor_bp.set_attribute('fov', '110')
            ss_sensor_bp.set_attribute('sensor_tick', '.1')

            ss_sensor = world.spawn_actor(ss_sensor_bp, transform)
            ss_sensor.attributes["sensor_uid"] = coordinate["id"]
            ss_sensor.listen(lambda image: ss_callback(
                coordinate["id"], image))
            ss_sensors.append(ss_sensor)


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

            optical_camera_bp = blueprint_library.find(
                'sensor.camera.optical_flow')
            optical_camera_bp.set_attribute('image_size_x', '1920')
            optical_camera_bp.set_attribute('image_size_y', '1280')
            optical_camera_bp.set_attribute('fov', '110')
            optical_camera_bp.set_attribute('sensor_tick', '.1')
            optical_camera_sensor = world.spawn_actor(
                optical_camera_bp, transform)
            optical_camera_sensor.attributes["sensor_uid"] = coordinate["id"]
            optical_camera_sensor.listen(lambda image: optical_camera_callback(
                coordinate["id"], image))
            optical_camera_sensors.append(optical_camera_sensor)

            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '120')  # Adjust range as needed
            lidar_bp.set_attribute('channels', '64')
            lidar_bp.set_attribute('sensor_tick', '.10')
            lidar_bp.set_attribute('points_per_second', '3000000')
            lidar_bp.set_attribute('horizontal_fov', '110')
            lidar_bp.set_attribute('rotation_frequency', '30')
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '0.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
            lidar_sensor = world.spawn_actor(lidar_bp, transform)
            lidar_sensor.attributes["sensor_uid"] = coordinate["id"]
            lidar_sensor.listen(lambda image: lidar_callback(
                coordinate["id"], image))
            lidar_sensors.append(lidar_sensor)


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

for sensor in lidar_sensors:
    sensor.stop()
    sensor.destroy()
