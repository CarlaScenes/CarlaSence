import os
import carla
from carla import ColorConverter
import numpy as np
import pygame
import cv2


def saveAllSensors(out_root_folder, sensor_data, sensor_types):

    #TODO have it find the snapshot object dynamically instead of using the hardcoded 0 index
    saveSnapshot(out_root_folder, sensor_data[0])
    sensor_data.pop(0)

    for i in range(len(sensor_data)):
        sensor_name = sensor_types[i]

        # if(sensor_name == 'sensor.camera.rgb' or sensor_name.find('rgb') != -1):
        if(sensor_name.find('rgb_camera') != -1):
            saveRgbImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('dvs') != -1):
            dvs_callback(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('optical_flow') != -1):
            optical_camera_callback(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('instance_segmentation_camera') != -1):
            saveISImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('semantic_segmentation_camera') != -1):
            saveSegImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('depth_camera') != -1):
            saveDepthImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('imu') != -1):
            saveImu(sensor_data[i], os.path.join(out_root_folder, sensor_name), sensor_name)

        if(sensor_name.find('gnss') != -1):
            saveGnss(sensor_data[i], os.path.join(out_root_folder, sensor_name), sensor_name)

        # if(sensor_name == 'sensor.lidar.ray_cast' or sensor_name == 'sensor.lidar.ray_cast_semantic' or sensor_name.find('lidar') != -1):
        if(sensor_name.find('lidar') != -1):
            saveLidar(sensor_data[i], os.path.join(out_root_folder, sensor_name))
    return

def saveSnapshot(output, filepath):
    return

def saveSteeringAngle(value, filepath):
    with open(filepath + "/steering_norm.txt", 'a') as fp:
        fp.writelines(str(value) + ", ")
    with open(filepath + "/steering_true.txt", 'a') as fp:
        fp.writelines(str(70*value) + ", ")

def saveGnss(output, filepath, sensor_name):
    with open(filepath + "/" + sensor_name + ".txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

def saveImu(output, filepath, sensor_name):
    with open(filepath + "/" + sensor_name + ".txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

def saveLidar(output, filepath):
    output.save_to_disk(filepath + '/%05d'%output.frame)
    with open(filepath + "/lidar_metadata.txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

def saveRgbImage(output, filepath):
    try:
        output.save_to_disk(filepath + '/%05d'%output.frame)
        with open(filepath + "/rgb_camera_metadata.txt", 'a') as fp:
            fp.writelines(str(output) + ", ")
            fp.writelines(str(output.transform) + "\n")
    except Exception as error:
        # handle the exception
        print("An exception occurred:", error)

def saveISImage(output, filepath):
    try:
        output.save_to_disk(filepath + '/%05d'%output.frame)
        with open(filepath + "/rgb_camera_metadata.txt", 'a') as fp:
            fp.writelines(str(output) + ", ")
            fp.writelines(str(output.transform) + "\n")
    except Exception as error:
        # handle the exception
        print("An exception occurred:", error)

def dvs_callback(data, filepath):
    timestamp = data.timestamp
    dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
        ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
    dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
    dvs_img[dvs_events[:]['y'], dvs_events[:]
            ['x'], dvs_events[:]['pol'] * 2] = 255
    surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
    # output_folder = os.path.join(
    #     'out', 'dvs')
    # os.makedirs(output_folder, exist_ok=True)
    output_file = os.path.join(filepath, f'{data.frame}.png')
    pygame.image.save(surface, output_file)

def optical_camera_callback(image, filepath):
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

    # output_folder = os.path.join(
    #     'out', 'optical')
    # os.makedirs(output_folder, exist_ok=True)
    filename = os.path.join(filepath, f"{image.frame}.png")
    cv2.imwrite(filename, bgr_image)

def saveDepthImage(output, filepath):
    output.convert(carla.ColorConverter.Depth)
    output.save_to_disk(filepath + '/%05d'%output.frame)
    with open(filepath + "/depth_camera_metadata.txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

def saveSegImage(output, filepath):
    output.convert(carla.ColorConverter.CityScapesPalette)
    output.save_to_disk(filepath + '/%05d'%output.frame)
    with open(filepath + "/seg_camera_metadata.txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

def saveDvsImage(output, filepath):
    output.convert(carla.ColorConverter.CityScapesPalette)
    output.save_to_disk(filepath + '/%05d'%output.frame)
    with open(filepath + "/seg_camera_metadata.txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")