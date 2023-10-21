import os
import carla
from carla import ColorConverter

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
            saveRgbImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('optical_flow') != -1):
            saveRgbImage(sensor_data[i], os.path.join(out_root_folder, sensor_name))

        if(sensor_name.find('segmentation_camera') != -1):
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
    output.save_to_disk(filepath + '/%05d'%output.frame)
    with open(filepath + "/rgb_camera_metadata.txt", 'a') as fp:
        fp.writelines(str(output) + ", ")
        fp.writelines(str(output.transform) + "\n")

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