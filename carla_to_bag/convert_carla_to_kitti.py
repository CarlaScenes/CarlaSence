import numpy as np
import re
import os
import sys
import shutil
from distutils.dir_util import copy_tree
from vedo import Points, show
from shutil import copy
import argparse
import math


# ************************************************************************************************************
# Template of the imu file:
# IMUMeasurement(
# frame=83,
# timestamp=11.504859,
# accelerometer=Vector3D(x=-0.003081, y=-0.029036, z=9.811818),
# gyroscope=Vector3D(x=0.000073, y=-0.001988, z=-0.036563), compass=6.280235),
# Transform(Location(x=-77.962997, y=-36.355682, z=1.662928),
# Rotation(pitch=-0.843351, yaw=-90.170631, roll=0.000478))
# Return: four different lists for each part (frame, timestamp, accelerometer, gyroscope, Location, Rotation)
# ************************************************************************************************************
def read_imu(path, num_frames):
    data = {}
    f = open(path, "r")
    lines = f.readlines()
    frames = []
    timestamps = []
    locations = []
    accelerometer = []
    gyroscope = []
    rotations = []

    if eval(num_frames) is not None:
        limit = int(num_frames)
    else:
        limit = len(lines)

    for c, line in enumerate(lines):

        if c >= limit:
            break

        init_line = line.strip()

        tmp = re.search('frame\=[^,]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("frame=", "")
        frames.append(float(tmp))

        tmp = re.search('timestamp\=[^,]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("timestamp=", "")
        timestamps.append(float(tmp))

        tmp = re.search('Location\([^)]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("Location(x=", "")
        tmp = tmp.replace("y=", "")
        tmp = tmp.replace("z=", "")
        tmp = tmp.split(", ")
        locations.append(tmp)

        tmp = re.search('Rotation\([^)]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("Rotation(pitch=", "")
        tmp = tmp.replace("yaw=", "")
        tmp = tmp.replace("roll=", "")
        tmp = tmp.split(", ")
        tmp = [str(math.radians(float(x))) for x in tmp]
        rotations.append(tmp)


        tmp = re.search('accelerometer=Vector3D\([^)]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("accelerometer=Vector3D(x=", "")
        tmp = tmp.replace("y=", "")
        tmp = tmp.replace("z=", "")
        tmp = tmp.split(", ")
        accelerometer.append(tmp)

        tmp = re.search('gyroscope=Vector3D\([^)]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("gyroscope=Vector3D(x=", "")
        tmp = tmp.replace("y=", "")
        tmp = tmp.replace("z=", "")
        tmp = tmp.split(", ")
        gyroscope.append(tmp)

    data["frames"] = frames
    data["timestamps"] = timestamps
    data["locations"] = locations
    data["rotations"] = rotations
    data["accelerometer"] = accelerometer
    data["gyroscope"] = gyroscope

    return data


# ************************************************************************************************************
# Template of the gnss file:
# GnssMeasurement(frame=83, timestamp=11.504859, lat=0.000187, lon=0.000730, alt=2.865521)
# Transform(Location(x=-77.962997, y=-36.355682, z=1.662928),
# Rotation(pitch=-0.843351, yaw=-90.170631, roll=0.000478))
# Return: four different lists for each part (frame, timestamp, accelerometer, gyroscope, Location, Rotation)
# ************************************************************************************************************
def read_gnss(path, num_frames):
    f = open(path, "r")
    lines = f.readlines()
    gnss_coordinates = []

    if eval(num_frames) is not None:
        limit = int(num_frames)
    else:
        limit = len(lines)

    for c, line in enumerate(lines):

        if c >= limit:
            break

        init_line = line.strip()
        tmp = re.search('lat=[^)]*', init_line)
        tmp = tmp.group(0)
        tmp = tmp.replace("lat=", "")
        tmp = tmp.replace("lon=", "")
        tmp = tmp.replace("alt=", "")
        tmp = tmp.split(", ")
        gnss_coordinates.append(tmp)

    return gnss_coordinates


# ************************************************************************************************************
# kitti format
# lat:   latitude of the oxts-unit (deg) (in carla)
# lon:   longitude of the oxts-unit (deg) (in carla)
# alt:   altitude of the oxts-unit (m) (in carla)
# roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi (in carla)
# pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2 (in carla)
# yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi (in carla)
# vn:    velocity towards north (m/s)
# ve:    velocity towards east (m/s)
# vf:    forward velocity, i.e. parallel to earth-surface (m/s)
# vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
# vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
# ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
# ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
# ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
# af:    forward acceleration (m/s^2) (in carla)
# al:    leftward acceleration (m/s^2) (in carla)
# au:    upward acceleration (m/s^2) (in carla)
# wx:    angular rate around x (rad/s)
# wy:    angular rate around y (rad/s)
# wz:    angular rate around z (rad/s)
# wf:    angular rate around forward axis (rad/s) (in carla)
# wl:    angular rate around leftward axis (rad/s) (in carla)
# wu:    angular rate around upward axis (rad/s) (in carla)
# pos_accuracy:  velocity accuracy (north/east in m)
# vel_accuracy:  velocity accuracy (north/east in m/s)
# navstat:       navigation status (see navstat_to_string)
# numsats:       number of satellites tracked by primary GPS receiver
# posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
# velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
# orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
#
# For the parameters that do not exist in carla, we replace them with zero values
# We convert gps and imu data from carla to oxts format from kitti (WE SUPPOSE THAT GPS AND IMU POSITIONS ARE THE SAME)
# ************************************************************************************************************
def store_oxts(data, path_to_store):
    res = []

    for c, i in enumerate(data["locations"]):
        res.clear()

        # append lat, lon, alt
        res.append(data["gnss_coordinates"][c][0])
        res.append(data["gnss_coordinates"][c][1])
        res.append(data["gnss_coordinates"][c][2])

        # res.append(data["locations"][c][0])
        # res.append(data["locations"][c][1])
        # res.append(data["locations"][c][2])

        # # append roll, pitch, yaw
        res.append(data["rotations"][c][2])
        res.append(data["rotations"][c][0])
        res.append(data["rotations"][c][1])

        # append 8 zero values
        [res.append('0') for j in range(0, 8)]

        # append forward, leftward and upward acceleration
        res.append(data["accelerometer"][c][0])
        res.append(data["accelerometer"][c][1])
        res.append(data["accelerometer"][c][2])

        # append 3 zero values
        [res.append('0') for j in range(0, 3)]

        # append  angular rate around forward axis, angular rate around leftward axis, angular rate around upward axis
        res.append(data["gyroscope"][c][0])
        res.append(data["gyroscope"][c][1])
        res.append(data["gyroscope"][c][2])

        # append 7 zero values
        [res.append('0') for j in range(0, 7)]

        file = open(path_to_store + '{0:0>10}'.format(str(c)) + ".txt", 'w')
        for c in res:
            file.write(c + " ")
        file.close()


# def _poses_from_oxts(self, oxts_packets):
#     """Helper method to compute SE(3) pose matrices from OXTS packets."""
#     er = 6378137.  # earth radius (approx.) in meters

#     # compute scale from first lat value
#     scale = np.cos(oxts_packets[0].lat * np.pi / 180.)

#     t_0 = []    # initial position
#     poses = []  # list of poses computed from oxts
#     for packet in oxts_packets:
#         # Use a Mercator projection to get the translation vector
#         tx = scale * packet.lon * np.pi * er / 180.
#         ty = scale * er * \
#             np.log(np.tan((90. + packet.lat) * np.pi / 360.))
#         tz = packet.alt
#         t = np.array([tx, ty, tz])

#         # We want the initial position to be the origin, but keep the ENU
#         # coordinate system
#         if len(t_0) == 0:
#             t_0 = t

#         # Use the Euler angles to get the rotation matrix
#         Rx = utils.rotx(packet.roll)
#         Ry = utils.roty(packet.pitch)
#         Rz = utils.rotz(packet.yaw)
#         R = Rz.dot(Ry.dot(Rx))

#         # Combine the translation and rotation into a homogeneous transform
#         poses.append(utils.transform_from_rot_trans(R, t - t_0)) 
#     return poses

# Compute a SE(3) pose matrix from an OXTS packet
def pose_from_oxts_packet(data, args):
    er = 6378137.  # earth radius (approx.) in meters
    poses = []
    scale = None
    origin = None
    prev = None
    for c, i in enumerate(data["locations"]):
        if scale is None:
            scale = np.cos(float(data["gnss_coordinates"][c][0]) * np.pi / 180.)

        # Use a Mercator projection to get the translation vector
        tx = scale * float(data["gnss_coordinates"][c][1]) * np.pi * er / 180.
        ty = scale * er * math.log(np.tan((90. + float(data["gnss_coordinates"][c][
                                                           0])) * np.pi / 360.))  # TO-DO: replace tan with atan in order not to have nan values
        tz = float(data["gnss_coordinates"][c][2])
        t = np.array([tx, ty, tz])

        # Use the Euler angles to get the rotation matrix

        Rx = rotx(float(data["rotations"][c][2]))
        Ry = roty(float(data["rotations"][c][0]))
        Rz = rotz(float(data["rotations"][c][1]))
        R = Rz.dot(Ry.dot(Rx))



        if origin is None:
            origin = t

        res = transform_from_rot_trans(R, t - origin)
        # Combine the translation and rotation into a homogeneous transform
        pose = str(res[0, 0]) + " " + str(res[0, 1]) + " " + str(res[0, 2]) + " " + str(res[0, 3]) + " " + str(
            res[1, 0]) + " " + str(res[1, 1]) + " " + str(res[1, 2]) + " " + str(res[1, 3]) + " " + str(
            res[2, 0]) + " " + str(res[2, 1]) + " " + str(res[2, 2]) + " " + str(res[2, 3]) + "\n"

        # pose = str(R[0,0]) + " " + str(R[0,1]) + " " + str(R[0,2]) + " " + str(t[0]) + " " + str(R[1,0]) + " " + str(R[1,1]) + " " + str(R[1,2]) + " " + str(t[1]) + " " + str(R[2,0]) + " " + str(R[2,1]) + " " + str(R[2,2]) + " " + str(t[2]) + "\n"        
        poses.append(pose)

    # write poses to file
    poses_file = open(
        os.path.join(args.path_to_carla, args.date + "/" + args.date + "_drive_" + args.seq + "_sync/poses.txt"), "w")
    for pose in poses:
        poses_file.write(pose)
    poses_file.close()


def transform_from_rot_trans(R, t):
    """Transforation matrix from rotation matrix and translation vector."""
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


# Rotation about the x-axis
def rotx(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


# Rotation about the y-axis
def roty(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


# Rotation about the z-axis
def rotz(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def store_timestamp(data, path_to_store):
    file = open(path_to_store + "timestamps.txt", 'w')
    for c in data:
        file.write(str(c) + "\n")
    file.close()


def visualize(data):
    p1 = Points(data, r=4, c='red')  # create the vedo object
    axes = (p1).buildAxes(xtitle='x', ytitle='y', ztitle='z', c='k')
    # Show the two clouds superposed on a new plotter window:
    show([(p1, axes), "Add a title here"],
         shape="10/1",  # 1 spaces above and 5 below
         sharecam=0, axes=0, zoom=2, interactive=True).close()


def main(args):
    # export data from imu sensor
    data = read_imu(args.path_to_carla + "imu/imu.txt", args.frames)
    data["gnss_coordinates"] = read_gnss(args.path_to_carla + "gnss/gnss.txt", args.frames)



    if eval(args.ignore_frames) is not None:
        data["frames"] = data["frames"][int(args.ignore_frames):]
        data["timestamps"] = data["timestamps"][int(args.ignore_frames):]
        data["locations"] = data["locations"][int(args.ignore_frames):]
        data["rotations"] = data["rotations"][int(args.ignore_frames):]
        data["accelerometer"] = data["accelerometer"][int(args.ignore_frames):]
        data["gyroscope"] = data["gyroscope"][int(args.ignore_frames):]
        data["gnss_coordinates"] = data["gnss_coordinates"][int(args.ignore_frames):]

    if eval(args.visualize) == False:
        if (os.path.exists(os.path.join(args.path_to_carla, args.date)) == False):
            os.mkdir(os.path.join(args.path_to_carla, args.date))

        if (os.path.exists(os.path.join(args.path_to_carla,
                                        args.date + "/" + args.date + "_drive_" + args.seq + "_sync")) == False):
            os.mkdir(os.path.join(args.path_to_carla, args.date + "/" + args.date + "_drive_" + args.seq + "_sync"))

        if (os.path.exists(os.path.join(args.path_to_carla,
                                        args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts")) == False):
            os.mkdir(os.path.join(args.path_to_carla,
                                  args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts"))

        if (os.path.exists(os.path.join(args.path_to_carla,
                                        args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts/data")) == False):
            os.mkdir(os.path.join(args.path_to_carla,
                                  args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts/data"))

        # store oxts values
        store_oxts(data, os.path.join(args.path_to_carla,
                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts/data/"))

        # store poses
        pose_from_oxts_packet(data, args)

        # copy rgb_camera
        if (eval(args.frames) is None) and (eval(args.ignore_frames) is None):
            copy_tree(os.path.join(args.path_to_carla, "rgb_camera-front/"), os.path.join(args.path_to_carla,
                                                                                    args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/"))
        else:
            if (os.path.exists(os.path.join(args.path_to_carla,
                                            args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/")) == False):
                os.mkdir(os.path.join(args.path_to_carla,
                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/"))

            if (os.path.exists(os.path.join(args.path_to_carla,
                                            args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/")) == False):
                os.mkdir(os.path.join(args.path_to_carla,
                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/"))

            if (eval(args.ignore_frames) is not None) and (eval(args.frames) is None):
                cmd = 'cd ' + os.path.join(args.path_to_carla, "rgb_camera-front/") + ' && cp $(ls | tail -n +' + str(
                    int(args.ignore_frames) + 1) + ') ' + os.path.join(args.path_to_carla,
                                                                       args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/")

            if (eval(args.ignore_frames) is None) and (eval(args.frames) is not None):
                cmd = 'cd ' + os.path.join(args.path_to_carla, "rgb_camera-front/") + ' && cp $(ls | head -n ' + str(
                    args.frames) + ') ' + os.path.join(args.path_to_carla,
                                                       args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/")

            if (eval(args.ignore_frames) is not None) and (eval(args.frames) is not None):
                cmd = 'cd ' + os.path.join(args.path_to_carla, "rgb_camera-front/") + ' && cp $(ls | tail -n +' + str(
                    int(args.ignore_frames) + 1) + ' | head -n ' + str(
                    int(args.frames) - int(args.ignore_frames)) + ') ' + os.path.join(args.path_to_carla,
                                                                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/data/")

            os.system(cmd)

        store_timestamp(data["timestamps"], os.path.join(args.path_to_carla,
                                                         args.date + "/" + args.date + "_drive_" + args.seq + "_sync/image_00/"))

        # copy velodyne_points
        if (eval(args.frames) is None) and (eval(args.ignore_frames) is None):
            copy_tree(os.path.join(args.path_to_carla, "sem_lidar_" + args.channels + "/"),
                      os.path.join(args.path_to_carla,
                                   args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/"))
        else:
            if (os.path.exists(os.path.join(args.path_to_carla,
                                            args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/")) == False):
                os.mkdir(os.path.join(args.path_to_carla,
                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/"))

            if (os.path.exists(os.path.join(args.path_to_carla,
                                            args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")) == False):
                os.mkdir(os.path.join(args.path_to_carla,
                                      args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/"))

            if (eval(args.ignore_frames) is not None) and (eval(args.frames) is None):
                if eval(args.semantic_lidar) == False:
                    cmd = 'cd ' + os.path.join(args.path_to_carla,
                                               "lidar_" + args.channels + "/") + ' && cp $(ls | tail -n +' + str(
                        int(args.ignore_frames) + 1) + ') ' + os.path.join(args.path_to_carla,
                                                                           args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")
                else:
                    cmd = 'cd ' + os.path.join(args.path_to_carla,
                                               "sem_lidar_" + args.channels + "/") + ' && cp $(ls | tail -n +' + str(
                        int(args.ignore_frames) + 1) + ') ' + os.path.join(args.path_to_carla,
                                                                           args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")

            if (eval(args.ignore_frames) is None) and (eval(args.frames) is not None):
                if eval(args.semantic_lidar) == False:
                    cmd = 'cd ' + os.path.join(args.path_to_carla,
                                               "lidar_" + args.channels + "/") + ' && cp $(ls | head -n ' + str(
                        args.frames) + ') ' + os.path.join(args.path_to_carla,
                                                           args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")
                else:
                    cmd = 'cd ' + os.path.join(args.path_to_carla,
                                               "sem_lidar_" + args.channels + "/") + ' && cp $(ls | head -n ' + str(
                        args.frames) + ') ' + os.path.join(args.path_to_carla,
                                                           args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")

            if (eval(args.ignore_frames) is not None) and (eval(args.frames) is not None):
                if eval(args.semantic_lidar) == False:
                    cmd =  (
                        'cd ' + 
                        os.path.join(args.path_to_carla, "lidar_" + str(args.channels) + "/") + 
                        ' && cp $(ls | tail -n +' + 
                        str( int(args.ignore_frames) + 1) 
                        + ' | head -n ' + str(
                        int(args.frames) - int(args.ignore_frames)) 
                    )
                    
                    
                    # + ') ' + os.path.join(args.path_to_carla,
                    #                                                                       args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")
                else:
                    cmd = 'cd ' + os.path.join(args.path_to_carla,
                                               "sem_lidar_" + args.channels + "/") + ' && cp $(ls | tail -n +' + str(
                        int(args.ignore_frames) + 1) + ' | head -n ' + str(
                        int(args.frames) - int(args.ignore_frames)) + ') ' + os.path.join(args.path_to_carla,
                                                                                          args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/data/")

            os.system(cmd)

        # store_timestamp(data["timestamps"], os.path.join(args.path_to_carla,
        #                                                  args.date + "/" + args.date + "_drive_" + args.seq + "_sync/velodyne_points/"))

        # copy calibration matrices
        # copy(os.path.join(args.path_to_carla, "calib_cam_to_cam.txt"), os.path.join(args.path_to_carla, args.date))
        # copy(os.path.join(args.path_to_carla, "calib_imu_to_velo.txt"), os.path.join(args.path_to_carla, args.date))
        # copy(os.path.join(args.path_to_carla, "calib_velo_to_cam.txt"), os.path.join(args.path_to_carla, args.date))
        # copy(os.path.join(args.path_to_carla, "calib_velo_to_world.txt"), os.path.join(args.path_to_carla, args.date))

        # store timestamps for all sensors
        # store_timestamp(data["timestamps"], os.path.join(args.path_to_carla,
        #                                                  args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/velodyne_points/"))
        # store_timestamp(data["timestamps"], os.path.join(args.path_to_carla,
        #                                                  args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/image_00/"))
        # store_timestamp(data["timestamps"], os.path.join(args.path_to_carla,
                                                        #  args.date + "/" + args.date + "_drive_" + args.seq + "_sync" + "/oxts/"))

        # visualize trajectory
        # visualize(data["locations"])
    else:
        # visualize trajectory
        visualize(data["locations"])


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert CARLA dataset to ROS bag file the easy way!")
    parser.add_argument("-p", "--path_to_carla", help="define the path to the carla dataset", required=True)
    parser.add_argument("-d", "--date", help="define the date of the carla dataset", required=True)
    parser.add_argument("-s", "--seq", help="define the sequence id", required=True)
    parser.add_argument("-f", "--frames", help="define the number of frames", default=None)
    parser.add_argument("-if", "--ignore_frames", help="define the number of first frames to ignore", default=None)
    parser.add_argument("-sem_lidar", "--semantic_lidar", help="defne whether you have semantic lidar or not",
                        default=True)
    parser.add_argument("-c", "--channels", help="define the number of lasers", default=64)
    parser.add_argument("-v", "--visualize", help="only visualize the data", default=False)

    # example
    # python3 convert_carla_to_kitti.py -p /media/andreas/Seagate/carla_dataset_cavsense/scenario_1/Town03_15_09_2021_14_12_09_to_keep/ego0/ -d 2021_09_16 -s 0003 -f 10

    args = parser.parse_args()

    # response = input('Have you removed the folder "' + args.date + '" from "' + args.path_to_carla + '"? (y/n) ')

    # print("response = ", response)

    # if (response.lower() != 'y' and response.lower() != 'n'):
    #     print("Wrong answer, try again!")
    #
    # if (response.lower() == 'y'):
    main(args)
