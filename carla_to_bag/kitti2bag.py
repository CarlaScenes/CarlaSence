#!env python
# -*- coding: utf-8 -*-

import sys

import pykitti

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)
import sys

sys.path.append("/opt/ros/noetic/lib/python3/dist-packages/")

import tf
import os
import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse
import pykitti.utils as utils
from scipy.spatial.transform import Rotation as R


def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    # tmp=0
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        # print ("oxts = ", oxts)
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        # imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.header.stamp = rospy.Time.from_sec(float(timestamp))

        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = 1
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)


def save_dynamic_tf(bag, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations")
    if kitti_type.find("raw") != -1:
        # tmp=0
        # print ("oxts = ", kitti.oxts)
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            # tf_oxts_transform.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
            tf_oxts_transform.header.stamp = rospy.Time.from_sec(float(timestamp))

            tf_oxts_transform.header.frame_id = 'map'
            tf_oxts_transform.child_frame_id = 'base_link'

            transform = (oxts.T_w_imu)

            t = transform[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(transform)
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write('/tf', tf_oxts_msg, tf_oxts_msg.transforms[0].header.stamp)

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = 'map'
            tf_stamped.child_frame_id = 'camera_left'

            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)

def save_camera_data(bag, kitti_type, kitti, util, bridge, crop, camera, camera_frame_id, topic, initial_time):
    print("Exporting camera {}".format(camera))
    if kitti_type.find("raw") != -1:
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        image_filenames = sorted(os.listdir(image_path))
        # with open(os.path.join(image_dir, 'timestamps.txt')) as f:
        #     image_datetimes = map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines())

        # timestamp_path = os.path.join(image_dir
        # image_path = os.path.join(timestamp_path, '')
        # image_filenames = sorted(os.listdir(timestamp_path))
        image_datetimes = []
        with open(os.path.join(image_dir, 'timestamps.txt')) as f:
            lines = f.readlines()
            for line in lines:
                if len(line) == 1:
                    continue
                dt = line.strip()
                image_datetimes.append(dt)

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.width, calib.height = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
        calib.distortion_model = 'plumb_bob'
        calib.K = util['K_{}'.format(camera_pad)]
        calib.R = util['R_rect_{}'.format(camera_pad)]
        calib.D = util['D_{}'.format(camera_pad)]
        calib.P = util['P_rect_{}'.format(camera_pad)]

    elif kitti_type.find("odom") != -1:
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(kitti.sequence_path, 'image_{}'.format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.P = util['P{}'.format(camera_pad)]

    iterable = zip(image_datetimes, image_filenames)
    # bar = progressbar.ProgressBar()
    for dt, filename in iterable:
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)

        # crop image
        if eval(crop) == True:
            height, width = cv_image.shape[:2]
            cv_image_cropped = cv_image[height - 640:, 0:width]
            cv_image = cv_image_cropped
            cv2.imshow("test.png", cv_image)
            cv2.waitKey(1)

        calib.height, calib.width = cv_image.shape[:2]
        # if camera in (0, 1):
        #     cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # encoding = "mono8" if camera in (0, 1) else "bgr8"
        encoding = "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id
        if kitti_type.find("raw") != -1:
            image_message.header.stamp = rospy.Time.from_sec(float(dt))
            # image_message.header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))
            topic_ext = "/image_raw"
        elif kitti_type.find("odom") != -1:
            image_message.header.stamp = rospy.Time.from_sec(dt)
            topic_ext = "/image_rect"
        calib.header.stamp = image_message.header.stamp
        bag.write(topic + topic_ext, image_message, t=image_message.header.stamp)
        bag.write(topic + '/camera_info', calib, t=calib.header.stamp)

def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            # dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            dt = line.strip()
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)
    # bar = progressbar.ProgressBar()
    for dt, filename in iterable:
        if dt is None:
            continue
        from plyfile import PlyData, PlyElement
        # print ("read velodyne from = ", os.path.join(velo_data_dir, filename))
        plydata = PlyData.read(os.path.join(velo_data_dir, filename))

        counter = 0
        all_points = []
        for p in plydata['vertex']['x']:
            all_points.append(plydata['vertex']['x'][counter])
            all_points.append(plydata['vertex']['y'][counter])
            all_points.append(plydata['vertex']['z'][counter])
            all_points.append(plydata['vertex']['I'][counter])  # hereee
            # all_points.append(plydata['vertex']['z'][counter]*(-1)) # hereee
            counter = counter + 1

        scan = np.array(all_points)

        scan = scan.reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        # header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))
        header.stamp = rospy.Time.from_sec(float(dt))
        # print ("header.stamp = ", header.stamp)

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)

def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg

def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transformsl(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    # tmp = 0
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])

        tfm.transforms.append(t)
    # tmp=0
    for timestamp in timestamps:
        # time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        # print ("timestamp = ", timestamp)
        time = rospy.Time.from_sec(float(timestamp))

        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    # tmp = 0
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])

        tfm.transforms.append(t)
    # tmp=0
    for timestamp in timestamps:
        # time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        # print ("timestamp = ", timestamp)
        time = rospy.Time.from_sec(float(timestamp))

        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    # tmp=0
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        # if (tmp >= frames_length):
        # break
        # tmp = tmp + 1        
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        # navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    # tmp=0
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        # twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp))

        # t = rospy.Time.from_sec(time.time())
        # seconds = twist_msg.header.stamp.to_sec()  # floating point

        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


def load_calib_rigid(args, filename):
    """Read a rigid transform calibration file as a numpy.array."""
    filepath = os.path.join(args.dir, args.date)
    filepath = os.path.join(filepath, filename)
    # print ("filepath = ", filepath)
    data = utils.read_calib_file(filepath)
    return utils.transform_from_rot_trans(data['R'], data['T'])


def main():
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS bag file the easy way!")
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))

    parser.add_argument("kitti_type", choices=kitti_types, help="KITTI dataset type")
    parser.add_argument("dir", nargs="?", default=os.getcwd(),
                        help="base directory of the dataset, if no directory passed the deafult is current working directory")
    # parser.add_argument("dir", nargs = "?", default = "/media/andreas/Seagate/carla_dataset_cavsense/scenario_1/Town03_15_09_2021_14_12_09_to_keep/ego0", help = "base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date",
                        help="date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive",
                        help="drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-c", "--channels", help="define the number of channels")
    parser.add_argument("-s", "--sequence", choices=odometry_sequences,
                        help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    parser.add_argument("-crop", "--crop_image", help="define whether to crop the images or not", default=False)

    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    # compression = rosbag.Compression.BZ2
    # compression = rosbag.Compression.LZ4

    # CAMERAS
    cameras = [
        (0, 'camera_left', '/kitti/camera_left')
    ]

    if args.kitti_type.find("raw") != -1:

        if args.date == None:
            print("Date option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        elif args.drive == None:
            print("Drive option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)

        # create bag directory
        path_to_bag = args.dir.replace("ego", "bag")
        if (os.path.exists(os.path.join(path_to_bag)) == False):
            os.mkdir(path_to_bag)

        bag = rosbag.Bag(
            path_to_bag + "/kitti_{}_drive_{}_{}_channels_{}.bag".format(args.date, args.drive, args.kitti_type[4:],
                                                                         str(args.channels)), 'w',
            compression=compression)
        # bag = rosbag.Bag("kitti_{}_drive_{}_{}.bag".format(args.date, args.drive, args.kitti_type[4:]), 'w', compression=compression)
        kitti = pykitti.raw(args.dir, args.date, args.drive)
        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        try:
            # IMU
            imu_frame_id = 'imu_link'
            imu_topic = '/kitti/oxts/imu'
            gps_fix_topic = '/kitti/oxts/gps/fix'
            gps_vel_topic = '/kitti/oxts/gps/vel'
            velo_frame_id = 'velo_link'
            velo_topic = '/kitti/velo'


            # T_base_link_to_imu = np.array([[0.0, 1.0, 0.0, 0.0],
            #                                [0.0, 0.0, -1.0, 0.0],
            #                                [1.0, 0.0, 0.0, 0.0],
            #                                [0.0, 0.0, 0.0, 1.0]])
            T_base_link_to_imu = np.array([[1.0, 0.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0, 0.0],
                                            [0.0, 0.0, 1.0, 0.0],
                                            [0.0, 0.0, 0.0, 1.0]])

            # from https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/
            # 1. base_link has its origin directly at the pivot point or center of the robot. 
            # This coordinate frame moves as the robot moves
            # 2. Base Link to IMU base_link -> imu transform gives us the position and orientation of the IMU 
            # inside the base_link’s coordinate frame. This transform is static.
            # T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93] # kitti measurements (check sensor setup in the paper)

            T_base_link_to_imu[0:3, 3] = [0.0, 0.0, 0.0]  # carla measurements

            # Load the rigid transformation from imu to velodyne
            T_velo_imu = load_calib_rigid(args, os.path.join(kitti.calib_path, 'calib_imu_to_velo.txt'))

            # Load the rigid transformation from velodyne to world
            T_world_velo = load_calib_rigid(args, os.path.join(kitti.calib_path, 'calib_velo_to_world.txt'))

            # Load the rigid transformation from velodyne to camera
            T_cam_velo = load_calib_rigid(args, os.path.join(kitti.calib_path, 'calib_velo_to_cam.txt'))

            # Load the rigid transformation from imu to camera
            T_cam0_imu = T_velo_imu.dot(T_cam_velo)



            # T_world_velo = Rot * T_world_velo
            transforms = [
                ('base_link', imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(T_cam0_imu)),
                ('velo', 'map', T_world_velo)
            ]

            util = pykitti.utils.read_calib_file(os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

            # Export
            save_static_transforms(bag, transforms, kitti.timestamps)
            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=None)
            save_imu_data(bag, kitti, imu_frame_id, imu_topic)
            save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
            save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)
            for camera in cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, args.crop_image, camera=camera[0],
                                 camera_frame_id=camera[1], topic=camera[2], initial_time=None)
            save_velo_data(bag, kitti, velo_frame_id, velo_topic)



        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()

    elif args.kitti_type.find("odom") != -1:

        if args.sequence == None:
            print("Sequence option is not given. It is mandatory for odometry dataset.")
            print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
            sys.exit(1)

        bag = rosbag.Bag("kitti_data_odometry_{}_sequence_{}.bag".format(args.kitti_type[5:], args.sequence), 'w',
                         compression=compression)

        kitti = pykitti.odometry(args.dir, args.sequence)
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
            sys.exit(1)

        kitti.load_calib()
        kitti.load_timestamps()

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        if args.sequence in odometry_sequences[:11]:
            print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
            kitti.load_poses()

        try:
            util = pykitti.utils.read_calib_file(os.path.join(args.dir, 'sequences', args.sequence, 'calib.txt'))
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            # Export
            if args.kitti_type.find("gray") != -1:
                used_cameras = cameras[:2]
            elif args.kitti_type.find("color") != -1:
                used_cameras = cameras[-2:]
            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=current_epoch)
            for camera in used_cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, args.crop_image, camera=camera[0],
                                 camera_frame_id=camera[1], topic=camera[2], initial_time=current_epoch)

        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()


if __name__ == '__main__':
    main()

# python2 -m kitti2bag -t 2021_09_15 -r 0003 raw_synced /media/andreas/Seagate/carla_dataset_cavsense/scenario_1/Town03_15_09_2021_14_12_09_to_keep/ego0

# rostopic pub /load_small_dataset_action_server/goal vloam_main/vloam_mainActionGoal "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# goal_id:
#   stamp:
#     secs: 0
#     nsecs: 0
#   id: ''
# goal:
#   date: '2021_10_01'
#   seq: '02' 
#   start_frame: 0
#   end_frame: 2400"
