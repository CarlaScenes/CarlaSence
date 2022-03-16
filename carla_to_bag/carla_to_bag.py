import argparse
import os

parser = argparse.ArgumentParser(description = "Convert CARLA dataset to ROS bag file the easy way!")
parser.add_argument("-d", "--dir", help = "define the path to the carla dataset", required=True)
parser.add_argument("-t", "--date", help = "define the date of the carla dataset", required=True)
parser.add_argument("-r", "--drive", help = "define the sequence id", required=True)
parser.add_argument("-f", "--frames", help = "define the total number of frames (if the user wants to ignore some frames, then the number of frames (--frames) will include the ignored frames as well)", default=None)
parser.add_argument("-if", "--ignore_frames", help = "define the number of first frames to ignore", default=None)
parser.add_argument("-sem_lidar", "--semantic_lidar", help = "defne whether you have semantic lidar or not", default=True)
parser.add_argument("-c", "--channels", help = "define the number of lasers", default=64)
parser.add_argument("-crop", "--crop_image", help = "define whether to crop the images or not", default=False)
parser.add_argument("-v", "--visualize", help = "only visualize the data", default=False)

# example
# python carla_to_bag.py 
# -d path/Town03_15_09_2021_14_12_09_to_keep/ego0/ 
# -t 2021_09_15 
# -r 0003 
# -f 10 

args = parser.parse_args()

cmd_convert_carla_to_kitti = "python3 convert_carla_to_kitti.py -p " + str(args.dir) + " -d " + str(args.date) + " -s " + str(args.drive) + " -f " + str(args.frames) + " -if " + str(args.ignore_frames) + " -sem_lidar " + str(args.semantic_lidar) + " -c " + str(args.channels) + " -v " + str(args.visualize)

os.system(cmd_convert_carla_to_kitti)
#
# file_path = '/home/andreas/upload/carla_dataset/carla_to_bag/kitti2bag.py'


cmd_kitti2bag = "python kitti2bag.py -t " + str(args.date) + " -r " + str(args.drive)  + " -c " + str(args.channels) + " raw_synced " + str(args.dir) + " -crop " + str(args.crop_image)
print(cmd_kitti2bag)
os.system(cmd_kitti2bag)


