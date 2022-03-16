# To generate data from carla
1. Run python multi_data_generator.py

# To convert carla data to bag files

1. Install conda

2. conda env create --name $conda name$ -f carla_to_bag/env.yml

3. Replace line 258 from /home/andreas/.local/lib/python2.7/site-packages/pykitti.raw.py:
	- t = dt.datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f') -> t = line

4. Type python carla_to_bag/carla_to_bag.py -h for the documentation
	- Example : python carla_to_bag/carla_to_bag.py -d path_to_ego/../Town03_15_09_2021_14_12_09_to_keep/ego0/ -t 2021_09_15 -r 0003 -f 10

5. To align poses between vloam and ground truth from carla data type:
	- python carla_to_bag/align_poses.py -h for the documentation

6. To evaluate results using evo, move LO1.txt, VO1.txt and MO1.txt and aligned_poses from vloam ros results to evo folder and type:
	- evo_traj kitti MO1.txt VO1.txt LO1.txt  --ref=aligned_poses.txt -p --plot_mode=xyz -as