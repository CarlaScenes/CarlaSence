import os
import shutil

source_folders = [
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-1/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-2/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-3/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-4/images/rgb_camera',

    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-5/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-6/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-7/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-8/images/rgb_camera',
]

for source_folder in source_folders:
    print(source_folder)
    target_folder = source_folder.replace("rgb", "dvs")
    os.makedirs(target_folder, exist_ok=True)
    for root, dirs, files in os.walk(source_folder):
        for file in files:
            if file.startswith('d'):
                file_path = os.path.join(root, file)
                new_file_path = os.path.join(target_folder, f'{file.replace("dvs-", "")}')
                shutil.move(file_path, new_file_path)

print("Files moved successfully.")
