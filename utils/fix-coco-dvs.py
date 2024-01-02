import os
import json


def convert_bbox_format(annotation):
    for ann in annotation['annotations']:
        bbox = ann['bbox']
        xmin, ymin, xmax, ymax = bbox
        width = xmax - xmin
        height = ymax - ymin
        ann['bbox'] = [xmin, ymin, width, height]


# List of folders containing JSON files
folder_list = [
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-1/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-2/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-3/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-4/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-5/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-6/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-7/images/dvs_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-8/images/dvs_camera',
]

for folder in folder_list:
    print(folder)
    for filename in os.listdir(folder):
        if filename.endswith('.json'):
            filepath = os.path.join(folder, filename)

            # Read JSON file
            with open(filepath, 'r') as file:
                data = json.load(file)

            # Convert bbox format
            convert_bbox_format(data)

            # Write updated JSON back to the file
            with open(filepath, 'w') as file:
                json.dump(data, file, indent=4)

            # print(f"Updated {filename}")
