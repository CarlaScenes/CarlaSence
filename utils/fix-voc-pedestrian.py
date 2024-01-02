import os
import xml.etree.ElementTree as ET


def fix_bbox_values(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    for obj in root.findall('.//object'):
        name = obj.find('name').text
        if name == 'pedestrian':
            xmin = int(obj.find('bndbox/xmin').text)
            ymin = int(obj.find('bndbox/ymin').text)
            xmax = int(obj.find('bndbox/xmax').text)
            ymax = int(obj.find('bndbox/ymax').text)

            width = xmax - (xmin * 2)
            height = ymax - (ymin * 2)

            ymax_new = ymin + height
            xmax_new = xmin + width

            obj.find('bndbox/xmax').text = str(xmax_new)
            obj.find('bndbox/ymax').text = str(ymax_new)

    tree.write(xml_file)

# List of folders containing XML files
folder_list = [
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-1/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-2/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-3/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town-10-ClearNight-21-12-23/fixed-4/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-5/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-6/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-7/images/rgb_camera',
    '/home/arpit/manideep/carla/out/Town03-SoftRainSunset-23-12-23-f5-8/fixed-8/images/rgb_camera',
]

for folder in folder_list:
    print(folder)
    # Iterate through files in the folder
    for filename in os.listdir(folder):
        if filename.endswith('.xml'):
            filepath = os.path.join(folder, filename)

            # Fix bbox values
            fix_bbox_values(filepath)

            # print(f"Updated {filename}")
