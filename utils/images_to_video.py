import os
import cv2
import re
import concurrent.futures

def create_video_from_images(image_dir, output_video, frame_width, frame_height, frame_rate):
    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and re.match(r'^\d', f)]
    if "dvs" in output_video:
        image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and f[0].isalpha()]
    image_files.sort()
    image_files = image_files[:4800]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
    out = cv2.VideoWriter(output_video, fourcc, frame_rate, (frame_width, frame_height))
    for image_file in image_files:
        image_path = os.path.join(image_dir, image_file)
        frame = cv2.imread(image_path)
        frame = cv2.resize(frame, (frame_width, frame_height))
        out.write(frame)
    out.release()
    print(f"Video '{output_video}' created successfully.")

folders = [
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-1/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-2/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-3/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-4/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-5/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-6/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-7/depth_camera',
    # '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/fixed-8/depth_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/ego0/depth_camera-front',
    '/home/apg/manideep/carla/out/Town10HD_Opt_01_12_2023_17_33_14/ego1/depth_camera-back',
]

frame_width = 1280
frame_height = 960
frame_rate = 20  

# for folder in folders:
#     folder_name = folder.split('/')[-1]
#     perception = folder.split('/')[-2]
#     output_video = f'/home/apg/manideep/carla/out/{folder_name}-{perception}.mp4'
#     print("Starting with ", folder_name)
#     create_video_from_images(folder, output_video, frame_width, frame_height, frame_rate)

def process_folder(folder):
    folder_name = folder.split('/')[-1]
    perception = folder.split('/')[-2]
    output_video = f'/home/apg/manideep/carla/out/{folder_name}-{perception}.mp4'
    print("Starting with ", perception, folder_name)
    create_video_from_images(folder, output_video, frame_width, frame_height, frame_rate)

with concurrent.futures.ThreadPoolExecutor() as executor:
    executor.map(process_folder, folders)