import os
import cv2
import re
import concurrent.futures
from tqdm import tqdm

def create_video_from_images(image_dir, output_video, frame_width, frame_height, frame_rate):
    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and re.match(r'^\d', f)]
    if "dvs" in output_video:
        image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and f[0].isalpha()]
    image_files.sort()
    image_files = image_files[:2400]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
    out = cv2.VideoWriter(output_video, fourcc, frame_rate, (frame_width, frame_height))
    for image_file in tqdm(image_files, desc="Processing images", unit="image"):
        image_path = os.path.join(image_dir, image_file)
        frame = cv2.imread(image_path)
        frame = cv2.resize(frame, (frame_width, frame_height))
        out.write(frame)
    out.release()
    print(f"Video '{output_video}' created successfully.")

folders = [
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-1/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-2/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-3/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-4/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-5/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-6/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-7/optical_flow_camera',
    '/home/apg/manideep/carla/out/Town10HD_Opt_30_11_2023_17_20_33/fixed-8/optical_flow_camera',
]

frame_width = 1280
frame_height = 960
frame_rate = 30  

def process_folder(folder):
    folder_name = folder.split('/')[-1]
    perception = folder.split('/')[-2]
    output_video = f'/home/apg/manideep/carla/out/{folder_name}-{perception}.mp4'
    print("Starting with ", perception, folder_name)
    create_video_from_images(folder, output_video, frame_width, frame_height, frame_rate)

with concurrent.futures.ThreadPoolExecutor() as executor:
    executor.map(process_folder, folders)