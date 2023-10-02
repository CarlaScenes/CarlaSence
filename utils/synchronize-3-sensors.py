import os
import cv2
import numpy as np

# Define the paths to the DVS, RGB, and Lidar data directories
dvs_data_dir = '../out/dvs'
rgb_data_dir = '../out/rgb'
lidar_data_dir = '../out/lidar_png'  # Added Lidar data directory
output_video = 'synchronized_video.mp4'

# Other parameters (frame size and frame rate)
frame_width = 1920
frame_height = 1080
frame_rate = 10

# Initialize OpenCV VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video, fourcc, frame_rate, (frame_width, frame_height))

def find_closest_file(files, target_file):
    closest_file = None
    min_timestamp_diff = float('inf')

    target_timestamp = float(os.path.splitext(target_file)[0])

    for file in files:
        file_timestamp = float(os.path.splitext(file)[0])
        timestamp_diff = abs(target_timestamp - file_timestamp)

        if timestamp_diff < min_timestamp_diff:
            min_timestamp_diff = timestamp_diff
            closest_file = file

    return closest_file

# Get lists of timestamped files from all three directories
dvs_files = sorted(os.listdir(dvs_data_dir))
rgb_files = sorted(os.listdir(rgb_data_dir))
lidar_files = sorted(os.listdir(lidar_data_dir))

# Synchronize and combine DVS, RGB, and Lidar data into the video
for dvs_file in dvs_files:
    dvs_path = os.path.join(dvs_data_dir, dvs_file)
    rgb_file = find_closest_file(rgb_files, dvs_file)
    lidar_file = find_closest_file(lidar_files, dvs_file)

    # If corresponding RGB and Lidar files are found
    if rgb_file is not None and lidar_file is not None:
        rgb_path = os.path.join(rgb_data_dir, rgb_file)
        lidar_path = os.path.join(lidar_data_dir, lidar_file)

        # Load DVS, RGB, and Lidar images
        dvs_image = cv2.imread(dvs_path)
        rgb_image = cv2.imread(rgb_path)
        lidar_image = cv2.imread(lidar_path)

        # Check if all images were loaded successfully
        if dvs_image is not None and rgb_image is not None and lidar_image is not None:
            # Resize DVS, RGB, and Lidar images to match the frame size
            dvs_image_resized = cv2.resize(dvs_image, (frame_width // 4, frame_height))
            rgb_image_resized = cv2.resize(rgb_image, (frame_width // 4, frame_height))
            lidar_image_resized = cv2.resize(lidar_image, (frame_width // 2, frame_height))

            # Combine DVS, RGB, and Lidar images side by side
            synchronized_frame = np.hstack([dvs_image_resized, rgb_image_resized, lidar_image_resized])

            # Write the synchronized frame to the video
            out.write(synchronized_frame)

# Release the video writer
out.release()

print(f"Video '{output_video}' created successfully.")
