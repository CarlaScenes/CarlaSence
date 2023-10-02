# import os
# import cv2

# # Define the paths to the DVS and RGB data directories
# dvs_data_dir = '../out/dvs'  # Updated path
# rgb_data_dir = '../out/rgb'  # Updated path
# output_video = 'synchronized_video.mp4'  # Updated output video format

# # Parameters for video frame size and frame rate
# frame_width = 1280  # Double the width for side-by-side frames
# frame_height = 480
# frame_rate = 1  # Frames per second

# # Get a list of timestamped files from both directories
# dvs_files = sorted(os.listdir(dvs_data_dir))
# rgb_files = sorted(os.listdir(rgb_data_dir))

# # Initialize OpenCV VideoWriter
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
# out = cv2.VideoWriter(output_video, fourcc, frame_rate,
#                       (frame_width, frame_height))

# # Synchronize and combine DVS and RGB data into the video
# for dvs_file, rgb_file in zip(dvs_files, rgb_files):
#     dvs_path = os.path.join(dvs_data_dir, dvs_file)
#     rgb_path = os.path.join(rgb_data_dir, rgb_file)

#     # Load DVS and RGB images
#     dvs_image = cv2.imread(dvs_path)
#     rgb_image = cv2.imread(rgb_path)

#     # Check if both images were loaded successfully
#     if dvs_image is not None and rgb_image is not None:
#         # Resize DVS and RGB images to half of the frame width
#         dvs_image_resized = cv2.resize(
#             dvs_image, (frame_width // 2, frame_height))
#         rgb_image_resized = cv2.resize(
#             rgb_image, (frame_width // 2, frame_height))

#         # Combine DVS and RGB images side by side
#         synchronized_frame = cv2.hconcat(
#             [dvs_image_resized, rgb_image_resized])

#         # Write the synchronized frame to the video
#         out.write(synchronized_frame)

# # Release the video writer
# out.release()

# print(f"Video '{output_video}' created successfully.")


import os
import cv2

# Define the paths to the DVS and RGB data directories
dvs_data_dir = '../out/dvs'
rgb_data_dir = '../out/rgb'
output_video = 'synchronized_video.mp4'  # Updated output video format

# Parameters for video frame size and frame rate
frame_width = 1280  # Double the width for side-by-side frames
frame_height = 480
frame_rate = 10  # Frames per second

# Get a list of timestamped files from both directories
dvs_files = sorted(os.listdir(dvs_data_dir))
rgb_files = sorted(os.listdir(rgb_data_dir))

# Initialize OpenCV VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
out = cv2.VideoWriter(output_video, fourcc, frame_rate,
                      (frame_width, frame_height))


def find_closest_rgb_file(rgb_files, dvs_file):
    closest_rgb_file = None
    min_timestamp_diff = float('inf')  # Initialize with a large value

    # Extract the timestamp from the DVS file (excluding the file extension)
    dvs_timestamp = float(os.path.splitext(dvs_file)[0])

    for rgb_file in rgb_files:
        # Extract the timestamp from the RGB file (excluding the file extension)
        rgb_timestamp = float(os.path.splitext(rgb_file)[0])

        timestamp_diff = abs(dvs_timestamp - rgb_timestamp)

        # Check if the current RGB file has a smaller timestamp difference
        if timestamp_diff < min_timestamp_diff:
            min_timestamp_diff = timestamp_diff
            closest_rgb_file = rgb_file

    return closest_rgb_file


# Synchronize and combine DVS and RGB data into the video
for dvs_file in dvs_files:
    dvs_path = os.path.join(dvs_data_dir, dvs_file)
    rgb_file = find_closest_rgb_file(rgb_files, dvs_file)

    # If a corresponding RGB file is found
    if rgb_file is not None:
        rgb_path = os.path.join(rgb_data_dir, rgb_file)

        # Load DVS and RGB images
        dvs_image = cv2.imread(dvs_path)
        rgb_image = cv2.imread(rgb_path)

        # Check if both images were loaded successfully
        if dvs_image is not None and rgb_image is not None:
            # Resize DVS and RGB images to half of the frame width
            dvs_image_resized = cv2.resize(
                dvs_image, (frame_width // 2, frame_height))
            rgb_image_resized = cv2.resize(
                rgb_image, (frame_width // 2, frame_height))

            # Combine DVS and RGB images side by side
            synchronized_frame = cv2.hconcat(
                [dvs_image_resized, rgb_image_resized])

            # Write the synchronized frame to the video
            out.write(synchronized_frame)

# Release the video writer
out.release()

print(f"Video '{output_video}' created successfully.")

# Helper function to find a corresponding RGB file for a given DVS file based on timestamps
