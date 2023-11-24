# import os
# import cv2

# # Directory containing the images
# image_dir = '/home/apg/manideep/carla/out/Town10HD_Opt_08_11_2023_11_41_47/ego0/rgb_camera-front'

# # Output video filename
# output_video = '/home/apg/manideep/carla/out/bike-rgb.mp4'

# # Parameters for video frame size and frame rate
# frame_width = 1280
# frame_height = 960
# frame_rate = 20  # Frames per second

# # Get a list of image files in the directory
# image_files = [f for f in os.listdir(
#     image_dir) if f.endswith(('.jpg', '.png'))]
# image_files.sort()  # Sort the files for correct sequence
# image_files = image_files[:3000]

# # Initialize OpenCV VideoWriter
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
# out = cv2.VideoWriter(output_video, fourcc, frame_rate,
#                       (frame_width, frame_height))

# # Loop through each image and create frames for the video
# for image_file in image_files:
#     image_path = os.path.join(image_dir, image_file)
#     frame = cv2.imread(image_path)
#     frame = cv2.resize(frame, (frame_width, frame_height))

#     # Write the frame to the video
#     out.write(frame)

# # Release the video writer
# out.release()

# print(f"Video '{output_video}' created successfully.")


import os
import cv2
import re

# Function to create video from images in a directory
def create_video_from_images(image_dir, output_video, frame_width, frame_height, frame_rate):
    # Get a list of image files in the directory
    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and re.match(r'^\d', f)]
    if "dvs" in output_video:
        image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png')) and f[0].isalpha()]
    image_files.sort()  # Sort the files for correct sequence
    image_files = image_files[20:]

    # Initialize OpenCV VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4
    out = cv2.VideoWriter(output_video, fourcc, frame_rate, (frame_width, frame_height))

    # Loop through each image and create frames for the video
    for image_file in image_files:
        image_path = os.path.join(image_dir, image_file)
        frame = cv2.imread(image_path)
        frame = cv2.resize(frame, (frame_width, frame_height))
        # Write the frame to the video
        out.write(frame)

    # Release the video writer
    out.release()
    print(f"Video '{output_video}' created successfully.")

# List of folders containing images
folders = [

    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow_camera-back',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow_camera-back-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow_camera-back-right',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow-front',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow_camera-front-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/optical_flow_camera-front-right',

    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-back',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-back-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-back-right',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-front',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-front-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/depth_camera-front-right',

    #    '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-back',
    #    '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-back-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-back-right',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-front',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-front-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/instance_segmentation_camera-front-right',

    #         '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-back',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-back-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-back-right',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-front',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-front-left',
    #        '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/semantic_segmentation_camera-front-right',
           
    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/rgb_camera-back-left',
    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/rgb_camera-front-left',
    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/rgb_camera-back',
    '/home/apg/manideep/carla/out/Town10HD_Opt_22_11_2023_19_28_30/ego0/rgb_camera-front',
    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/rgb_camera-back-right',
    # '/home/apg/manideep/out/Town10HD_Opt_19_11_2023_16_27_25/ego0/rgb_camera-front-right',


           ]

# Parameters for video frame size and frame rate
frame_width = 1280
frame_height = 960
frame_rate = 20  # Frames per second

# Process images in each folder and create videos
for folder in folders:
    # Output video filename based on the folder name
    folder_name = folder.split('/')[-1]
    output_video = f'/home/apg/manideep/carla/out/{folder_name}.mp4'
    print("Starting with ", folder_name)
    create_video_from_images(folder, output_video, frame_width, frame_height, frame_rate)
