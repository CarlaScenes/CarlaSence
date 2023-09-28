import os
import cv2

# Directory containing the images
image_dir = '/home/apg/manideep/carla/out/rgb'

# Output video filename
output_video = 'rgb.mp4'

# Parameters for video frame size and frame rate
frame_width = 640
frame_height = 480
frame_rate = 1  # Frames per second

# Get a list of image files in the directory
image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
image_files.sort()  # Sort the files for correct sequence

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
