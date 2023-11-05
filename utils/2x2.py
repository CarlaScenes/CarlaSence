from moviepy.editor import VideoFileClip, clips_array

# File paths to your four videos
# video_paths = [
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
# ]

# video_paths = [
#     "/home/apg/manideep/carla/out/clearnoon-rgb.mp4",
#     "/home/apg/manideep/carla/out/clearnoon-dvs.mp4",
#     "/home/apg/manideep/carla/out/clearnoon-optical.mp4",
#     "/home/apg/manideep/carla/out/clearsunset-rgb.mp4",
#     "/home/apg/manideep/carla/out/clearsunset-dvs.mp4",
#     "/home/apg/manideep/carla/out/clearsunset-optical.mp4",
#     "/home/apg/manideep/carla/out/clearnight-rgb.mp4",
#     "/home/apg/manideep/carla/out/clearnight-dvs.mp4",
#     "/home/apg/manideep/carla/out/clearnight-optical.mp4",
#     "/home/apg/manideep/carla/out/midrainynoon-rgb.mp4",
#     "/home/apg/manideep/carla/out/midrainynoon-dvs.mp4",
#     "/home/apg/manideep/carla/out/midrainynoon-optical.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-rgb.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-dvs.mp4",
#     "/home/apg/manideep/carla/out/foggynoon-optical.mp4",
# ]

video_paths = [
   "/home/apg/manideep/carla/out/rgb-front-left.mp4",
    "/home/apg/manideep/carla/out/rgb-front.mp4",
    "/home/apg/manideep/carla/out/rgb-front-right.mp4",
    "/home/apg/manideep/carla/out/rgb-back-left.mp4",
    "/home/apg/manideep/carla/out/rgb-back.mp4",
    "/home/apg/manideep/carla/out/rgb-back-right.mp4"
]

sensor = "rgb-full-sim"


# Load video clips
video_clips = [VideoFileClip(path) for path in video_paths]

# # Define the text to be added to specific videos
# text = "Weather on top left"

# # Create TextClips for the text elements
# text_clips = [TextClip(text, fontsize=24, color='white') for _ in video_clips]

# Iterate through the videos and add the text to the specified videos
# for i in [0, 3, 6, 9, 12]:
#     video_clips[i] = video_clips[i].set_duration(
#         video_clips[i].duration).set_position(('left', 'top'))
#     video_clips[i] = clips_array([[text_clips[i], video_clips[i]]])

# Stack the clips in a 5x3 grid
final_clip = clips_array([

    [video_clips[0], video_clips[1], video_clips[2]],
    [video_clips[3], video_clips[4], video_clips[5]],

])

# Write the merged video to an output file
final_clip.write_videofile(f"{sensor}.mp4", codec='libx264')

# Close video clips
for clip in video_clips:
    clip.reader.close()

print("Merged video saved as merged_video.mp4")
