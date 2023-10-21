from moviepy.editor import VideoFileClip, clips_array, TextClip, CompositeVideoClip

# List of video file paths
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
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
    "/home/apg/manideep/carla/out/foggynoon-rgb-test.mp4",
]

# Load video clips
video_clips = [VideoFileClip(path) for path in video_paths]

# Define the metadata texts for each row
metadata_texts = [
    "Weather: Clear Noon",
    "Weather: Clear Sunset",
    "Weather: Clear Night",
    "Weather: Mid Rainy Noon",
    "Weather: Foggy Noon",
]

# Create TextClips for the metadata texts
text_clips = [TextClip(text, fontsize=24, color='white')
              for text in metadata_texts]

# Create a CompositeVideoClip to overlay the text on the first video in each row
row_1 = CompositeVideoClip([text_clips[0].set_position(('left', 'top')).set_duration(
    video_clips[0].duration), video_clips[0], video_clips[1], video_clips[2]])
row_2 = CompositeVideoClip([video_clips[3], video_clips[4], video_clips[5]])
row_3 = CompositeVideoClip([video_clips[6], video_clips[7], video_clips[8]])
row_4 = CompositeVideoClip([video_clips[9], video_clips[10], video_clips[11]])
row_5 = CompositeVideoClip([video_clips[12], video_clips[13], video_clips[14]])

# Concatenate the rows vertically to create the final merged video
final_video = clips_array([[row_1], [row_2], [row_3], [row_4], [row_5]])

# Write the merged video with metadata to an output file
final_video.write_videofile("merged_video_with_metadata.mp4", codec='libx264')
