import time
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

# Load the PLY file
# pcd = o3d.io.read_point_cloud(
#     '/home/apg/manideep/carla/out/lidar/22.85948852263391.ply')
# points = np.asarray(pcd.points)

# # Extract the XYZ coordinates from the loaded point cloud
# point_cloud = points[:, :3]


# # Optional: Visualize the point cloud
# o3d_pc = o3d.geometry.PointCloud()
# o3d_pc.points = o3d.utility.Vector3dVector(point_cloud)
# o3d.visualization.draw_geometries( [o3d_pc],
#                                   zoom=0.15999999999999984,
#                                   front=[-0.90981715783548767, -
#                                          0.35379372132251835, 0.21693948939950897],
#                                   lookat=[10.0, 10.0, 10.0],
#                                   up=[0.09401892479298736, 0.33343013896937823, 0.93807504188504653])

# def rotate_view(vis):
#         print("rotate_view")
#         ctr = vis.get_view_control()
#         ctr.rotate(100.0, 10.0)
#         vis.poll_events()
#         vis.update_renderer()
#         # vis.capture_screen_image("/home/apg/manideep/carla/utils/filename23.png", True)
#         # vis.destroy_window()
#         return True


# o3d.visualization.draw_geometries_with_animation_callback([o3d_pc],
#                                                               rotate_view)

# def custom_draw_geometry_with_rotation(pcd):

#     def rotate_view(vis):
#         vis.create_window()
#         ctr = vis.get_view_control()
#         ctr.rotate(10.0, 0.0)
#         print("ok")
#         vis.update_geometry(pcd)
#         vis.poll_events()
#         vis.update_renderer()
#         return False


#     o3d.visualization.draw_geometries_with_animation_callback([pcd],
#                                                               rotate_view)
    
# custom_draw_geometry_with_rotation(o3d_pc)



# vis = o3d.visualization.Visualizer()
# vis.create_window(visible=True, width = 1920, height = 1080, left = 10000, top = 50000)
# vis.add_geometry(o3d_pc)

# # zoom = 0.15999999999999984
# # front = [-0.90981715783548767, -0.35379372132251835, 0.21693948939950897]
# # lookat = [10.0, 10.0, 10.0]
# # up = [0.09401892479298736, 0.33343013896937823, 0.93807504188504653]

# # vis.get_view_control().set_zoom(zoom)
# # vis.get_view_control().set_front(front)
# # vis.get_view_control().set_lookat(lookat)
# # vis.get_view_control().set_up(up)
# # vis.update_geometry()
# vis.poll_events()
# vis.update_renderer()

# vis.capture_screen_image("/home/apg/manideep/carla/utils/filename.png", True)
# vis.destroy_window()


# import open3d as o3d
import os

# # Directory containing the PLY files
ply_directory = '/home/apg/manideep/carla/out/lidar/'

# Output directory for PNG images
output_directory = '/home/apg/manideep/carla/out/lidar_png/'

# Create the output directory if it doesn't exist
os.makedirs(output_directory, exist_ok=True)

# List all PLY files in the directory
ply_files = [f for f in os.listdir(ply_directory) if f.endswith('.ply')]

# Loop through each PLY file
for ply_file in ply_files:
    # Load the PLY file
    pcd = o3d.io.read_point_cloud(os.path.join(ply_directory, ply_file))
    points = np.asarray(pcd.points)

    # Extract the XYZ coordinates from the loaded point cloud
    point_cloud = points[:, :3]

    # Create a point cloud object
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(point_cloud)

    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True, width=1920, height=1080)
    vis.add_geometry(o3d_pc)

    # Render and capture the image
    vis.poll_events()
    vis.update_renderer()

    # Save the captured image as a PNG with the same filename
    image_filename = os.path.splitext(ply_file)[0] + '.png'
    image_path = os.path.join(output_directory, image_filename)
    vis.capture_screen_image(image_path, True)

    # Close the visualizer
    vis.destroy_window()
    # break

print("Images saved to:", output_directory)
