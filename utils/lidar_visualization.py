import numpy as np
import open3d as o3d

# Load the PLY file
pcd = o3d.io.read_point_cloud('/home/apg/manideep/carla/out/lidar/5297.4809078808175.ply')
points = np.asarray(pcd.points)

# Extract the XYZ coordinates from the loaded point cloud
point_cloud = points[:, :3]

# Optional: Visualize the point cloud
o3d_pc = o3d.geometry.PointCloud()
o3d_pc.points = o3d.utility.Vector3dVector(point_cloud)
o3d.visualization.draw_geometries([o3d_pc])
