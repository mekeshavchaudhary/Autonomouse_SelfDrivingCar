# import pyrealsense2 as rs
# import numpy as np
# import matplotlib.pyplot as plt
# import open3d

# # Setup:
# file_path = "20231026_154034.bag"
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_device_from_file(file_path, repeat_playback=False)

# # Start streaming from file
# pipeline.start(config)

# # Get the frames
# frames = pipeline.wait_for_frames()
# depth_frame = frames.get_depth_frame()
# color_frame = frames.get_color_frame()

# # Get intrinsics and extrinsics
# depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
# color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
# depth_to_color_extrinsics = depth_frame.profile.get_extrinsics_to(color_frame.profile)

# # Get depth and color data
# depth_image = np.asanyarray(depth_frame.get_data())
# color_image = np.asanyarray(color_frame.get_data())

# # Create point cloud and map color
# points = []
# colors = []

# for v in range(depth_image.shape[0]):
#     for u in range(depth_image.shape[1]):
#         depth = depth_image[v, u]
#         if depth:  # if depth is not 0
#             # Map depth pixel to 3D space
#             x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth)
            
#             # Map 3D point to color pixel
#             color_pixel = rs.rs2_transform_point_to_point(depth_to_color_extrinsics, [x, y, z])
#             color_pixel = rs.rs2_project_point_to_pixel(color_intrinsics, color_pixel)
#             color_pixel = [int(cp) for cp in color_pixel]

#             # Check if the projection is valid
#             if 0 <= color_pixel[0] < color_image.shape[1] and 0 <= color_pixel[1] < color_image.shape[0]:
#                 points.append([x, y, z])
#                 colors.append(color_image[color_pixel[1], color_pixel[0], :])

# # Convert lists to numpy arrays
# points = np.array(points)
# colors = np.array(colors)

# # # Visualize using matplotlib
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# # ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors/255, s=0.5)
# # plt.show()

# # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)
# v= o3d.io.write_point_cloud("sync.ply", pcd)
# vd = o3d.io.read_point_cloud("sync.ply")
# o3d.visualization.draw_geometries([vd],zoom=0.3412, front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])


# # Stop streaming
# pipeline.stop()


import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# Setup:
file_path = "20231026_154034.bag"
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(file_path, repeat_playback=False)

# Start streaming from file
pipeline.start(config)

# Get the frames
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Get intrinsics and extrinsics
depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrinsics = depth_frame.profile.get_extrinsics_to(color_frame.profile)

# Get depth and color data
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

# Create point cloud and map color
points = []
colors = []

for v in range(depth_image.shape[0]):
    for u in range(depth_image.shape[1]):
        depth = depth_image[v, u]
        if depth:  # if depth is not 0
            # Map depth pixel to 3D space
            x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth)
            
            # Map 3D point to color pixel
            color_pixel = rs.rs2_transform_point_to_point(depth_to_color_extrinsics, [x, y, z])
            color_pixel = rs.rs2_project_point_to_pixel(color_intrinsics, color_pixel)
            color_pixel = [int(cp) for cp in color_pixel]

            # Check if the projection is valid
            if 0 <= color_pixel[0] < color_image.shape[1] and 0 <= color_pixel[1] < color_image.shape[0]:
                points.append([x, y, z])
                colors.append(color_image[color_pixel[1], color_pixel[0], :])

# Convert lists to numpy arrays
points = np.array(points)
colors = np.array(colors)

# Create an Open3D PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Visualize using Open3D
o3d.visualization.draw_geometries([pcd])

# Stop streaming
pipeline.stop()
