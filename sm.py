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

# Create filters
spatial_filter = rs.spatial_filter()
temporal_filter = rs.temporal_filter()
hole_filling_filter = rs.hole_filling_filter()

# Get the frames
frames = pipeline.wait_for_frames()

# Apply filters
filtered_depth = spatial_filter.process(frames.get_depth_frame())
filtered_depth = temporal_filter.process(filtered_depth)
filtered_depth = hole_filling_filter.process(filtered_depth)

color_frame = frames.get_color_frame()

# Get intrinsics and extrinsics
depth_intrinsics = filtered_depth.profile.as_video_stream_profile().intrinsics
color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrinsics = filtered_depth.profile.get_extrinsics_to(color_frame.profile)

# Get depth and color data
depth_image = np.asanyarray(filtered_depth.get_data())
color_image = np.asanyarray(color_frame.get_data())

# Create point cloud and map color
points = []
colors = []

step = 1  # reduce this for more points, e.g., 0.5 or 0.25
for v in range(0, depth_image.shape[0], int(step)):
    for u in range(0, depth_image.shape[1], int(step)):
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

# Create open3d point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Visualize using open3d
o3d.visualization.draw_geometries([pcd])

# Stop streaming
pipeline.stop()
