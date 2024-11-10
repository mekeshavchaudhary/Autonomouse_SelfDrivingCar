import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

# Path to the .bag file
bag_file_path = "20231026_154034.bag"

# Configure the streams
pipeline = rs.pipeline()
config = rs.config()

# Tell config that we will use a recorded device from file to be used by the pipeline through playback.
config.enable_device_from_file(bag_file_path)

# Configure the streams from the bag file
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Create a series of filters to enhance depth data
spatial_filter = rs.spatial_filter()  # Spatial - edge-preserving spatial smoothing
temporal_filter = rs.temporal_filter()  # Temporal - reduces temporal noise
hole_filling_filter = rs.hole_filling_filter()  # Hole filling - fills the holes in depth data

# Start streaming from file
pipeline.start(config)

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Apply filters
        depth_frame = spatial_filter.process(depth_frame)
        depth_frame = temporal_filter.process(depth_frame)
        depth_frame = hole_filling_filter.process(depth_frame)

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert depth and color frames to Open3D format
        depth_o3d = o3d.geometry.Image(depth_image)
        color_o3d = o3d.geometry.Image(color_image)

        # Create an RGBD image
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)

        # Create a point cloud from the RGBD image
        intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)

        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd])

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
