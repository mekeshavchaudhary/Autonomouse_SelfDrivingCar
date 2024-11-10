import pyrealsense2 as rs
import numpy as np
import cv2

# Setup:
file_path = "20231026_091514.bag"
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(file_path, repeat_playback=False)

# Start streaming from file
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Extract depth, RGB, and motion data
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        motion_frame = frames.first_or_default(rs.stream.motion)

        # Convert depth and RGB frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Visualize depth data
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('Depth', depth_colormap)

        # Visualize RGB data
        cv2.imshow('Color', color_image)

        # Print motion data (if available)
        if motion_frame:
            motion_data = motion_frame.get_motion_data()
            print(f"Acceleration: {motion_data.acceleration.x}, {motion_data.acceleration.y}, {motion_data.acceleration.z}")

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
