import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import torch
import torchvision.transforms as T
from torchvision.models.detection import fasterrcnn_resnet50_fpn

# Setup:
file_path = "path_to_your_file.bag"
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(file_path, repeat_playback=False)

# Load pre-trained Faster R-CNN model
model = fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

# Define the transformation
transform = T.Compose([T.ToPILImage(), T.ToTensor()])

# Start streaming from file
pipeline.start(config)

# ... [rest of the previous code for filtering and point cloud creation]
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

# Object detection on color image
input_tensor = transform(color_image)
input_tensor = input_tensor.unsqueeze(0)  # add batch dimension
with torch.no_grad():
    prediction = model(input_tensor)

# Extract bounding boxes and labels
boxes = prediction[0]['boxes'].numpy()
labels = prediction[0]['labels'].numpy()
scores = prediction[0]['scores'].numpy()

# Filter out low confidence detections
threshold = 0.5
selected_indices = scores > threshold
boxes = boxes[selected_indices]
labels = labels[selected_indices]

# Estimate distance for each detected object
for box in boxes:
    x_min, y_min, x_max, y_max = map(int, box)
    region_depth = depth_image[y_min:y_max, x_min:x_max]
    distance = np.nanmedian(region_depth) * depth_scale  # Convert to meters or appropriate scale
    print(f"Object: {labels[i]}, Distance: {distance:.2f} meters")

# ... [rest of the previous code for visualization]
# Visualize using open3d
o3d.visualization.draw_geometries([pcd])
# Stop streaming
pipeline.stop()
