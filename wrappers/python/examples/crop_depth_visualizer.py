## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import time

import open3d as o3d
# Create a pipeline
pipeline = rs.pipeline()
import pickle

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


def get_depth():
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        return None

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    #color_image = np.asanyarray(color_frame.get_data())

    # Remove background - Set pixels further than clipping_distance to grey
    background = clipping_distance

    
    bg_depth = np.where((depth_image > clipping_distance) | (depth_image <= 0), background, depth_image)
    bg_depth = bg_depth[220:580,600:830] #[height,width]
    
    #bg_depth = bg_depth[230:550,650:750]

    
    xyz =  [[(width, height , bg_depth[width,[height]][0] ) for width in range(bg_depth.shape[0])] for height in range(bg_depth.shape[1])] 
    xyzs  = np.array( [item for sublist in xyz for item in sublist] )
    
    

    return [xyzs, bg_depth]
    
    

# Create a random point cloud
xyzs = get_depth()[0]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyzs)

def rotate_view(vis):
        ctr = vis.get_view_control()
        ctr.rotate(10.0, 0.0)
        return False

# Set up visualization window
vis = o3d.visualization.Visualizer()
#vis = o3d.visualization.draw_geometries_with_animation_callback([pcd],rotate_view)
vis.create_window()

# Add point cloud to the window
vis.add_geometry(pcd)

# Get render option for better visualization
render_option = vis.get_render_option()
render_option.point_size = 5.0  # Set the point size

prev = get_depth()[0]

file_number = 1
# Run the visualization loop
while True:
    depth_1 = get_depth()
    depth_2 = get_depth()
    
    depth_array = (depth_1[1] + depth_2[1]) / 2
    
    file = open(f"./depth_arrays/depth_array_{file_number}.bin","wb")   #binary file not pickle
    pickle.dump(depth_array,file)
    file.close()
    
    new_xyz = (depth_1[0] + depth_2[0]) / 2
    
    new_xyz = (new_xyz + prev) / 2
    
    pcd.points = o3d.utility.Vector3dVector(new_xyz)
    prev = new_xyz
    vis.update_geometry(pcd)  # Update geometry
    vis.poll_events()
    vis.update_renderer()
    file_number += 1
    #time.sleep(1)  # Wait for 1 secondl

# Close the visualization window when the loop ends
vis.destroy_window()
