## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

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

# Streaming loop
try:
    while True:
        time.sleep(.5)
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
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 0
        background = clipping_distance
        
        # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        
        bg_depth = np.where((depth_image > clipping_distance) | (depth_image <= 0), background, depth_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(bg_depth, alpha=0.03), cv2.COLORMAP_HOT)
        # images = np.hstack((bg_removed, depth_colormap))
        
        # bg_flatten = list(bg_depth.flatten())
        
        # single_width = np.arange(0,1280,1)
        # width = list( np.tile(single_width,720))
        
        # single_height = np.arange(0,720,1)
        # height = list( np.tile(single_height,1280) )
        
        # xyz = [(bg_flatten[i],width[i],height[i]) for i in range(921500)]
        # xyz = np.array(xyz)
        
        xyz =  [[(width, height , bg_depth[width,[height]][0] ) for width in range(bg_depth.shape[0])] for height in range(bg_depth.shape[1])] 
        xyzs  = np.array( [item for sublist in xyz for item in sublist] )
        
        

        # print(xyz.shape)
        # print(len(width))
        # print(len(height))
        # print(len(bg_flatten))
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyzs)
        o3d.visualization.draw_geometries([pcd])
        

        # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        # cv2.imshow('Align Example', bg_depth)
        
        # print(depth_colormap.shape)
        # print(depth_colormap.min())
        # print(depth_colormap.max())
        # print(depth_colormap[0,0,:])

        break
        
        #cv2.imwrite(f"/home/tron/librealsense/filtered_rgb_captures/{np.random.random()}.jpg",bg_removed)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
