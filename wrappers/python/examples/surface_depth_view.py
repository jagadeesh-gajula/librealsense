import cv2                                
import numpy as np                       
import pyrealsense2 as rs
import matplotlib as mpl
import matplotlib.pyplot as plt          
import time
from mpl_toolkits.mplot3d import Axes3D
import time

fig1 = plt.figure(1)
ar = fig1.add_subplot(projection='3d')
ar.set_xlim(-1, 1)
ar.set_ylim(-1, 1)
ar.set_zlim(-1, 1)


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
pipe_profile =pipeline.start(config)

points =[[[]]]
def get_3dPoints():#Function to be written
    pass    

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
colorizer = rs.colorizer()
depth_frame = colorizer.colorize(depth_frame)
prev_depth_colormap = np.asanyarray(depth_frame.get_data())#[170:700,600:800]

print(f"prev_depth_colormap min: {prev_depth_colormap.min()}, Max: {prev_depth_colormap.max()}")

try:
    while True:
        #time.sleep(0.5)
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        

        if not depth_frame or not color_frame: #or not infrared_frame:
            continue
        
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        colorizer = rs.colorizer()
        depth_frame = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(depth_frame.get_data())#[170:700,600:800]
        
        prev_depth_colormap = depth_colormap
        depth_colormap[depth_colormap < 80 ] = 0
        
        depth_colormap = np.maximum(depth_colormap , prev_depth_colormap)
        #depth_colormap = (depth_colormap + prev_depth_colormap) // 2
        #depth_colormap = np.clip(depth_colormap,a_max = 255,a_min=0)
        
        #heatmap = cv2.applyColorMap(depth_colormap[180:600,600:700], cv2.COLORMAP_HOT)
        #heatmap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
        
        cv2.imshow('RealSense', depth_colormap)
        #cv2.imwrite(f"./surface_captures/{np.random.random()}.jpg",depth_colormap)
        
        cv2.waitKey(1)

finally:

    # Stop streaming#img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_AREA)
    pipeline.stop()