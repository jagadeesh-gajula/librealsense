import cv2                                
import numpy as np                       
import pyrealsense2 as rs
import matplotlib as mpl
import matplotlib.pyplot as plt          
from mpl_toolkits.mplot3d import Axes3D
import time



# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
pipe_profile =pipeline.start(config)

try:
    printed = 0
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        

        if not depth_frame or not color_frame: 
            continue
        
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # colorizer = rs.colorizer()
        # depth_frame = colorizer.colorize(depth_frame)
        
        depth_colormap = np.asanyarray(depth_frame.get_data())
        color_frame = np.asanyarray(color_frame.get_data())
        
        depth_colormap = np.clip(depth_colormap,a_min=0,a_max=1600)
        color_frame[depth_colormap > 800] = 0
        
        depth_colormap = (depth_colormap/256).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
    
        if printed == 0:
            print(f"Depthmap details: max_depth:{depth_colormap.max()} min_depth:{depth_colormap.min()} avg_depth:{depth_colormap.mean()}")
            printed = None
        
        cv2.imshow('RealSense',color_frame) 
        cv2.waitKey(1)

finally:

    pipeline.stop()