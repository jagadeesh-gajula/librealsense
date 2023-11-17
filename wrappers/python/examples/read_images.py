import cv2                                
import numpy as np                       
import pyrealsense2 as rs
import matplotlib as mpl
import matplotlib.pyplot as plt          
import time
# from mpl_toolkits.mplot3d import Axes3D
import random
from matplotlib.animation import FuncAnimation

fig1 = plt.figure(1)
ar = fig1.add_subplot(projection='3d')
ar.set_xlim(-1, 1)
ar.set_ylim(-1, 1)
ar.set_zlim(-1, 1)

depths = []

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

universal_y  = None

def animate(i, x=[], y=[]):
    plt.cla()
    
    try:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        

        # if not depth_frame or not color_frame: #or not infrared_frame:
        #     continue
        
        # Convert images to numpy arrays
        # depth_image = np.asanyarray(depth_frame.get_data())[:,450:850]
        # color_image = np.asanyarray(color_frame.get_data())

        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        
        colorizer = rs.colorizer()
        dc = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        depth_colormap = dc[:,:,[0]] + dc[:,:,[1]] + dc[:,:,[2]]
        
        
        # y = depth_colormap[180:600,650:651].reshape(420)
        # y = moving_average(y,10)
        # x = np.array([i for i in range(y.shape[0])])
        
        # if y == None:
        #     global universal_y 
        #     universal_y = y 
        # plt.yscale('log')
        # plt.plot(x, y,linewidth = '2.0')
        
        #print(depth_frame.get_data().shape)
        cv2.imshow('RealSense', depth_colormap[150:650,600:700])
        
        # heatmap = cv2.applyColorMap(depth_colormap[150:650,600:700], cv2.COLORMAP_JET)
        # cv2.imshow('heatmap', heatmap)
        
        cv2.waitKey(1)
        
    except Exception as error:
        print(error)

    # finally:

    #     # Stop streaming#img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_AREA)
    #     pipeline.stop()
        

# Start streaming
pipe_profile =pipeline.start(config)
    
fig = plt.figure(figsize=(10,5))
ani = FuncAnimation(fig, animate, interval=100)
plt.show()