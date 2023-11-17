import cv2                                
import numpy as np                       
import pyrealsense2 as rs
import matplotlib as mpl
import matplotlib.pyplot as plt          
import time
from mpl_toolkits.mplot3d import Axes3D
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
pipe_profile =pipeline.start(config)



while True:
    frames = pipeline.wait_for_frames()
    #depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    color_frame = np.asanyarray(color_frame.get_data())

    #color_frame = cv2.cvtColor(color_frame,cv2.COLOR_BGR2RGB)

    # plt.imshow(color_frame)
    # plt.show()


    cv2.imshow("preview",color_frame)
    cv2.waitKey(1)