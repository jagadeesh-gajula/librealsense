import numpy as np
import pickle
import matplotlib.pyplot as plt

file = open("/home/tron/librealsense/depth_arrays/depth_array_1.bin","rb")

array = pickle.load(file)

file.close()

plt.imshow(array)
plt.show()

