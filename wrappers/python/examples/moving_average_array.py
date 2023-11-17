import numpy as np


def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w


ma = moving_average([i for i in range(10)],2)

print(ma)