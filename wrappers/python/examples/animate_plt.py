import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def animate(i, x=[], y=[]):
    plt.cla()
    x = [i for i in range(100)]
    y = [random.random() for i in range(100)]
    plt.plot(x, y)


if __name__ == "__main__":
    fig = plt.figure()
    ani = FuncAnimation(fig, animate, interval=10)
    plt.show()