# Test program for MLX90640 class, uses matplotlib for plotting temperatures
# Based on example from https://matplotlib.org/stable/gallery/animation/animation_demo.html
# UBC PHAS E-lab, Nov 2022
# Required Packages:
# pyserial
# matplotlib


from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib.pyplot as plt
from MLX90640 import MLX90640
from celluloid import Camera

# conda install -c conda-forge ffmpeg

import matplotlib
matplotlib.use('Tkagg')

plt.rcParams['animation.ffmpeg_path'] = '"D:\Downloads\ffmpeg-master-latest-win64-gpl\ffmpeg-master-latest-win64-gpl\bin\ffmpeg.exe"'
# MLX Framerate values 0-7 are 0.5-64Hz
# 0 = 0.5Hz
# 1 = 1Hz
# 2 = 2Hz
# 3 = 4Hz
# 4 = 8Hz
# 5 = 16Hz
# 6 = 32Hz
# 7 = 64Hz
# Actual com port name will depend on system
sensor = MLX90640(port="COM5", baud=115200, framerate=3, pattern=1)
fps = 10
nSeconds = 5

# fig, ax = plt.subplots()
# plt.inferno()

# floatarray = [[sensor.getCompensatedPixDataRAM(
#     i+1, j+1) for i in range(24)] for j in range(32)]
# im = ax.imshow(floatarray)  # Show the image
# ax.set_title("Temperature Map")
# cb = fig.colorbar(im, ax=ax)  # Show a colorbar


loop = 0


fig, ax = plt.subplots()

plt.inferno()
im = plt.imshow(sensor.getImage(), origin='lower')
ax.set_title("Temperature Map")
cb = fig.colorbar(im, ax=ax)  # Show a colorbar
camera = Camera(fig)



def animate(n):
    print(n)
    ax.set_title("Temperature Map")
    data = sensor.getImage()
    im.set_data(data)
    im.set_clim(np.min(data), np.max(data))
    return [im]
    

def video():
    anim = FuncAnimation(fig, animate, interval=1000 / fps)
    plt.show()
    f = r"animate_func.gif"
    anim.save(f, fps=fps)
    #writergif = PillowWriter(fps=fps)
    #anim.save(f, writer=writergif)
    sensor.close()
    plt.close()


def normal():
    try:
        while True:
            # Calculate temperature values from MLX RAM
            floatarray = [[sensor.getCompensatedPixDataRAM(
                i+1, j+1) for i in range(24)] for j in range(32)]
            cmap = ax.imshow(floatarray)  # Show the image
            ax.set_title("Temperature Map")
            cb = fig.colorbar(cmap, ax=ax)  # Show a colorbar
            plt.pause(0.001)
            sensor.updateRAM()  # get copy new of RAM from MLX90640
            cb.remove()  # remove old plots
            ax.cla()
    finally:
        sensor.close()


video()
