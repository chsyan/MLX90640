# Test program for MLX90640 class, uses matplotlib for plotting temperatures
# Based on example from https://matplotlib.org/stable/gallery/animation/animation_demo.html
# UBC PHAS E-lab, Nov 2022
# Required Packages:
# pyserial
# matplotlib

# conda install -c conda-forge ffmpeg
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np
import matplotlib.pyplot as plt
from MLX90640 import MLX90640
import time
import datetime
import matplotlib

is_recording = False
video_start_time = 0
frames = []
clims = []

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
ir_framerate = 4

def on_key_press(event, fig, im, sensor):
    global is_recording, frames, clims, video_start_time, ir_framerate
    if event.key == 'c':
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"img_{timestamp}.png"
        print("Saved image as: " + filename)
        plt.savefig(filename)
    elif event.key == 'v':
        if ~is_recording:
            print("Starting recording")
            frames = []
            clims = []
            video_start_time = time.time()
        else:
            video_elapsed_time = time.time() - video_start_time
            fps = len(frames) / video_elapsed_time
            print("Stopping recording")
            # Set up the file writer
            writer = FFMpegWriter(fps=15)

            def update(frame):
                # Update the data for the image plot
                i = frame - 1
                im.set_data(frames[i])
                im.set_clim(clims[i][0], clims[i][1])
            
                # Return the image object so that it gets updated
                return im,
            
            # Create the animation from the saved frames and axis limits
            ani = FuncAnimation(fig, update, frames=len(frames), interval=1000/fps)            
            
            # Save the animation to a file
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"vid_{timestamp}.mp4"
            print("Saved video as: " + filename)
            ani.save(filename, writer=writer)
        is_recording = ~is_recording
    elif event.key == 'j':
        if ir_framerate > 0:
            ir_framerate -= 1
            sensor.setFramerate(ir_framerate)
            print("Set framerate: " + str(ir_framerate))
    elif event.key == 'k':
        if ir_framerate < 7:
            ir_framerate += 1
            im.set_data(sensor.getImage()) # Fix a bug with the plot axis not updating
            sensor.setFramerate(ir_framerate)
            print("Set framerate: " + str(ir_framerate))
        
def show(sensor, calib_interval):
    fig, ax = plt.subplots()
    plt.inferno()
    im = plt.imshow(sensor.getImage(), origin='lower')
    ax.set_title("Temperature Map")
    fig.colorbar(im, ax=ax)  # Show a colorbar
    fig.canvas.mpl_connect('key_press_event', lambda event: on_key_press(event, fig, im, sensor))
    
    prev_calib = 0
    data = sensor.getImage()
    curr_clim = [np.min(data), np.max(data)]
    while True:
        try:
            data = sensor.getImage()
            im.set_data(data)
            if time.time() - prev_calib > calib_interval:
                prev_calib = time.time()
                curr_clim = [np.min(data), np.max(data)]
                im.set_clim(curr_clim[0], curr_clim[1])
                print("Recalibrating")
            plt.show()
            if is_recording:
                frames.append(data)
                clims.append(curr_clim)
            plt.pause(0.01)
        except:
            break
    plt.close()


def main():
    matplotlib.use('Tkagg')
    # Setup the sensor
    sensor = MLX90640(port="COM5", baud=115200, framerate=ir_framerate, pattern=1)
    show(sensor, calib_interval=5)
    sensor.close()
    print("Done")

if __name__ == "__main__":
    try:
        main()
    except:
      pass

