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

# Constants
INTERLEAVED_PATTERN = 0
CHESS_PATTERN = 1
FRAMERATE_MIN = 0
FRAMERATE_MAX = 7

# Settings
# MLX Framerate values 0-7 are 0.5-64Hz
# 0 = 0.5Hz
# 1 = 1Hz
# 2 = 2Hz
# 3 = 4Hz
# 4 = 8Hz
# 5 = 16Hz
# 6 = 32Hz
# 7 = 64Hz
ir_framerate = 4
com_port = "COM5" # Actual com port name will depend on system
pattern = CHESS_PATTERN # 1 = chess (default optimized), 0 = interleaved
calibration_interval = 10 # How long in seconds between each calibration of temperature range
auto_calibrate = 1 # Whether to auto calibrate on calibration interval or not

# State vars
is_recording = False
video_start_time = 0
frames = []
clims = []
prev_calib_time = 0

def on_key_press(event, fig, im, sensor):
    global is_recording, frames, clims, video_start_time, ir_framerate, prev_calib_time
    if event.key == 'i':
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"img_{timestamp}.png"
        print("Saved image as: " + filename)
        plt.savefig(filename)
    elif event.key == 'v':
        if ~is_recording:
            frames = []
            clims = []
            video_start_time = time.time()
            print("Start recording")
        else:
            curr_time = time.time()
            video_elapsed_time = curr_time - video_start_time
            fps = len(frames) / video_elapsed_time
            print("Stop recording. Saving video...")
            # Set up the file writer
            writer = FFMpegWriter(fps=fps)

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
            ani.save(filename, writer=writer)
            print("Saved video as: " + filename)
        is_recording = ~is_recording
    elif event.key == 'h':
        if ir_framerate > 0:
            ir_framerate -= 1
            sensor.setFramerate(ir_framerate)
            print("Set framerate: " + str(ir_framerate))
    elif event.key == 'l':
        if ir_framerate < 7:
            ir_framerate += 1
            im.set_data(sensor.getImage()) # Fix a bug with the plot axis not updating
            sensor.setFramerate(ir_framerate)
            print("Set framerate: " + str(ir_framerate))
    elif event.key == 'r' or event.key == 'c':
        prev_calib_time = 0
        
def show(sensor, calib_interval=5):
    fig, ax = plt.subplots()
    plt.inferno()
    im = plt.imshow(sensor.getImage(), origin='lower')
    ax.set_title("Temperature Map")
    fig.colorbar(im, ax=ax)  # Show a colorbar
    fig.canvas.mpl_connect('key_press_event', lambda event: on_key_press(event, fig, im, sensor))
    
    global prev_calib_time
    prev_calib_time = 0
    data = sensor.getImage()
    curr_clim = [np.min(data), np.max(data)]
    while True:
        try:
            data = sensor.getImage()
            im.set_data(data)
            curr_time = auto_calibrate * time.time()
            if prev_calib_time == 0 or curr_time - prev_calib_time > calib_interval:
                prev_calib_time = time.time()
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
    sensor = MLX90640(port=com_port, framerate=ir_framerate, pattern=pattern)
    show(sensor, calib_interval=calibration_interval)
    sensor.close()
    print("Done")

if __name__ == "__main__":
    try:
        main()
    except:
      pass

