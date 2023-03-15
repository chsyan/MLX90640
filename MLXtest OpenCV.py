#Test program for MLX90640 class, uses OpenCV (faster framerate) for plotting temperatures
#UBC PHAS E-lab, Nov 2019
#Required Packages:
#pyserial
#numpy
#opencv-python

from MLX90640 import MLX90640
import numpy as np
import cv2    #run in anaconda command mode 'pip install opencv-python'

#MLX Framerate values 0-7 are 0.5-64Hz
#0 = 0.5Hz
#1 = 1Hz
#2 = 2Hz
#3 = 4Hz
#4 = 8Hz
#5 = 16Hz
#6 = 32Hz
#7 = 64Hz
sensor = MLX90640("COM5", baud=115200, framerate=5, pattern=0)#Actual com port name will depend on system

loop = 0
cv2.namedWindow("img", cv2.WINDOW_NORMAL)
try:
    while True:
        #Calculate temperature values from MLX RAM
        floatarray = [[sensor.getCompensatedPixDataRAM(i+1,j+1) for i in range(24)] for j in range(32)]
        floatarray = np.array(floatarray)
        scalemin = np.min(floatarray)
        scalemax = np.max(floatarray)
        #scalemin = 20
        #scalemax = 35
        #normalize array from 0-1.0
        arrayscaled = (floatarray - scalemin)/(scalemax-scalemin)
        #print(arrayscaled)
        cv2.imshow("img", arrayscaled)
        cv2.waitKey(1)

        sensor.updateRAM() #get copy new of RAM from MLX90640
        loop = loop + 1
        print(loop)

finally:
    sensor.close()
