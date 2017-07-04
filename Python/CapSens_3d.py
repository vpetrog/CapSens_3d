# 1 Imports
import serial
import numpy as np
import matplotlib.pyplot as plt
import sys
from select import select
from time import sleep
from mpl_toolkits.mplot3d import Axes3D


# 2 GLOBAL FUNCTIONS

# Makesphere
def sphere():
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x1 = 1 * np.outer(np.cos(u), np.sin(v))
    y1 = 1 * np.outer(np.sin(u), np.sin(v))
    z1 = 1 * np.outer(np.ones(np.size(u)), np.cos(v))
    return x1, y1, z1


# Read values from serial port
def read_values():
    values = []
    temp = (ser.readline()).decode()
    print(temp[:-1])
    values = list(map(float, temp.split('\t')))
    return values[0], values[1], values[2], values[3]


# Redraw Axis
def Axis():
    ax.set_xlim3d([0.0, 5.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([0.0, 5.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, 5.0])
    ax.set_zlabel('Z')

    ax.set_title('CapSense')

# ReadWrite reads or writes from to serial buffer
def ReadWrite():
    isdata = False
    inp, outp, err = select([sys.stdin, ser], [], [], 0.1)
    if sys.stdin in inp:
        line = sys.stdin.readline()
        ser.write(line.encode())
    if ser in inp:
        line = ser.readline().decode()
        print(line[:-1])
        isdata = line[:-2].replace('\t', '').isdigit() and not line[:-2].isdigit()
    return isdata


# Main LOOP

# Serial Port Initialization
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flush()
sleep(0.5)

# Figure Generation
fig = plt.figure()
plt.ion()
ax = fig.add_subplot(111, projection='3d')
x, y, z = sphere()

# loop
while True:
    Axis()
    isData = ReadWrite()
    if isData:
        t, x0, y0, z0 = read_values()
        # Plot the surface
        plt.gca().invert_yaxis()
        ax.plot_surface(x+(x0-0.5), y+(y0-0.5), z+(z0-0.5), color='r',rcount = 20, ccount=20)
        plt.pause(0.01)
        ax.clear()
