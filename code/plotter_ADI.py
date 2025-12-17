import serial                                 
import matplotlib.pyplot as plt               
from matplotlib.animation import FuncAnimation
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

data_points = 200
var_names = ["x", "xm", "a_hat", "b_hat"]

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
ser.flushInput()

# data buffers
n_vars = len(var_names)
data_arrays = [np.zeros(data_points) for _ in range(n_vars)]

# FIGURE WITH 2 SUBPLOTS
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# --- first subplot (x, xm)
line1, = ax1.plot(data_arrays[0], label="$x$")
line2, = ax1.plot(data_arrays[1], label="$x_m$")
ax1.set_xlim(0, data_points)
ax1.set_ylim(0, 28)
ax1.legend()
ax1.set_ylabel("Amplitude (RPM)")
ax1.set_title("ADI Controller")

# --- second subplot (a_hat, b_hat)
line3, = ax2.plot(data_arrays[2], label="$\hat{a}$")
line4, = ax2.plot(data_arrays[3], label="$\hat{b}$")
ax2.set_xlim(0, data_points)
ax2.set_ylim(-2, 100)
ax2.set_xlabel("Samples")
ax2.set_ylabel("Parameters' Value")
ax2.legend()

lines = [line1, line2, line3, line4]

def update(frame):
    line_bytes = ser.readline().decode(errors='ignore').strip()

    if not line_bytes:
        return lines

    parts = line_bytes.split(",")

    if len(parts) != n_vars:
        return lines

    try:
        vals = [float(x) for x in parts]
    except:
        return lines

    # update buffers
    for arr, v in zip(data_arrays, vals):
        arr[:-1] = arr[1:]
        arr[-1] = v

    # update plot lines
    for line, arr in zip(lines, data_arrays):
        line.set_ydata(arr)

    return lines

ani = FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
plt.show()

ser.close()
