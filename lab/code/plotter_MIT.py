import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

data_points = 200
var_names = ["RPM", "ym", "theta1", "theta2"]

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
ser.flushInput()

# Data buffers
n_vars = len(var_names)
data_arrays = [np.zeros(data_points) for _ in range(n_vars)]

# Figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# --- First subplot (RPM, xm) ---
line1, = ax1.plot(data_arrays[0], label="x")
line2, = ax1.plot(data_arrays[1], label="xm")
ax1.set_xlim(0, data_points)
ax1.set_ylim(0, 28)
ax1.legend()
ax1.set_ylabel("Amplitude (RPM)")
ax1.set_title("MIT Controller")

# --- Second subplot (kr, kx) ---
line3, = ax2.plot(data_arrays[2], label="θ1")
line4, = ax2.plot(data_arrays[3], label="θ2")
ax2.set_xlim(0, data_points)
ax2.set_ylim(-10, 20)
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

    # Update buffers
    for arr, v in zip(data_arrays, vals):
        arr[:-1] = arr[1:]
        arr[-1] = v

    # Update plot lines
    for line, arr in zip(lines, data_arrays):
        line.set_ydata(arr)

    return lines

ani = FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
plt.show()

ser.close()
