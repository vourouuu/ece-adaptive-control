import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

SERIAL_PORT = 'COM5'
BAUD_RATE = 9600

data_points = 200
var_names = ["RPM", "ref"] # 2 variables from Arduino

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
ser.flushInput()

# Data buffers
n_vars = len(var_names)
data_arrays = [np.zeros(data_points) for _ in range(n_vars)]

# --- SINGLE PLOT WITH 2 LINES ---
fig, ax = plt.subplots(figsize=(8, 5))

line_rpm, = ax.plot(data_arrays[0], label="RPM")
line_ref, = ax.plot(data_arrays[1], label="Reference")

ax.set_xlim(0, data_points)
ax.set_ylim(0, 30)
ax.set_ylabel("RPM")
ax.set_title("Real-Time Motor RPM vs Reference")
ax.legend()

lines = [line_rpm, line_ref]

def update(frame):
    line_bytes = ser.readline().decode(errors="ignore").strip()

    if not line_bytes:
        return lines

    parts = line_bytes.split(",")

    if len(parts) != n_vars:
        return lines

    try:
        vals = [float(x) for x in parts]
    except:
        return lines

    # Shift arrays & insert new values
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
