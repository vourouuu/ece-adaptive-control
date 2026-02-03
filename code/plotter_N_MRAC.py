import serial                                 
import matplotlib.pyplot as plt               
from matplotlib.animation import FuncAnimation
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

data_points = 200
var_names = ["theta", "theta_m", "omega", "omega_m", "kx_theta", "kx_omega", "kr"]

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
ser.flushInput()

# Data buffers
n_vars = len(var_names)
data_arrays = [np.zeros(data_points) for _ in range(n_vars)]

# Figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# --- First subplot (θ, ω) ---
line_theta,    = ax1.plot(data_arrays[0], label=r"$\theta$")
line_theta_m,  = ax1.plot(data_arrays[1], label=r"$\theta_m$", linestyle='--')
line_omega,    = ax1.plot(data_arrays[2], label=r"$\omega$")
line_omega_m,  = ax1.plot(data_arrays[3], label=r"$\omega_m$", linestyle='--')

ax1.set_xlim(0, data_points)
ax1.set_ylim(-120, 480)
ax1.legend(loc="upper right", fontsize='small', ncol=2)
ax1.set_ylabel("States (deg & RPM)")
ax1.set_title("Vector MRAC")

# --- Second subplot (kx_theta, kx_omega, kr) ---
line_kx_th, = ax2.plot(data_arrays[4], label="$k_{x,\\theta}$")
line_kx_om, = ax2.plot(data_arrays[5], label="$k_{x,\\omega}$")
line_kr,    = ax2.plot(data_arrays[6], label="$k_r$")

ax2.set_xlim(0, data_points)
ax2.set_ylim(-100, 200)
ax2.set_xlabel("Samples")
ax2.set_ylabel("Parameters' Value")
ax2.legend(loc="upper right", fontsize='small')

lines = [line_theta, line_theta_m, line_omega, line_omega_m, line_kx_th, line_kx_om, line_kr]

def update(frame):
    line_bytes = ser.readline().decode(errors='ignore').strip()

    if not line_bytes:
        return lines

    parts = line_bytes.split(",")

    if len(parts) != n_vars:
        return lines

    try:
        vals = [float(x) for x in parts]
    except ValueError:
        return lines

    # Update buffers
    for arr, v in zip(data_arrays, vals):
        arr[:-1] = arr[1:]
        arr[-1] = v

    # Update plot lines
    for line, arr in zip(lines, data_arrays):
        line.set_ydata(arr)
    
    return lines

ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
plt.show()

ser.close()