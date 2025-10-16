
import pandas as pd
import matplotlib.pylab as plt
import numpy as np

df = pd.read_csv("./results/states.csv")

x = df['pos_0']
y = df['pos_1']
z = df['pos_2']
x_dot = df['vel_0']
y_dot = df['vel_1']
z_dot = df['vel_2']

# Normalize positions
x_norm = df['pos_0'] / df['pos_0'].max()
y_norm = df['pos_1'] / df['pos_1'].max()
z_norm = df['pos_2'] / df['pos_2'].max()

# Normalize velocities
x_dot_norm = df['vel_0'] / df['vel_0'].max()
y_dot_norm = df['vel_1'] / df['vel_1'].max()
z_dot_norm = df['vel_2'] / df['vel_2'].max()

# Compute range (distance from origin) for each time step
range_vals = np.sqrt(x**2 + y**2 + z**2)
speed_vals = np.sqrt(x_dot**2 + y_dot**2 + z_dot**2)

# Get time column (replace 'time' with your actual time column name if different)
time = df['time']

fig, axs = plt.subplots(2, 3, figsize=(12, 6), sharex=True)

# Positions
axs[0, 0].plot(time, df['pos_0'])
axs[0, 0].set_title('X Position')
axs[0, 1].plot(time, df['pos_1'])
axs[0, 1].set_title('Y Position')
axs[0, 2].plot(time, df['pos_2'])
axs[0, 2].set_title('Z Position')

# Velocities
axs[1, 0].plot(time, df['vel_0'])
axs[1, 0].set_title('X Velocity')
axs[1, 1].plot(time, df['vel_1'])
axs[1, 1].set_title('Y Velocity')
axs[1, 2].plot(time, df['vel_2'])
axs[1, 2].set_title('Z Velocity')

for ax in axs.flat:
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.grid(True)

plt.tight_layout()
plt.show()