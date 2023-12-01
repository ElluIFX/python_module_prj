import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from drivers.interface import register_interface
from drivers.mlx90614 import MLX90614

register_interface("ch347", "i2c", clock=100000)
sensor = MLX90614()

# Initialize empty lists to store temperature data
time_data = []
ambient_temp_data = []
object_temp_data = []

# Create a figure and axis for the chart

fig, ax = plt.subplots()

# Create empty line objects for the temperature data
(ambient_temp_line,) = ax.plot([], [], label="Ambient Temperature")
(object_temp_line,) = ax.plot([], [], label="Object Temperature")

# Set up the chart labels and legend
ax.set_xlabel("Time")
ax.set_ylabel("Temperature (C)")
ax.legend()

start_time = time.time()


# Function to update the chart with new temperature data
def update_chart(frame):
    global time_data, ambient_temp_data, object_temp_data, start_time
    current_time = time.time() - start_time
    ambient_temp = sensor.ambient_temperature
    object_temp = sensor.object_temperature

    # Append new data to the lists
    time_data.append(current_time)
    ambient_temp_data.append(ambient_temp)
    object_temp_data.append(object_temp)
    time_data = time_data[-200:]
    ambient_temp_data = ambient_temp_data[-200:]
    object_temp_data = object_temp_data[-200:]
    max_temp = max(ambient_temp_data + object_temp_data)
    min_temp = min(ambient_temp_data + object_temp_data)

    # Update the line objects with the new data
    ambient_temp_line.set_data(time_data, ambient_temp_data)
    object_temp_line.set_data(time_data, object_temp_data)
    ambient_temp_line.set_label(f"Ambient Temperature ({ambient_temp:.2f} C)")
    object_temp_line.set_label(f"Object Temperature ({object_temp:.2f} C)")
    # Adjust the x-axis limits to show a rolling window of data
    ax.set_xlim(time_data[0], time_data[-1])
    ax.set_ylim(min_temp - 1, max_temp + 1)
    ax.legend(loc="upper left")
    ax.figure.canvas.draw()
    return ambient_temp_line, object_temp_line


# Create the animation
animation = FuncAnimation(fig, update_chart, interval=100)
# Show the chart
plt.show()
