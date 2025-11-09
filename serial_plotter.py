import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

import matplotlib
from serial.serialutil import SerialException

matplotlib.use('qt5agg')

# This is a program for plotting graphs from esp logs.
# Current plotted values: desired position, current position, pid integral, pid output value 
#


# Configuration
SERIAL_PORTS = ['/dev/ttyACM0','/dev/ttyACM1']  # Change to your port (Linux: /dev/ttyUSB0, Mac: /dev/cu.usbserial-*)
current_port=0
BAUD_RATE = 9600
MAX_POINTS = 200  # Number of points to display

# Data storage
timestamps = deque(maxlen=MAX_POINTS)
actual_values = deque(maxlen=MAX_POINTS)
desired_values = deque(maxlen=MAX_POINTS)
pid_outputs = deque(maxlen=MAX_POINTS)
integrals = deque(maxlen=MAX_POINTS)

# Initialize serial connection
ser = None

# Create figure with subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
ax1.set_ylim(1500, 2900)    # For actual and desired positions
ax2.set_ylim(-1.0, 1.0)
fig.suptitle('ESP32 PID Controller Real-Time Monitor')

# Initialize empty lines
line_actual, = ax1.plot([], [], 'b-', label='Actual', linewidth=2)
line_desired, = ax1.plot([], [], 'r--', label='Desired', linewidth=2)
line_pid, = ax2.plot([], [], 'g-', label='PID Output', linewidth=2)
line_integral, = ax2.plot([], [], 'm-', label='Integral', linewidth=1)

# Configure axes
ax1.set_xlabel('Sample')
ax1.set_ylabel('Position')
ax1.legend(loc='upper right')
ax1.grid(True)

ax2.set_xlabel('Sample')
ax2.set_ylabel('PID / Integral')
ax2.legend(loc='upper right')
ax2.grid(True)

sample_count = 0


def parse_line(line):
    """Parse ESP32 log line and extract values"""
    # Example line: "I (89326) PID System: A: 2163.492/D: 2148.637,I: 5.7 , dt: 0.10000, -0.0699"

    # Extract A (Actual)
    match_actual = re.search(r'A:\s*([\d.]+)', line)
    actual = float(match_actual.group(1)) if match_actual else None

    # Extract D (Desired)
    match_desired = re.search(r'D:\s*([\d.]+)', line)
    desired = float(match_desired.group(1)) if match_desired else None

    # Extract I (Integral)
    match_integral = re.search(r'I:\s*([-\d.]+)', line)
    integral = float(match_integral.group(1)) if match_integral else None

    # Extract PID output (last number)
    match_pid = re.search(r',\s*([-\d.]+)\s*$', line)
    pid = float(match_pid.group(1)) if match_pid else None

    return actual, desired, integral, pid


def update_plot(frame):
    global sample_count,ser,current_port

    try:
        # Read line from serial

        if ser is None:
            raise Exception("initial exception")

        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Only process PID System lines
            if 'PID System' in line:
                actual, desired, integral, pid = parse_line(line)

                if actual is not None and desired is not None:
                    # Add data
                    timestamps.append(sample_count)
                    actual_values.append(actual)
                    desired_values.append(desired)
                    integrals.append(integral/100 if integral else 0)
                    pid_outputs.append(pid if pid else 0)

                    sample_count += 1

                    # Update plot data
                    line_actual.set_data(timestamps, actual_values)
                    line_desired.set_data(timestamps, desired_values)
                    line_pid.set_data(timestamps, pid_outputs)
                    line_integral.set_data(timestamps, integrals)

                    # # Auto-scale axes
                    ax1.relim()
                    ax1.autoscale_view()
                    ax2.relim()
                    ax2.autoscale_view()

                    # Print to console
                    print(f"Sample {sample_count}: A={actual:.2f}, D={desired:.2f}, I={integral:.2f}, PID={pid:.3f}")

    except Exception as e:
        if "Errno 5" in str(e) or "initial" in str(e):
            current_port+=1
            if current_port>=len(SERIAL_PORTS):
                current_port=0
            try:
                ser=serial.Serial(SERIAL_PORTS[current_port], BAUD_RATE, timeout=1)
            except SerialException:
                pass
            print("reconnecting with other port...")
        else:
            print(f"Error: {e}")

    return line_actual, line_desired, line_pid, line_integral


# Start animation
ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)

plt.tight_layout()
plt.show()

# Cleanup
ser.close()
