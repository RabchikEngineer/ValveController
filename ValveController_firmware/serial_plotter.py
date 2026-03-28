import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, TextBox
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

# PID coefficients
kp, ki, kd = 2, 1, 2
initial_values_loaded = False

# Initialize serial connection
ser = None

# Create figure with subplots
fig_plot = plt.figure(figsize=(12, 8))
ax1 = fig_plot.add_subplot(2, 1, 1)  # Position plot
ax2 = fig_plot.add_subplot(2, 1, 2)  # PID plot

# Create separate control window
fig_control = plt.figure(figsize=(8, 2))
fig_control.canvas.manager.set_window_title('PID Control Panel')

ax1.set_ylim(0, 1)
ax2.set_ylim(-1.0, 1.0)


fig_plot.suptitle('ESP32 PID Controller Real-Time Monitor')


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



# Create sliders (use fig_control instead of plt.axes)
slider_height = 0.1
slider_left = 0.15
slider_width = 0.65
slider_spacing = 0.3
slider_position = 0.8
val_precision = 0.01

# Position sliders from TOP of control window
ax_kp = fig_control.add_axes([slider_left, slider_position, slider_width, slider_height])
ax_ki = fig_control.add_axes([slider_left, slider_position - slider_spacing, slider_width, slider_height])
ax_kd = fig_control.add_axes([slider_left, slider_position - 2 * slider_spacing, slider_width, slider_height])

slider_kp = Slider(ax_kp, 'Kp', 0.0, 10.0, valinit=kp, valstep=val_precision)
slider_ki = Slider(ax_ki, 'Ki', 0.0, 10.0, valinit=ki, valstep=val_precision)
slider_kd = Slider(ax_kd, 'Kd', 0.0, 10.0, valinit=kd, valstep=val_precision)

# Create text boxes for manual entry
textbox_left = slider_left + slider_width + 0.02
textbox_width = 0.12

ax_text_kp = fig_control.add_axes([textbox_left, slider_position, textbox_width, slider_height])
ax_text_ki = fig_control.add_axes([textbox_left, slider_position - slider_spacing, textbox_width, slider_height])
ax_text_kd = fig_control.add_axes([textbox_left, slider_position - 2 * slider_spacing, textbox_width, slider_height])

text_kp = TextBox(ax_text_kp, '', initial=f'{kp:.4f}')
text_ki = TextBox(ax_text_ki, '', initial=f'{ki:.4f}')
text_kd = TextBox(ax_text_kd, '', initial=f'{kd:.4f}')


def send_pid_values():
    """Send PID values to ESP32 via serial"""
    global ser, kp, ki, kd
    if ser is not None and ser.is_open:
        try:
            message = f"set-pid {kp:.6f} {ki:.6f} {kd:.6f}\n"
            ser.write(message.encode())
            print(f"Sent to ESP32: Kp={kp:.6f}, Ki={ki:.6f}, Kd={kd:.6f}")
        except Exception as e:
            print(f"Error sending data: {e}")


def update_kp(val):
    global kp
    kp = val
    text_kp.set_val(f'{kp:.4f}')
    send_pid_values()


def update_ki(val):
    global ki
    ki = val
    text_ki.set_val(f'{ki:.4f}')
    send_pid_values()


def update_kd(val):
    global kd
    kd = val
    text_kd.set_val(f'{kd:.4f}')
    send_pid_values()


def submit_kp(text):
    try:
        val = float(text)
        if 0.0 <= val <= 5.0:
            slider_kp.set_val(val)
    except ValueError:
        text_kp.set_val(f'{kp:.4f}')


def submit_ki(text):
    try:
        val = float(text)
        if 0.0 <= val <= 5.0:
            slider_ki.set_val(val)
    except ValueError:
        text_ki.set_val(f'{ki:.4f}')


def submit_kd(text):
    try:
        val = float(text)
        if 0.0 <= val <= 5.0:
            slider_kd.set_val(val)
    except ValueError:
        text_kd.set_val(f'{kd:.4f}')


# Connect callbacks
slider_kp.on_changed(update_kp)
slider_ki.on_changed(update_ki)
slider_kd.on_changed(update_kd)

text_kp.on_submit(submit_kp)
text_ki.on_submit(submit_ki)
text_kd.on_submit(submit_kd)


def parse_line(line):
    """Parse ESP32 log line and extract values"""
    global initial_values_loaded, kp, ki, kd

    # Check for initial PID values: "Current PID: x y z"
    if 'Current PID:' in line and not initial_values_loaded:
        match = re.search(r'Current PID:\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)', line)
        if match:
            kp = float(match.group(1))
            ki = float(match.group(2))
            kd = float(match.group(3))

            # Update sliders and textboxes
            slider_kp.set_val(kp)
            slider_ki.set_val(ki)
            slider_kd.set_val(kd)
            text_kp.set_val(f'{kp:.4f}')
            text_ki.set_val(f'{ki:.4f}')
            text_kd.set_val(f'{kd:.4f}')

            initial_values_loaded = True
            print(f"Initial PID loaded: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")
            return None, None, None, None

    # print(line)
    # Extract A (Actual)
    match_actual = re.search(r'A:\s*([-\d.]+)', line)
    # print(match_actual)
    actual = float(match_actual.group(1)) if match_actual else None

    # Extract D (Desired)
    match_desired = re.search(r'D:\s*([-\d.]+)', line)
    desired = float(match_desired.group(1)) if match_desired else None

    # Extract I (Integral)
    match_integral = re.search(r'I:\s*([-\d.]+)', line)
    integral = float(match_integral.group(1)) if match_integral else None

    # Extract PID output (last number)
    match_pid = re.search(r',\s*([-\d.]+)\s*$', line)
    pid = float(match_pid.group(1)) if match_pid else None

    return actual, desired, integral, pid


def update_plot(frame):
    global sample_count, ser, current_port

    try:
        if ser is None:
            raise Exception("initial exception")

        if sample_count%100==0:
            print("Requesting pid values")
            ser.write(f"get-pid\n".encode())

        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Parse for both PID data and initial config
            if 'PID System' in line or 'Current PID:' in line:
                actual, desired, integral, pid = parse_line(line)

                if actual is not None and desired is not None:
                    # Add data
                    timestamps.append(sample_count)
                    actual_values.append(actual)
                    desired_values.append(desired)
                    integrals.append(integral if integral else 0)
                    pid_outputs.append(pid if pid else 0)

                    sample_count += 1

                    # Update plot data
                    line_actual.set_data(timestamps, actual_values)
                    line_desired.set_data(timestamps, desired_values)
                    line_pid.set_data(timestamps, pid_outputs)
                    line_integral.set_data(timestamps, integrals)

                    # Auto-scale axes
                    ax1.relim()
                    ax1.autoscale_view()
                    ax2.relim()
                    ax2.autoscale_view()

                    # Print to console
                    print(f"Sample {sample_count}: A={actual:.2f}, D={desired:.2f}, "
                          f"delta={abs(desired - actual):.2f}, I={integral:.2f}, PID={pid:.3f}")

    except Exception as e:
        if "Errno 5" in str(e) or "initial" in str(e):
            current_port += 1
            if current_port >= len(SERIAL_PORTS):
                current_port = 0
            try:
                ser = serial.Serial(SERIAL_PORTS[current_port], BAUD_RATE, timeout=1)
                print(f"Connected to {SERIAL_PORTS[current_port]}")
            except SerialException:
                pass
            print("Reconnecting with other port...")
        else:
            print(f"Error: {e}")

    return line_actual, line_desired, line_pid, line_integral


# Start animation
ani = animation.FuncAnimation(fig_plot, update_plot, interval=10, blit=True, cache_frame_data=False)

plt.show()

# Cleanup
if ser is not None:
    ser.close()