import serial
import time
import csv
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================
# SETTINGS
# ==========================
SERIAL_PORT = "COM4"   # CHANGE THIS
BAUD_RATE = 115200
CSV_PATH = "log.csv"

HEADERS = [
    "a_n.x", "a_n.y", "a_n.z",
    "t_us",
    "p_x", "p_y", "p_z",
    "v_x", "v_y", "v_z",
    "roll_deg", "pitch_deg", "yaw_deg",
    "r_baro",
    "r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z",
    "r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z",
    "v_north_mps"
]

TIME_KEY = "t_us"
V_NORTH_KEY = "v_north_mps"

MAX_POINTS = 1000

# ==========================
# INIT
# ==========================
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)

csv_file = open(CSV_PATH, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(HEADERS)

data = {h: deque(maxlen=MAX_POINTS) for h in HEADERS}

t0 = None

# ==========================
# PARSE
# ==========================
def parse_line(line):
    parts = line.split(",")
    if len(parts) != len(HEADERS):
        return None
    try:
        return [float(x) for x in parts]
    except:
        return None

# ==========================
# PLOT SETUP
# ==========================
fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

ax_pos, ax_vel, ax_res = axes

lines = {}
v_north_text = fig.text(0.02, 0.98, "v_north_mps: --", ha="left", va="top")

def setup_lines():
    for key in ["p_x", "p_y", "p_z"]:
        lines[key], = ax_pos.plot([], [], label=key)

    for key in ["v_x", "v_y", "v_z"]:
        lines[key], = ax_vel.plot([], [], label=key)

    for key in ["r_baro", "r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z",
                "r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z"]:
        lines[key], = ax_res.plot([], [], label=key)

    ax_pos.set_title("Position")
    ax_vel.set_title("Velocity")
    ax_res.set_title("Residuals")

    for ax in axes:
        ax.grid(True)
        ax.legend(loc="upper right")

# ==========================
# UPDATE LOOP
# ==========================
def update(_):
    global t0

    t_idx = HEADERS.index(TIME_KEY)

    while ser.in_waiting:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        row = parse_line(raw)
        if row is None:
            continue

        if t0 is None:
            t0 = row[t_idx]

        # convert to seconds
        row[t_idx] = (row[t_idx] - t0) * 1e-6

        for h, val in zip(HEADERS, row):
            data[h].append(val)

        writer.writerow(row)

    if len(data[TIME_KEY]) == 0:
        return

    t = list(data[TIME_KEY])

    for key in lines:
        if key in data:
            lines[key].set_data(t, list(data[key]))

    for ax in axes:
        ax.relim()
        ax.autoscale_view()

    if len(data[V_NORTH_KEY]) > 0:
        v_north_text.set_text(f"v_north_mps: {data[V_NORTH_KEY][-1]:.3f}")
    else:
        v_north_text.set_text("v_north_mps: --")

# ==========================
# RUN
# ==========================
setup_lines()

ani = animation.FuncAnimation(fig, update, interval=50)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

ser.close()
csv_file.close()