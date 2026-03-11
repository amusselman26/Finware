import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time

# ---------- config ----------
PORT = "COM4"      # Windows example: "COM5"
# PORT = "/dev/ttyUSB0"  # Linux example
# PORT = "/dev/tty.usbmodemXXXX"  # macOS example
BAUD = 115200

# ---------- quaternion utils ----------
def q_norm(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1,0,0,0], dtype=float)
    return q / n

def q_to_R(q):
    # q = [w,x,y,z] body->nav rotation
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ], dtype=float)

def R_to_euler_zyx(R):
    # yaw(Z), pitch(Y), roll(X) for nav frame
    # handles typical ranges; good enough for live viz
    yaw = np.arctan2(R[1,0], R[0,0])
    pitch = np.arcsin(-np.clip(R[2,0], -1.0, 1.0))
    roll = np.arctan2(R[2,1], R[2,2])
    return roll, pitch, yaw

# ---------- cube model ----------
def make_cube(size=1.0):
    s = size / 2.0
    verts = np.array([
        [-s, -s, -s],
        [ s, -s, -s],
        [ s,  s, -s],
        [-s,  s, -s],
        [-s, -s,  s],
        [ s, -s,  s],
        [ s,  s,  s],
        [-s,  s,  s],
    ], dtype=float)

    faces = [
        [0,1,2,3],  # bottom
        [4,5,6,7],  # top
        [0,1,5,4],  # front
        [2,3,7,6],  # back
        [1,2,6,5],  # right
        [3,0,4,7],  # left
    ]
    return verts, faces

# ---------- serial ----------
ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(1.5)
ser.reset_input_buffer()

# ---------- plot ----------
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Quaternion Live Visualization")

# fixed axis limits
L = 1.2
ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

# cube
verts0, faces = make_cube(size=1.0)
poly = Poly3DCollection([], alpha=0.35)
ax.add_collection3d(poly)

# body axes lines
body_axes = {
    "x": ax.plot([0,1],[0,0],[0,0])[0],
    "y": ax.plot([0,0],[0,1],[0,0])[0],
    "z": ax.plot([0,0],[0,0],[0,1])[0],
}

text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

def update_plot(q):
    q = q_norm(q)
    R = q_to_R(q)

    # rotate cube vertices (body frame cube -> nav frame)
    verts = (R @ verts0.T).T

    # update faces
    face_verts = [[verts[i] for i in face] for face in faces]
    poly.set_verts(face_verts)

    # update body axes
    ex = R @ np.array([1,0,0], dtype=float)
    ey = R @ np.array([0,1,0], dtype=float)
    ez = R @ np.array([0,0,1], dtype=float)

    body_axes["x"].set_data([0, ex[0]], [0, ex[1]])
    body_axes["x"].set_3d_properties([0, ex[2]])

    body_axes["y"].set_data([0, ey[0]], [0, ey[1]])
    body_axes["y"].set_3d_properties([0, ey[2]])

    body_axes["z"].set_data([0, ez[0]], [0, ez[1]])
    body_axes["z"].set_3d_properties([0, ez[2]])

    roll, pitch, yaw = R_to_euler_zyx(R)
    text.set_text(
        f"q=[{q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f}]\n"
        f"roll={np.degrees(roll):+.1f}°, pitch={np.degrees(pitch):+.1f}°, yaw={np.degrees(yaw):+.1f}°"
    )

    fig.canvas.draw()
    fig.canvas.flush_events()

print("Listening... (CSV: t_us,w,x,y,z)")
try:
    buf = ""
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) != 5:
            continue
        # t_us = int(parts[0])  # unused
        q = [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]
        update_plot(q)
except KeyboardInterrupt:
    pass
finally:
    ser.close()