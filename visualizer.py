# -----------------------
# FILE: visualizer.py
# -----------------------
import matplotlib.pyplot as plt
from robot_kinematics import robot_chain

def plot_robot(joint_angles, ax, color='blue'):
    frames = robot_chain.forward_kinematics(joint_angles, full_kinematics=True)
    xs, ys, zs = [], [], []

    for frame in frames:
        pos = frame[:3, 3]
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])

    ax.plot(xs, ys, zs, '-o', color=color)
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.7])
    ax.set_title("Robot Arm")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.pause(0.05)

def draw_frame(ax, T, scale=0.05):
    origin = T[:3, 3]
    x_axis = T[:3, 0] * scale
    y_axis = T[:3, 1] * scale
    z_axis = T[:3, 2] * scale

    ax.quiver(*origin, *x_axis, color='r', linewidth=2)
    ax.quiver(*origin, *y_axis, color='g', linewidth=2)
    ax.quiver(*origin, *z_axis, color='b', linewidth=2)
