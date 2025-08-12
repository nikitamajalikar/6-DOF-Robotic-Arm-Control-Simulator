# Directory structure:
# robotics_challenge/
# ├── main.py
# ├── robot_kinematics.py
# ├── trajectory.py
# ├── visualizer.py
# ├── requirements.txt
# ├── README.md
# └── tests/
#     └── test_kinematics.py

# -----------------------
# FILE: main.py
# -----------------------
import numpy as np
from robot_kinematics import forward_kinematics, inverse_kinematics, within_joint_limits
from trajectory import interpolate_cartesian
#from visualizer import plot_robot, draw_frame
#import matplotlib.pyplot as plt

def pose_from_xyz_rpy(x, y, z, roll, pitch, yaw):
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

#def print_pose(pose):
 #   pos = pose[:3, 3]
  #  print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

def print_pose(pose):
    pos = pose[:3, 3]
    print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    # Extract rotation matrix
    R = pose[:3, :3]

    # Convert rotation matrix to roll, pitch, yaw
    pitch = np.arcsin(-R[2, 0])
    if abs(R[2, 0]) != 1:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock cases
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    # Convert to degrees
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    print(f"Orientation: roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°, yaw={yaw_deg:.1f}°")

def is_near_obstacle(pose, obstacles, threshold=0.05):
    end_effector_pos = pose[:3, 3]
    for obs in obstacles:
        if np.linalg.norm(end_effector_pos - obs) < threshold:
            return True
    return False

def execute_trajectory(from_pose, to_pose, obstacles, steps=20):
    motion_steps = interpolate_cartesian(from_pose, to_pose, num_steps=steps)
    for i, pose in enumerate(motion_steps):
        joint_angles = inverse_kinematics(pose)
        if not within_joint_limits(joint_angles):
            print(f"Step {i+1:02}: ❌ Joint limits exceeded.")
            continue
        if is_near_obstacle(pose, obstacles):
            print(f"Step {i+1:02}: ⚠️ Too close to obstacle. Skipping.")
            continue
        print(f"Step {i+1:02}:")
        print_pose(pose)

def main():
    home_angles = [0] * 7  # 6 joints + base
    current_pose = forward_kinematics(home_angles)

    print("Initial pose:")
    #print(current_pose)  # NEW DEBUG LINE
    print_pose(current_pose)

    obstacles = []
    num_obstacles = int(input("How many obstacles? "))
    for i in range(num_obstacles):
        ox = float(input(f"Obstacle {i + 1} X (m): "))
        oy = float(input(f"Obstacle {i + 1} Y (m): "))
        oz = float(input(f"Obstacle {i + 1} Z (m): "))
        obstacles.append(np.array([ox, oy, oz]))

        print("\nEnter PICK position:")
        x_pick = float(input("Pick X (m): "))
        y_pick = float(input("Pick Y (m): "))
        z_pick = float(input("Pick Z (m): "))
        roll_pick = float(input("Pick Roll (deg): "))
        pitch_pick = float(input("Pick Pitch (deg): "))
        yaw_pick = float(input("Pick Yaw (deg): "))

        print("\nEnter PLACE position:")

        x_place = float(input("Place X (m): "))
        y_place = float(input("Place Y (m): "))
        z_place = float(input("Place Z (m): "))
        roll_place = float(input("Place Roll (deg): "))
        pitch_place = float(input("Place Pitch (deg): "))
        yaw_place = float(input("Place Yaw (deg): "))

        pose_pick = pose_from_xyz_rpy(x_pick, y_pick, z_pick, roll_pick, pitch_pick, yaw_pick)
        pose_lift = pose_from_xyz_rpy(x_pick, y_pick, z_pick + 0.05, roll_pick, pitch_pick, yaw_pick)
        pose_place_lift = pose_from_xyz_rpy(x_place, y_place, z_place + 0.05, roll_place, pitch_place, yaw_place)
        pose_place = pose_from_xyz_rpy(x_place, y_place, z_place, roll_place, pitch_place, yaw_place)
        print("\nExecuting trajectory: home → pick")

        execute_trajectory(current_pose, pose_pick, obstacles)

        print("🤖 Gripper closing...")
        print("\nExecuting trajectory: pick → lift")
        execute_trajectory(pose_pick, pose_lift, obstacles)
        print("\nExecuting trajectory: lift → over place")
        execute_trajectory(pose_lift, pose_place_lift, obstacles)
        print("\nExecuting trajectory: over place → place")
        execute_trajectory(pose_place_lift, pose_place, obstacles)
        print("🤖 Gripper opening...")
        print("\nExecuting trajectory: place → lift")
        execute_trajectory(pose_place, pose_place_lift, obstacles)
        print("\nExecuting trajectory: lift → home")
        execute_trajectory(pose_place_lift, current_pose, obstacles)

if __name__ == "__main__":
    main()
