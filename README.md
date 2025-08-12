# 6-DOF-Robotic-Arm-Control-Simulator
# -----------------------
# FILE: README.md
# -----------------------
# 6-Axis Robotic Arm - Cartesian Motion Challenge

This project implements a simulation of a 6-axis robotic arm that accepts Cartesian coordinates and orientation commands (x, y, z, roll, pitch, yaw) and generates smooth, visualised movement using Python.

## 🚀 Features
- Forward and Inverse Kinematics using `ikpy`
- Cartesian pose interpolation for smooth motion
- Joint limit validation
- Optional obstacle avoidance
- Optional pick-and-place sequence
- Real-time 3D visualization (optional)
- Unit tests for kinematics modules

## 📦 Requirements
See `requirements.txt` below.

## 📂 How to Run
1. Clone or download the repository
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the main program:
   ```bash
   python main.py
   ```
4. Follow prompts to enter target poses, obstacles, or pick-and-place commands.

## ✅ Example Input (Pick & Place with Obstacle)
```
How many obstacles? 1
Obstacle 1 X (m): 0.25
Obstacle 1 Y (m): 0.0
Obstacle 1 Z (m): 0.3

Pick X (m): 0.2
Pick Y (m): 0.0
Pick Z (m): 0.25
Pick Roll (deg): 0
Pick Pitch (deg): 0
Pick Yaw (deg): 0

Place X (m): 0.35
Place Y (m): 0.0
Place Z (m): 0.25
Place Roll (deg): 0
Place Pitch (deg): 0
Place Yaw (deg): 0
```

## 🧪 Running Tests
```bash
python -m unittest discover tests
```

## 🧠 Design Decisions / Assumptions
- Inverse kinematics is based only on end-effector **position**, not orientation, due to `ikpy`'s limitations.
- Obstacles are modeled as **points** in 3D space and checked only against the end-effector.
- Pick-and-place sequence assumes the **gripper behavior** is simulated with printed messages.
- Orientation matching is supported in interpolation and printing, but IK matching to rotation is limited.
- Collisions are skipped; no replanning is done for avoided steps.

