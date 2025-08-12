# -----------------------
# FILE: trajectory.py
# -----------------------

def interpolate_cartesian(start_pose, end_pose, num_steps=20):
    """
    Linearly interpolate between two 4x4 transformation matrices.
    """
    import numpy as np
    interpolated_poses = []
    for alpha in np.linspace(0, 1, num_steps):
        interp_pose = (1 - alpha) * start_pose + alpha * end_pose
        interp_pose[3] = [0, 0, 0, 1]  # keep homogeneous row valid
        interpolated_poses.append(interp_pose)
    return interpolated_poses
