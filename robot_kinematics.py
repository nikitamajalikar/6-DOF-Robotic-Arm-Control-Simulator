# -----------------------
# FILE: robot_kinematics.py
# -----------------------
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# Define the robot chain
def get_robot_chain():
    return Chain(name='6dof_robot', links=[
        OriginLink(),
        URDFLink(
            name="joint_1",
            origin_translation=[0, 0, 0.1],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1]
        ),
        URDFLink(
            name="joint_2",
            origin_translation=[0, 0, 0.2],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),
        URDFLink(
            name="joint_3",
            origin_translation=[0.2, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),
        URDFLink(
            name="joint_4",
            origin_translation=[0.1, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0]
        ),
        URDFLink(
            name="joint_5",
            origin_translation=[0, 0, 0.1],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),
        URDFLink(
            name="joint_6",
            origin_translation=[0, 0, 0.1],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1]
        )
    ])

JOINT_LIMITS = [
    (-np.pi, np.pi),
    (-np.pi/2, np.pi/2),
    (-np.pi/2, np.pi/2),
    (-np.pi, np.pi),
    (-np.pi/2, np.pi/2),
    (-np.pi, np.pi)
]

robot_chain = get_robot_chain()

def forward_kinematics(joint_angles):
    return robot_chain.forward_kinematics(joint_angles)

def inverse_kinematics(target_pose):
    target_position = target_pose[:3, 3]  # extract x, y, z
    return robot_chain.inverse_kinematics(target_position)

def within_joint_limits(joint_angles):
    for i, angle in enumerate(joint_angles[1:]):  # Skip base
        min_limit, max_limit = JOINT_LIMITS[i]
        if not (min_limit <= angle <= max_limit):
            return False
    return True
