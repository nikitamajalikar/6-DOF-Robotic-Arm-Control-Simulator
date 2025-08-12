# -----------------------
# FILE: tests/test_kinematics.py
# -----------------------
import unittest
import numpy as np
from robot_kinematics import forward_kinematics, inverse_kinematics, within_joint_limits

class TestKinematics(unittest.TestCase):

    def test_forward_kinematics_output_shape(self):
        joint_angles = [0] * 7
        pose = forward_kinematics(joint_angles)
        self.assertEqual(pose.shape, (4, 4))

    def test_inverse_kinematics_within_limits(self):
        pose = np.eye(4)
        pose[0, 3] = 0.2
        pose[2, 3] = 0.3
        joint_angles = inverse_kinematics(pose)
        self.assertTrue(within_joint_limits(joint_angles))

    def test_joint_limit_checking(self):
        invalid_angles = [0, 10, 0, 0, 0, 0, 0]  # way out of range
        self.assertFalse(within_joint_limits(invalid_angles))

if __name__ == '__main__':
    unittest.main()

