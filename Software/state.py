"""
state.py

Stores the current state estimate of the probe.

This class contains only the navigation state and utility functions.
The prediction and correction algorithms are implemented in eskf.py.

State vector:

Position (3)
Velocity (3)
Quaternion (4)
Accelerometer Bias (3)
Gyroscope Bias (3)

Total Nominal State = 16 variables
"""

import numpy as np

from quaternion import Quaternion


class ProbeState:
    """
    Stores the nominal state of the probe.

    Position and velocity are expressed in the world frame.

    Quaternion represents the orientation of the probe
    relative to the world frame.

    Biases are estimated by the ESKF.
    """

    def __init__(self):

        self.reset()

    def reset(self):
        """
        Reset the probe to its initial state.

        Called when a new scan begins
        (i.e., probe placed on the nipple).
        """

        # Position (m)
        self.position = np.zeros(3)

        # Velocity (m/s)
        self.velocity = np.zeros(3)

        # Orientation (identity quaternion)
        self.quaternion = np.array([
            1.0,
            0.0,
            0.0,
            0.0
        ])

        # Estimated accelerometer bias (m/s²)
        self.accel_bias = np.zeros(3)

        # Estimated gyroscope bias (rad/s)
        self.gyro_bias = np.zeros(3)

    def get_rotation_matrix(self):
        """
        Returns the current body-to-world
        rotation matrix.
        """

        return Quaternion.to_rotation_matrix(self.quaternion)

    def get_pose(self):
        """
        Returns the current pose.

        Useful for visualization.
        """

        return {
            "position": self.position.copy(),
            "quaternion": self.quaternion.copy()
        }

    def set_position(self, position):
        """
        Update position.
        """

        self.position = np.asarray(position, dtype=float)

    def set_velocity(self, velocity):
        """
        Update velocity.
        """

        self.velocity = np.asarray(velocity, dtype=float)

    def set_quaternion(self, quaternion):
        """
        Update orientation.
        """

        self.quaternion = Quaternion.normalize(quaternion)

    def set_accel_bias(self, bias):
        """
        Update estimated accelerometer bias.
        """

        self.accel_bias = np.asarray(bias, dtype=float)

    def set_gyro_bias(self, bias):
        """
        Update estimated gyroscope bias.
        """

        self.gyro_bias = np.asarray(bias, dtype=float)

    def print_state(self):
        """
        Print the current state.
        """

        print("\n========== Probe State ==========")

        print("Position (m):")
        print(self.position)

        print("\nVelocity (m/s):")
        print(self.velocity)

        print("\nQuaternion:")
        print(self.quaternion)

        print("\nAccel Bias (m/s²):")
        print(self.accel_bias)

        print("\nGyro Bias (rad/s):")
        print(self.gyro_bias)

        print("=================================\n")