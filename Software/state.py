# Stores the current state estimate of the probe.

# State vector:
# Position (3)
# Velocity (3)
# Quaternion (4)
# Accelerometer Bias (3)
# Gyroscope Bias (3)

import numpy as np

from quaternion import Quaternion
import config


class ProbeState:
    # Stores the nominal state of the probe.

    # Position and velocity are expressed in the world frame.

    # Quaternion represents the orientation of the probe
    # relative to the world frame.

    # Biases are estimated by the ESKF
    
    position = np.zeros(3)
    velocity = np.zeros(3)
    quaternion = np.zeros(4)
    # Estimated accelerometer bias (m/s²)
    # Estimated gyroscope bias (rad/s)
    accel0_bias = np.zeros(3)
    gyro0_bias = np.zeros(3)
    accel1_bias = np.zeros(3)
    gyro1_bias = np.zeros(3)
    calibrated_accel0_bias = np.zeros(3)
    calibrated_gyro0_bias = np.zeros(3)
    calibrated_accel1_bias = np.zeros(3)
    calibrated_gyro1_bias = np.zeros(3)
    offset = 0

    def __init__(self):

        self.reset()

    def reset(self):
        # Reset the probe to its initial state

        # Call when a new scan begins
        # (i.e., probe placed on the nipple)

        # Position (m)
        self.position = np.array([
            0.0,
            0.0,
            config.BREAST_C
        ])

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
        self.accel0_bias = self.calibrated_accel0_bias
        self.accel1_bias = self.calibrated_accel1_bias

        # Estimated gyroscope bias (rad/s)
        self.gyro0_bias = self.calibrated_gyro0_bias
        self.gyro1_bias = self.calibrated_gyro1_bias

    def get_rotation_matrix(self):
        # Returns the current body-to-world rotation matrix
        return Quaternion.to_rotation_matrix(self.quaternion)

    def get_pose(self):
        # Returns the current pose

        return {
            "position": self.position.copy(),
            "quaternion": self.quaternion.copy()
        }

    def set_position(self, position):
        # Update position

        self.position = np.asarray(position, dtype=float)

    def set_velocity(self, velocity):
        # Update velocity

        self.velocity = np.asarray(velocity, dtype=float)

    def set_quaternion(self, quaternion):
        # Update orientation

        self.quaternion = Quaternion.normalize(quaternion)

    def set_accel0_bias(self, bias):
        # Update estimated accelerometer bias.
        self.accel0_bias = np.asarray(bias, dtype=float)

    def set_gyro0_bias(self, bias):
        # Update estimated gyroscope bias.
        self.gyro0_bias = np.asarray(bias, dtype=float)

    def set_accel1_bias(self, bias):
        # Update estimated accelerometer bias.
        self.accel1_bias = np.asarray(bias, dtype=float)

    def set_gyro1_bias(self, bias):
        # Update estimated gyroscope bias.
        self.gyro1_bias = np.asarray(bias, dtype=float)

    def set_calibrated_accel0_bias(self, bias):
        self.calibrated_accel0_bias = np.asarray(bias, dtype=float)

    def set_calibrated_gyro0_bias(self, bias):
        self.calibrated_gyro0_bias = np.asarray(bias, dtype=float)

    def set_calibrated_accel1_bias(self, bias):
        self.calibrated_accel1_bias = np.asarray(bias, dtype=float)
    
    def set_calibrated_gyro1_bias(self, bias):
        self.calibrated_gyro1_bias = np.asarray(bias, dtype=float)

    def print_state(self):
        # Print states

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