# Error-State Kalman Filter implementation for IMU-based probe tracking.

# Current version:
#     - Nominal state propagation
#     - Quaternion orientation update
#     - Position and velocity integration
#     - Framework for future covariance and correction steps

# State:
#     Position
#     Velocity
#     Quaternion
#     Accelerometer bias
#     Gyroscope bias

import numpy as np

from state import ProbeState
from quaternion import Quaternion
import config


class ESKF:

    def __init__(self):
        # Initialize filter.


        # Nominal state
        self.state = ProbeState()

        # Error covariance matrix
        # 15 error states:
        #
        # dp (3)
        # dv (3)
        # dtheta (3)
        # dba (3)
        # dbg (3)
        #
        self.P = np.eye(15)

        # Debug storage
        self.specific_force_samples = []


    def reset(self):
        # Reset filter to initial probe position

        # Called when probe is placed at nipple

        self.state.reset()

        self.P = np.eye(15)


    def predict(self, accel, gyro, dt):
        # Prediction step

        # 1. Remove estimated sensor bias
        accel_corrected = (
            accel -
            self.state.accel_bias
        )

        gyro_corrected = (
            gyro -
            self.state.gyro_bias
        )

        # 2. Update orientation
        self.state.quaternion = (
            Quaternion.integrate_gyro(
                self.state.quaternion,
                gyro_corrected,
                dt
            )
        )


        # 3. Convert acceleration to world frame
        R = self.state.get_rotation_matrix()

        accel_world = R @ accel_corrected

        # 4. Remove gravity

        #accel_world -= config.GRAVITY

        specific_force = accel_world - config.GRAVITY
        self.specific_force_samples.append(
            specific_force.copy()
        )

        accel_world = specific_force

        # 5. Update position
        old_velocity = self.state.velocity.copy()

        self.state.position += (
            old_velocity * dt
            +
            0.5 * accel_world * dt**2
        )

        # 6. Update velocity

        self.state.velocity += (
            accel_world * dt
        )

        # 7. Covariance prediction placeholder
        self.predict_covariance(
            accel_corrected,
            gyro_corrected,
            dt
        )

    def predict_covariance(
        self,
        accel,
        gyro,
        dt
    ):
        # Propagate error covariance.

        # Future implementation:
        #     P = FPF^T + Q

        # Placeholder
        #
        # Full ESKF requires:
        #
        # F = State transition Jacobian
        # Q = Process noise matrix
        #

        pass


    def update(self, measurement):
        # Measurement update.

        # Future implementation:

        # 1. Compute innovation
        # 2. Calculate Kalman gain
        # 3. Estimate error state
        # 4. Inject correction

        pass


    def inject_error(self, dx):
        # Inject estimated error state.
        # Error state:
        #     dx = [dp, dv, dtheta, dba, dbg]
  
        dp = dx[0:3]
        dv = dx[3:6]
        dtheta = dx[6:9]
        dba = dx[9:12]
        dbg = dx[12:15]

        # Position correction
        self.state.position += dp


        # Velocity correction
        self.state.velocity += dv


        # Orientation correction
        dq = np.array([
            1.0,
            dtheta[0]/2,
            dtheta[1]/2,
            dtheta[2]/2
        ])

        self.state.quaternion = Quaternion.normalize(
            Quaternion.multiply(
                dq,
                self.state.quaternion
            )
        )


        # Bias correction
        self.state.accel_bias += dba
        self.state.gyro_bias += dbg


    def get_state(self):
        # Return current estimated state

        return self.state