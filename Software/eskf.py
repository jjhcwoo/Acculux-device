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

def skew(v):

    x,y,z = v

    return np.array([
        [0,-z,y],
        [z,0,-x],
        [-y,x,0]
    ])


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
        # Position uncertainty
        self.P[0:3,0:3] *= 0.001

        # Velocity uncertainty
        self.P[3:6,3:6] *= 0.1

        # Orientation uncertainty
        self.P[6:9,6:9] *= 0.01

        # Bias uncertainty
        self.P[9:15,9:15] *= 0.001


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

        F = np.eye(15)

        R = self.state.get_rotation_matrix()


        # velocity depends on attitude error
        F[3:6,6:9] = (
            -R @ skew(accel)
        ) * dt


        # velocity depends on accel bias
        F[3:6,9:12] = (
            -R
        ) * dt


        # attitude depends on gyro bias
        F[6:9,12:15] = (
            -np.eye(3)
        ) * dt


        # position depends on velocity
        F[0:3,3:6] = (
            np.eye(3)
        ) * dt



        Q = np.zeros((15,15))


        Q[3:6,3:6] = (
            config.ACCEL_NOISE**2 *
            np.eye(3)
        )

        Q[6:9,6:9] = (
            config.GYRO_NOISE**2 *
            np.eye(3)
        )

        Q[9:12,9:12] = (
            config.ACCEL_BIAS_NOISE**2 *
            np.eye(3)
        )

        Q[12:15,12:15] = (
            config.GYRO_BIAS_NOISE**2 *
            np.eye(3)
        )


        self.P = (
            F @ self.P @ F.T
            +
            Q * dt
        )


    def update(self, innovation, H, R):
        # Generic ESKF measurement update.

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Estimated error state
        dx = K @ innovation

        # Correct nominal state
        self.inject_error(dx)

        # Covariance update (Joseph form)
        I = np.eye(15)

        self.P = (
            (I - K @ H)
            @ self.P
            @ (I - K @ H).T
            +
            K @ R @ K.T
        )

    def update_hemisphere(self, radius):

        p = self.state.position

        distance = np.linalg.norm(p)

        if distance < 1e-8:
            return

        innovation = np.array([
            radius - distance
        ])

        H = np.zeros((1,15))
        H[0,0:3] = p / distance

        R = np.array([[0.001]])

        self.update(
            innovation,
            H,
            R
        )

    def constrain_velocity(self):

        p=self.state.position

        n=p/np.linalg.norm(p)

        self.state.velocity -= (
            np.dot(self.state.velocity,n)*n
        )

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