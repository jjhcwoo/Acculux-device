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
import breast

class ESKF:

    counter = 0
    r_imu0 = np.array([config.IMU0_OFFSET,
                      0.0,
                      config.PCB_OFFSET])
    
    r_imu1 = np.array([config.IMU1_OFFSET,
                      0.0,
                      config.PCB_OFFSET])

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
        self.P = np.eye(21)

        # Debug storage
        self.specific_force_samples = []


    def reset(self):
        # Reset filter to initial probe position

        # Called when probe is placed at nipple
        print(self.state.offset)
        self.state.reset()

        self.P = np.eye(21)
        # Position uncertainty
        self.P[0:3,0:3] *= 0.001

        # Velocity uncertainty
        self.P[3:6,3:6] *= 0.001

        # Orientation uncertainty
        self.P[6:9,6:9] *= 0.01

        # Bias uncertainty
        self.P[9:21,9:21] *= 0.001


    def predict(self, accel0, gyro0, accel1, gyro1, dt):
        # Prediction step

        # 1. Remove estimated sensor bias
        gyro0_corrected = (
            gyro0 -
            self.state.gyro0_bias
        )

        gyro1_corrected = (
            gyro1 - 
            self.state.gyro1_bias
        )

        accel0_corrected = (
            accel0 -
            self.state.accel0_bias -
            - 0 # dw r R ≈ 0
            - np.cross(gyro0_corrected,
                        np.cross(gyro0_corrected,
                                self.r_imu0))
        )

        accel1_corrected = (
            accel1 -
            self.state.accel1_bias -
            - 0 # dw r R ≈ 0
            - np.cross(gyro1_corrected,
                       np.cross(gyro1_corrected,
                                self.r_imu1))
        )

        gyro_corrected = (gyro0_corrected + gyro1_corrected) / 2
        accel_corrected = (accel0_corrected + accel1_corrected) / 2

        old_rotation = self.state.get_rotation_matrix()

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

        specific_force = accel_world + config.GRAVITY
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
            old_rotation,
            dt
        )

        if self.counter >= 1:
            position = breast.get_projection(self.state.quaternion)
            self.orientation_update(position)
            self.counter = 0
        self.counter += 1

    def predict_covariance(
        self,
        accel,
        gyro,
        rotation,
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

        R = rotation
        I = np.eye(3)
        F = np.eye(21)
        F[0:3, 3:6] = I * dt
        F[3:6, 6:9] = (-R @ Quaternion.skew(accel)) * dt
        F[3:6, 9:12] = -R / config.IMU_COUNT * dt
        F[3:6, 15:18] = -R / config.IMU_COUNT * dt
        F[6:9, 6:9] = np.transpose(Quaternion.from_vector_to_rotation(gyro * dt))
        F[6:9, 12:15] = -I / config.IMU_COUNT * dt
        F[6:9, 18:21] = -I / config.IMU_COUNT * dt

        Q = np.zeros((21, 21))
        Q[3:6, 3:6] = config.ACCEL_NOISE ** 2 / config.IMU_COUNT * dt ** 2 * np.eye(3)
        Q[6:9, 6:9] = config.GYRO_NOISE ** 2 / config.IMU_COUNT * dt ** 2 * np.eye(3)
        Q[9:12, 9:12] = config.ACCEL_BIAS_NOISE ** 2 * dt * np.eye(3)
        Q[12:15, 12:15] = config.GYRO_BIAS_NOISE ** 2 * dt * np.eye(3)
        Q[15:18, 15:18] = config.ACCEL_BIAS_NOISE ** 2 * dt * np.eye(3)
        Q[18:21, 18:21] = config.GYRO_BIAS_NOISE ** 2 * dt * np.eye(3)
        
        self.P = F @ self.P @ np.transpose(F) + Q

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
        I = np.eye(21)

        self.P = (
            (I - K @ H)
            @ self.P
            @ (I - K @ H).T
            +
            K @ R @ K.T
        )

    def orientation_update(self, position):
        # assume 0.1cm accuracy for prediction
        V = np.diag([0.01 ** 2, 0.01 ** 2, 0.01 ** 2])
        
        H = np.zeros((3, 21))
        H[0:3, 0:3] = np.eye(3)
        K = self.P @ np.transpose(H) @ np.linalg.inv(H @ self.P @ np.transpose(H) + V)
        dx = K @ (position - self.state.position)
        self.inject_error(dx)
        # poor stability reset
        self.P = (np.eye(21) - K @ H) @ self.P

    def constrain_velocity(self):
        '''
        p=self.state.position
        
        n=p/np.linalg.norm(p)

        self.state.velocity -= (np.dot(self.state.velocity,n)*n)
        '''
        p = self.state.position

        a = config.BREAST_A / 2
        b = config.BREAST_B / 2
        c = config.BREAST_C

        x, y, z = p

        # Ellipsoid surface normal
        n = np.array([
        x / (a * a),
        y / (b * b),
        z / (c * c)
        ])

        norm = np.linalg.norm(n)

        if norm < 1e-8:
            return

        n /= norm

        # Remove normal velocity component
        self.state.velocity -= np.dot(self.state.velocity, n) * n

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
        #self.state.accel_bias += dba
        #self.state.gyro_bias += dbg


    def get_state(self):
        # Return current estimated state

        return self.state