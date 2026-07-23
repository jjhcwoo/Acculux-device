"""
imu_simulator.py

Simulated IMU data source for testing ESKF.

Outputs:
    acceleration [m/s^2]
    angular velocity [rad/s]
    timestep [s]

The simulated IMU measures:
    - gravity
    - motion acceleration
    - sensor noise
    - sensor bias

Author: Justin
"""

import numpy as np

from quaternion import Quaternion


class IMUSimulator:

    

    def __init__(self, sample_rate=10):
        self.true_quaternion = np.array([
            0.966,
            0.0,
            0.259,
            0.0
        ])

        self.true_velocity = np.zeros(3)
        self.true_position = np.zeros(3)

        """
        Initialize simulator.

        sample_rate:
            IMU frequency in Hz
        """

        self.dt = 1 / sample_rate

        self.time = 0


        # -----------------------------
        # True motion parameters
        # -----------------------------

        self.acceleration_world = np.zeros(3)

        self.angular_velocity = np.zeros(3)


        # -----------------------------
        # Sensor biases
        # -----------------------------

        self.accel_bias = np.array([
            0.05,
            -0.03,
            0.02
        ])

        self.gyro_bias = np.array([
            0.002,
            -0.001,
            0.003
        ])


        # -----------------------------
        # Noise parameters
        # -----------------------------

        self.accel_noise_std = 0.02

        self.gyro_noise_std = 0.001


        # Gravity vector

        self.gravity = np.array([
            0,
            0,
            9.81
        ])



    def motion_profile(self):
        """
        Defines the simulated probe motion.

        Returns:
            acceleration in world frame
            angular velocity

        """

        t = self.time


        # Start stationary
        if t < 1:

            acceleration = np.zeros(3)

            gyro = np.zeros(3)


        # Accelerate forward
        elif t < 3:

            acceleration = np.array([
                0.5,
                0,
                0
            ])

            gyro = np.array([
                0,
                0,
                0.3
            ])



        # Constant velocity
        elif t < 5:

            acceleration = np.zeros(3)

            gyro = np.zeros(3)


        # Stop
        else:

            acceleration = np.zeros(3)

            gyro = np.zeros(3)


        return acceleration, gyro



    def read(self):
        """
        Simulates IMU measurement.

        Returns:

            accel:
                [ax, ay, az] m/s^2

            gyro:
                [gx, gy, gz] rad/s

            dt:
                seconds

        """


        # True motion

        accel_world, gyro_true = self.motion_profile()

        # Update true velocity and position

        self.true_velocity += accel_world * self.dt

        self.true_position += (
            self.true_velocity * self.dt
            + 0.5 * accel_world * self.dt**2
        )
        
        self.true_quaternion = Quaternion.integrate_gyro(
        self.true_quaternion,
        gyro_true,
        self.dt
)


        # For now assume:
        # body frame = world frame
        #
        # Later we will rotate this using
        # the simulated quaternion

        R = Quaternion.to_rotation_matrix(
            self.true_quaternion
        )


        gravity = np.array([
            0,
            0,
            9.81
        ])


        accel_body = R.T @ (
            accel_world + gravity
        )


        gyro_body = gyro_true



        # Add sensor errors

        accel_measurement = (
            accel_body
            +
            self.accel_bias
            +
            np.random.normal(
                0,
                self.accel_noise_std,
                3
            )
        )


        gyro_measurement = (
            gyro_body
            +
            self.gyro_bias
            +
            np.random.normal(
                0,
                self.gyro_noise_std,
                3
            )
        )


        self.time += self.dt


        return (
            accel_measurement,
            gyro_measurement,
            self.dt
        )
    
    def get_true_orientation(self):
        return self.true_quaternion.copy()
    def get_true_position(self):
        return self.true_position.copy()