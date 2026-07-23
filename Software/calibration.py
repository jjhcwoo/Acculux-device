import numpy as np
from quaternion import Quaternion


class IMUCalibration:

    def __init__(self):
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.initial_quaternion = np.array([
            1.0,
            0.0,
            0.0,
            0.0
        ])


    def calibrate(
        self,
        measurements
    ):

        accel_values = []
        gyro_values = []


        for accel, gyro in measurements:

            accel_values.append(accel)
            gyro_values.append(gyro)


        accel_values = np.array(accel_values)
        gyro_values = np.array(gyro_values)


        # Gyro average is bias
        self.gyro_bias = np.mean(
            gyro_values,
            axis=0
        )


        # Accelerometer should measure gravity
        accel_average = np.mean(
            accel_values,
            axis=0
        )
        self.calculate_initial_orientation(
            accel_average
        )


        # Calculate initial orientation first

        self.initial_quaternion = (
            self.calculate_initial_orientation(
                accel_average
            )
        )
        



        # Rotate gravity into IMU frame

        R = Quaternion.to_rotation_matrix(
            self.initial_quaternion
        )


        gravity_world = np.array([
            0,
            0,
            9.81
        ])


        gravity_body = (
            R.T @ gravity_world
        )


        # Bias is remaining acceleration

        self.accel_bias = (
            accel_average - gravity_body
        )


        return (
            self.accel_bias,
            self.gyro_bias,
            self.initial_quaternion
        )
    
    def calculate_initial_orientation(self, accel_average):

        # Measured gravity direction in body frame

        g_body = accel_average / np.linalg.norm(accel_average)


        # Gravity direction in world frame

        g_world = np.array([
            0,
            0,
            1
        ])


        # Quaternion rotating g_body -> g_world

        cross = np.cross(
            g_body,
            g_world
        )

        dot = np.dot(
            g_body,
            g_world
        )


        q = np.zeros(4)


        q[0] = np.sqrt(
            (1 + dot) / 2
        )


        q[1:] = (
            cross /
            (2*q[0])
        )


        q = q / np.linalg.norm(q)

        return q



    # def calculate_initial_orientation(
    #         self,
    #         accel_average
    # ):

    #     # Normalize measured gravity direction

    #     measured_gravity = (
    #         accel_average /
    #         np.linalg.norm(accel_average)
    #     )


    #     # Desired world gravity direction

    #     world_gravity = np.array([
    #         0,
    #         0,
    #         1
    #     ])


    #     # Find rotation between vectors

    #     v = np.cross(
    #         measured_gravity,
    #         world_gravity
    #     )


    #     c = np.dot(
    #         measured_gravity,
    #         world_gravity
    #     )


    #     # Handle case where vectors are parallel

    #     if np.linalg.norm(v) < 1e-8:

    #         self.initial_quaternion = np.array([
    #             1,
    #             0,
    #             0,
    #             0
    #         ])

    #         return self.initial_quaternion


    #     # Quaternion from axis-angle

    #     s = np.sqrt(
    #         (1+c)*2
    #     )


    #     q = np.array([
    #         s/2,
    #         v[0]/s,
    #         v[1]/s,
    #         v[2]/s
    #     ])


    #     self.initial_quaternion = q / np.linalg.norm(q)


    #     return self.initial_quaternion