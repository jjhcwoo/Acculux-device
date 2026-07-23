"""
quaternion.py

Quaternion utilities for IMU orientation tracking.

Quaternion convention:
    q = [w, x, y, z]

Author: Justin
"""

import numpy as np


class Quaternion:

    @staticmethod
    def normalize(q):
        """
        Normalize quaternion.
        """
        q = np.asarray(q, dtype=float)
        return q / np.linalg.norm(q)

    @staticmethod
    def conjugate(q):
        """
        Quaternion conjugate.
        """
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

    @staticmethod
    def inverse(q):
        """
        Quaternion inverse.
        """
        q = np.asarray(q, dtype=float)
        return Quaternion.conjugate(q) / np.dot(q, q)

    @staticmethod
    def multiply(q1, q2):
        """
        Hamilton quaternion multiplication.

        q = q1 ⊗ q2
        """

        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,

            w1*x2 + x1*w2 + y1*z2 - z1*y2,

            w1*y2 - x1*z2 + y1*w2 + z1*x2,

            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def from_axis_angle(axis, angle):
        """
        Create quaternion from axis-angle.

        axis : length-3 vector
        angle : radians
        """

        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)

        half = angle / 2

        s = np.sin(half)

        return Quaternion.normalize(np.array([
            np.cos(half),
            axis[0]*s,
            axis[1]*s,
            axis[2]*s
        ]))

    @staticmethod
    def to_rotation_matrix(q):
        """
        Convert quaternion to 3x3 rotation matrix.
        """

        q = Quaternion.normalize(q)

        w, x, y, z = q

        return np.array([

            [1 - 2*(y*y + z*z),
             2*(x*y - z*w),
             2*(x*z + y*w)],

            [2*(x*y + z*w),
             1 - 2*(x*x + z*z),
             2*(y*z - x*w)],

            [2*(x*z - y*w),
             2*(y*z + x*w),
             1 - 2*(x*x + y*y)]

        ])

    @staticmethod
    def rotate_vector(q, v):
        """
        Rotate vector v using quaternion q.
        """

        R = Quaternion.to_rotation_matrix(q)

        return R @ np.asarray(v)
    
    @staticmethod
    def from_small_angle(delta_theta):
        """
        Convert a small rotation vector into
        a quaternion correction.

        Parameters:
            delta_theta: np.array([rx, ry, rz])

        Returns:
            quaternion [w, x, y, z]
        """

        dq = np.zeros(4)

        dq[0] = 1.0

        dq[1:] = 0.5 * delta_theta

        # Normalize to avoid drift

        dq = dq / np.linalg.norm(dq)

        return dq

    @staticmethod
    def integrate_gyro(q, gyro, dt):
        """
        Integrate quaternion using gyroscope.

        gyro:
            angular velocity (rad/s)

        dt:
            timestep (s)
        """

        gyro = np.asarray(gyro, dtype=float)

        omega = np.linalg.norm(gyro)

        if omega < 1e-10:
            return Quaternion.normalize(q)

        axis = gyro / omega

        angle = omega * dt

        dq = Quaternion.from_axis_angle(axis, angle)

        q_new = Quaternion.multiply(q, dq)

        return Quaternion.normalize(q_new)

    @staticmethod
    def from_accel(accel):
        """
        Estimate roll and pitch from gravity.
        Assumes accel is body-frame [ax, ay, az].
        """

        ax, ay, az = accel

        roll = np.arctan2(
            ay,
            az
        )

        pitch = np.arctan2(
            -ax,
            np.sqrt(ay*ay + az*az)
        )

        # yaw is unknowable from accelerometer
        yaw = 0.0

        cr = np.cos(roll/2)
        sr = np.sin(roll/2)
        cp = np.cos(pitch/2)
        sp = np.sin(pitch/2)
        cy = np.cos(yaw/2)
        sy = np.sin(yaw/2)

        return np.array([
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        ])