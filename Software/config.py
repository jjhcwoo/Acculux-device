import numpy as np

IMU_COUNT = 2

GRAVITY = np.array([0.0, 0.0, 9.80655])

ACCEL_NOISE = 0.0070 * 9.80655

GYRO_NOISE = np.deg2rad(0.0028)

#ACCEL_BIAS_NOISE = 0.001

#GYRO_BIAS_NOISE = np.deg2rad(0.01)

ACCEL_BIAS_NOISE = 0.001

GYRO_BIAS_NOISE = np.deg2rad(0.01)

IMU0_OFFSET = 0.035922

IMU1_OFFSET = -0.035922

IMU_DISTANCE = 0.071844

PCB_OFFSET = 0.01425

# Investigate the cause of half angle rotations. Suspect it is related to IMU code not sampling properly.
SAMPLE_RATE = 500

SAMPLE_PERIOD = 1 / SAMPLE_RATE

SCAN_TIME = 10.0

PORT = 'COM4'

BAUD = 115200

ACC_FS = 16 * 9.80655

GYRO_FS = np.deg2rad(2000)

# Width
BREAST_A = 0.12

# Height
BREAST_B = 0.11

# Projection
BREAST_C = 0.03