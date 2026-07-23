import numpy as np
import matplotlib.pyplot as plt

from simulation import IMUSimulator
from eskf import ESKF
from calibration import IMUCalibration


# ==================================
# Configuration
# ==================================

sample_rate = 10          # Hz
calibration_time = 1      # seconds
simulation_time = 6       # seconds


# ==================================
# Initialize system
# ==================================

imu = IMUSimulator(
    sample_rate=sample_rate
)

eskf = ESKF()


# ==================================
# Calibration phase
# ==================================

print("Starting calibration...")


calibration_data = []


calibration_steps = int(
    calibration_time * sample_rate
)


for i in range(calibration_steps):

    accel, gyro, dt = imu.read()

    calibration_data.append(
        (accel, gyro)
    )


# Estimate biases

calibrator = IMUCalibration()


accel_bias, gyro_bias, initial_quaternion = calibrator.calibrate(
    calibration_data
)

print("Calibration quaternion:")
print(initial_quaternion)

print("True simulator quaternion:")
print(imu.get_true_orientation())


print("\nCalibration complete")

print("Estimated accelerometer bias:")
print(accel_bias)

print("\nTrue accelerometer bias:")
print(imu.accel_bias)


print("\nEstimated gyro bias:")
print(gyro_bias)

print("\nTrue gyro bias:")
print(imu.gyro_bias)

print("Average accelerometer:")
print(np.mean(
    np.array([x[0] for x in calibration_data]),
    axis=0
))


from quaternion import Quaternion
R = Quaternion.to_rotation_matrix(
    initial_quaternion
)

gravity = np.array([
    0,
    0,
    9.81
])


print("\nGravity rotated using calibration quaternion:")
print(R.T @ gravity)

print("\nMeasured gravity:")
print(
    np.mean(
        np.array([x[0] for x in calibration_data]),
        axis=0
    )
)


# Apply calibration to ESKF

eskf.state.accel_bias = accel_bias
eskf.state.gyro_bias = gyro_bias
eskf.state.quaternion = initial_quaternion



# ==================================
# Tracking phase
# ==================================

print("\nStarting tracking...")


estimated_position = []
true_position = []

time = []


steps = int(
    simulation_time * sample_rate
)


for i in range(steps):

    accel, gyro, dt = imu.read()


    # ESKF prediction

    eskf.predict(
        accel,
        gyro,
        dt
    )


    # Store estimated position

    estimated_position.append(
        eskf.state.position.copy()
    )


    # Store true position

    true_position.append(
        imu.get_true_position()
    )


    time.append(
        i * dt
    )



# Convert to numpy arrays

estimated_position = np.array(
    estimated_position
)

true_position = np.array(
    true_position
)

time = np.array(time)



# ==================================
# Results
# ==================================

print("\nFinal Results")

print("\nTrue position:")
print(true_position[-1])


print("\nEstimated position:")
print(estimated_position[-1])


print("\nPosition error:")
print(
    estimated_position[-1]
    -
    true_position[-1]
)



# ==================================
# Plot XYZ trajectory
# ==================================

fig = plt.figure()

ax = fig.add_subplot(
    111,
    projection="3d"
)


ax.plot(
    true_position[:,0],
    true_position[:,1],
    true_position[:,2],
    label="True"
)


ax.plot(
    estimated_position[:,0],
    estimated_position[:,1],
    estimated_position[:,2],
    label="Estimated"
)


ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")


ax.legend()

plt.show()



# ==================================
# Plot X position over time
# ==================================

plt.figure()


plt.plot(
    time,
    true_position[:,0],
    label="True X"
)


plt.plot(
    time,
    estimated_position[:,0],
    label="Estimated X"
)


plt.xlabel("Time (s)")
plt.ylabel("X Position (m)")


plt.grid()
plt.legend()

plt.show()