"""
main.py

Runs the ESKF over IMU data pulled from a CSV file.

Usage:
    python main.py path/to/data.csv --sample-rate 100
    python main.py path/to/data.csv --timestamp-column TIME

Author: Justin
"""

import argparse
import numpy as np

from eskf import ESKF
from IMUread import IMUCSVReader
from quaternion import Quaternion
from gui import create_gui


def quaternion_to_euler(q):
    """
    Convert quaternion [w,x,y,z] to Euler angles (degrees).

    Returns:
        roll, pitch, yaw in degrees
    """

    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = np.arctan2(
        sinr_cosp,
        cosr_cosp
    )

    # Pitch (y-axis rotation)
    sinp = 2 * (w*y - z*x)

    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi/2
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = np.arctan2(
        siny_cosp,
        cosy_cosp
    )

    return np.degrees([
        roll,
        pitch,
        yaw
    ])


def run(csv_path, sample_rate=None, timestamp_column=None):
    app, window = create_gui()

    imu = IMUCSVReader(
        csv_path,
        sample_rate=sample_rate,
        timestamp_column=timestamp_column
    )

    filt = ESKF()


    # ---------------------------------------
    # Sensor bias values
    # ---------------------------------------

    filt.state.set_accel_bias([
        0.005,
        0.001,
        -0.058
    ])

    filt.state.set_gyro_bias([
        -0.00071,
        -0.01172,
        0.00289
    ])


    # ---------------------------------------
    # Initial orientation calibration
    # ---------------------------------------

    print("Calibrating initial orientation...")

    accel_samples = []

    calibration_samples = 200

    for i in range(calibration_samples):

        accel, gyro, dt = imu.read()

        accel_samples.append(accel)


    accel_average = np.mean(
        accel_samples,
        axis=0
    )


    q0 = Quaternion.from_accel(
        accel_average
    )

    filt.state.set_quaternion(q0)

    R = filt.state.get_rotation_matrix()

    print("\nGravity check:")

    g_body = accel_average

    g_world = R @ g_body

    print("Measured accel:")
    print(g_body)

    print("Rotated accel:")
    print(g_world)

    print("Gravity magnitude:")
    print(np.linalg.norm(g_world))


    print("Initial accelerometer:")
    print(accel_average)

    print("\nInitial quaternion:")
    print(q0)

    roll, pitch, yaw = quaternion_to_euler(q0)

    print("\nInitial orientation:")
    print(f"Roll:  {roll:.2f} deg")
    print(f"Pitch: {pitch:.2f} deg")
    print(f"Yaw:   {yaw:.2f} deg")


    positions = []
    orientations = []


    sample = 0
    gui_update_rate = 10


    while imu.has_next() and sample < 2100:

        accel, gyro, dt = imu.read()


        filt.predict(
            accel,
            gyro,
            dt
        )


        state = filt.get_state()

        roll, pitch, yaw = quaternion_to_euler(
            state.quaternion
        )

        positions.append(
            state.position.copy()
        )

        orientations.append(
            state.quaternion.copy()
        )


        # Update GUI point
        window.plot3D.update_orientation(
            roll,
            pitch
        )

        if sample % gui_update_rate == 0:
            window.bottom.sensorPanel.update_angles(
                roll,
                pitch,
                yaw
            )


        # Keep GUI alive
        app.processEvents()

        if sample % 100 == 0:
            print(f"\nSample {sample}")

            print(
                "Position:",
                state.position
            )

            print("Orientation:")
            print(f"Roll:  {roll:.2f}")
            print(f"Pitch: {pitch:.2f}")
            print(f"Yaw:   {yaw:.2f}")


        sample += 1

    print("\nAverage specific force:")
    print(
        np.mean(
            filt.specific_force_samples,
            axis=0
        )
    )

    app.exec()
    return positions, orientations



def main():

    parser = argparse.ArgumentParser(
        description="Run ESKF over CSV IMU data"
    )

    parser.add_argument(
        "csv_path",
        help="Path to IMU CSV file"
    )


    rate_group = parser.add_mutually_exclusive_group(
        required=True
    )

    rate_group.add_argument(
        "--sample-rate",
        type=float,
        help="Fixed IMU sample rate in Hz"
    )

    rate_group.add_argument(
        "--timestamp-column",
        type=str,
        help="Name of timestamp column"
    )


    args = parser.parse_args()


    positions, orientations = run(
        args.csv_path,
        sample_rate=args.sample_rate,
        timestamp_column=args.timestamp_column
    )


    print("\nProcessed samples:")
    print(len(positions))


    print("\nFinal position:")
    print(positions[-1])


    print("\nFinal orientation quaternion:")
    print(orientations[-1])


    roll, pitch, yaw = quaternion_to_euler(
        orientations[-1]
    )

    print("\nFinal orientation:")
    print(f"Roll:  {roll:.2f} deg")
    print(f"Pitch: {pitch:.2f} deg")
    print(f"Yaw:   {yaw:.2f} deg")



if __name__ == "__main__":
    main()