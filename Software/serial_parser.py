import threading
import serial
from config import PORT, BAUD, ACC_FS, GYRO_FS, GRAVITY, SAMPLE_PERIOD
import time
from eskf import ESKF
from quaternion import Quaternion

import numpy as np

import state

def quaternion_to_euler(q):
    # Convert quaternion [w,x,y,z] to Euler angles (degrees)

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

class Serial_Parser:

    def __init__(self, filter, window):
        serial_port = serial.Serial(PORT, BAUD, timeout = None)
        self.window = window

        # dirty way of flushing the serial port
        for i in range(0, 100):
            s = serial_port.readline()

        # calibration
        calibration_values = []
        for i in range(0, 1000):
            s = np.array(serial_port.readline().decode().strip('\r\n').split(','), dtype=float)
            s[4:7] = s[4:7] * ACC_FS / 32768
            s[7:10] = s[7:10] * GYRO_FS / 32768
            s[10:13] = s[10:13] * ACC_FS / 32768
            s[13:16] = s[13:16] * GYRO_FS / 32768
            calibration_values.append(s[4:16])


        bias_values = np.mean(calibration_values, axis=0)
        bias_values[0:3] = bias_values[0:3] + GRAVITY
        bias_values[6:9] = bias_values[6:9] + GRAVITY

        filter.state.set_accel_bias(bias_values[0:3])
        filter.state.set_gyro_bias(bias_values[3:6])

        # safe to assume that probe is on a flat surface

        q0 = Quaternion.from_accel([0, 0, -9.81])
        filter.state.set_quaternion(q0)

        print(quaternion_to_euler(filter.state.quaternion))

        thread = threading.Thread(target=self.read_from_port, args=(serial_port, filter, window), daemon=True)
        thread.start()

    def read_from_port(self, serial_port, filter, window):

        last_print = time.time()
        while True:
            try:
                # Read one complete serial line
                line = serial_port.readline().decode(errors="ignore").strip()

                # Ignore blank lines
                if not line:
                    continue

                # Split into values
                values = line.split(',')

                # Make sure packet has the expected number of values
                if len(values) != 16:
                    # print(f"Bad packet ({len(values)} values): {repr(line)}")
                    continue

                # Convert to numpy array
                s = np.array(values, dtype=float)

                s[4:7] = s[4:7] * ACC_FS / 32768
                s[7:10] = s[7:10] * GYRO_FS / 32768
                s[10:13] = s[10:13] * ACC_FS / 32768
                s[13:16] = s[13:16] * GYRO_FS / 32768
                filter.predict(s[4:7], s[7:10], SAMPLE_PERIOD)

                filter.update_hemisphere(
                    radius = 0.15  # example radius in metres
                )
                filter.constrain_velocity()
                print(
                    filter.specific_force_samples[-1]
                )

                if time.time() - last_print > 1.0:
                    print("Position:", state.position)
                    print("Quaternion:", state.quaternion)
                    print("Euler:", quaternion_to_euler(state.quaternion))
                    print()
                    last_print = time.time()



                state = filter.get_state()
                
                roll, pitch, yaw = quaternion_to_euler(
                    state.quaternion
                )

                window.latest_angles = np.array([
                    roll,
                    pitch,
                    yaw
                ])

                window.latest_position = state.position.copy()
                
                window.plot3D.update_orientation(
                    roll,
                    pitch
                )

            # Bandaid fix, will have to use GUI signals in PyQt
            except RuntimeError:
                print("GUI closed")
                break

            except ValueError as e:
                print("Invalid serial packet:", e)
                continue

            except UnicodeDecodeError as e:
                print("Serial decode error:", e)
                continue

            except Exception as e:
                print("Unexpected serial parser error:", e)
                continue