import threading
import serial
from config import PORT, BAUD, ACC_FS, GYRO_FS, GRAVITY, SAMPLE_PERIOD
import time
from eskf import ESKF
from quaternion import Quaternion
from PyQt6.QtCore import QObject, pyqtSignal
from force import Force
from hardware import find_probe_port

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

class Serial_Parser(QObject):
    status_signal = pyqtSignal(str)
    connection_signal = pyqtSignal(bool)

    def __init__(self, filter, window):
        super().__init__()
        port = find_probe_port()
        self.serial_port = serial.Serial(port, BAUD, timeout = None)
        self.connection_signal.emit(True)
        self.window = window
        self.filter = filter
        window.filter = filter
        self.calibration_requested = False
        self.force = Force()

        # dirty way of flushing the serial port
        for i in range(0, 100):
            s = self.serial_port.readline()
        thread = threading.Thread(target=self.read_from_port, args=(window,), daemon=True)
        thread.start()

    def request_calibration(self):
        self.calibration_requested = True

    # calibration
    def calibrate(self):

        self.status_signal.emit("Calibration started. Pleast place the probe on a flat surface.")
        print("Calibrate button pressed")
    
        samples = []
        forceTotal = []
        for i in range(0, 1000):
            s = np.array(self.serial_port.readline().decode().strip('\r\n').split(','), dtype=float)
            samples.append(s[0:4])

        samples = np.array(samples)

        self.force.capture_zero(samples)
        time.sleep(3)

        samples2 = []
        calibration_values = []
        for i in range(0, 1000):
            s = np.array(self.serial_port.readline().decode().strip('\r\n').split(','), dtype=float)
            samples2.append(s[0:4])
            s[4:7] = s[4:7] * ACC_FS / 32768
            s[7:10] = s[7:10] * GYRO_FS / 32768
            s[10:13] = s[10:13] * ACC_FS / 32768
            s[13:16] = s[13:16] * GYRO_FS / 32768
            calibration_values.append(s[4:16])

        samples2 = np.array(samples2)
        self.force.calculate(samples2, known_force = 1.102)

        bias_values = np.mean(calibration_values, axis=0)
        bias_values[0:3] = bias_values[0:3] + GRAVITY
        bias_values[6:9] = bias_values[6:9] + GRAVITY

        print("Calibration complete. Bias values:")
        print(bias_values)

        self.filter.state.set_accel_bias(bias_values[0:3])
        self.filter.state.set_gyro_bias(bias_values[3:6])

        # safe to assume that probe is on a flat surface
        q0 = Quaternion.from_accel([0, 0, -9.81])
        self.filter.state.set_quaternion(q0)
        self.status_signal.emit("Calibration complete")

        # print(quaternion_to_euler(self.filter.state.quaternion))

    def read_from_port(self, window):

        last_print = time.time()
        while True:
            if self.calibration_requested:
                self.calibrate()
                self.calibration_requested = False
                continue

            try:
                # Read one complete serial line
                line = self.serial_port.readline().decode(errors="ignore").strip()

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
                raw_force = s[0:4]
                s[4:7] = s[4:7] * ACC_FS / 32768
                s[7:10] = s[7:10] * GYRO_FS / 32768
                s[10:13] = s[10:13] * ACC_FS / 32768
                s[13:16] = s[13:16] * GYRO_FS / 32768
           
                if window.reset_request:
                    self.filter.reset()
                    window.reset_request = False

                if window.scanning:
                    self.filter.predict(s[4:7], s[7:10], SAMPLE_PERIOD)

                    self.filter.update_hemisphere(radius = 0.15)
                    self.filter.constrain_velocity()
                    window.latest_force = self.force.convert(raw_force)

                # if time.time() - last_print > 1.0:
                #     print("Position:", state.position)
                #     print("Quaternion:", state.quaternion)
                #     print("Euler:", quaternion_to_euler(state.quaternion))
                #     print()
                #     last_print = time.time()



                state = self.filter.get_state()

                if time.time() - last_print > 1.0:
                    print("Position:", state.position)
                    print("Velocity:", state.velocity)
                    print("Speed:", np.linalg.norm(state.velocity))
                    print()
                    last_print = time.time()
                
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
                self.connection_signal.emit(False)
                break

            except ValueError as e:
                print("Invalid serial packet:", e)
                continue

            except UnicodeDecodeError as e:
                print("Serial decode error:", e)
                continue

            except serial.SerialException:
                self.connection_signal.emit(False)
                while True:
                    try:
                        time.sleep(1)
                        
                        port = find_probe_port()
                        self.serial_port = serial.Serial(port, BAUD, timeout=None)

                        self.connection_signal.emit(True)
                        break

                    except serial.SerialException:
                        continue