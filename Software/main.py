# Usage:
#     python main.py Data.csv --sample-rate 250

import argparse
import numpy as np

from eskf import ESKF
from IMUread import IMUCSVReader
from quaternion import Quaternion
from gui import create_gui
from serial_parser import Serial_Parser

import time


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

def run(csv_path = None, sample_rate=None, timestamp_column=None):
    app, window = create_gui()

    filt = ESKF()

    Serial_Parser(filt, window)

    app.processEvents()
    app.exec()



def main():

    run()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()