import argparse
import numpy as np

from eskf import ESKF
from quaternion import Quaternion
from gui import create_gui
from serial_parser import Serial_Parser

import time
import config


def run():
    app, window = create_gui()

    filt0 = ESKF(config.IMU0_OFFSET)
    filt1 = ESKF(config.IMU1_OFFSET)

    parser = Serial_Parser(filt0, filt1, window)

    parser.status_signal.connect(window.show_status_popup)
    parser.connection_signal.connect(window.update_connection_status)
    window.bottom.calibrateButton.clicked.connect(parser.request_calibration)

    app.processEvents()
    app.exec()

def main():
    run()

if __name__ == "__main__":
    main()