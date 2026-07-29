import argparse
import numpy as np

from eskf import ESKF
from IMUread import IMUCSVReader
from quaternion import Quaternion
from gui import create_gui
from serial_parser import Serial_Parser

import time


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