"""
imu_csv_reader.py

Reads real IMU data from a CSV file and serves it row-by-row,
mimicking the IMUSimulator interface (accel, gyro, dt) so it can be
dropped into existing ESKF code without changing predict()/main loop.

Expected CSV columns:
    ACC1X, ACC1Y, ACC1Z, GYR1X, GYR1Y, GYR1Z

Author: Justin
"""

import numpy as np
import pandas as pd


class IMUCSVReader:

    REQUIRED_COLUMNS = [
        "ACC1X", "ACC1Y", "ACC1Z",
        "GYR1X", "GYR1Y", "GYR1Z"
    ]

    def __init__(self, csv_path, sample_rate=None, timestamp_column=None):
        """
        Initialize reader.

        Inputs:
            csv_path:
                Path to CSV file.

            sample_rate:
                IMU frequency in Hz. Use this if the CSV has no
                timestamp column and rows are evenly spaced in time.

            timestamp_column:
                Name of a column containing timestamps (seconds).
                If given, dt is computed from consecutive timestamp
                differences instead of a fixed sample_rate.

        Exactly one of sample_rate or timestamp_column must be given.
        """

        if (sample_rate is None) == (timestamp_column is None):
            raise ValueError(
                "Provide exactly one of sample_rate or timestamp_column"
            )

        self.data = pd.read_csv(csv_path)

        missing = [
            c for c in self.REQUIRED_COLUMNS
            if c not in self.data.columns
        ]
        if missing:
            raise ValueError(f"CSV missing required columns: {missing}")

        self.accel = self.data[
            ["ACC1X", "ACC1Y", "ACC1Z"]
        ].to_numpy(dtype=float)

        self.gyro = self.data[
            ["GYR1X", "GYR1Y", "GYR1Z"]
        ].to_numpy(dtype=float)

        self.timestamp_column = timestamp_column

        if timestamp_column is not None:
            if timestamp_column not in self.data.columns:
                raise ValueError(
                    f"timestamp_column '{timestamp_column}' not in CSV"
                )
            self.timestamps = self.data[timestamp_column].to_numpy(dtype=float)
            self.fixed_dt = None
        else:
            self.timestamps = None
            self.fixed_dt = 1.0 / sample_rate

        self.n_rows = len(self.data)
        self.index = 0

    def has_next(self):
        """
        True if there is another row left to read.
        """

        return self.index < self.n_rows

    def read(self):
        """
        Returns one row of IMU data, same shape as IMUSimulator.read():

            accel:
                [ax, ay, az] m/s^2

            gyro:
                [gx, gy, gz] rad/s

            dt:
                seconds since previous row
        """

        if not self.has_next():
            raise StopIteration("No more IMU data in CSV")

        accel = self.accel[self.index] * 9.81      # g -> m/s²
        gyro = np.deg2rad(self.gyro[self.index])   # deg/s -> rad/s

        if self.timestamps is not None:
            if self.index == 0:
                # No previous row to diff against; assume the gap
                # to the second sample as a stand-in for the first dt.
                dt = (
                    self.timestamps[1] - self.timestamps[0]
                    if self.n_rows > 1 else 0.0
                )
            else:
                dt = self.timestamps[self.index] - self.timestamps[self.index - 1]
        else:
            dt = self.fixed_dt

        self.index += 1

        return accel, gyro, dt

    def reset(self):
        """
        Reset reader back to the first row.
        """

        self.index = 0