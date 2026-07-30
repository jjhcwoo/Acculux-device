import numpy as np


class Force:

    def __init__(self):

        self.zero_values = None
        self.gain = None
        self.offset = None

        # Low-pass filter state
        self.filtered_force = 0.0
        self.alpha = 0.2

    def capture_zero(self, data):

        # data shape:
        # [FSR1, FSR2, FSR3, FSR4]

        self.zero_values = np.mean(
            data,
            axis=0
        )

        print("Zero force:")
        print(self.zero_values)



    def calculate(self, loaded_data, known_force):
        force_per_sensor = known_force / 4
        loaded_values = np.mean(
            loaded_data,
            axis=0
        )
        print("Known force:")
        print(loaded_values)

        self.gain = (
            force_per_sensor /
            (loaded_values - self.zero_values)
        )
        self.gain[2] *= -1

        self.offset = (
            -self.gain *
            self.zero_values
        )

        print("Force gains:")
        print(self.gain)

        print("Force offsets:")
        print(self.offset)



    def convert(self, raw):

        if self.gain is None:
            return 0.0
        force = self.gain * raw + self.offset
        force[force < 0] = 0

        # Low-pass filter
        self.filtered_force = (self.alpha * force + (1 - self.alpha) * self.filtered_force)

        totalForce = np.sum(force)
        return totalForce