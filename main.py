"""main.py - executes the simulation as described in the paper."""

import numpy
import noise_generator


TIME_RESOLUTION_S = 1 / 250
END_TIME_S = 60


def main():

    t = 0
    while t <= END_TIME_S:
        print(t)

        t += TIME_RESOLUTION_S


if __name__ == '__main__':
    main()
