"""reference_trajectory.py - Given a point in time, generates the desired velocity of the end effector."""

import math
import numpy


def generate_reference_trajectory(time_value):

    if time_value <= 55:
        z_velocity = 0.1
        x_velocity = math.cos(3.5 * (time_value / 55) * 2*math.pi)
        y_velocity = math.sin((time_value / 55) * 2*math.pi) + math.sin(2 * (time_value / 55) * 2*math.pi)

        return numpy.Matrix([[x_velocity], [y_velocity], [z_velocity]])

    else:
        return numpy.zeros((3, 1))