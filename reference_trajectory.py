"""reference_trajectory.py - Given a point in time, generates the desired velocity of the end effector."""

import math
import numpy
import sympy

t = sympy.symbols("t")

def generate_reference_trajectory(time_value):

    if time_value <= 55:
        z_position = t * 0.1
        x_position = sympy.sin(3.5 * (t / 55) * 2*sympy.pi)
        y_position = -sympy.cos((t / 55) * 2*sympy.pi) - sympy.cos(2 * (t / 55) * 2*sympy.pi)

        desired_position = [x_position.subs([t, time_value]),
                            y_position.subs([t, time_value]),
                            z_position.subs([t, time_value])]
        desired_velocity = [sympy.diff(x_position, t).subs([t, time_value]),
                            sympy.diff(y_position, t).subs([t, time_value]),
                            sympy.diff(z_position, t).subs([t, time_value])]
        desired_accel = [sympy.diff(x_position, t, 2).subs([t, time_value]),
                         sympy.diff(y_position, t, 2).subs([t, time_value]),
                         sympy.diff(z_position, t, 2).subs([t, time_value])]
        return desired_position, desired_velocity, desired_accel

    else:
        null_matrix = numpy.zeros((3, 1))
        return null_matrix, null_matrix, null_matrix
