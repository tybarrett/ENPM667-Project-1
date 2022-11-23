"""inverse_kinematics.py - Calculates the desired behavior of the controllable variables."""


import jacobian
import gain_constants
import numpy
import math


def generate_controllable_variable_accelerations(state, end_effector_position, desired_ee_position,
                                                 desired_ee_rotation, desired_ee_velocity, desired_ee_accel):
    controlled_vars_deriv = numpy.Matrix((9, 1))
    controlled_vars_deriv[0:4, :] = numpy.Matrix([[state.vx], [state.vy], [state.vz], [state.rotational_velocity_yaw]])
    controlled_vars_deriv[4:, :] = numpy.Matrix(state.joint_velocities).transpose()

    uncontrolled_vars_deriv = numpy.Matrix([[state.rotational_velocity_pitch], [state.rotational_velocity_roll]])
    uncontrolled_vars_accel = numpy.Matrix([[state.rot_accel_pitch], [state.rot_accel_roll]])

    # The jacobian of controllable variables
    controllable_jac = jacobian.get_jacobian_of_controllable_variables(state.joint_positions, state, end_effector_position)

    pseudo_inv_controllable_jac = controllable_jac * controllable_jac.transpose()
    pseudo_inv_controllable_jac = controllable_jac.transpose() * pseudo_inv_controllable_jac.inv()

    derivative_of_controlled_jac = None # TODO
    derivative_of_unc_jac = None

    uncontrollable_jac = jacobian.get_jacobian_of_uncontrolled_variables(state, end_effector_position)

    e = _calculate_ee_error(desired_ee_position, state, desired_ee_rotation)

    ee_velocity = jacobian.get_current_jacobian(state.joint_positions) * state.joint_velocities
    ee_velocity_error = desired_ee_velocity - ee_velocity
    controllable_accelerations = pseudo_inv_controllable_jac * (desired_ee_accel + gain_constants.K_V * ee_velocity_error + gain_constants.K_P * e) - \
        pseudo_inv_controllable_jac * (derivative_of_controlled_jac * controlled_vars_deriv + uncontrollable_jac * uncontrolled_vars_accel + derivative_of_unc_jac * uncontrolled_vars_deriv)

    return controllable_accelerations


def _calculate_ee_error(desired_ee_position, state, desired_ee_rotation):
    e = numpy.Matrix((6, 1))
    e[0:3, :] = desired_ee_position - state.get_current_ee_position()

    mutual_orientation_matrix = desired_ee_rotation.transpose() * state.get_current_ee_rotation()

    # Convert to quaternion
    yaw = mutual_orientation_matrix[0]
    pitch = mutual_orientation_matrix[1]
    roll = mutual_orientation_matrix[2]
    qx = numpy.sin(roll / 2) * numpy.cos(pitch / 2) * numpy.cos(yaw / 2) - numpy.cos(roll / 2) * numpy.sin(
        pitch / 2) * numpy.sin(yaw / 2)
    qy = numpy.cos(roll / 2) * numpy.sin(pitch / 2) * numpy.cos(yaw / 2) + numpy.sin(roll / 2) * numpy.cos(
        pitch / 2) * numpy.sin(yaw / 2)
    qz = numpy.cos(roll / 2) * numpy.cos(pitch / 2) * numpy.sin(yaw / 2) - numpy.sin(roll / 2) * numpy.sin(
        pitch / 2) * numpy.cos(yaw / 2)
    normalize_factor = math.sqrt(qx**2 + qy**2 + qz**2)
    qx /= normalize_factor
    qy /= normalize_factor
    qz /= normalize_factor

    e[3:6, :] = desired_ee_position * numpy.Matrix([[qx], [qy], [qz]])

    return e
