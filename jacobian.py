"""jacobian.py - contains methods pertaining to the Jacobian and its use."""

import numpy
import sympy

import transformation_matrices


q1, q2, q3, q4, q5 = sympy.symbols("q1, q2, q3, q4, q5")
GENERIC_JACOBIAN = sympy.Matrix((6, 6)) # TODO - create this based on properties of the arm


def _generate_skew_matrix(a):
    skew_matrix = numpy.matrix([[0, -a[2, 0], a[1, 0]],
                                [a[2, 0], 0, -a[0, 0]],
                                [-a[1, 0], a[0, 0], 0]])
    return skew_matrix


def get_effector_velocity_in_base_frame(arm_state, joint_velocities):
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    effector_velocity = instantaneous_jacobian * joint_velocities
    return effector_velocity


def get_effector_velocity_in_world_frame(state, end_effector_position, arm_state, joint_velocities):
    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix

    j_eb = numpy.zeros(6)
    j_eb[0:3, 0:3] = rotation_matrix
    j_eb[3:6, 3:6] = rotation_matrix
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    j_eb = j_eb * instantaneous_jacobian

    base_frame_velocity = sympy.Matrix([[state.vx], [state.vy], [state.vz], [state.vyaw], [state.vpitch], [state.vroll]])
    effector_velocity_in_world_frame = j_b * base_frame_velocity + j_eb * joint_velocities
    return effector_velocity_in_world_frame


def get_end_effector_velocity(state, end_effector_position, arm_state):
    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix

    t_a = numpy.eye(6)
    t_of_phi = sympy.Matrix([[0, -sympy.sin(state.yaw), sympy.cos(state.yaw) * sympy.cos(state.pitch)],
                             [0, sympy.cos(state.yaw), sympy.sin(state.yaw) * sympy.cos(state.pitch)],
                             [1, 0, -1 * sympy.sin(state.pitch)]])
    t_a[3:6, 3:6] = t_of_phi

    j_b_t_a = j_b * t_a
    j_sigma = j_b_t_a[:, -2:]

    j_nu = j_b_t_a[:, :4]

    j_eb = numpy.zeros(6)
    j_eb[0:3, 0:3] = rotation_matrix
    j_eb[3:6, 3:6] = rotation_matrix
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    j_eb = j_eb * instantaneous_jacobian

    j_epsilon = numpy.zeros((6, 10))
    j_epsilon[:, -6:] = j_eb
    j_epsilon[:, :4] = j_nu

    controllable_variable_velocities = numpy.matrix([[state.vx], [state.vy], [state.vz], [state.vyaw]])
    uncontrollable_variable_velocities = numpy.matrix([[state.vpitch], [state.vroll]])
    end_effector_velocity = j_epsilon * controllable_variable_velocities + j_sigma * uncontrollable_variable_velocities

    return end_effector_velocity


if __name__ == '__main__':
    n = numpy.matrix([[1], [2], [3]])
    print(_generate_skew_matrix(n))
