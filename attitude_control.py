"""attitude_control.py - Calculates control input values given desired roll and pitch angles"""


import numpy
import gain_constants
import transformation_matrices


def generate_attitude_control_input_estimates(desired_roll,
                                              desired_roll_deriv,
                                              desired_pitch,
                                              desired_pitch_deriv,
                                              robot_state):

    pitch_control_estimate = gain_constants.K_THETA_V * (desired_pitch_deriv - robot_state.rotational_velocity_pitch) + \
                             gain_constants.K_THETA_P * (desired_pitch - robot_state.pitch)

    roll_control_estimate = gain_constants.K_PHI_V * (desired_roll_deriv - robot_state.rotational_velocity_roll) + \
                            gain_constants.K_PHI_P * (desired_roll - robot_state.roll)

    return pitch_control_estimate, roll_control_estimate


def calculate_vehicle_torques(inertia_matrix, position_controls, rotational_control, joint_control_estimate, g, state):

    M_p_phi_transpose = inertia_matrix[3:6, 0:3]
    M_phi_phi = inertia_matrix[3:6, 3:6]
    M_phi_q = inertia_matrix[6:, 3:6]

    g_phi = g[3:6, :]

    u_mu = M_p_phi_transpose*position_controls + M_phi_phi*rotational_control + M_phi_q*joint_control_estimate + g_phi
    # TODO - include the C matrix in the above equation (as seen in equation 30)

    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    euler_deriv_to_angular_vel_transform = transformation_matrices.get_angular_velocity_transformation_matrix(state)

    vehicle_torques = rotation_matrix.transpose() * euler_deriv_to_angular_vel_transform.transpose().inv() * u_mu

    return vehicle_torques
