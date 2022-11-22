"""manipulator_control.py - determines the joint output forces based on inverse kinematics results."""


def calculate_joint_forces(inertia_matrix, position_control, attitude_control, joint_control, coriolis_matrix, gravity,
                           state):
    m_p_phi_transpose = inertia_matrix[3:6, 0:3]
    m_phi_q_transpose = inertia_matrix[6:, 3:6]
    m_q_q = inertia_matrix[6:, 6:]

    # TODO - disturbances as mentioned in equation (32)
    joint_torques = m_p_phi_transpose * position_control + m_phi_q_transpose * attitude_control + \
                    m_q_q * joint_control + coriolis_matrix * state.get_matrix_of_derivatives + gravity

    return joint_torques
