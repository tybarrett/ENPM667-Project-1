
import numpy
import gain_constants


def calculate_control_inputs(controllable_vars_accels, state, desired_yaw, desired_vel_yaw,
                             desired_link_positions, desired_link_velocities,
                             desired_position, desired_velocity):
    position_accel = controllable_vars_accels[:3, :]
    yaw_accel = controllable_vars_accels[3, :]
    link_accel = controllable_vars_accels[4:, :]

    yaw_control = yaw_accel + gain_constants.K_PSI_V * (desired_vel_yaw - state.rotational_velocity_yaw) + \
        gain_constants.K_PSI_P * (desired_yaw - state.yaw)
    q_control = link_accel + gain_constants.K_Q_V * (desired_link_velocities - numpy.Matrix(state.joint_velocities).transpose()) + \
        gain_constants.K_Q_P * (desired_link_positions - numpy.Matrix(state.joint_positions).transpose())

    current_velocity = numpy.Matrix([state.vx, state.vy, state.vz]).transpose()
    current_position = numpy.Matrix([state.x, state.y, state.z]).transpose()
    position_control = position_accel + gain_constants.K_P_V * (desired_velocity - current_velocity) + \
        gain_constants.K_P_P * (desired_position - current_position)

    return position_control, yaw_control, q_control