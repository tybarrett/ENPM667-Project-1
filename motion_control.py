

import gain_constants


def calculate_control_inputs(controllable_vars_accels, state):
    position_accel = controllable_vars_accels[:3, :]
    yaw_accel = controllable_vars_accels[3, :]
    link_accel = controllable_vars_accels[4:, :]

    yaw_control = yaw_accel + gain_constants.K_PSI_V * ()