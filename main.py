"""main.py - executes the simulation as described in the paper."""


import numpy
import transformation_matrices
import system_state
import reference_trajectory
import inverse_kinematics
import position_control
import attitude_control
import motion_control
import dynamics
import manipulator_control
import system_emulator


TIME_RESOLUTION_S = 1 / 250
END_TIME_S = 60


def main():

    state = system_state.SystemState()

    t = 0
    while t <= END_TIME_S:
        print(t)

        desired_pos, desired_vel, desired_accel, desired_rot = reference_trajectory.generate_reference_trajectory(t)
        ee_position = transformation_matrices.get_ee_position(state)
        controllable_vars = inverse_kinematics.generate_controllable_variable_accelerations(state, ee_position, desired_pos,
                                                                        desired_rot, desired_vel, desired_accel)

        pos_control_input, yaw_control_input, link_control_input = motion_control.calculate_control_inputs(controllable_vars,
                                                                                                           state,
                                                                                                           desired_yaw,
                                                                                                           desired_vel_yaw,
                                                                                                           desired_link_position,
                                                                                                           desired_link_velocities,
                                                                                                           desired_pos,
                                                                                                           desired_vel)

        thrust, pitch, roll = position_control.calculate_thrust_and_reference_angles(state, controllable_vars, gravity, inertia_matrix)

        pitch_control, roll_control = attitude_control.generate_attitude_control_input_estimates(roll, roll_deriv,
                                                                                                 pitch, pitch_deriv,
                                                                                                 state)

        rotational_control_input = numpy.Matrix([[yaw_control_input], [pitch_control], [roll_control]])
        vehicle_torques = attitude_control.calculate_vehicle_torques(inertia_matrix, pos_control_input, rotational_control_input,
                                                   link_control_input, g, coriolis_matrix, state)

        quad_motor_forces = dynamics.create_motor_forces_from_desired_torque_and_thrust(vehicle_torques, thrust)

        manipulator_control.calculate_joint_forces(inertia_matrix, pos_control_input, rotational_control_input,
                                                   link_control_input, coriolis_matrix, g, state)

        new_state = system_emulator.apply_system_inputs(TIME_RESOLUTION_S, state, pos_control_input,
                                                        rotational_control_input, link_control_input)

        state = new_state

        t += TIME_RESOLUTION_S


if __name__ == '__main__':
    main()
