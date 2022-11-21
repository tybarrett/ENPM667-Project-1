"""transformation_matrices.py - contains functions to express data in the world reference frame."""

import sympy


yaw, pitch, roll, robot_x, robot_y, robot_z = sympy.symbols("yaw, pitch, roll, robot_x, robot_y, robot_z")

ROTATION_MATRIX_BASE_TO_WORLD = sympy.Matrix([[sympy.cos(yaw) * sympy.cos(pitch),
                                               sympy.cos(yaw) * sympy.sin(pitch) * sympy.sin(roll) - sympy.sin(yaw) * sympy.cos(roll),
                                               sympy.cos(yaw) * sympy.sin(pitch) * sympy.cos(roll) + sympy.sin(yaw) * sympy.sin(roll)],
                                              [sympy.sin(yaw) * sympy.cos(pitch),
                                               sympy.sin(yaw) * sympy.sin(pitch) * sympy.sin(roll) + sympy.cos(yaw) * sympy.cos(roll),
                                               sympy.sin(yaw) * sympy.sin(pitch) * sympy.cos(roll) - sympy.cos(yaw) * sympy.sin(roll)],
                                              [-1 * sympy.sin(pitch),
                                               sympy.cos(pitch) * sympy.sin(roll),
                                               sympy.cos(pitch) * sympy.cos(roll)]])


def get_instantaneous_rotation_matrix(state):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                      (pitch, state.pitch),
                                                                      (roll, state.roll)])
    return rotate_base_to_world_matrix


def get_position_in_world_from_base_frame(state, point):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                       (pitch, state.pitch),
                                                                       (roll, state.roll)])

    translation_base_to_world = sympy.Matrix([[state.robot_x], [state.robot_y], [state.robot_z]])

    output_coord = translation_base_to_world + rotate_base_to_world_matrix * point
    return output_coord


def get_linear_velocity_in_world_from_base(state, point, point_velocity):

    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                       (pitch, state.pitch),
                                                                       (roll, state.roll)])

    base_velocity = sympy.Matrix([state.vx], [state.vy], [state.vz])
    base_rotational_velocity = sympy.Matrix([state.vyaw], [state.vpitch], [state.vroll])

    point_rotated_to_world = rotate_base_to_world_matrix * point
    output_velocity = base_velocity - point_rotated_to_world.cross(base_rotational_velocity) + rotate_base_to_world_matrix * point_velocity

    return output_velocity


def get_angular_velocity_in_world_from_base(state, point_rotational_vel):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                      (pitch, state.pitch),
                                                                      (roll, state.roll)])

    base_rotational_velocity = sympy.Matrix([state.vyaw], [state.vpitch], [state.vroll])
    output_rotational_vel = base_rotational_velocity + rotate_base_to_world_matrix * point_rotational_vel

    return output_rotational_vel
