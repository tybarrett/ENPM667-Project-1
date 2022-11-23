"""system_state.py - contains a class definition of the attributes of the system at any given time"""


class SystemState:
    def __init__(self):
        # Instantaneous attributes
        self.body_x = 0
        self.body_y = 0
        self.body_z = 0

        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        self.joint_positions = []

        # First derivative
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.rotational_velocity_yaw = 0
        self.rotational_velocity_pitch = 0
        self.rotational_velocity_roll = 0
        self.joint_velocities = []

        # Second derivative
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.rot_accel_yaw = 0
        self.rot_accel_pitch = 0
        self.rot_accel_roll = 0
        self.joint_accelerations = []


    def populate_derivative_from_history(self):
        pass # TODO