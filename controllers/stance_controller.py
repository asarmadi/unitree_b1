import numpy as np


class Controller:
    def __init__(self):
        self._joint_angles = None
        self.reset(0)

    def reset(self, current_time: float) -> None:
        """Called during the start of a swing cycle.

        Args:
          current_time: The wall time in seconds.
        """
        del current_time
        self._joint_angles = {}

    def update(self, current_time: float) -> None:
        """Called at each control step.

        Args:
          current_time: The wall time in seconds.
        """
        del current_time

    def get_action(self, robot, desired):
        contact_forces = {}
        for i in range(robot._num_legs):
            contact_forces[i] = np.array([0.0, 0.0, -9.8]) * robot.mass / 4

        action = {}
        for leg_id, force in contact_forces.items():
            motor_torques = robot.MapContactForceToJointTorques(leg_id, force)
            for joint_id, torque in motor_torques.items():
                action[joint_id] = (0, 0, 0, 0, torque)
        return action
