class Controller():
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
    
    action = []
    kps = robot.GetMotorPositionGains()
    kds = robot.GetMotorVelocityGains()
    for joint_id in range(len(desired)):
      action.extend((desired[joint_id], kps[joint_id], 0, kds[joint_id], 0))

    action = np.array(action, dtype=np.float32)

    return action
