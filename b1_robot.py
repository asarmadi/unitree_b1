# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# pytype: disable=attribute-error
"""Real robot interface of B1 robot."""

import numpy as np
import pybullet as pyb 
from rb_interface import RobotInterface

class B1Robot():
  def __init__(self, pybullet_client):
    """Initializes the robot class."""
    self._pybullet_client = pybullet_client

    # Robot state variables
    self._base_orientation = None
    self._motor_angles = np.zeros(12)
    self._motor_velocities = np.zeros(12)
    self._joint_states = None
    self.init_position = [0, 0, 0.8]
    self._urdf_filename = "./b1.urdf"
    self.motor_names   = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint"]

    # Initiate UDP for robot state and actions
    self._robot_interface = RobotInterface()
    self._LoadRobotURDF()
    self._BuildJointNameToIdDict()
    self._motor_id_list = [
        self._joint_name_to_id[motor_name]
        for motor_name in self._GetMotorNames()
    ]

  def ReceiveObservation(self):
    """Receives observation from robot.

    Synchronous ReceiveObservation is not supported in B1,
    so changging it to noop instead.
    """
    state = self._robot_interface.receive_observation()
    # Convert quaternion from wxyz to xyzw, which is default for Pybullet.
    q = state.imu.quaternion
    self._base_orientation = np.array([q[1], q[2], q[3], q[0]])
    self._motor_angles     = np.array([motor.q for motor in state.motorState[:12]])
    self._motor_velocities = np.array([motor.dq for motor in state.motorState[:12]])
    self._joint_states     = np.array(list(zip(self._motor_angles, self._motor_velocities)))
    self._SetRobotStateInSim(self._motor_angles, self._motor_velocities)

  def _SetRobotStateInSim(self, motor_angles, motor_velocities):
    self._pybullet_client.resetBasePositionAndOrientation(
        self.quadruped, self.GetBasePosition(), self.GetBaseOrientation())
    for i, motor_id in enumerate(self._motor_id_list):
      self._pybullet_client.resetJointState(self.quadruped, motor_id,
                                            motor_angles[i],
                                            motor_velocities[i])

  @property
  def motor_velocities(self):
    return self._motor_velocities.copy()

  def Terminate(self):
    self._is_alive = False

  def _LoadRobotURDF(self):
    _urdf_path = self.GetURDFFile()
    self.quadruped = self._pybullet_client.loadURDF(
          _urdf_path, self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation())

  def GetBasePosition(self):
    return self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[0]
  
  def _GetMotorNames(self):
    return self.motor_names

  def GetBaseOrientation(self):
    return self._base_orientation.copy()

  def GetURDFFile(self):
    return self._urdf_filename

  def _GetDefaultInitPosition(self):
    return self.init_position

  def _GetDefaultInitOrientation(self):
    init_orientation = pyb.getQuaternionFromEuler([0., 0., 0.])
    return init_orientation

  def _BuildJointNameToIdDict(self):
    num_joints = self._pybullet_client.getNumJoints(self.quadruped)
    self._joint_name_to_id = {}
    for i in range(num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
      self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

