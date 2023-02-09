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
import re
import math
import copy
from rb_interface import RobotInterface


class B1Robot:
    def __init__(self, pybullet_client):
        """Initializes the robot class."""
        self._pybullet_client = pybullet_client

        # Robot state variables
        self.mass = 50  # kg
        self._num_legs = 4
        self._num_motors = 12
        self._base_orientation = None
        self._motor_angles = np.zeros(12)
        self._motor_velocities = np.zeros(12)
        self._motor_kps = np.array(
            [30, 30, 30] * 4
        )  # ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
        self._motor_kds = np.array(
            [3, 3, 3] * 4
        )  # ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
        self._joint_states = None
        self.init_position = [0, 0, 0.8]
        self._urdf_filename = "./b1.urdf"
        self.motor_names = [
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
            "RL_calf_joint",
        ]

        # Initiate UDP for robot state and actions
        self._robot_interface = RobotInterface()
        self._LoadRobotURDF()
        self._BuildJointNameToIdDict()
        self._motor_id_list = [
            self._joint_name_to_id[motor_name] for motor_name in self._GetMotorNames()
        ]
        self._BuildUrdfIds()

    def ReceiveObservation(self):
        """Receives observation from robot.

        Synchronous ReceiveObservation is not supported in B1,
        so changging it to noop instead.
        """
        state = self._robot_interface.receive_observation()
        # Convert quaternion from wxyz to xyzw, which is default for Pybullet.
        q = state.imu.quaternion
        self._base_orientation = np.array([q[1], q[2], q[3], q[0]])
        self._motor_angles = np.array([motor.q for motor in state.motorState[:12]])
        self._motor_velocities = np.array([motor.dq for motor in state.motorState[:12]])
        self._joint_states = np.array(
            list(zip(self._motor_angles, self._motor_velocities))
        )
        self._SetRobotStateInSim(self._motor_angles, self._motor_velocities)

    def _SetRobotStateInSim(self, motor_angles, motor_velocities):
        self._pybullet_client.resetBasePositionAndOrientation(
            self.quadruped, self.GetBasePosition(), self.GetBaseOrientation()
        )
        for i, motor_id in enumerate(self._motor_id_list):
            self._pybullet_client.resetJointState(
                self.quadruped, motor_id, motor_angles[i], motor_velocities[i]
            )

    def _BuildUrdfIds(self):
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        self._foot_link_ids = []
        FOOT_NAME_PATTERN = re.compile(r"\w+_foot\w+")
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode("UTF-8")
            joint_id = self._joint_name_to_id[joint_name]
            if FOOT_NAME_PATTERN.match(joint_name):
                self._foot_link_ids.append(joint_id)

            self._foot_link_ids.sort()

    @property
    def motor_velocities(self):
        return self._motor_velocities.copy()

    def Terminate(self):
        self._is_alive = False

    def _LoadRobotURDF(self):
        _urdf_path = self.GetURDFFile()
        self.quadruped = self._pybullet_client.loadURDF(
            _urdf_path,
            self._GetDefaultInitPosition(),
            self._GetDefaultInitOrientation(),
        )

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
        init_orientation = pyb.getQuaternionFromEuler([0.0, 0.0, 0.0])
        return init_orientation

    def _BuildJointNameToIdDict(self):
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        self._joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    def ApplyAction(self, motor_commands):
        self._robot_interface.send_command(motor_commands)

    def GetMotorPositionGains(self):
        return self._motor_kps

    def GetMotorVelocityGains(self):
        return self._motor_kds

    def ComputeJacobian(self, leg_id):
        """Compute the Jacobian for a given leg."""
        motor_angles = self.GetMotorAngles()[leg_id * 3 : (leg_id + 1) * 3]
        return self.analytical_leg_jacobian(motor_angles, leg_id)

    def MapContactForceToJointTorques(self, leg_id, contact_force):
        """Maps the foot contact force to the leg joint torques."""
        jv = self.ComputeJacobian(leg_id)

        motor_torques_list = np.matmul(contact_force, jv)
        motor_torques_dict = {}
        motors_per_leg = self._num_motors // self._num_legs
        for torque_id, joint_id in enumerate(
            range(leg_id * motors_per_leg, (leg_id + 1) * motors_per_leg)
        ):
            motor_torques_dict[joint_id] = motor_torques_list[torque_id]
        return motor_torques_dict

    def GetMotorAngles(self):
        return self.MapToMinusPiToPi(self._motor_angles).copy()

    def analytical_leg_jacobian(self, leg_angles, leg_id):
        """
        Computes the analytical Jacobian.
        Args:
        leg_angles: a list of 3 numbers for current abduction, hip and knee angle.
        l_hip_sign: whether it's a left (1) or right(-1) leg.
        """
        l_up = 0.2
        l_low = 0.2
        l_hip = 0.08505 * (-1) ** (leg_id + 1)

        t1, t2, t3 = leg_angles[0], leg_angles[1], leg_angles[2]
        l_eff = np.sqrt(l_up**2 + l_low**2 + 2 * l_up * l_low * np.cos(t3))
        t_eff = t2 + t3 / 2
        J = np.zeros((3, 3))
        J[0, 0] = 0
        J[0, 1] = -l_eff * np.cos(t_eff)
        J[0, 2] = (
            l_low * l_up * np.sin(t3) * np.sin(t_eff) / l_eff
            - l_eff * np.cos(t_eff) / 2
        )
        J[1, 0] = -l_hip * np.sin(t1) + l_eff * np.cos(t1) * np.cos(t_eff)
        J[1, 1] = -l_eff * np.sin(t1) * np.sin(t_eff)
        J[1, 2] = (
            -l_low * l_up * np.sin(t1) * np.sin(t3) * np.cos(t_eff) / l_eff
            - l_eff * np.sin(t1) * np.sin(t_eff) / 2
        )
        J[2, 0] = l_hip * np.cos(t1) + l_eff * np.sin(t1) * np.cos(t_eff)
        J[2, 1] = l_eff * np.sin(t_eff) * np.cos(t1)
        J[2, 2] = (
            l_low * l_up * np.sin(t3) * np.cos(t1) * np.cos(t_eff) / l_eff
            + l_eff * np.sin(t_eff) * np.cos(t1) / 2
        )
        return J

    def MapToMinusPiToPi(self, angles):
        """Maps a list of angles to [-pi, pi].

        Args:
          angles: A list of angles in rad.

        Returns:
          A list of angle mapped to [-pi, pi].
        """
        mapped_angles = copy.deepcopy(angles)
        for i in range(len(angles)):
            mapped_angles[i] = math.fmod(angles[i], (2 * math.pi))
            if mapped_angles[i] >= math.pi:
                mapped_angles[i] -= 2 * math.pi
            elif mapped_angles[i] < -math.pi:
                mapped_angles[i] += 2 * math.pi
        return mapped_angles

    @property
    def joint_states(self):
        return self._joint_states
