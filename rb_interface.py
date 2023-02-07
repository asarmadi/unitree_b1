import sys

sys.path.append("/home/franka-ws/Documents/unitree_legged_sdk-3.8.3/lib/python/amd64/")
import numpy as np
import robot_interface as sdk


class RobotInterface:
    def __init__(self):
        LOWLEVEL = 0xFF
        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)

        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)

        self.safe = sdk.Safety(sdk.LeggedType.B1)

    def receive_observation(self):
        self.udp.Recv()
        self.udp.GetRecv(self.state)
        return self.state

    def send_command(self, command):
        for motor_id in range(12):
            self.cmd.motorCmd[motor_id].q = command[motor_id * 5]
            self.cmd.motorCmd[motor_id].Kp = command[motor_id * 5 + 1]
            self.cmd.motorCmd[motor_id].dq = command[motor_id * 5 + 2]
            self.cmd.motorCmd[motor_id].Kd = command[motor_id * 5 + 3]
            self.cmd.motorCmd[motor_id].tau = command[motor_id * 5 + 4]

        self.safe.PositionLimit(self.cmd)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
