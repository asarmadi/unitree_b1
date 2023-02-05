import sys
sys.path.append('/data/alireza/high_bo/BayesOpt/lib/')

import robot_interface as sdk

class RobotInterface():

 def __init__(self):
     LOWLEVEL  = 0xff
     self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)

     self.cmd   = sdk.LowCmd()
     self.state = sdk.LowState()
     self.udp.InitCmdData(self.cmd)

     self.safe  = sdk.Safety(sdk.LeggedType.B1)

 def receive_observation(self):
     self.udp.Recv()
     self.udp.GetRecv(self.state)
     return self.state

