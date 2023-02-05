import numpy as np
import pybullet
import pybullet_data

from pybullet_utils import bullet_client
import b1_robot

p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
p.setPhysicsEngineParameter(numSolverIterations=30)
p.setTimeStep(0.001)
p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(enableConeFriction=0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=10, cameraPitch=-20, cameraTargetPosition=[0, 0, 1.0])
robot = b1_robot.B1Robot(pybullet_client=p)

while True:
  robot.ReceiveObservation()
  
p.removeAllUserDebugItems()
p.resetSimulation()
p.disconnect()
