import numpy as np
import pybullet
import pybullet_data

from pybullet_utils import bullet_client
from controller import Controller
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

desired_joint_angles = np.array([0, 0.9, -1.8]*4)
controller = Controller()

while True:
  robot.ReceiveObservation()
  action = controller.get_action(robot, desired_joint_angles)
  robot.ApplyAction(action)

p.removeAllUserDebugItems()
p.resetSimulation()
p.disconnect()
