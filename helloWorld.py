import pybullet as p
import time
import pybullet_data
import numpy as np

def setFrontRightVelocity(p, q, v):
    p.setJointMotorControl2(q, 1, p.VELOCITY_CONTROL, targetVelocity=v)
    p.setJointMotorControl2(q, 4, p.VELOCITY_CONTROL, targetVelocity=-v)

def setFrontLeftVelocity(p, q, v):
    p.setJointMotorControl2(q, 14, p.VELOCITY_CONTROL, targetVelocity=v)
    p.setJointMotorControl2(q, 17, p.VELOCITY_CONTROL, targetVelocity=-v)

def setBackRightVelocity(p, q, v):
    p.setJointMotorControl2(q, 10, p.VELOCITY_CONTROL, targetVelocity=v)
    p.setJointMotorControl2(q, 7, p.VELOCITY_CONTROL, targetVelocity=-v)

def setBackLeftVelocity(p, q, v):
    p.setJointMotorControl2(q, 20, p.VELOCITY_CONTROL, targetVelocity=v)
    p.setJointMotorControl2(q, 23, p.VELOCITY_CONTROL, targetVelocity=-v)


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0 ,0 ,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("quadruped/minitaur.urdf", [-0.000046, -0.000068, 0.200774],
               [-0.000701, 0.000387, -0.000252, 1.000000],
               useFixedBase=False)
nJ = p.getNumJoints(quadruped)
print('No of joints: ',p.getNumJoints(quadruped))
for i in range(nJ):
    print('Join info: ',p.getJointInfo(quadruped, i))

for i in range (10000):
    #setFrontRightVelocity(p, quadruped, 2)
    #setFrontLeftVelocity(p, quadruped, 20)
    #setBackRightVelocity(p, quadruped, 5)
    setBackLeftVelocity(p, quadruped, 10)

    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(quadruped)
print(cubePos, cubeOrn)
p.disconnect()