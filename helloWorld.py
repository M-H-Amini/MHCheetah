import pybullet as p
import time
import pybullet_data
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
    for j in range(nJ):
        p.setJointMotorControl2(quadruped, j, p.VELOCITY_CONTROL, targetVelocity=20)
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(quadruped)
print(cubePos, cubeOrn)
p.disconnect()
