import pybullet as p
import time
import pybullet_data
import keras
import numpy as np
import MHutils as mh

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

def setFrontRightPosition(p, q, pos, max_v = 10):
    p.setJointMotorControl2(q, 1, p.POSITION_CONTROL, targetPosition=pos ,maxVelocity=max_v)
    p.setJointMotorControl2(q, 4, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v)

def setFrontLeftPosition(p, q, pos, max_v = 10):
    p.setJointMotorControl2(q, 14, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v)
    p.setJointMotorControl2(q, 17, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v)

def setBackRightPosition(p, q, pos, max_v = 10):
    p.setJointMotorControl2(q, 10, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v)
    p.setJointMotorControl2(q, 7, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v)

def setBackLeftPosition(p, q, pos, max_v = 10):
    p.setJointMotorControl2(q, 20, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v)
    p.setJointMotorControl2(q, 23, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v)

def lockKnees(p, q, pos, max_v = 10, mode = 0):
    '''
        mode = 0: All knees the same!
        mode = 1: Symmetric knees! Fat way!
        mode = 2: Symmetric knees! Thin way!
    '''
    if mode==0:
        right_knees = [3, 9, 19, 25]
        left_knees = [6, 12, 16, 22]
        for rk, lk in zip(right_knees, left_knees):
            p.setJointMotorControl2(q, rk, p.POSITION_CONTROL, targetPosition=pos ,maxVelocity=10)
            p.setJointMotorControl2(q, lk, p.POSITION_CONTROL, targetPosition=-pos ,maxVelocity=10)
    elif mode==1:
        right_front_knees = [3, 19]
        left_front_knees = [6, 16]
        right_back_knees = [9, 25]
        left_back_knees = [12, 22]
        for rk, lk in zip(right_front_knees, left_front_knees):
            p.setJointMotorControl2(q, rk, p.POSITION_CONTROL, targetPosition=pos ,maxVelocity=10)
            p.setJointMotorControl2(q, lk, p.POSITION_CONTROL, targetPosition=-pos ,maxVelocity=10)
        for rk, lk in zip(right_back_knees, left_back_knees):
            p.setJointMotorControl2(q, rk, p.POSITION_CONTROL, targetPosition=-pos ,maxVelocity=10)
            p.setJointMotorControl2(q, lk, p.POSITION_CONTROL, targetPosition=+pos ,maxVelocity=10)

    elif mode==2:
        right_front_knees = [3, 19]
        left_front_knees = [6, 16]
        right_back_knees = [9, 25]
        left_back_knees = [12, 22]
        for rk, lk in zip(right_front_knees, left_front_knees):
            p.setJointMotorControl2(q, rk, p.POSITION_CONTROL, targetPosition=-pos ,maxVelocity=10)
            p.setJointMotorControl2(q, lk, p.POSITION_CONTROL, targetPosition=pos ,maxVelocity=10)
        for rk, lk in zip(right_back_knees, left_back_knees):
            p.setJointMotorControl2(q, rk, p.POSITION_CONTROL, targetPosition=pos ,maxVelocity=10)
            p.setJointMotorControl2(q, lk, p.POSITION_CONTROL, targetPosition=-pos ,maxVelocity=10)


def getReward(prev_pos, current_pos, orientation):
    reward = 0
    reward += (current_pos[1] - prev_pos[1])*5
    reward -= abs(current_pos[0] - prev_pos[0])
    return reward

def resetEpisode(p, q):
    p.resetBasePositionAndOrientation(q, [0, 0, 0.4], [1, 1, 0, 0])

def getState(p, q, angles):
    state = []
    state = state + list(p.getJointInfo(q, 0)[-2])
    state = state + list(angles.values())
    return np.array(state)

def setLegs(p, q, angles):
    setFrontRightPosition(p, q, angles['rf'])
    setFrontLeftPosition(p, q, angles['lf'])
    setBackRightPosition(p, q, angles['rb'])
    setBackLeftPosition(p, q, angles['lb'])
    
def act(i):
    if 0<=i<=4:
        angles['rf'] = -(np.pi/2) + i * np.pi/4
    elif 5<=i<=9:
        angles['lf'] = -(np.pi/2) + (i-5) * np.pi/4
    elif 10<=i<=14:
        angles['rb'] = -(np.pi/2) + (i-10) * np.pi/4
    elif 15<=i<=19:
        angles['lb'] = -(np.pi/2) + (i-15) * np.pi/4

def epsilonGreedy(state, eps=0.1):
    n = np.random.rand()
    if n>eps:
        qs = model.predict(state[np.newaxis, :])
        action = np.max(qs[0])
        return action
    else:
        return np.random.randint(0, 20)

##  Neural Net...
model = keras.Sequential()
model.add(keras.layers.Dense(14, activation='sigmoid', input_shape=(7,)))
model.add(keras.layers.Dense(20, activation='linear'))
model.compile(loss='mse', optimizer='adam')

##  Simulator
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0 ,0 ,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("quadruped/minitaur.urdf", [0, 0, 0.4],
               [1, 1, 0, 0],
               useFixedBase=False)

angles = {'rf': 0, 'lf': 0, 'rb': 0, 'lb': 0}

nJ = p.getNumJoints(quadruped)
print('No of joints: ', p.getNumJoints(quadruped))
for i in range(nJ):
    print('Joint info: ', p.getJointInfo(quadruped, i))
print('No of constraints: ', p.getNumConstraints())
#p.resetBasePositionAndOrientation(quadruped, [0, 0, 0.4], [1, 1, 0, 0])
prev_pos, orient = p.getBasePositionAndOrientation(quadruped)
print(mh.quantize(p.getJointInfo(quadruped, 1)[-2]))
getState(p, quadruped, angles)
setLegs(p, quadruped, angles)
hist_reward = []
hist_state = []
hist_action = []

for i in range (10000):
    # pos, orient = p.getBasePositionAndOrientation(quadruped)
    # setFrontRightPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100))
    # setFrontLeftPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100 + np.pi/4))
    # setBackLeftPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100))
    # setBackRightPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100 + np.pi/4))
    if not (i%1000):
        resetEpisode(p, quadruped)
    lockKnees(p, quadruped, np.pi/6, mode=1)

    current_pos, orient = p.getBasePositionAndOrientation(quadruped)
    reward = getReward(prev_pos, current_pos, orient)
    #print('Pos and Ornt: ', prev_pos, current_pos, orient)
    #print('Reward: ', reward)
    prev_pos = current_pos
    if not(i%10):
        action = 
        hist_state.append(getState(p, quadruped, angles))
        hist_reward.append(reward)
        hist_action.append(action)
        act(np.random.randint(0, 20))
        setLegs(p, quadruped, angles)
    p.stepSimulation()
    time.sleep(1./240.)

print(current_pos, orient)
p.disconnect()