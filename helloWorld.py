import pybullet as p
import time
import pybullet_data
import keras
import keras.backend as kb
from keras.layers import Input, Dense
from keras.models import Sequential
import numpy as np
import os
from functools import partial
import MHutils as mh
import MHModels as mhm

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
    '''
    reward = 0
    reward += (current_pos[1] - prev_pos[1])*5
    if (abs(current_pos[0]) - 0.5) > 0:
        reward -= (abs(current_pos[0]) - 0.5)
    '''
    ##  Reward for jumping...
    reward = (current_pos[2] - 0.3)*10
    return reward

def resetEpisode(p, q):
    p.resetBasePositionAndOrientation(q, [0, 0, 0.4], [1, 1, 0, 0])
    
def getState(p, q, angles):
    '''
    States:
        8 numbers: 4 numbers for orientation and 4 number for angles of 4 legs
    '''
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

def isFallen(p, q):
    if p.getBasePositionAndOrientation(q)[0][2] < 0.13:
        return True
    return False

def setEpsilon(i, mode='exploration', *args):
    if mode=='explore' or mode=='exploration' or mode==0:
        eps = 0.2 + 0.8 * np.exp(-i/5000)
    elif mode=='exploit' or mode=='exploitation' or mode==1:
        if not len(args):
            eps = 0.1
        else:
            eps = args[0]
    elif mode in ['const', 'constant']:
        eps = args[0]
    return eps

def epsilonGreedy(state, eps=0.1):
    # print('epsilon greedy: ')
    n = np.random.rand()
    if n>eps:
        qs = model.predict(state[np.newaxis, :])[0, :]
        print('qs: ', qs)
        action = np.argmax(qs)
        print('argmax: ', action)
        return action
    else:
        return np.random.randint(0, 21)

def trainModels(discount=1., model_type=1):
    print('trainModel: ')
    targets = [list() for i in range(21)]
    Xs = [list() for i in range(21)]
    for i in range(len(hist_action)):
        discounted_rewards = [hist_reward[j] * (discount**(j-i)) for j in range(i, len(hist_reward))]
        ret = sum(discounted_rewards)

        print('ret: ', ret)
        print('len: ', len(hist_state), i)
        #input()
        Xs[hist_action[i]].append(hist_state[i])
        targets[hist_action[i]].append(ret)

        # print('target before: ', target)
        # print('predicted: ', target[0, hist_action[i]])
        # target[0, hist_action[i]] = ret

    Xs = [np.array(Xs[i]) for i in range(21)]
    ys = [np.array(targets[i]) for i in range(21)]
    print('Xs shape: ', [Xs[i].shape for i in range(21)])
    for i in range(21):
        if Xs[i].shape[0]:
            processModel(model_train, i)
            model_train.train_on_batch(Xs[i], ys[i])
    mhm.saveModels([model], 'model')


def processModel(model, i):
    w = np.zeros((21, 1))
    w[i, 0] = 1
    model.layers[-1].set_weights([w, np.array([0])])
    model.layers[-1].trainable = False

##  Neural Net...

# model = mhm.buildModel()
model, = mhm.loadModels('model', 1)
model_train = Sequential([model, Dense(1)])
model_train.compile('adam', loss='mse')

'''  processModel Test
print(model_train.layers[-1].get_weights())
print(model_train.layers[-1].trainable)
processModel(model_train, 2)
print(model_train.layers[-1].get_weights())
print(model_train.layers[-1].trainable)
'''

time.sleep(2)
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
# print(mh.quantize(p.getJointInfo(quadruped, 1)[-2]))
getState(p, quadruped, angles)
setLegs(p, quadruped, angles)
hist_reward = []
hist_state = []
hist_action = []
episode_len = 0
max_episode_len = 300

prev_state = getState(p, quadruped, angles)
for i in range (5000000):
    # pos, orient = p.getBasePositionAndOrientation(quadruped)
    # setFrontRightPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100))
    # setFrontLeftPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100 + np.pi/4))
    # setBackLeftPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100))
    # setBackRightPosition(p, quadruped, - (np.pi/6) *  np.sin(np.pi * i / 100 + np.pi/4))
    #if i>0 and (not (i%1000)):
    if isFallen(p, quadruped) or episode_len > max_episode_len:
        print('here')
        episode_len = 0
        trainModels(0.9, 2)
        resetEpisode(p, quadruped)
        hist_action = []
        hist_reward = []
        hist_state = []
        prev_state = getState(p, quadruped, angles)
        print('lens: ', len(hist_state), len(hist_action), len(hist_reward))
        
    episode_len += 1
    lockKnees(p, quadruped, np.pi/6, mode=1)
    if not(i%10):
        current_state = getState(p, quadruped, angles)
        state = np.concatenate((prev_state, current_state))
        current_pos, orient = p.getBasePositionAndOrientation(quadruped)
        reward = getReward(prev_pos, current_pos, orient)
        prev_pos = current_pos
        eps = setEpsilon(i, 'const', 0.2)
        print('eps: ', eps, 'reward: ', reward)
        action = epsilonGreedy(state, eps)
        act(action)
        setLegs(p, quadruped, angles)
        hist_state.append(state)
        hist_action.append(action)
        hist_reward.append(reward)
        prev_state = current_state.copy()

    p.stepSimulation()
    time.sleep(1./240.)

print(current_pos, orient)
p.disconnect()