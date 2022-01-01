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
    ##  Reward for moving forward...
    orientation_reward = 0  #  10 - np.linalg.norm(np.array(orientation) - np.array([1, 1, 0, 0])) * 10
    position_reward = 200 * (np.array(current_pos)[1] - np.array(prev_pos)[1]) #(current_pos[2] - 0.3)*20
    reward = orientation_reward + position_reward
    print('R: ', reward)
    # input('here')
    return reward

def resetEpisode(p, q):
    p.resetBasePositionAndOrientation(q, [0, 0, 0.4], [1, 1, 0, 0])
    
def getState(p, q, prev_angles, current_angles, normalized=False):
    '''
    States:
        15 numbers:
             3 numbers for position
             4 numbers for orientation
             4 number for previous angles of 4 legs
             4 number for current angles of 4 legs
    '''
    state = []
    # state = state + list(p.getJointInfo(q, 0)[-2])
    state = state + list(p.getBasePositionAndOrientation(q)[0])
    state = state + list(p.getBasePositionAndOrientation(q)[1])
    state = state + list(prev_angles.values())
    state = state + list(current_angles.values())
    if not normalized:
        return np.array(state)
    state = np.array(state)
    # state[-8:] = state[-8:] * np.pi / 180
    return state

def setLegs(p, q, angles):
    setFrontRightPosition(p, q, angles['rf'])
    setFrontLeftPosition(p, q, angles['lf'])
    setBackRightPosition(p, q, angles['rb'])
    setBackLeftPosition(p, q, angles['lb'])
    
def act(amp, freq, phase, n):
    print(f'Amp: {amp}\tFreq: {freq}\tPhase: {phase}\tn: {n}')
    angle = amp * np.sin(2*np.pi*freq*n + phase)
    angles['rf'] = angle
    angles['lf'] = -angle
    print('Generated angle...', angle)

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
        inputs = prepareModelInput(state, amps, frqs, phis)
        qs = np.squeeze(model.predict(inputs))
        max_index = np.argmax(qs)
        # print('qs: ', qs.shape)
        # print(qs[max_index])
        action = inputs[max_index][-3:]
        action = action.tolist()
        # print('action: ', action)
        # input('waiting...')
        return action
    else:
        print('Random action...')
        action = [np.random.choice(amps), np.random.choice(frqs), np.random.choice(phis)]
        return action

# def _trainModels(Xs, ys, n=100):
#     '''
#     Planning!
#     '''
#     for _ in range(n):
#         i = np.random.randint(0, 21)
#         if Xs[i].shape[0]:
#             processModel(model_train, i)
#             model_train.train_on_batch(Xs[i], ys[i])

def trainModels(discount=1.):
    print('trainModel: ')
    X = []
    y = []
    print('Lens: ', len(hist_state), len(hist_action), len(hist_reward))
    print('Rewards: ', hist_reward)
    for i in range(len(hist_action)):
        discounted_rewards = [hist_reward[j] * (discount**(j-i)) for j in range(i, len(hist_reward))]
        # print('Discounted!: ', discounted_rewards)
        ret = sum(discounted_rewards)

        # print('ret: ', ret)
        # print('len: ', len(hist_state), i)
        #input()
        X.append(np.concatenate((hist_state[i], hist_action[i])))
        y.append(ret)
        # print('X: ', X)
        # print('y: ', y)
        # input('Waiting...')
        # print('target before: ', target)
        # print('predicted: ', target[0, hist_action[i]])
        # target[0, hist_action[i]] = ret

    X = np.array(X)
    y = np.array(y)
    print('Shapes:', X.shape, y.shape)

    for _ in range(5):
        model.train_on_batch(X, y)
    # _trainModels(Xs, ys)
    mhm.saveModels([model], 'model')


def processModel(model, i):
    w = np.zeros((21, 1))
    w[i, 0] = 1
    model.layers[-1].set_weights([w, np.array([0])])
    model.layers[-1].trainable = False

def prepareModelInput(state, amps, frqs, phis):
    parameters = [(a, f, p) for a in amps for f in frqs for p in phis]
    inputs = [np.concatenate((state, np.array(parameter))) for parameter in parameters]
    return np.array(inputs)

##  Neural Net...
LOAD_MODEL = True

if LOAD_MODEL:
    model = keras.models.load_model('model0')
    print('Model Loaded!')
    input('Press any key to continue...')
else:
    model = Sequential()
    model.add(Dense(128, 'tanh', input_shape=(18,)))
    model.add(Dense(1))
    model.compile('adam', loss='mse')

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
# physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0 ,0 ,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("quadruped/minitaur.urdf", [0, 0, 0.4],
               [1, 1, 0, 0],
               useFixedBase=False)

angles = {'rf': 0, 'lf': 0, 'rb': 0, 'lb': 0}
prev_angles = angles.copy()

# amps = [60, 45, 30, 10]
amps = [np.pi/3, np.pi/4, np.pi/6, np.pi*10/180]
frqs = [0.1, 0.2, 0.25, 0.05]
phis = [0, np.pi/3, np.pi/4, np.pi/6]

nJ = p.getNumJoints(quadruped)
print('No of joints: ', p.getNumJoints(quadruped))
for i in range(nJ):
    print('Joint info: ', p.getJointInfo(quadruped, i))
print('No of constraints: ', p.getNumConstraints())
#p.resetBasePositionAndOrientation(quadruped, [0, 0, 0.4], [1, 1, 0, 0])
prev_pos, orient = p.getBasePositionAndOrientation(quadruped)
# print(mh.quantize(p.getJointInfo(quadruped, 1)[-2]))
getState(p, quadruped, prev_angles, angles)
setLegs(p, quadruped, angles)
hist_reward = []
hist_state = []
hist_action = []
episode_len = 0
max_episode_len = 600

prev_state = getState(p, quadruped, prev_angles, angles)
for i in range (5000000):
    if isFallen(p, quadruped) or episode_len > max_episode_len:
        print('here')
        episode_len = 0
        trainModels(0.95)
        resetEpisode(p, quadruped)
        hist_action = []
        hist_reward = []
        hist_state = []
        prev_state = getState(p, quadruped, prev_angles, angles)
        print('lens: ', len(hist_state), len(hist_action), len(hist_reward))
        
    episode_len += 1
    lockKnees(p, quadruped, np.pi/6, mode=1)
    if not(i%10):
        current_state = getState(p, quadruped, prev_angles, angles)
        state = current_state.copy()
        # state = np.concatenate((prev_state, current_state))
        current_pos, orient = p.getBasePositionAndOrientation(quadruped)
        reward = getReward(prev_pos, current_pos, orient)
        prev_pos = current_pos
        eps = setEpsilon(i, 'const', 0.3)
        print('eps: ', eps, 'reward: ', reward)
        prev_angles = angles.copy()
        action = epsilonGreedy(state, eps)
        act(*action, episode_len)
        print('Action: ', action)
        print('Angles: ', angles)
        setLegs(p, quadruped, angles)
        hist_state.append(state)
        hist_action.append(action)
        hist_reward.append(reward)
        prev_state = current_state.copy()

    p.stepSimulation()
    time.sleep(1./240.)

print(current_pos, orient)
p.disconnect()