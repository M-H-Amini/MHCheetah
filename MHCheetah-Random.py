import pybullet as p
import time
import pybullet_data
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


def setFrontRightPosition(p, q, pos, max_v=10):
    p.setJointMotorControl2(
        q, 1, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v
    )
    p.setJointMotorControl2(
        q, 4, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v
    )


def setFrontLeftPosition(p, q, pos, max_v=10):
    p.setJointMotorControl2(
        q, 14, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v
    )
    p.setJointMotorControl2(
        q, 17, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v
    )


def setBackRightPosition(p, q, pos, max_v=10):
    p.setJointMotorControl2(
        q, 10, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v
    )
    p.setJointMotorControl2(
        q, 7, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v
    )


def setBackLeftPosition(p, q, pos, max_v=10):
    p.setJointMotorControl2(
        q, 20, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=max_v
    )
    p.setJointMotorControl2(
        q, 23, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=max_v
    )


def lockKnees(p, q, pos, max_v=10, mode=0):
    """
    mode = 0: All knees the same!
    mode = 1: Symmetric knees! Fat way!
    mode = 2: Symmetric knees! Thin way!
    """
    if mode == 0:
        right_knees = [3, 9, 19, 25]
        left_knees = [6, 12, 16, 22]
        for rk, lk in zip(right_knees, left_knees):
            p.setJointMotorControl2(
                q, rk, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=10
            )
            p.setJointMotorControl2(
                q, lk, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=10
            )
    elif mode == 1:
        right_front_knees = [3, 19]
        left_front_knees = [6, 16]
        right_back_knees = [9, 25]
        left_back_knees = [12, 22]
        for rk, lk in zip(right_front_knees, left_front_knees):
            p.setJointMotorControl2(
                q, rk, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=10
            )
            p.setJointMotorControl2(
                q, lk, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=10
            )
        for rk, lk in zip(right_back_knees, left_back_knees):
            p.setJointMotorControl2(
                q, rk, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=10
            )
            p.setJointMotorControl2(
                q, lk, p.POSITION_CONTROL, targetPosition=+pos, maxVelocity=10
            )

    elif mode == 2:
        right_front_knees = [3, 19]
        left_front_knees = [6, 16]
        right_back_knees = [9, 25]
        left_back_knees = [12, 22]
        for rk, lk in zip(right_front_knees, left_front_knees):
            p.setJointMotorControl2(
                q, rk, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=10
            )
            p.setJointMotorControl2(
                q, lk, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=10
            )
        for rk, lk in zip(right_back_knees, left_back_knees):
            p.setJointMotorControl2(
                q, rk, p.POSITION_CONTROL, targetPosition=pos, maxVelocity=10
            )
            p.setJointMotorControl2(
                q, lk, p.POSITION_CONTROL, targetPosition=-pos, maxVelocity=10
            )


def getReward(prev_pos, current_pos, orientation):
    """
    reward = 0
    reward += (current_pos[1] - prev_pos[1])*5
    if (abs(current_pos[0]) - 0.5) > 0:
        reward -= (abs(current_pos[0]) - 0.5)
    """
    ##  Reward for jumping...
    orientation_reward = (
        10 - np.linalg.norm(np.array(orientation) - np.array([1, 1, 0, 0])) * 10
    )
    position_reward = (
        10 - np.linalg.norm(np.array(current_pos) - np.array([0, 0, 0.3])) * 10
    )  # (current_pos[2] - 0.3)*20
    reward = orientation_reward + position_reward
    return reward


def resetEpisode(p, q):
    p.resetBasePositionAndOrientation(q, [0, 0, 0.4], [1, 1, 0, 0])


def getState(p, q, angles):
    """
    States:
        11 numbers:
             3 numbers for position
             4 numbers for orientation
             4 number for angles of 4 legs
    """
    state = []
    # state = state + list(p.getJointInfo(q, 0)[-2])
    state = state + list(p.getBasePositionAndOrientation(q)[0])
    state = state + list(p.getBasePositionAndOrientation(q)[1])
    state = state + list(angles.values())
    return np.array(state)


def setLegs(p, q, angles):
    setFrontRightPosition(p, q, angles["rf"])
    setFrontLeftPosition(p, q, angles["lf"])
    setBackRightPosition(p, q, angles["rb"])
    setBackLeftPosition(p, q, angles["lb"])


def act(i):
    if 0 <= i <= 4:
        angles["rf"] = -(np.pi / 2) + i * np.pi / 4
    elif 5 <= i <= 9:
        angles["lf"] = -(np.pi / 2) + (i - 5) * np.pi / 4
    elif 10 <= i <= 14:
        angles["rb"] = -(np.pi / 2) + (i - 10) * np.pi / 4
    elif 15 <= i <= 19:
        angles["lb"] = -(np.pi / 2) + (i - 15) * np.pi / 4


def isFallen(p, q):
    if p.getBasePositionAndOrientation(q)[0][2] < 0.13:
        return True
    return False


def setEpsilon(i, mode="exploration", *args):
    if mode == "explore" or mode == "exploration" or mode == 0:
        eps = 0.2 + 0.8 * np.exp(-i / 5000)
    elif mode == "exploit" or mode == "exploitation" or mode == 1:
        if not len(args):
            eps = 0.1
        else:
            eps = args[0]
    elif mode in ["const", "constant"]:
        eps = args[0]
    return eps


time.sleep(2)
##  Simulator
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
quadruped = p.loadURDF(
    "quadruped/minitaur.urdf", [0, 0, 0.4], [1, 1, 0, 0], useFixedBase=False
)

angles = {"rf": 0, "lf": 0, "rb": 0, "lb": 0}

nJ = p.getNumJoints(quadruped)
print("No of joints: ", p.getNumJoints(quadruped))
for i in range(nJ):
    print("Joint info: ", p.getJointInfo(quadruped, i))
print("No of constraints: ", p.getNumConstraints())
# p.resetBasePositionAndOrientation(quadruped, [0, 0, 0.4], [1, 1, 0, 0])
prev_pos, orient = p.getBasePositionAndOrientation(quadruped)
getState(p, quadruped, angles)
setLegs(p, quadruped, angles)
episode_len = 0
max_episode_len = 300

prev_state = getState(p, quadruped, angles)
for i in range(5000000):
    if isFallen(p, quadruped) or episode_len > max_episode_len:
        episode_len = 0
        resetEpisode(p, quadruped)

    episode_len += 1
    lockKnees(p, quadruped, np.pi / 6, mode=2)
    if not (i % 10):
        current_state = getState(p, quadruped, angles)
        state = current_state.copy()
        current_pos, orient = p.getBasePositionAndOrientation(quadruped)
        reward = getReward(prev_pos, current_pos, orient)
        action = np.random.choice(20)
        act(action)
        setLegs(p, quadruped, angles)

    p.stepSimulation()
    time.sleep(1.0 / 240.0)

print(current_pos, orient)
p.disconnect()
