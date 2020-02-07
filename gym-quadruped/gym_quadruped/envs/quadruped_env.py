import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np

import pybullet as p
import time
import pybullet_data
import time

class QuadrupedEnv(gym.Env):
    def __init__(self, timeStep=0.01, visualize=False):

        if(visualize):
            physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        else:
            physicsClient = p.connect(p.DIRECT)
            p.setRealTimeSimulation(0)
        # physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.2]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.quadrupedId = p.loadURDF("/home/manu/catkin_ws/src/URDF_Quadruped/URDF/quadruped.urdf",cubeStartPos, cubeStartOrientation)

        self.timeStep = timeStep


        self.observation_space = spaces.Box(low=-1.57, high=1.57, shape=(12,))
        self.action_space = spaces.Box(low=-1.57, high=1.57, shape=(12,))

        self.visualize = visualize


        self.nJoints = p.getNumJoints(self.quadrupedId)
        self.revoluteJoints = []
        for i in range(self.nJoints):
            jointInfo = p.getJointInfo(self.quadrupedId, i)
            if(jointInfo[2] == p.JOINT_REVOLUTE):
                self.revoluteJoints.append(i)

        # variables for computing reward
        self.prev_X = 0.0
        self.prev_Y = 0.0

        self.maxMotorForce = [3.5 for i in range(len(self.revoluteJoints))]

    def stepTime(self):
        for i in range(int(0.5/self.timeStep)):
            p.stepSimulation()
            if self.visualize:
                time.sleep(1/240)

    def getState(self):
        jointStates = p.getJointStates(self.quadrupedId, self.revoluteJoints)
        movementData = []
        for i in range(len(jointStates)):
            movementData.append(round(jointStates[i][0], 3))
        self.state = np.array(movementData)
        
    def isDone(self):

        ypr = p.getEulerFromQuaternion(self.orientation)
        if(ypr[0]>1.57 or ypr[0]<-1.57 or ypr[1]>1.57 or ypr[1]<-1.57): # Bigger than 100 degrees
            self.reward -= 0.05
            return True
        else:
            return False


    def step(self, action):
        # info current state
        posAndOrient = p.getBasePositionAndOrientation(self.quadrupedId)
        self.position = posAndOrient[0]
        self.orientation = posAndOrient[1]
        X = round(self.position[0], 3) * 100 # to cm
        Y = round(self.position[1], 3) * 100
        Z = round(self.position[2], 3) * 100

        # take action       
        p.setJointMotorControlArray(self.quadrupedId,
                                    jointIndices=self.revoluteJoints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=action,
                                    forces=self.maxMotorForce)
        self.stepTime()
        # reward computation
        dX = X - self.prev_X
        dY = Y - self.prev_Y
        # print("dX" + str(dX))
        self.reward = dX*1.8 - abs(dY)
        
        #print("dY" + str(Y - prev_Y))
        self.prev_X = X
        self.prev_Y = Y

        if(self.prev_X<0.1): # Look if it is okay respect to time elapsed
            self.reward -= 0.0001

        if(Z<14):
            self.reward -= 0.0001

        self.getState()

        # Observation, reward, done, info
        return self.state, self.reward, self.isDone(), {} 
        
        


    def reset(self):
        p.resetSimulation()
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.2]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.quadrupedId = p.loadURDF("/home/manu/catkin_ws/src/URDF_Quadruped/URDF/quadruped.urdf",cubeStartPos, cubeStartOrientation)
        self.getState()
        return self.state
    
    def render(self, mode='human'):
        pass

    def close(self):
        pass
