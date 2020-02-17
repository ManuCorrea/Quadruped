'''
This is the main definition of the environment
# TODO Add something like contact_cost in https://github.com/openai/gym/blob/master/gym/envs/mujoco/ant.py but with rewards and just the tibias or the "feet".
       For example: With the feet in contact with the plane or supports it will be more stable and is something close to learn to walk(can't walk with the 
       feet in the air)
'''

import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np

import pybullet as p
import time
import pybullet_data
import time

class QuadrupedEnv(gym.Env):
    def __init__(self, timeStep=0.005, visualize=False):

        if(visualize):
            physicsClient = p.connect(p.GUI) # graphical version of pybullet
        else:
            physicsClient = p.connect(p.DIRECT) # non-graphical version of enviroment, faster computation
            p.setRealTimeSimulation(0) # disable real time simulation

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.2]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.quadrupedId = p.loadURDF("URDF_Quadruped/URDF/quadruped.urdf",cubeStartPos, cubeStartOrientation)

        self.timeStep = timeStep

        self.observation_space = spaces.Box(low=-1.57, high=1.57, shape=(15,)) # +-90 degrees of observation
        self.action_space = spaces.Box(low=-1.57, high=1.57, shape=(12,)) #' # +-90 degrees of action per joint
        
        self.visualize = visualize

        # Need to define this Order to an easy implementation later in the real world, names taken from URDF file
        self.jointsOrder = ["S1_rot", "S2_rot", "S3_rot", "S4_rot",
                            "rot_femur1", "rot_femur2", "rot_femur3", "rot_femur4",
                            "knee1", "knee2", "knee3", "knee4"]

        self.nJoints = p.getNumJoints(self.quadrupedId)

        self.revoluteJoints = [i for i in range(12)]

        # For loop to get joint index according to the order in jointsOrder
        for idx, name in enumerate(self.jointsOrder):
            for i in range(self.nJoints):
                jointInfo = p.getJointInfo(self.quadrupedId, i)
                jointName = jointInfo[1].decode("utf-8")
                if jointName == name:
                    self.revoluteJoints[idx] = i

        # variables for computing reward
        self.prev_X = 0.0
        self.prev_Y = 0.0
        self.getState()
        self.prevJoint = self.state[:12] 
        self.episodeRewards = []
        self.steps = 0

        # TODO check real motor force with proper setup
        self.maxMotorForce = [3.5 for i in range(len(self.revoluteJoints))]

    def stepTime(self):
        """
        Function for make time advance
        """
        # TODO use setTimeStep function from API and making it works
        # previous use of setTimeStep collisions were not computed properly
        for _ in range(int(0.5/self.timeStep)):
            p.stepSimulation()
            if self.visualize:
                time.sleep(1/240)

    def getState(self):
        posAndOrient = p.getBasePositionAndOrientation(self.quadrupedId)
        self.position = posAndOrient[0]
        self.orientation = posAndOrient[1]

        jointStates = p.getJointStates(self.quadrupedId, self.revoluteJoints)
        self.ypr = p.getEulerFromQuaternion(self.orientation)
        movementData = []
        for i in range(len(jointStates)):
            movementData.append(round(jointStates[i][0], 3))
        self.ypr = [round(i, 3) for i in self.ypr]
        movementData += list(self.ypr)
        self.state = np.array(movementData)
        
    def isDone(self):    
        # When it falls over it is done    
        if(self.ypr[0]>1.57 or self.ypr[0]<-1.57 or self.ypr[1]>1.57 or self.ypr[1]<-1.57): # X or Y bigger than 100 degrees
            self.reward -= 4
            self.episodeRewards = []
            return True
        # if it get low very low rewards it is done
        elif sum(self.episodeRewards) < -200.0 or self.steps > 200:
            self.episodeRewards = []
            return True
        else:
            return False


    def step(self, action):
        self.steps += 1
        # take action       
        p.setJointMotorControlArray(self.quadrupedId,
                                    jointIndices=self.revoluteJoints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=action,
                                    forces=self.maxMotorForce)
        self.stepTime()

        # info current state
        self.getState()
        
        # All units in pybullet are in meters
        X = round(self.position[0], 3) * 100 # to cm
        Y = round(self.position[1], 3) * 100
        Z = round(self.position[2], 3) * 100

        # reward computation
        dX = X - self.prev_X
        dY = Y - self.prev_Y

        # print("{} {} {}".format(self.prev_X, X, dX))

        if (self.ypr[2]<0.5236 and self.ypr[2] > -0.5236): # orientation -30 to 30 degrees
            if(dX>0):
                self.reward = dX*4.0 - abs(dY) #encourage going forward more than penalizing backwards
            elif (dX > -2):
                self.reward = - abs(dY)
            else:
                self.reward = dX - abs(dY)
        else:
            self.reward = -abs(dY)
        
        movementCost = self.prevJoint - self.state[:12]
        movementCost = [abs(diff) for diff in movementCost]
        self.reward -= sum(movementCost)

        self.prev_X = X
        self.prev_Y = Y
        self.prevJoint = self.state[:12]

        if(self.prev_X<0.1): # TODO Look if it is okay respect to time elapsed
            self.reward -= 0.01

        # Observation, reward, done, info
        self.episodeRewards.append(self.reward)
        return self.state, self.reward, self.isDone(), {}     

    def reset(self):
        p.resetSimulation()
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.2]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.quadrupedId = p.loadURDF("/home/manu/catkin_ws/src/URDF_Quadruped/URDF/quadruped.urdf",cubeStartPos, cubeStartOrientation)
        self.getState()
        self.prevJoint = self.state[:12]
        return self.state
    
    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()
