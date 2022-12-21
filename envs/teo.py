import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data

JOINT_IDS = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,23,24,25,26,27,28]
USELESS_JOINTS = [2,3, 8,9, 14,15] # head and wrists, not usefull for balance
VALID_JOINT_IDS = [id for id in JOINT_IDS if id not in USELESS_JOINTS]

class TEO:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep

    self.reset()

  def reset(self):

    # measured at root waist
    teoStartOrientation = p.getQuaternionFromEuler([0,0,0])
    teoStartPosition = [0,0,0.851]
    self.teoUid = p.loadURDF(os.path.join(self.urdfRootPath, "TEO.urdf"), teoStartPosition ,teoStartOrientation)
    self.numJoints = p.getNumJoints(self.teoUid)
    

    # set up the robot parameters
    self.jointIndices = list(range(self.numJoints))
    self.jointPositions = [0.0] * self.numJoints
    self.maxVelocities = [math.radians(1)] * self.numJoints
    self.maxForces = [0.0] * self.numJoints
    self.jointPositions = [0] * self.numJoints  

    for i in range(self.numJoints):
        self.maxForces[i] = p.getJointInfo(self.teoUid,i)[10]

    # Set the starting position/action
    for jointIndex in self.jointIndices:
      p.resetJointState(self.teoUid, jointIndex, self.jointPositions[jointIndex])
      p.setJointMotorControl2(self.teoUid,
                              jointIndex,
                              p.POSITION_CONTROL,
                              targetPosition=self.jointPositions[jointIndex],
                              force=self.maxForces[jointIndex])

    self.motorNames = []
    self.motorIndices = []


    # set motor names and indices
    for i in VALID_JOINT_IDS:#self.jointIndices:
      jointInfo = p.getJointInfo(self.teoUid, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)
    


  def getActionDimension(self):
    return len(self.motorIndices)


  def getObservationDimension(self):
    return len(self.getObservation())


  def getObservation(self):
    observation = []

    linear, angular = p.getBaseVelocity(self.teoUid)
    joint_states = p.getJointStates(self.teoUid, VALID_JOINT_IDS)
    joint_positions = [joint_state[0] for joint_state in joint_states]
    joint_velocities = [joint_state[1] for joint_state in joint_states]

    observation.extend(list(linear))
    observation.extend(list(angular))
    observation.extend(list(joint_positions))
    observation.extend(list(joint_velocities))

    return observation


  def applyAction(self, motorCommands):
    for action in range(len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.teoUid,
                                motor,
                                p.POSITION_CONTROL,
                                targetPosition=motorCommands[action],
                                force=self.maxForces[action])






















