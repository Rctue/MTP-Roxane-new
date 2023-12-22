import pandas as pd
from datetime import datetime
import time
import numpy as np
import pybullet as p
import math
import os
from Definitions_VR import *

###########################################################################################################################
# Functions
###########################################################################################################################

def checkPoint(target, currentLoc):
    marge = 0.01*s
    if not target[0]-marge < currentLoc[0] < target[0]+marge:
        return False
    if not target[1]-marge < currentLoc[1] < target[1]+marge:
        return False
    if not target[2]-marge < currentLoc[2] < target[2]+marge:
        return False
    return True

########################################################################################################
##                                         PANDA CODE                                                 ##
########################################################################################################

class Panda:
    ll = [] #lower limits for null space
    ul = [] #upper limits for null space
    jr = [] #joint ranges for null space
    rp = [] #restposes for null space
    jd = [] #joint damping coefficents

    def __init__(self, type, position, orientation, scale):
        self.initialize_parameters(type, position, orientation, scale)
        self.setup_joint_information()

    # Initialisation Methods
    def initialize_parameters(self, type, position, orientation, scale):
        """Initializes robot parameters."""
        self.type = type
        self.position = position
        self.load_robot_urdf(type, position, orientation, scale)

        self.basePose = (0, -math.pi/6, 0, -5*math.pi/6, 0, 1.1*math.pi, math.pi/4, 0.08*scale, 0.08*scale)
        self.basePoint = (self.position[0]-0.5169*s, self.position[1]+0*s, self.position[2]+0.5654*s)
        self.setJoints(self.basePose)
        self.openGripper()

    def load_robot_urdf(self, type, position, orientation, scale):
        """Loads the appropriate robot URDF based on type."""
        if type == 0:          # Normal 
            file = "panda.urdf"     
        elif type == 1:        # Anticipatory 
            file = "panda.urdf"
        elif type == 2:        # Legibile
            file = "panda_snake.urdf"
        elif type == 3:        # Anticipatory + Legible
            file = "panda_snake.urdf"
        
        print(path + "PyBullet/franka_panda/" + file)
        self.id = p.loadURDF(path + "franka_panda/" + file, position, orientation,
                             useFixedBase=1,
                             flags=p.URDF_USE_INERTIA_FROM_FILE |
                             p.URDF_USE_SELF_COLLISION |
                             p.URDF_MAINTAIN_LINK_ORDER,
                             globalScaling=scale)
        p.changeDynamics(self.id, -1, linearDamping=0, angularDamping=0)

    def setup_joint_information(self):
        """Sets up joint information."""    
        for j in range(p.getNumJoints(self.id)):
          info = p.getJointInfo(self.id, j)
          jointLL = info[8]
          jointUL = info[9]
          jointRange = jointUL-jointLL
          self.ll.append(jointLL)
          self.ul.append(jointUL)
          self.jr.append(jointRange)
          self.rp.append(0)
          self.jd.append(info[7])

    # Gripper Control Methods
    def openGripper(self):
        """Opens the gripper."""
        p.setJointMotorControl2(self.id, 9, p.POSITION_CONTROL, 0.08*s)
        p.setJointMotorControl2(self.id, 10, p.POSITION_CONTROL, 0.08*s)

    def closeGripper(self):
        """Closes the gripper."""
        p.setJointMotorControl2(self.id, 9, p.POSITION_CONTROL, 0.0, force = 500)
        p.setJointMotorControl2(self.id, 10, p.POSITION_CONTROL, 0.0, force = 500)

    # Joint Control Methods
    def setJoints(self, jointsPositions, gradual=False, steps=20, t=time_to_return):
        """Sets the robot joints."""
        start = datetime.now()
        if steps == 0:
            return True
        j = 0
        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                if (jointType != p.JOINT_PRISMATIC):
                    if gradual:
                        self.setJointGradually(i, jointsPositions[j], steps)
                    else:
                        p.setJointMotorControl2(bodyIndex=self.id,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=jointsPositions[j],
                                        force=2000, positionGain=0.05)
                j+=1
        if gradual:
            dt = (datetime.now() - start).total_seconds()
            if dt < t / steps:
                time.sleep(t / steps - dt)
            self.setJoints(jointsPositions, True, steps-1, t - t/steps)

    def setJointGradually(self, joint, targetPos, steps):
        """Sets a joint's position gradually."""
        if steps == 0:
            return True
        currPos = p.getJointState(self.id, joint)[0]
        if currPos == targetPos:
            return True
        diffStep = abs(currPos-targetPos)/steps
        if targetPos > currPos:
            newPos = currPos + diffStep
        else:
            newPos = currPos - diffStep
        p.setJointMotorControl2(bodyIndex=self.id,
                                        jointIndex=joint,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=newPos,
                                        force=2000, positionGain=0.05)

    # Inverse Kinematics Methods
    def calcJoints(self, targetPos):
        """Calculates joint positions."""
        jointPoses = p.calculateInverseKinematics(self.id,
                                                    self.end(),
                                                    targetPos,
                                                    lowerLimits=self.ll,
                                                    upperLimits=self.ul,
                                                    jointRanges=self.jr,
                                                    restPoses=self.rp,
                                                    maxNumIterations=500,
                                                    residualThreshold=.005
                                                    )
        return jointPoses

    def calcJoints2(self, list, targetPos):
        """"Calculates joint positions using another method."""
        jointPoses = p.calculateInverseKinematics2(self.id,
                                                    list,
                                                    targetPos,
                                                    self.ll, self.ul,
                                                    self.jr, self.rp,
                                                    maxNumIterations=1000,
                                                    residualThreshold=.001)                                   
        return jointPoses

    # Motion Methods
    def realTimeSim(self, totaltime, step, path, final):
        """Real-time simulation of robot movement."""
        startTime = datetime.now()
        if step >= len(path):
            return True
        tStep = totaltime/(len(path)-step)
        if final != None and checkPoint(self.getEndLocation(), final):
            time.sleep(max(tStep*8,0.001))
            return True

        if final != None:
            if self.type == 2 or self.type == 3:
                if step == 0:
                    # print(f"step: {step}, \n, path : {path}")
                    self.moveToPoint(path[step][0:3], final, True, tStep, close=path[step][4])
                else:
                    self.moveToPoint(path[step][0:3], final, close=path[step][4])
        else: 
            self.moveToPoint(path[step][0:3], final)
        time_elapsed = (datetime.now()-startTime).total_seconds()
        if time_elapsed < tStep:
            time.sleep(tStep-time_elapsed)
        newTime = totaltime - (datetime.now()-startTime).total_seconds()
        step += 1
        self.realTimeSim(newTime, step, path, final)

    def moveToPoint(self, target, final, grad=False, tStep=0, close=False):
        """Moves the robot to a point."""

        # Condition 2 behaves differently
        if self.type == 2 or self.type == 3 and final and not close:
            hand = self.calcHand(final,target)
            jointPoses = self.calcJoints2([8,11,12], [hand, target, self.calcGlasses(hand,target)])
        else:
            jointPoses = self.calcJoints(target)

        if grad and self.type == 2 or self.type == 3:
            self.setJoints(jointPoses, True, 20, tStep)
        else:
            self.setJoints(jointPoses)
        return checkPoint(target, self.getEndLocation())

    def moveToBase(self):
        """Moves the robot to its base position."""
        self.setJoints(self.basePose, True, 30, time_to_return)
        self.openGripper()
        return checkPoint(self.basePoint, self.getEndLocation())

    # Utility Methods
    def getID(self):
        """Returns the robot's ID."""
        return self.id
    
    def end(self):
        """"Returns the end effect index."""
        return 11 #pandasEndEffectorIndex = 11

    def getEndLocation(self):
        """Returns the location of the end effector."""
        linkWorldPosition,_,_,_,_,_ = p.getLinkState(self.id, self.end())
        return linkWorldPosition

    def distTwoLinks(self, one=8, two=11):
        """"Calculates distance between two links."""
        pos1 = p.getLinkState(self.id, one)[0]
        pos2 = p.getLinkState(self.id, two)[0]
        
        diff = [pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]]
        dist = math.sqrt(math.pow(diff[0],2) + math.pow(diff[1],2) + math.pow(diff[2],2))
        return dist

    def distToObject(self, pos1, pos2):
        """"Calculates the distance to an object."""
        diff = [pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]]
        dist = math.sqrt(math.pow(diff[0],2) + math.pow(diff[1],2) + math.pow(diff[2],2))
        return dist
    
    def lookAtPoint(self, point, t):
        pos = p.getLinkState(self.id, 8)[0]
        endPos = self.calcEnd(point,pos)
        jointPoses = self.calcJoints2([8,11, 12], [pos, endPos, self.calcGlasses(pos, endPos)])
        self.setJoints(jointPoses, True, 20, t)

    def calcHand(self, can, path, hand=8,end=11):
        """Calculates hand position."""
        trajectory = path
        object = can
        links= self.distTwoLinks(hand,end)
        r = math.sqrt(math.pow(links,2) / (math.pow(trajectory[0]-object[0],2) + math.pow(trajectory[1]-object[1],2) + math.pow(trajectory[2]-object[2],2)))
        ax = trajectory[0] - r*(object[0]-trajectory[0])
        ay = trajectory[1] - r*(object[1]-trajectory[1])
        az = trajectory[2] - r*(object[2]-trajectory[2])
        targetPos = [ax, ay, az]
        return targetPos
    
    def calcGlasses(self, hand, end):
        ##calc height:
        cd = self.distTwoLinks(8,12)
        bc = self.distTwoLinks()
        dz_bc = abs(hand[2]-end[2])
        d = (dz_bc/bc) %1
        ch = cd * math.cos(math.asin(d))
        z = hand[2]+ch

        #calc x&y
        dh = math.sqrt(math.pow(cd,2) - math.pow(ch,2))
        bh = math.sqrt(math.pow((hand[0]-end[0]),2) + math.pow((hand[1]-end[1]),2))
        dp = (abs(hand[1]-end[1])*dh)/bh
        y = hand[1] - dp

        hp = ((end[0]-hand[0])*dh)/bh
        x = hand[0] + hp

        pos = [x,y,z]
        return pos

    def calcEnd(self, can, handloc, hand=8,end=11):
        """Calculates end position."""
        trajectory = handloc
        object = can
        links = self.distTwoLinks(hand,end)
        r = math.sqrt(math.pow(links,2) / (math.pow(trajectory[0]-object[0],2) + math.pow(trajectory[1]-object[1],2) + math.pow(trajectory[2]-object[2],2)))
        ax = trajectory[0] + r*(object[0]-trajectory[0])
        ay = trajectory[1] + r*(object[1]-trajectory[1])
        az = trajectory[2] + r*(object[2]-trajectory[2])
        targetPos = [ax, ay, az]
        return targetPos