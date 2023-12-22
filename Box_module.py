import pandas as pd
import numpy as np
import pybullet as p
import math
from Definitions_VR import *

########################################################################################################
##                                        BOX CODE                                                    ##
########################################################################################################

class Box:
    def __init__(self, position, scale, max):
        self.boxOpen = True
        self.pos = position
        self.id = p.loadURDF(path+"box.urdf",basePosition=position, globalScaling=scale, useFixedBase=1)
        self.cans_in_box = []
        self.max = max
        self.spots = self.getSpots()

    def getID(self):
        return self.id
    def getPos(self):
        return p.getBasePositionAndOrientation(self.id)[0]
    def setPos(self,pos):
        p.getBasePositionAndOrientation(self.id)[1]
        p.resetBasePositionAndOrientation(self.id, pos, p.getQuaternionFromEuler([0,0,0]))
    
    def reset(self):
        self.openBox()
        self.cans_in_box = []
        self.setPos(self.pos)
    
    def emptySpot(self):
        self.spots = self.getSpots()
        return self.spots[len(self.cans_in_box)]

    def getSpots(self):
        dim = math.ceil(math.sqrt(self.max))
        spots = []
        loc = self.getPos()
        bounds = p.getAABB(self.id)
        sizeX = (bounds[1][0]-bounds[0][0])/dim 
        sizeY = (bounds[1][1] - bounds[0][1])/dim
        dx = sizeX/2
        for x in range(dim):
            dy = sizeY/2
            for y in range(dim):
                spot = (bounds[0][0]+dx, bounds[0][1]+dy, loc[2]+0.03*s)
                spots.append(spot)
                dy += sizeY
            dx += sizeX
        return spots

    def openBox(self):
        p.setJointMotorControl2(self.id, 3, p.POSITION_CONTROL, 0)
        p.setJointMotorControl2(self.id, 1, p.POSITION_CONTROL, 0)
        self.boxOpen = True

    def fellOnFloor(self, current_can):
        location = current_can.getPos()
        return (location[2]<0.5) # table is 70 cm high, so if it is smaller we assume it fell
          

    def inBox(self, current_can):
        location = current_can.getPos()
        bounds = p.getAABB(self.id)
        if bounds[0][0] > location[0] or location[0] > bounds[1][0]:
            return False
        if bounds[0][1] > location[1] or location[1] > bounds[1][1]:
            return False

        inList = False
        current_can.setBox(True) # can.setBox(True) would set the box attribute of the can to True

        for a_can in self.cans_in_box:
            if a_can.getID() == current_can.getID():
                inList = True

        if not inList:
            self.cans_in_box.append(current_can)
            # print(self.cans_in_box)
        return True

    def isFull(self):
        if len(self.cans_in_box) < self.max:
            return False
        self.closeBox()
        return True
    
    def closeBox(self):
        p.setJointMotorControl2(self.id, 3, p.POSITION_CONTROL, 3.927)
        p.setJointMotorControl2(self.id, 1, p.POSITION_CONTROL, 3.927)
        self.boxOpen = False