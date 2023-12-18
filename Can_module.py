import pandas as pd
from pynput import keyboard
from datetime import datetime
import time
import numpy as np
import pybullet as p
import math
import pybullet_data
import threading
import os
from Definitions_VR import *

########################################################################################################
##                                     SPHERE CODE                                                    ##
########################################################################################################

class Can:
    scale = 0.03*s

    def __init__(self, position):
        self.orn=p.getQuaternionFromEuler([math.pi/2,0,0])
        self.id = p.loadURDF(path+"cups/cup.urdf",basePosition=position, baseOrientation=self.orn, globalScaling=self.scale)
        self.pos = self.getPos()
        #self.color = color
        self.box = False
        self.state = "on_table"

    def getID(self):
        return self.id
    def getPos(self):
        pos = p.getBasePositionAndOrientation(self.id)[0]
        return pos
    
    # can.setBox(True) would set the box attribute of the can to True
    def setBox(self, box):
        self.box = box

    def setPos(self,pos):
        orn = p.getBasePositionAndOrientation(self.id)[1]
        p.resetBasePositionAndOrientation(self.id, pos, orn)
    def reset(self):
        p.removeBody(self.id)
        self.box = False
        self.id = p.loadURDF(path+"cups/cup.urdf",basePosition=self.pos, baseOrientation=self.orn, globalScaling=self.scale)