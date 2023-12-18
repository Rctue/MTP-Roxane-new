import pandas as pd
from pynput import keyboard
from datetime import datetime
import numpy as np
import pybullet as p
from Can_module import Can
from Definitions_VR import *

y_target = 0.8

class VRControllers:

    def __init__(self):
        pass

    
    def infer_human_intention():
        pass


    

    def checkPoint(target, currentLoc):
        margin = 0.01
        if not target[0]-margin < currentLoc[0] < target[0]+margin:
            return False
        if not target[1]-margin < currentLoc[1] < target[1]+margin:
            return False
        if not target[2]-margin < currentLoc[2] < target[2]+margin:
            return False
        return True

    # def drawDebugLines(self, pos, dirX, dirY, dirZ, dirZ_opposite,lineLen): # OPTIONAL
    #     mat = p.getMatrixFromQuaternion(event[ORIENTATION])
    #     dirX = [mat[0], mat[3], mat[6]] # to the right
    #     dirY = [mat[1], mat[4], mat[7]] # upwards
    #     dirZ = [mat[2], mat[5], mat[8]] # forward
    #     dirZ_opposite = [-mat[2], -mat[5], -mat[8]] # backward
        
    #     lineLen = 0.1

    #     t0X = [pos[0] + lineLen * dirX[0], pos[1] + lineLen * dirX[1], pos[2] + lineLen * dirX[2]]
    #     t0Y = [pos[0] + lineLen * dirY[0], pos[1] + lineLen * dirY[1], pos[2] + lineLen * dirY[2]]
    #     t0Z = [pos[0] + lineLen * dirZ[0], pos[1] + lineLen * dirZ[1], pos[2] + lineLen * dirZ[2]]
    #     t0Z_opposite = [pos[0] + lineLen * dirZ_opposite[0], pos[1] + lineLen * dirZ_opposite[1], pos[2] + lineLen * dirZ[2]]

    #     p.addUserDebugLine(pos, t0X, [1, 0, 0], 1)
    #     p.addUserDebugLine(pos, t0Y, [0, 1, 0], 1)
    #     p.addUserDebugLine(pos, t0Z, [0, 0, 1], 1)
    #     p.addUserDebugLine(pos, t0Z_opposite, [0.5,0.5,0.],1,3)