import pandas as pd
from pynput import keyboard
from datetime import datetime
import random
import time
import numpy as np
import pybullet as p
import math
import pybullet_data
import threading
import os
from Definitions_VR import *

########################################################################################################
##                                     TRAJECTORY CODE                                                ##
########################################################################################################

class TrajectoryCalculator:
    def __init__(self):
        self.y_target=0.8

    @staticmethod
    def linearTrajectory(startPos, endPos, nrsteps=100):
        """
        Calculates the trajectory based on a normal linear method
        Condition: 0
        """
        locs = []

        for i in range(nrsteps + 1):  # +1 to include the endpoint
            distance_traveled = i / nrsteps  # proportion of the distance traveled

            x = startPos[0] + distance_traveled * (endPos[0] - startPos[0])
            y = startPos[1] + distance_traveled * (endPos[1] - startPos[1])
            z = startPos[2] + distance_traveled * (endPos[2] - startPos[2])

            targetPos = [x, y, z]
            locs.append(targetPos)

        return locs

    def linearRegression(self, start_position, current_position):
        """ Using linear regression, predict the x value for a given
            y value using the start and current positions.

            y_target = 0.8 (taken from the fixed placement of the cans in the environment)
        """
        x_coords = [start_position[0], current_position[0]]
        y_coords = [start_position[1], current_position[1]]

        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        m, b = np.linalg.lstsq(A, y_coords, rcond=None)[0]
        
        predicted_x = (self.y_target - b) / m
       
        return predicted_x

    def predictHumanTrajectory(self):
        """
        Calculates the trajectory based on start and current position using linear regression.
        Condition: 1
        """

        if not trajectory['start'] or not trajectory['current']:
            print("Either start or current position is not available.")
            return None

        predicted_x = self.linearRegression(trajectory['start'], trajectory['current'])
        predicted_position = (predicted_x, self.y_target, trajectory['current'][2])
        
        # You can use the following line to draw a line between current position and predicted position in PyBullet
        p.addUserDebugLine(trajectory['current'], predicted_position, [1,0,0], lifeTime=1)  # Red color line for 1 second

        return predicted_position

    @staticmethod
    def minimumJerk(start, stop):
        t0 = start[0]
        t1 = stop[0]  
        minjerkmat = np.matrix([[1, t0, math.pow(t0,2), math.pow(t0,3), math.pow(t0,4), math.pow(t0,5)],
                                [0, 1,  2*t0, 3*math.pow(t0,2), 4*math.pow(t0,3),  5*math.pow(t0,4)],
                                [0, 0,  2,    6*t0,   12*math.pow(t0,2), 20*math.pow(t0,3)],
                                [1, t1, math.pow(t1,2), math.pow(t1,3),   math.pow(t1,4),    math.pow(t1,5)],
                                [0, 1,  2*t1, 3*math.pow(t1,2), 4*math.pow(t1,3),  5*math.pow(t1,4)],
                                [0, 0,  2,    6*t1,   12*math.pow(t1,2), 20*math.pow(t1,3)]])
        loc_vector= start[1:4] + stop[1:4]
        x_vec = np.linalg.lstsq(minjerkmat,loc_vector, rcond=None)[0]
        return x_vec.reshape((6,1))

    @staticmethod
    def evaluateJerk(x_vec, tval):
        tmat = np.matrix([[ 1, tval, math.pow(tval,2), math.pow(tval,3),   math.pow(tval,4),    math.pow(tval,5)],
                        [0, 1,    2*tval, 3*math.pow(tval,2), 4*math.pow(tval,3),  5*math.pow(tval,4)],
                        [0, 0,    2,      6*tval,   12*math.pow(tval,2), 20*math.pow(tval,3)]])
        ret= tmat*x_vec
        return ret

    @staticmethod
    def legibleTrajectory(startPos, endPos, startV = [0,0,0], endV = [0,0,0], startA=[0,0,0], endA=[0,0,0], nrsteps=100, distance=False): 
        """
        Calculates the trajectory based on an animal-like legible method
        Condition: 2
        """
        xstart = [0,startPos[0],startV[0],startA[0]]    # Creates start info based on (t(0), start(x), start(Vx), start(Ax))
        ystart = [0,startPos[1],startV[1],startA[1]]    # ... (t(0), start(y), start(Vy), start(Ay))
        zstart = [0,startPos[2],startV[2],startA[2]]    # ... (t(0), start(z), start(Vz), start(Az))
        xstop = [1,endPos[0],endV[0],endA[0]]     # Creates end info based on (t(1), end(x), end(Vx), end(Ax))
        ystop = [1,endPos[1],endV[1],endA[1]]     # ... (t(1), start(y), start(Vy), start(Ay))
        zstop = [1,endPos[2],endV[2],endA[2]]     # ... (t(1), start(z), start(Vz), start(Az))

        x_vec = TrajectoryCalculator.minimumJerk(xstart, xstop)
        y_vec = TrajectoryCalculator.minimumJerk(ystart, ystop)
        z_vec = TrajectoryCalculator.minimumJerk(zstart,zstop)

        dt =1/nrsteps
        locs = []
        
        changed = False
        for i in range(nrsteps):
            x = TrajectoryCalculator.evaluateJerk(x_vec, i*dt)[0].item()
            y = TrajectoryCalculator.evaluateJerk(y_vec, i*dt)[0].item()
            z = TrajectoryCalculator.evaluateJerk(z_vec, i*dt)[0].item()
            pos = [x,y,z]
            if distance:
                diff = [pos[0]-endPos[0], pos[1]-endPos[1], pos[2]-endPos[2]]
                dist = math.sqrt(math.pow(diff[0],2) + math.pow(diff[1],2) + math.pow(diff[2],2))
                turningPoint = False
                if dist < distance_until_orientation*s:
                    if not changed:
                        turningPoint = True
                    changed = True
                pos.append(turningPoint)
                pos.append(changed)
            locs.append(pos)

        return locs