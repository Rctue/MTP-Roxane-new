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

###########################################################################################################################
# Settings
###########################################################################################################################

# Constants
path = "C:/Users/roxan/Dropbox/TUe/Master/Master Thesis/Python/Pybullet/"
dataPath = "C:/Users/roxan/Dropbox/TUe/Master/Master Thesis/Python/PyBullet/Data/"
participantID = 999     #ID of the participant, used in file paths for data output
conditionID = 0             #Index of the counterbalanced group from list "groups"                        

# Data directory setup
try:
    os.mkdir(dataPath+"P"+str(participantID))
except:
    print("Directory already created")
dataPath = dataPath + "P" + str(participantID) + "/"

# Task configurations
tasks = [0,1,2,3,4]
max_task_nr = 5

# Time configurations
track_time = 0.5
time_between_tasks = 0.3    #seconds
time_between_cans = 0.5     #seconds
time_to_grab = 3            #seconds
time_to_correct = 0.5       #seconds
time_to_box = 2             #seconds
time_to_return = 1        #seconds
trajectory = {'start': None, 'current': None} # to store the start and current position

# Other configurations
distance_until_orientation = 0
s = 1                       # Scale factor
tracking = False            # Tracking
doPractice = False          # Include practice rounds + explanations
SLEEP_TIME = 1              # Normally 10
controllers_present = False # Check for use VR

# Environment constants
box = np.nan
tl = [0*s,1*s,0*s]
startOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
list_cans = [0, 1, 2]
nrCans = 3
boxsize = 0.25*s

# Information position from VR Events
CONTROLLER_ID = 0
CURRENT_POSITION = 1
ORIENTATION = 2
ANALOG = 3
BUTTONS = 6