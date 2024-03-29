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
path = "C:/Users/participant/Downloads/MTP Roxane/"
dataPath = "C:/Users/participant/Downloads/MTP Roxane/Data"
# participantID = 999     #ID of the participant, used in file paths for data output
# conditionID = 0         #Index of the counterbalanced group from list "groups"                        

# Task configurations
tasks = [0,1,2]
max_task_nr = 3

# Time configurations
track_time = 0.5
#time_between_tasks = 3     #seconds
time_between_cans = 0.5     #seconds
time_to_grab = 1            #seconds
time_to_correct = 0.5       #seconds
time_to_box = 3             #seconds
time_to_return = 1          #seconds
trajectory = {'start': None, 'current': None} # to store the start and current position

# Other configurations
distance_until_orientation = 0
s = 1                       # Scale factor
tracking = False            # Tracking
doPractice = False          # Include practice rounds + explanations
SLEEP_TIME = 10             # Normally 10
controllers_present = True  # Check for use VR
task_state = "not_done"

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