import pandas as pd
from pynput import keyboard
from datetime import datetime
import time
import numpy as np
import pybullet as p
import math
import pybullet_data
import threading
import Definitions
from Panda_module import Panda
from Box_module import Box
from Can_module import Can
from Trajectory_module import TrajectoryCalculator
from VR_module import VRControllers

path = Definitions.path
dataPath = Definitions.dataPath
participantID = Definitions.participantID
groupID = Definitions.groupID                     
doPractice = Definitions.doPractice
groups = Definitions.groups
track_time = Definitions.track_time
time_between_cans = Definitions.time_between_cans
time_to_grab = Definitions.time_to_grab
time_to_correct = Definitions.time_to_correct
time_to_box = Definitions.time_to_box
time_to_return = Definitions.time_to_return
distance_until_orientation = Definitions.distance_until_orientation
trajectory = Definitions.trajectory
s = Definitions.s
tracking = Definitions.tracking
nrCans = Definitions.nrCans
boxsize = Definitions.boxsize
controllers_present = Definitions.controllers_present

CURRENT_POSITION = Definitions.CURRENT_POSITION
ORIENTATION = Definitions.ORIENTATION
ANALOG = Definitions.ANALOG
CONTROLLER_ID = Definitions.CONTROLLER_ID
BUTTONS = Definitions.BUTTONS
SLEEP_TIME = Definitions.SLEEP_TIME

###########################################################################################################################
# Functions
###########################################################################################################################

def checkPoint(target, currentLoc):
    margin = 0.01*s
    if not target[0]-margin < currentLoc[0] < target[0]+margin:
        return False
    if not target[1]-margin < currentLoc[1] < target[1]+margin:
        return False
    if not target[2]-margin < currentLoc[2] < target[2]+margin:
        return False
    return True
###########################################################################################################################
# Classes
###########################################################################################################################

class Experiment:
    END_TASK = "This was the end of this task \n You can take off the headset \n and continue with the questionnaire."

    def __init__(self, participant, group):
        self.group_index = group
        self.group = groups[group]
        self.id = participant
        self.data = pd.DataFrame(columns=['participantID', 'Group'])
        self.startNow = False
        self.stop = False

    def main(self):

        # Attempt to connect to the shared memory of pybullet.
        self.clid = p.connect(p.SHARED_MEMORY)
        print("VR is connected.", self.clid)
        # If connection to shared memory fails, connect using the GUI.
        if (self.clid < 0):
            p.connect(p.GUI)

    
        if doPractice:
            prac = Practice()
            prac.startPractice()

        # Start condition with specific task_nr
        for task_nr, condition in enumerate(self.group):
            task = Task(condition, task_nr)
            data = task.startCondition()
            self.data = pd.concat([self.data,data], axis=0, ignore_index=True)
        
        # Define data
        self.data['participantID'] = self.id
        self.data['Group'] = self.group_index
        self.data.to_excel(dataPath+str(self.id)+'_ReactionTimes.xlsx')

        # End Experiment (STILL HAVE TO DECIDE WHEN)
        if self.stop:
            self.env.displayText(self.END_TASK)
            time.sleep(1*SLEEP_TIME)
            self.env.stopEnvironment()

class Practice:
    HAND_CHECK = "Make sure you can see the grippers, \n and that they are kalibrated \n to your actual hands."
    GRAB_OBJECT = "Try to grab one of the objects \n and drop it in the box. \n Click and hold down to grab. \n Let go to let the object go."
    ALL_OBJECTS = "Try to put all objects \n into the box."
    REPEAT = "To repeat this practice task:\n Press the big button. \n Otherwise indicate to the leader \n that you want to start the experiment."

    def __init__(self):
        self.env = Environment(99)

    def startPractice(self):
        """ Start the practice task. """
        print("------ Start Practice: "+ str(datetime.now()))
        self.env.newPackage()

        # Check hand calibration
        self.env.displayText(self.HAND_CHECK)
        time.sleep(0.7*SLEEP_TIME)

        # Try to grab object
        self.env.displayText(self.GRAB_OBJECT)
        time.sleep(0.7*SLEEP_TIME)

        # Throw all objects in box
        self.env.displayText(self.ALL_OBJECTS)
        time.sleep(0.5*SLEEP_TIME)

        # Repeat the practice round
        self.env.displayText(self.REPEAT)
        l = keyboard.Listener(on_press=self.monitorKeyboard)    # Listen to keyboard
        l.start()
        self.practiceVRInput()  # Listen to VR controller
        
        time.sleep(2*SLEEP_TIME)
    
        # Shut down environment
        self.env.stopEnvironment()
        l.stop()
        l.join()
    
    def monitorKeyboard(self, key):
       """Check if spacebar is pressed."""
       if hasattr(key, 'name') and key.name == 'space':
            self.startNow = True
            self.env.newPackage()

    def practiceVRInput(self):
        """Check if button is pressed."""
        while not self.startNow:
            events = p.getVREvents()
            for e in (events):
                if e[BUTTONS][33] == p.VR_BUTTON_WAS_TRIGGERED:
                    self.startNow = True
                    self.env.newPackage()  
        
class Task:   
    """Represents a task where the robot interacts with red objects."""
    START_EXPERIMENT = "Now the real experiment starts. \n Make sure to work as fast as \n possible to put the objects in \n the box."
    START_TASK = "Press the BIG button \n to start with the first task."      
    CONTINUE_TEXT = "To continue with the next box \n press the BIG button"

    def __init__(self, panda_type, task_nr):
        """ Initialize the Task with a type and number.
            
            type: Type of the panda
            nr: Number or index of the can
        """        
        self.task_nr = task_nr
        self.panda_type = panda_type
        self.startNow = False
        self.list_cans = [0, 1, 2]
        self.start_time = datetime.now()
        self.data = pd.DataFrame(columns=['Condition', 'Start_condition'])
        self.env = Environment(self.panda_type)

    def startCondition(self):
        """Begin the condition and collect data."""
        print("----- Condition: " + str(self.panda_type))
        print("----- Task: " + str(self.task_nr))

        # Load new environment for task
        self.env.newPackage()
        if self.task_nr>0:
            self.env.displayText(self.CONTINUE_TEXT)

        
        self.env.displayText(self.START_EXPERIMENT)
        time.sleep(0.5*SLEEP_TIME)
        self.env.displayText(self.START_TASK)
        l = keyboard.Listener(on_press=self.monitorKeyboard)  # Listen to keyboard
        l.start()
        
        # Wait for button presses
        if controllers_present:
            while not self.startNow:
                self.startVRInput()  # Listen to VR controller
                time.sleep(0.05*SLEEP_TIME)

        l.stop()
        l.join()
        
        print("------ Start Experiment: " + str(datetime.now()))
        self.env.removeText()

        data = self.startTask()
        self.data = pd.concat([self.data, data], axis=0, ignore_index=True)
        self.data['Condition'] = self.panda_type
        self.data['Start_condition'] = datetime.now()
       
    def startTask(self):
        """Begin the task, run tasks, and collect data."""
        print("----Task Started----")

        # Loop through list to grab objects
        for can_nr in self.list_cans:
            if isinstance(can_nr, int):
                print("-----Object state: ", self.env.list_cans[can_nr].state)

                # Return object state to initial state
                if self.env.list_cans[can_nr].state == "predicted_target":
                    self.env.list_cans[can_nr].state == "on_table"
                    print("-----Object state: ", self.env.list_cans[can_nr].state)

                # Check if any objects on the table
                if self.env.list_cans[can_nr].state == "on_table":
                    results = self.executeCanTasks(can_nr)
                    self.data = pd.concat([self.data, results], axis=0, ignore_index=True)
                    print("-----Object state: ", self.env.list_cans[can_nr].state)

                # Check if object already in box
                elif self.env.list_cans[can_nr].state == "in_hand" or "in_box":
                    self.panda.moveToBase()
                    self.released = False
        
        return self.compileTaskData(self.data)

    def executeCanTasks(self, can_nr):
        """Execute the can grabbing task with the robot while collecting data."""
        print(f"--- Can: {can_nr}")
        if p.isConnected():
            print("Connected to the physics server.")
        else:
            print("Not connected to the physics server.")
        self.start_time = datetime.now()

        reached_time = self.env.robotExecuteTask(can_nr)
        time_reach_can = (reached_time - self.start_time).total_seconds()
        
        return pd.DataFrame([[can_nr, time_reach_can, self.start_time, reached_time]],
                            columns=['can', 'Time reaching','Start_time_can', 'Robot reached time'])

    def compileTaskData(self, data):
        """Add metadata to the task data and return it."""
        data['Task'] = "Task Data"
        data['Start_task'] = datetime.now()
        return data
    
    def monitorKeyboard(self, key):
       """Check if spacebar is pressed."""
       if hasattr(key, 'name') and key.name == 'space':
            self.startNow = True

    def startVRInput(self):
        """Check if button is pressed."""
        events = p.getVREvents()
        for e in (events):
            if e[BUTTONS][32] == p.VR_BUTTON_IS_DOWN:
                self.startNow = True

class Environment:

    def __init__(self, condition):
         # VR setup
        if controllers_present:
            self.setupVR()
        
        # Initialisation and setup
        self.setupEnvironment(condition)
        
        # Panda setup 
        self.setupPanda()
        # Tracking
        self.setupTracking()

    # Initialisation and Setup Methods
    def setupEnvironment(self, condition):
        self.panda_type = condition
        self.writer_ready = False
        self.stop_tracking = False
        self.update_ready = False

        

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        
        # Configure the visualizer to disable rendering and other settings.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, lightPosition=[0,-2,2])
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

        

        # Load plane and environment
        p.loadURDF("plane.urdf", [0, 0, 0])
        p.setGravity(0, 0, -10)

        # Define the table location and load the table into the simulation.
        self.tl = [0*s,1*s,0*s]
        startOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
        tableUid = p.loadURDF(path+"table_top/table.urdf",basePosition=self.tl, baseOrientation=startOrientation, globalScaling=s)
        p.loadURDF(path+"table/table.urdf",basePosition=self.tl, baseOrientation=startOrientation, globalScaling=s)
        self.th = p.getAABB(tableUid)[1][2]
        
        # Initialize box properties
        self.box = np.nan
        self.boxlocation = [self.tl[0]-0.35*s, self.tl[1], self.th]

        # Initialise variables
        self.list_cans = []
        self.texts = []
        self.explanation = None
        p.setRealTimeSimulation(1)
        print("Environment is setup.")

    def setupVR(self):
        p.resetSimulation()
        p.stepSimulation()
        self.vr = VRControllers()
        # self.vr_events = threading.Thread(target=self.vrEvents(), args=())
        self.vr_events = threading.Thread(target=self.vr.startEventLoop, args=())
        self.vr_events.start()
        # while 1:
        #     print("started the event loop")
        # pass

    def vrEvents(self):
        self.vr = VRControllers()
        print("VR is setup.")
        self.vr.startEventLoop()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def setupPanda(self):
        pandalocation = [self.tl[0], self.tl[1]+0.5*s, self.th]
        startOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
        if self.panda_type != 99:
            self.panda = Panda(self.panda_type, pandalocation,startOrientation, s)

        p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=0, cameraPitch=-20, cameraTargetPosition=[self.tl[0],self.tl[1],self.th])

        #camera position for screenshots:
        #p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=35, cameraPitch=-15, cameraTargetPosition=[pandalocation[0], pandalocation[1], pandalocation[2] + 0.4])

    def setupTracking(self):
        p.setRealTimeSimulation(1)
        if self.panda_type != 99:
            if tracking:
                self.track = threading.Thread(target=self.tracking, args=())
                self.track.start()
                while not self.writer_ready:
                    time.sleep(0.1*SLEEP_TIME)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Task Execution Methods
    def robotExecuteTask(self, can_nr):
        """Execute a given task with a specified can."""
        logging = [self.panda.getID()]
        self.reachedTime = None

        # Pause real-time simulation
        p.setRealTimeSimulation(0)
        
        # Start state logging for VR controllers, robot data, contact points
        log1 = p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS,
                                   dataPath+str(participantID)+"_con"+str(self.panda_type)+"_can"+ str(can_nr)+"_vr_hmd.txt",
                                   deviceTypeFilter=p.VR_DEVICE_HMD)
        log2 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                                   dataPath+str(participantID)+"_con"+str(self.panda_type)+"_can"+ str(can_nr)+"_generic_data.txt",
                                   objectUniqueIds =logging)
        log3 = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "contact_points.bin")
        
        # Resume real-time simulation
        p.setRealTimeSimulation(1)

        # Start grabbing the cans simultaneously
        arm = threading.Thread(target=self.grabOneCan, args=(self.list_cans[can_nr],))
        arm.start()

        # While the arm is moving, wait till task is complete
        while arm.is_alive():
            time.sleep(0.01*SLEEP_TIME)

        # Close the box
        self.box.Full()
        time.sleep(time_between_cans)

        # Stop state logging
        p.stopStateLogging(log1)
        p.stopStateLogging(log2)
        p.stopStateLogging(log3)
        return self.reachedTime
    
    def grabOneCan(self, can_pos):
        """Logic to grab a single can."""
        self.robot_reached = False
        self.human_reached = False
        self.released = False
        calculator = TrajectoryCalculator()
        
        # CONDITION 0
        if self.panda_type == 0:

            # Calculate trajectory to can
            pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
            self.panda.realTimeSim(time_to_grab, 0, pathcan, can_pos.getPos())

            # Reach the can
            self.reachedTime = datetime.now()

            while not self.robot_reached:
                pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
                self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
                self.robot_reached = checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            self.putCanInBox(can_pos)

        # CONDITION 1
        elif self.panda_type == 1:
            self.reachedTime = datetime.now()
            print("Anticipatory: Can: "+ str(can_pos.getID()))

            # Calculate trajectory to can (still relevant if also in while loop?)
            pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
            self.panda.realTimeSim(time_to_grab, 0, pathcan, can_pos.getPos())
            self.robot_reached = checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            while not self.robot_reached:
                if controllers_present:
                    # Predict human trajectory to can
                    if trajectory['start'] and trajectory['current']:
                        self.predicted_position = calculator.predictHumanTrajectory()
                        print(f"Predicted position: {self.predicted_position}")

                    # Check own target
                    if self.isnearObject(self.predicted_position[0], can_pos.getPos()[0]):
                        print("Predicted can:", can_pos.getID())
                        self.state = "predicted_target"
                        print("-----Object state: ", self.state)

                        # Change the can ID with ID below 2
                        self.id = can_pos.getID()
                        if self.id < 2:
                            self.new_id = self.id + 1
                            self.can_pos = p.getBasePositionAndOrientation(self.new_id)[0]
                            self.state = "on_table"

                            # Continue trajectory to different can
                            pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
                            self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
                            self.robot_reached = checkPoint(can_pos.getPos(), self.panda.getEndLocation())
                        else:
                            # Robot is done
                            self.state = "done"
                            print("-----Object state: ", self.state)
                            break
                
                self.state = "on_table" # Maybe redundant?
                print("-----Object state: ", self.state)
                pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
                self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
                self.robot_reached = checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            # Grab object based on state
            if self.state == "on_table":
                self.putCanInBox(can_pos)

            elif self.state == "done":
                # Return to base
                self.panda.moveToBase()
                self.released = False

        # CONDITION 2        
        elif self.panda_type == 2:
            # Initialise variables
            v = [0,0,0]
            a = [0,0,0.1]
            
            # Start gaze orientation
            startlook = datetime.now()
            self.panda.lookAtPoint(can_pos.getPos(),1)
            print("Legible: Can: "+ str(can_pos.getID()) + " Time to look: " + str((datetime.now()-startlook).total_seconds()))
            time_to_grab2 = time_to_grab*1.18 - (datetime.now()-startlook).total_seconds()
            
            # Calculate trajectory to can
            pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), np.array(can_pos.getPos()) + np.array([0,0,0.05*s]), startV=v, endA=a, distance=True)
            self.panda.realTimeSim(time_to_grab2, 0, pathcan, can_pos.getPos())

            # Reach the can
            self.reachedTime = datetime.now()

            while not self.robot_reached:
                pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), can_pos.getPos(), nrsteps=20, distance=True)
                self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
                self.robot_reached = checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            self.putCanInBox(can_pos)

        # STILL TO BE WRITTEN
        elif self.panda_type == 3:    
            # Calculate trajectory to can
            pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), can_pos.getPos(), startV=v, endA=a, distance=True)
            self.panda.realTimeSim(time_to_grab2, 0, pathcan, can_pos.getPos())

    def putCanInBox(self, can_pos):
        self.released = False
        calculator = TrajectoryCalculator()

        # Grip the Can
        self.panda.closeGripper()

        # Move to Box
        h = 0.4
        boxRelease = self.box.emptySpot()
        boxRelease = [boxRelease[0], boxRelease[1], boxRelease[2]+h*s]

        # Choose trajectory
        if self.panda_type == 2 or 3:
            pathBox = calculator.legibleTrajectory(self.panda.getEndLocation(), boxRelease, startA=[0,0,10*s])
        else:
            pathBox = calculator.linearTrajectory(self.panda.getEndLocation(), boxRelease)
        
        self.panda.realTimeSim(time_to_box, 0, pathBox, None)

        # Release can
        self.released = True
        self.panda.openGripper()
        self.box.inBox(can_pos)

        # Return to base
        self.panda.moveToBase()
        self.released = False

    # Utility Methods
    def newPackage(self):
        """Setup a new package with cans and box."""
        if self.box is not np.nan:
            self.box.reset()
        else: 
            self.box = Box(self.boxlocation, boxsize, 3)
        if len(self.list_cans)>0:
            for i, reset_can in enumerate(self.list_cans):
                #can.setColor('green')
                reset_can.reset()
        else:
            for i, c in enumerate(self.getCanLocations(nrCans)):
                load_can = Can([c[0]*s, c[1]*s, c[2]*s], 'green')
                self.list_cans.append(load_can)
        
    def isnearObject(self, predicted_x, can_pos):
        """Check if the predicted x is near the object."""
        object_position = can_pos.getPos(can_pos)
        distance = abs(predicted_x - object_position[0])  # We consider only the x-coordinates
        return distance < 0.1
       
    def getCanLocations(self, cup_nr):
        """Get the locations for a number of cans."""
        locs = []
        spacing = 0.2

        for i in range(cup_nr):
            loc = [self.tl[0]+i*spacing, self.tl[1]-0.2*s, self.th+0.03*s]
            locs.append(loc)
        return locs

    def displayText(self, text):
        """"Display given text in the environment."""
        if text == ' ' and self.explanation != None:
            p.removeBody(self.explanation)
            self.explanation = None
            return True
        
        for i, t in enumerate(self.texts):
            p.removeUserDebugItem(t)
        
        self.texts = []
        texts = text.split("\n")
    
        color = [0,0,0]
        size = 0.05
        orn = p.getQuaternionFromEuler([math.pi/2,0,0])
        if self.explanation == None:
            self.explanation = p.loadURDF(path+"explanation_small.urdf",basePosition=[self.tl[0], self.tl[1], self.th+0.3*s], useFixedBase=1)
        
        bounds = p.getAABB(self.explanation)
        dz = (bounds[1][2] - bounds[0][2])
        dy = (bounds[1][1] - bounds[0][1])
        positions = []

        for i in range(len(texts)):
            z = dz/2 - (i+1)*dz/len(texts) + (dz/len(texts) -0.04)/2
            positions.append([-0.3, -dy/2 - 0.001, z])

        while len(self.texts) < len(texts):
            self.texts.append(' ')

        for i, t in enumerate(texts):
            if self.texts[i] != ' ':    
                self.texts[i] = p.addUserDebugText(t, positions[i], color, textSize=size, replaceItemUniqueId=self.texts[i], textOrientation=orn, parentObjectUniqueId=self.explanation)
            else:
                self.texts[i] = p.addUserDebugText(t, positions[i], color, textSize=size, textOrientation=orn,  parentObjectUniqueId=self.explanation) 

    def removeText(self):
        p.removeBody(self.explanation)
        for i, t in enumerate(self.texts):
            p.removeUserDebugItem(t)

    def stopEnvironment(self):
        """"Stops the environment and disconnects."""
        self.stop_tracking = True
        time.sleep(0.1*SLEEP_TIME)
        p.disconnect()

    # Tracking Method
    def tracking(self):
        path2 = dataPath+str(participantID)+'_Con'+str(self.panda_type)+'_Tracking.xlsx'
        writer = pd.ExcelWriter(path2)
        title = pd.DataFrame([['Time', 'Gripper_location', 'Head_location', 'Head_orientation']], columns=['Time', 'Gripper_location', 'Head_location', 'Head_orientation'])
        title.to_excel(writer,sheet_name='Sheet1', index = False,header= False)
        writer.save()
        self.writer_ready = True

        while True:
            events = p.getVREvents(p.VR_DEVICE_HMD + p.VR_DEVICE_GENERIC_TRACKER)
            loc = self.panda.getEndLocation()
            t = datetime.now()
            if len(events) > 0:
                pos = events[-1][1]
                mat = p.getMatrixFromQuaternion(events[-1][2])
                dir = [-mat[2], -mat[5], -mat[8]]
                d = pd.DataFrame([[t, loc, pos, dir]])
            else:
                d = pd.DataFrame([[t, loc, np.nan, np.nan]])
            d.to_excel(writer,sheet_name='Sheet1', startrow = writer.sheets['Sheet1'].max_row, index = False,header= False)
            writer.save()
            time.sleep(track_time)
            if self.stop_tracking:
                break

class_experiment = Experiment(participantID,groupID)
class_experiment.main()