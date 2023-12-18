import pandas as pd
from pynput import keyboard
from datetime import datetime
import time
import numpy as np
import pybullet as p
import math
import pybullet_data
import threading
from Definitions_VR import *
from Panda_module import Panda
from Box_module import Box
from Can_module import Can
from Trajectory_module import TrajectoryCalculator
#from VR_module import VRControllers

###########################################################################################################################
##                                                  EXPERIMENT                                                           ##
###########################################################################################################################

class Experiment:
    HAND_CHECK = "Welcome to this experiment. \n Make sure you can see the grippers, \n and that they are kalibrated \n to your actual hands."
    GRAB_OBJECT = "Try to grab one of the objects \n and drop it in the box. \n Pull the finger trigger to grab an object. \n Let go to let the object go."
    ALL_OBJECTS = "Try to put all objects \n into the box."
    REPEAT = "To repeat this practice task:\n Press the big button. \n Otherwise indicate to the experimenter \n that you want to start the experiment."
    START_EXPERIMENT = "Now the real experiment starts. \n Make sure to work as fast as \n possible to put the objects in \n the box."
    START_TASK = "Press the BIG button \n to start with the first task."
    CONTINUE_TASK = "To continue with the next box \n press the BIG button"
    END_TASK = "This was the end of a trial \n You can take off the headset \n and continue with the questionnaire."

    def __init__(self, participant, condition):
        """ Initialise the experiment with a participant number and condition.
            
            participant: participant id
            condition: type of condition
        """
        self.condition = condition
        self.participant = participant
        self.data = pd.DataFrame(columns=['participantID', 'Condition', 'Task', 'Robot Chosen Can',
                                          'Human Chosen Can', 'Human Predicted Can', 'Completion Time (stf)',
                                          'Robot Reach Time (stc)', 'Human Reach Time (stc)'])
        # Maybe only keep ParticipantId and Condition, because I add the rest later on
        self.startNow = False
        self.stop = False

    def main(self):
        
        ### START PRACTICE ###
        if doPractice:
            self.env = Environment(99)
            self.startPractice()

        ### START EXPERIMENT ###
        self.env = Environment(self.condition)
        self.startExperiment()
 
        ### START TASKS ###
        for task_nr in tasks:
            if task_nr>0:
                self.startNow = False
                self.env.newPackage()
                self.env.displayText(self.CONTINUE_TASK)
                time.sleep(0.5*SLEEP_TIME)

                self.waitForStart()

            task_instance = Task(self.env, conditionID, task_nr)
            task_data = task_instance.startTask()
            #self.data = pd.concat([self.data, task_data], axis=0, ignore_index=True)
            
        # Define data
        self.data['participantID'] = self.participant
        self.data['Condition'] = self.condition
        #self.data.to_excel(dataPath+str(self.participant)+'_Data.xlsx')

        # End Experiment (STILL HAVE TO DECIDE WHEN)
        self.env.displayText(self.END_TASK)
        time.sleep(1*SLEEP_TIME)
        self.env.stopEnvironment()
    
    def startPractice(self):
        """ Start the practice task. """
        print("------ Start Practice: "+ str(datetime.now()))

        # Check hand calibrations
        self.env.displayText(self.HAND_CHECK)
        time.sleep(0.7*SLEEP_TIME)

        ### GIVE INSTRUCTIONS ### 
        # Try to grab object
        self.env.displayText(self.GRAB_OBJECT)
        time.sleep(0.7*SLEEP_TIME)

        # Throw all objects in box
        self.env.displayText(self.ALL_OBJECTS)
        time.sleep(0.7*SLEEP_TIME)

        # Repeat the practice round
        self.env.displayText(self.REPEAT)
        
        self.waitForStart()

        if self.startNow:
            self.env.newPackage()

    def startExperiment(self):
        """ Start the real experiment. """

        # Check hand calibrations
        self.env.displayText(self.HAND_CHECK)
        time.sleep(0.7*SLEEP_TIME)

        ### GIVE INSTRUCTIONS ###  
        self.env.displayText(self.START_EXPERIMENT)
        time.sleep(0.7*SLEEP_TIME)
        self.env.displayText(self.START_TASK)

        self.waitForStart()

        self.start_time = datetime.now()
        print("------ Start Experiment: " + str(self.start_time))
        print("------ Condition: " + str(self.condition))
    
    def waitForStart(self):
        l = keyboard.Listener(on_press=self.monitorKeyboard)  # Listen to keyboard
        l.start()
        
        # Wait for button/spacebar presses
        while not self.startNow:
            if controllers_present:
                self.monitorVRController()  # Listen to VR controller
        l.stop()
        l.join()

    def monitorKeyboard(self, key):
       """Check if spacebar is pressed."""
       if hasattr(key, 'name') and key.name == 'space':
            self.startNow = True
            self.env.displayText('')

    def monitorVRController(self):
        """Check if VR button is pressed."""
        events = p.getVREvents()
        for e in (events):
            if e[BUTTONS][33] == p.VR_BUTTON_IS_DOWN or e[BUTTONS][32] == p.VR_BUTTON_IS_DOWN:
                self.startNow = True
                self.env.displayText('')
        
class Task:   
    """Represents a task where the robot interacts with objects."""      

    def __init__(self, env, panda_type, task_nr):
        """ Initialize the task with a type and number.
            
            panda_type: Type of the panda
            task_nr: Number of the task
        """        
        self.task_nr = task_nr
        self.panda_type = panda_type
        self.startNow = False
        self.start_time = datetime.now()
        self.env = env
        self.data = pd.DataFrame(columns=['Task', 'Can', 'Robot Chosen Can'])
        self.list_cans = [0,1,2] # Real ID's are 4, 5, 6
       
    def startTask(self):
        print("------ Start Task: " + str(self.task_nr))

        self.data['Task'] = self.task_nr
        self.task_start_time = datetime.now()

        self.env.task_state = "not done"
        while self.env.task_state == "not done":
            # Loop through list to grab objects while task_state is not done
            for can_nr in self.list_cans:
                print(f"--- Can: {can_nr}")
                print("-----Object state at start: ", self.env.list_cans[can_nr].state)

                self.start_can_time = datetime.now()
                # Object state is changed if can is in box
                can_end_time, robot_chosen_can_nr = self.env.robotExecuteTask(can_nr)
                robot_can_reach_time = (can_end_time - self.start_can_time).total_seconds()

                print("-----Object state at end: ", self.env.list_cans[can_nr].state)

                # Do we also want to know the start time and reached time?
                # results = pd.DataFrame([can_nr, robot_chosen_can_nr, human_chosen_can, 
                #                         human_predicted_can, robot_can_reach_time, human_can_reach_time],
                #                         columns=['Can', 'Robot Chosen Can', 'Human Chosen Can', 'Human Predicted Can',
                #                                 'Robot Reach time (stc)', 'Human Reach time (stc)'])
                # self.data = pd.concat([self.data, results], axis=0, ignore_index=True)

            if controllers_present: # Not dependent on can_nr, but still need to track it for the reach time
                human_chosen_can, human_predicted_can = self.env.humanTaskEvents()
                human_can_reach_time = (can_end_time - self.start_can_time).total_seconds()

            # Close the box
            if self.env.box.isFull():
                self.env.task_state = "task done"
            print(" task_state: ", self.env.task_state)

        self.task_end_time = datetime.now()
        total_time = (self.task_end_time - self.task_start_time).total_seconds()

        results2 = pd.DataFrame([total_time], columns=['Completion Time (stf)'])
        self.data = pd.concat([self.data, results2], axis=0, ignore_index=True)

        return self.data

###########################################################################################################################
##                                               ENVIRONMENT                                                             ##
###########################################################################################################################

class Environment:
    SYNC_CONTROLLER = "Press the BIG button to \n activate the controller."

    def __init__(self, condition):
        self.state = "on_table"        
        # Initialisation and setup
        self.setupEnvironment(condition)
        # Panda setup 
        self.setupPanda()
        # Tracking
        self.setupTracking()

    def setupEnvironment(self, condition):
        """ Initialisation of environment:
            Plane, table, box, VR
        """
        self.panda_type = condition
        self.writer_ready = False
        self.stop_tracking = False
        self.update_ready = False

        # Attempt to connect to the shared memory of pybullet.
        self.clid = p.connect(p.SHARED_MEMORY)
        # If connection to shared memory fails, connect using the GUI.
        if (self.clid < 0):
            p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

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
        self.boxlocation = [self.tl[0]-0.35*s, self.tl[1]-0.2*s, self.th]

        # Initialise variables
        self.list_cans = []
        self.texts = []
        self.explanation = None

        # Load new environment for task
        self.newPackage()

        # Setup VR controllers
        if controllers_present:
            self.displayText(self.SYNC_CONTROLLER)
            self.pr2_gripper = self.loadGripper()
            self.pr2_constraintId, self.pr2_constraintId2 = self.initializePR2Gripper(self.pr2_gripper)
            self.pickControllerId = self.controllerInput()
            self.startEventLoop()

        # Configure the visualizer to disable rendering and other settings.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, lightPosition=[0,-2,2])
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

        p.setRealTimeSimulation(1)

    def loadGripper(self):
        """Load gripper into environment."""
        objects = [p.loadURDF(path + "controllers/pr2_gripper.urdf",
                                0.500000, 0.300006, 0.700000,
                                -0.000000, -0.000000, -0.000031, 1.000000)]
        pr2_gripper = objects[0]
        print("pr2_gripper=", pr2_gripper)
        return pr2_gripper

    def initializePR2Gripper(self, pr2_gripper):
        """"Initialise PR2 gripper settings."""
        # 1. List of initial joint positions for PR2 gripper
        jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
        for jointIndex in range(p.getNumJoints(pr2_gripper)):
            # Resets each joint to the positions in the list
            p.resetJointState(pr2_gripper, jointIndex, jointPositions[jointIndex])
            # Each joint tries to reach position 0, but with force 0, so no movement
            p.setJointMotorControl2(pr2_gripper, jointIndex, p.POSITION_CONTROL, targetPosition=0, force=0)

        # 2. Fix PR2 gripper at certain position and orientation
        pr2_constraintId = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0.2, 0, 0],
                                            [0.500000, 0.300006, 0.700000])
        print("pr2_constraintId", pr2_constraintId)

        # 3. Synchronize motion between joint 0 and 2
        pr2_constraintId2 = p.createConstraint(pr2_gripper, 0,
                                    pr2_gripper, 2,
                                    jointType=p.JOINT_GEAR,
                                    jointAxis=[0, 1, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
        p.changeConstraint(pr2_constraintId2, gearRatio=1, erp=0.5, relativePositionTarget=0.5, maxForce=3)
        return pr2_constraintId, pr2_constraintId2

    def controllerInput(self):
        """Identify which controller will be used for for picking up objects."""
        # First define the pick-up-objects controller
        pickControllerId = -1

        print("waiting for VR picking controller trigger")
        while (pickControllerId < 0):
            events = p.getVREvents()
            for e in (events):
                if e[BUTTONS][33] == p.VR_BUTTON_IS_DOWN or e[BUTTONS][32] == p.VR_BUTTON_IS_DOWN:
                    #print(e)
                    pickControllerId = e[CONTROLLER_ID]
                    pos = e[CURRENT_POSITION]

                    # Find start pos
                    trajectory['start'] = pos
                    print("----- Start position: ", pos)

        print("Using pickControllerId=" + str(pickControllerId))
        return pickControllerId

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

    # Utility Methods
    def newPackage(self):
        """Setup a new package with cans and box."""
        if self.box is not np.nan:
            self.box.reset()
        else: 
            self.box = Box(self.boxlocation, boxsize, 3)

        if len(self.list_cans)>0:
            for reset_can in self.list_cans:
                reset_can.reset()
                self.state = "on_table" # reset all the cans to the starting state
        else:
            for can in self.getCanLocations(nrCans):
                load_can = Can([can[0]*s, can[1]*s, can[2]*s])
                self.list_cans.append(load_can)
                self.state = "on_table" # set all new cans on the table to starting state

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
        if text == '' and self.explanation != None:
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

    # Task Execution Methods
    def robotExecuteTask(self, can_nr):
        """Execute a given task with a specified can."""
        logging = [self.panda.getID()]
        self.reachedTime = None
        self.robot_chosen_can = can_nr

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

        # Stop state logging
        p.stopStateLogging(log1)
        p.stopStateLogging(log2)
        p.stopStateLogging(log3)

        return self.reachedTime, self.robot_chosen_can
    
    def humanTaskEvents(self):
        #self.list_cans = [0,1,2]
        self.human_reached = False
        calculator = TrajectoryCalculator()
        #print("list_cans:", self.list_cans)

        while not self.human_reached:
            self.hand_pos = trajectory['current']
            # Predict human trajectory to can
            if trajectory['start'] and trajectory['current']:
                self.predicted_position = calculator.predictHumanTrajectory()
                
                for can_nr in self.list_cans:
                    if abs(self.predicted_position[0] - can_nr.getPos()[0]) < 0.1:
                    #if self.isnearObject(self.predicted_position[0], can_nr.getPos()[0]):
                        self.predicted_can = can_nr
                        #print(f"----- Predicted Can: {self.predicted_can}")
                        self.state = "predicted_target"
                        #print("----- Object state: ", self.state)

                self.human_reached = self.checkPoint(can_nr.getPos(), self.hand_pos)
                #print(f"Predicted position: {self.predicted_position}")
        
        self.state = "in_hand"
        print("------- Object state: ", self.state)

        self.data = pd.DataFrame([])
        
        return self.state       

    def isnearObject(self, predicted_x, can_pos):
        """Check if the predicted x is near the object."""
        object_position = can_pos.getPos(can_pos)
        distance = abs(predicted_x - object_position[0])  # We consider only the x-coordinates
        return distance < 0.1       
    
    ################################################        THREAD 1        ##########################################################
    
    def startEventLoop(self):
        #p.resetSimulation()
        #p.stepSimulation()
        self.vr_events = threading.Thread(target=self.gripperEvents, args=())
        self.vr_events.start()

    def gripperEvents(self):
        while True:     # If you want to keep updating the position continuously
            events = p.getVREvents()
            for e in events:
                # CHECK IF SELF. NAMES ARE CORRECT ETC. THE EVENTS CHANGE THE CONTROLLER ID
                self.syncGripper(e, self.pr2_gripper, self.pickControllerId, self.pr2_constraintId, self.pr2_constraintId2)
                pos = self.updateControllerPosition(e)
                #print(e)
                #self.humanTaskEvents(pos)

    def syncGripper(self, e, pr2_gripper, pickControllerId, pr2_constraintId, pr2_constraintId2):
        """Sync the gripper with the controller and allow for gripping."""
        #print("Sync and move gripper")

        # Keep gripper centered/symmetric
        b = p.getJointState(pr2_gripper, 2)[0]
        p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=1000)
            
        if e[CONTROLLER_ID] == pickControllerId:
            # Sync the PR2 gripper with the controller position
            p.changeConstraint(pr2_constraintId, e[CURRENT_POSITION], e[ORIENTATION], maxForce=10000)
            #print(e[CURRENT_POSITION])
            # Determine relative position target for PR2
            relPosTarget = 1 - e[ANALOG]
            # Open/close gripper based on ANALOG value
            p.changeConstraint(pr2_constraintId2,
                                gearRatio=1,
                                erp=1,
                                relativePositionTarget=relPosTarget,
                                maxForce=1000)

    def updateControllerPosition(self, e):
        """Handle individual device event."""

        global trajectory
        #print(" Update current positions ")
        
        pos = e[CURRENT_POSITION] # Make this a global as well?

        # Update the current position
        trajectory['current'] = pos
        #print("----- Current position: ", pos)
        
        return pos   
        
    ################################################        THREAD 2        ##########################################################

    def grabOneCan(self, can_pos):
        """Logic to grab a single can."""
        self.robot_reached = False
        calculator = TrajectoryCalculator()
        
        # CONDITION 0
        if self.panda_type == 0:
            
            # Check if object already in box
            self.checkAvailability()
            
            # Reach for can
            self.linearReach(can_pos)

            # Put can into box
            self.putCanInBox(can_pos)

        # CONDITION 1
        elif self.panda_type == 1:
            self.reachedTime = datetime.now()
            print("Anticipatory: Can: "+ str(can_pos.getID()))

            # Check if object already in box
            self.checkAvailability(can_pos)

            # Calculate trajectory to can
            pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())

            # Start reaching for the can
            self.panda.realTimeSim(time_to_grab, 0, pathcan, can_pos.getPos())
            #self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            while not self.robot_reached:
                # Reach for can with anticipation
                self.anticipateHuman(can_pos)

                #print("-----Object state: ", self.state)
                pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
                self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
                self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())

                # while reaching, check if in hand, otherwise return to base
                self.checkInHand()

            self.reachedTime = datetime.now()

            # Put can in box
            self.putCanInBox(can_pos)

        # CONDITION 2        
        elif self.panda_type == 2:
            # Initialise variables
            v = [0,0,0]
            a = [0,0,0.1]

            # Check if object already in box
            self.checkAvailability()
            
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
                self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())

                # while reaching, check if in hand, otherwise return to base
                self.checkInHand()

            # Put can into the box
            self.putCanInBox(can_pos)

            #self.robot_chosen_can = can_pos.getID()

        # STILL TO BE WRITTEN
        elif self.panda_type == 3:   
            # Check if object already in box
            self.checkAvailability()

            # Calculate trajectory to can
            pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), can_pos.getPos(), startV=v, endA=a, distance=True)
            self.panda.realTimeSim(time_to_grab2, 0, pathcan, can_pos.getPos())

            # while reaching, check if in hand, otherwise return to base
            self.checkInHand()

            # Put can into the box
            self.putCanInBox(can_pos)

            #self.robot_chosen_can = can_pos.getID()

    def checkPoint(self, target, currentLoc):
        margin = 0.01*s
        if not target[0]-margin < currentLoc[0] < target[0]+margin:
            return False
        if not target[1]-margin < currentLoc[1] < target[1]+margin:
            return False
        if not target[2]-margin < currentLoc[2] < target[2]+margin:
            return False
        return True
    
    def checkAvailability(self):
        # Check if on table
        if self.state == "on_table":
            return self.state
        # Reset if 'predicted_target'
        elif self.state == "predicted_target":
            self.state = "on_table"
            print("----- Object state at start (reset): ", self.state)
        # Check if (at this exact moment) in hand (maybe redundant)
        elif self.state == "in_hand":
            self.panda.moveToBase()
            self.state = "done"
        # Check if done
        elif self.state == "done":
            self.panda.moveToBase()
        
    def checkInHand(self):
        # Check if in hand
        if self.state == "in_hand":
            self.panda.moveToBase()
            self.state = "done"

    def linearReach(self, can_pos):
        calculator = TrajectoryCalculator()
        
        # Calculate trajectory to can
        pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())

        # Start reaching for the can
        self.panda.realTimeSim(time_to_grab, 0, pathcan, can_pos.getPos())
        #self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())
        
        while not self.robot_reached:
            pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can_pos.getPos())
            self.panda.realTimeSim(time_to_correct, 0, pathcan, can_pos.getPos())
            self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            # while reaching, check if in hand, otherwise return to base
            self.checkInHand()

        self.reachedTime = datetime.now()

    def anticipateHuman(self, can_pos):
        if self.state == "predicted_target":
            # Continue trajectory to different can
            for can_pos in self.list_cans:
                print("-----Object state (replan): ", self.state)
                # Check if on table
                if self.state == "on_table":
                    self.linearReach(can_pos)

                    # Put can into the box
                    self.putCanInBox(can_pos)
                    self.robot_chosen_can = can_pos
                    break
    
    def putCanInBox(self, can_pos):
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
        
        # Move to box
        self.panda.realTimeSim(time_to_box, 0, pathBox, None)

        # Release can
        self.panda.openGripper()
        self.box.inBox(can_pos) # Changes state to "done"

        # Return to base
        self.panda.moveToBase()
    
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

class_experiment = Experiment(participantID, conditionID)
class_experiment.main()