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
    HAND_CHECK = "Welcome to this experiment. \n Make sure you can see the gripper, \n and that it is kalibrated \n to your actual hand."
    SYNC_CONTROLLER = "Press the BIG ROUND button \n near your thumb to \n activate the hand gripper."
    GRAB_OBJECT = "Try to grab one of the objects \n and drop it in the box. \n Pull the finger trigger to grab an object. \n Let go to let the object go."
    ALL_OBJECTS = "Try to put all objects \n into the box."
    REPEAT = "To repeat this practice task:\n Press the big button. \n Otherwise indicate to the experimenter \n that you want to start the experiment."
    START_EXPERIMENT = "Now the real experiment starts. \n Make sure to work as fast as \n possible to put the objects in \n the box."
    START_TASK = "Press the BIG button \n to start with the first task. \n The gripper will attach to \n the controller now."
    CONTINUE_TASK = "To continue with the next box \n press the BIG button"
    END_TASK = "This was the end of a trial \n You can take off the headset \n and continue with the questionnaire."

    def __init__(self):
        """ Initialise the experiment with a participant number and condition.
            
            participant: participant id
            condition: type of condition
        """
        self.exp_data = pd.DataFrame(columns=['participantID', 'Condition', 'Task', 'Can', 'Robot Chosen Can',
                                          'Human Chosen Can', 'Human Predicted Can', 'Completion Time (stf)',
                                          'Robot Reach Time (stc)', 'Human Reach Time (stc)'])
        self.startNow = False
        self.stop = False
        # self.task_state = "not_done"

    def main(self):
        #self.condition_list = random.sample([0,1,2,3], 4)
        self.condition_list = [0,1,2,3]
        
        ### START PRACTICE ###
        if doPractice:
            print("Starting DoPractice")
            self.env = Environment(99)
            self.startPractice()

        #### START EXPERIMENT ###
        for self.conditionID in self.condition_list:
            condition_data = self.startExperiment(self.conditionID)
            self.exp_data = pd.concat([self.exp_data, condition_data], axis = 0, ignore_index=True)

        # Data directory setup
        try:
            os.mkdir(dataPath+"P"+str(participantID))
        except:
            print("Directory already created")
        dataPath = dataPath + "P" + str(participantID) + "/"

        # Write data to excel
        #self.exp_data.to_excel(dataPath+str(participantID) + str(datetime.now())+'_data.xlsx')
        
        # Stop the environment
        the_experiment.env.stopEnvironment()

    def startExperiment(self, conditionID):
        global task_state
        print("------ Condition: " + str(conditionID))

        ### START INSTRUCTIONS ###
        self.env = Environment(conditionID)
        self.startInstructions() # waiting for controller action
 
        ### START TASKS ###
        self.env.displayText(self.START_TASK)
        for self.task_nr in tasks:
            # When not the first task
            if self.task_nr > 0:
                self.startNow = False
                self.env.newPackage()
                task_state = "not_done"
                self.env.displayText(self.CONTINUE_TASK)
                time.sleep(0.5*SLEEP_TIME)

            # waiting for controller action
            self.waitForStart()

            self.human_start_time = datetime.now()
            self.env.human_boxed = False
            print("----- Start Experiment: " + str(datetime.now()))

            # Start controller tracking
            if controllers_present:
                self.env.humanEventThread()
                time.sleep(1*SLEEP_TIME) # make this into 0.1 for the real experiment

            # Robot start moving
            self.task_data = self.startRobotTask()

            # Get data from human controller hand
            self.human_data = self.getHumanData()
            
            # Combine data
            self.exp_data = pd.concat([self.task_data, self.human_data], axis=1, ignore_index=True) # checken met axis en zo of misschien met append doen

        # End Experiment condition (Do Questionnaire)
        self.env.displayText(self.END_TASK)
        time.sleep(1*SLEEP_TIME)
        
        # return self.data
    
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

    def startInstructions(self):
        """ Start the real experiment. """

        if controllers_present:
            # Check hand calibrations
            self.env.displayText(self.HAND_CHECK)
            time.sleep(0.7*SLEEP_TIME)

            # Activate gripper and start position
            self.env.displayText(self.SYNC_CONTROLLER)
            self.env.controllerInput()
            time.sleep(0.7*SLEEP_TIME)

        ### GIVE INSTRUCTIONS ###  
        self.env.displayText(self.START_EXPERIMENT)
        time.sleep(0.7*SLEEP_TIME)
        
    
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
    
    def startRobotTask(self):
        global task_state
        print("------ Start Task: " + str(self.task_nr))

        #self.data['Task'] = self.task_nr
        self.task_start_time = datetime.now()

        while task_state == "not_done":
            # Loop through list to grab objects while task_state is not done
            self.random_can_nr_list = random.sample([0,1,2], 3)
            for can_nr in self.random_can_nr_list:

                self.robot_start_time = datetime.now()
                self.robot_end_time, robot_chosen_can_nr = self.env.robotEventThread(can_nr)
                robot_reach_time = (self.robot_end_time - self.robot_start_time).total_seconds()

                # Do we also want to know the start time and reached time?

                # Define data
                results = pd.DataFrame([[participantID, self.conditionID, self.task_nr, can_nr, robot_chosen_can_nr, robot_reach_time]],
                                        columns=['ParticipantID', 'ConditionID', 'Task', 'Can', 'Robot Chosen Can', 'Robot Reach time (stc)'])
            
            if not controllers_present:
                # Check if all objects in box or on floor
                for can_nr, can in self.env.list_cans:
                    if self.env.box.inBox(can) or self.env.box.fellOnFloor(can):
                        self.env.state[can_nr] = "done"

            # Close the box
            if all(state == "done" for state in self.env.state):
                time.sleep(1)
                task_state = "task_done"
            print(" task_state: ", task_state)

        self.task_end_time = datetime.now()
        self.completion_time = (self.task_end_time - self.task_start_time).total_seconds()

        # How do I add this correctly?
        results2 = pd.DataFrame([self.completion_time], columns=['Completion Time (stf)'])
        self.task_data = pd.concat([results, results2], axis=1, ignore_index=True)

        return self.task_data
    
    def getHumanData(self):
        if self.env.human_boxed:
            self.human_reach_time = (self.env.human_end_time - self.human_start_time).total_seconds()

            human_data = pd.DataFrame([self.env.predicted_can_nr, self.human_reach_time], columns=['Human Chosen Can (stc)', 'Human Reach Time'])
            self.data = pd.concat([self.data, human_data], axis = 1, ignore_index=True)     

###########################################################################################################################
##                                               ENVIRONMENT                                                             ##
###########################################################################################################################

class Environment:
    SYNC_CONTROLLER = "Press the BIG button to \n activate the controller."

    def __init__(self, condition):    
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
        self.state = [None]*nrCans
        self.human_state = [None]*nrCans
        self.human_reached = False
        self.human_predict = True
        
        # Load new environment for task
        self.newPackage()

        # Setup VR controllers
        if controllers_present:
            # self.displayText(self.SYNC_CONTROLLER)
            self.pr2_gripper = self.loadGripper()
            self.pr2_constraintId, self.pr2_constraintId2 = self.initializePR2Gripper(self.pr2_gripper)
            # self.pickControllerId = self.controllerInput()
            # self.humanEventThread()

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
        self.pickControllerId = -1

        print("waiting for VR picking controller trigger")
        while (self.pickControllerId < 0):
            events = p.getVREvents()
            for e in (events):
                if e[BUTTONS][33] == p.VR_BUTTON_IS_DOWN or e[BUTTONS][32] == p.VR_BUTTON_IS_DOWN:
                    #print(e)
                    self.pickControllerId = e[CONTROLLER_ID]
                    pos = e[CURRENT_POSITION]

                    # Find start pos
                    trajectory['start'] = pos
                    print("----- Start position: ", pos)

        print("Using pickControllerId=" + str(self.pickControllerId))
        return self.pickControllerId

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
            for can_nr, can in enumerate(self.list_cans):
                can.reset()
                self.state[can_nr] = "on_table" # reset all the cans to the starting state
        else:
            for can_nr, can_pos in enumerate(self.getCanLocations(nrCans)):
                load_can = Can([can_pos[0]*s, can_pos[1]*s, can_pos[2]*s])
                self.list_cans.append(load_can)
                self.state[can_nr] = "on_table" # set all new cans on the table to starting state

    def getCanLocations(self, nrCans):
        """Get the locations for a number of cans."""
        locs = []
        spacing = 0.2

        for i in range(nrCans):
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

    ################################################        THREAD 1        ##########################################################
    
    # CONTINUOUS UPDATING OF GRIPPER EVENTS
    def humanEventThread(self):
        #p.resetSimulation()
        #p.stepSimulation()
        self.vr_events = threading.Thread(target=self.gripperEventLoop, args=())
        self.vr_events.start()

    def gripperEventLoop(self):
        """ Continuous updating of gripper events such as:
            
            syncGripper()
            updateControllerPositions()
            humanTaskEvents()
            """
        global task_state
        while True:
            events = p.getVREvents()
            for e in events:
                self.syncGripper(e, self.pr2_gripper, self.pickControllerId, self.pr2_constraintId, self.pr2_constraintId2)
                self.updateControllerPosition(e)
                if task_state == "task_done":
                    for self.predicted_can_nr, can in enumerate(self.list_cans):
                        self.human_state[self.predicted_can_nr] = "no_target"
                if task_state == "not_done":
                    self.humanTaskEvents(e) 

    def syncGripper(self, e, pr2_gripper, pickControllerId, pr2_constraintId, pr2_constraintId2):
        """Sync the gripper with the controller and allow for gripping."""

        # Keep gripper centered/symmetric
        b = p.getJointState(pr2_gripper, 2)[0]
        p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=10000)
            
        if e[CONTROLLER_ID] == pickControllerId:
            # Sync the PR2 gripper with the controller position
            p.changeConstraint(pr2_constraintId, e[CURRENT_POSITION], e[ORIENTATION], maxForce=100000)
            # Determine relative position target for PR2
            relPosTarget = 1 - e[ANALOG]
            # Open/close gripper based on ANALOG value
            p.changeConstraint(pr2_constraintId2,
                                gearRatio=1,
                                erp=1,
                                relativePositionTarget=relPosTarget,
                                maxForce=100000)

    def updateControllerPosition(self, e):
        """Handle individual device event."""

        global trajectory
        pos = e[CURRENT_POSITION]

        # Update the current position
        trajectory['current'] = pos
        
        return pos

    def humanTaskEvents(self, e):
        calculator = TrajectoryCalculator()
        
        for self.predicted_can_nr, can in enumerate(self.list_cans):
            # Check if all objects in box or on floor
            if self.box.inBox(can) or self.box.fellOnFloor(can):
                self.state[self.predicted_can_nr] = "done"

            # Check if human has reached one of the cans
            self.human_reached = self.checkPointHuman(can.getPos(), trajectory['current'])
            
            # Predict human trajectory (if not grabbed)
            if not self.human_reached:
                if self.human_predict:
                    if not any(human_state == "in_hand" for human_state in self.human_state):
                        self.predicted_position = calculator.predictHumanTrajectory()
                    
                        # Check if predicted position is close to predicted_can_nr
                        if abs(self.predicted_position[0] - can.getPos()[0]) < 0.05:
                            if self.state[self.predicted_can_nr] != "done":
                                self.human_state[self.predicted_can_nr] = "predicted_target"
            
                            # There can only be one predicted_target
                            for can_nr, a_can in enumerate(self.list_cans):
                                if self.human_state[can_nr] == "predicted_target" and can_nr != self.predicted_can_nr:
                                    self.human_state[can_nr] = "no_target"
                        # else:
                        #     self.human_state[self.predicted_can_nr] = "no_target"

            # If human has reached the can, break out of the for-loop
            # elif self.human_reached:
            #     self.human_end_time = datetime.now()
            #     self.human_chosen_can = self.predicted_can_nr
            #     break

            # Check if human is holding the can
            if self.human_reached and e[ANALOG] > 0.7:
                self.human_predict = False
                self.human_state[self.predicted_can_nr] = "in_hand"
                self.state[self.predicted_can_nr] = "unavailable"
                # print("Object State grabbed: ", self.state)
                # print("Human State grabbed: ",self.human_state)

            # Check with prints if correct things happen
            if self.human_state[self.predicted_can_nr] == "in_hand":
                self.human_predict = False
                if self.box.inBox(can) or self.box.fellOnFloor(can):
                    self.human_state[self.predicted_can_nr] = "human_boxed"
                    print("Human finished a can")
                    # print("Human State after human finish: ",self.human_state)
        
            # Check if previously held by human and if it is now in the box
            # if self.human_state[self.predicted_can_nr] == "in_hand":
            #     if self.box.inBox(can) or self.box.fellOnFloor(can): #eigenlijk can state variable checken?
            #         self.human_state[self.predicted_can_nr] = "human_boxed"
            #         #self.human_boxed = True
            #         self.state[self.predicted_can_nr] = "done"
            #         #print("Object State after human finish: ", self.state) # already done in RobotTaskEvent
            #         print("Human State after human finish: ",self.human_state)

                    #return self.human_boxed, self.human_chosen_can
        
        # Get the starting position and check when back at start to start predicting again
        if any(human_state == "human_boxed" for human_state in self.human_state):
            if self.checkPointStart(trajectory['start'], trajectory['current']):
                if not self.human_predict:
                    self.human_predict = True
                    self.human_reached = False
                    print("Human back at start")
            
    ################################################        THREAD 2        ##########################################################

    # ROBOT GRABS A CAN
    def robotEventThread(self, can_nr):
        """Start state logging and start the thread of the robot grabbing a can."""
        logging = [self.panda.getID()]
        self.robot_end_time = None
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
        
        # Resume real-time simulation
        p.setRealTimeSimulation(1)

        # Start grabbing the cans simultaneously
        arm = threading.Thread(target=self.robotGrabEvents, args=(self.list_cans[can_nr], can_nr,))
        arm.start()

        # While the arm is moving, wait till task is complete
        while arm.is_alive():
            time.sleep(0.01*SLEEP_TIME)

        # Stop state logging
        p.stopStateLogging(log1)
        p.stopStateLogging(log2)

        return self.robot_end_time, self.robot_chosen_can 
    
    def robotGrabEvents(self, can, can_nr):
        """Logic to grab a single can."""
        self.robot_reached = False
        calculator = TrajectoryCalculator()

        # Reset cans
        #self.resetCans(can_nr)
        
        # CONDITION 0
        if self.panda_type == 0:
            print("Control: Can: ", can_nr)
            print("Object State start: ", self.state)
            print("Human State start: ",self.human_state)

            # Check availability
            if self.checkAvailability(can_nr):
            
                # Calculate trajectory to can
                pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())
                
                # Start reaching for the can
                self.panda.realTimeSim(time_to_grab, 0, pathcan, can.getPos())
                self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())
                
                while not self.robot_reached:
                    pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())
                    self.panda.realTimeSim(time_to_correct, 0, pathcan, can.getPos())
                    self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())

                    # while reaching, check if available, otherwise return to base
                    if not self.checkAvailability(can_nr):
                        self.panda.moveToBase()
                        print(f"-----Object state {can_nr} not available (during reach): ", self.state[can_nr])
                        print(self.state)
                        break
                
                self.robot_end_time = datetime.now()

                # Put can into box
                if self.robot_reached:
                     self.putCanInBox(can, can_nr)
                    
            else:
                self.robot_end_time = datetime.now()
                self.panda.moveToBase()
                print(f"-----Object state {can_nr} not available: ", self.state[can_nr])
                print(self.state)

        # CONDITION 1
        elif self.panda_type == 1:
            print("Anticipatory: Can: ", can_nr)
            print("-----Object state at start: ", self.state[can_nr])
            print("Human State start: ",self.human_state)

            # Check availability
            if self.checkAvailability(can_nr):

                # Calculate trajectory to can
                pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())

                # Start reaching for the can
                self.panda.realTimeSim(time_to_grab, 0, pathcan, can.getPos())
                self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())

                while not self.robot_reached:
                    # Reach for can with anticipation
                    # self.anticipateHuman(can_nr)
                    if not self.state[can_nr] == "predicted_target":

                        pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())
                        self.panda.realTimeSim(time_to_correct, 0, pathcan, can.getPos())
                        self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())

                        # while reaching, check if available, otherwise return to base
                        if not self.checkAvailability(can_nr):
                            self.panda.moveToBase()
                            print(f"-----Object state {can_nr} not available (during reach): ", self.state[can_nr])
                            print(self.state)
                            break

                self.robot_end_time = datetime.now()

                # Put can into box
                if self.robot_reached:
                     self.putCanInBox(can, can_nr)
        
            else:
                self.robot_end_time = datetime.now()
                self.panda.moveToBase()
                print(f"-----Object state {can_nr} not available: ", self.state[can_nr])
                print(self.state)

        # CONDITION 2        
        elif self.panda_type == 2:
            print("Legible: Can: ", can_nr)
            print("-----Object state at start: ", self.state[can_nr])

            # Initialise variables
            v = [0,0,0]
            a = [0,0,0.1]
            
            # Start gaze orientation
            startlook = datetime.now()
            self.panda.lookAtPoint(can.getPos(),0.5)
            print("Time to look: " + str((datetime.now()-startlook).total_seconds()))
            time_to_grab2 = time_to_grab*1.18 - (datetime.now()-startlook).total_seconds()
            
            # Start reaching for the can
            pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), np.array(can.getPos()) + np.array([0,0,0.05*s]), startV=v, endA=a, distance=True)
            self.panda.realTimeSim(time_to_grab2, 0, pathcan, can.getPos())
            #self.robot_reached = self.checkPoint(can_pos.getPos(), self.panda.getEndLocation())

            while not self.robot_reached:
                pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), can.getPos(), nrsteps=20, distance=True)
                self.panda.realTimeSim(time_to_correct, 0, pathcan, can.getPos())
                self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())

                # while reaching, check if in hand, otherwise return to base
                if not self.checkAvailability(can_nr):
                    self.panda.moveToBase()
                    print("-----Object state not available: ", self.state[can_nr])
                    break

            self.robot_end_time = datetime.now()

            # Put can into the box
            if self.robot_reached:
                self.putCanInBox(can, can_nr)

        # STILL TO BE WRITTEN
        elif self.panda_type == 3: 
            print("Anticipatory + Legible: Can: ", can_nr)
            print("-----Object state at start: ", self.state[can_nr])

            # Calculate trajectory to can
            pathcan = calculator.legibleTrajectory(self.panda.getEndLocation(), can.getPos(), startV=v, endA=a, distance=True)
            self.panda.realTimeSim(time_to_grab2, 0, pathcan, can.getPos())


            # Put can into the box
            self.putCanInBox(can, can_nr)

            #self.robot_chosen_can = can_pos.getID()
    
    # def resetCans(self, can_nr):
    #     # Reset if 'predicted_target'
    #     if self.human_state[can_nr] == "predicted_target":
    #         self.state[can_nr] = "on_table"
    #         print(f"----- Object state {can_nr} at start (reset): ", self.state[can_nr])
    
    def checkAvailability(self, can_nr):
        # Check if on table
        if self.state[can_nr] != "on_table":
            return False
        else:
            return True

    def checkPoint(self, target, currentLoc):
        margin = 0.01*s
        if not target[0]-margin < currentLoc[0] < target[0]+margin:
            return False
        if not target[1]-margin < currentLoc[1] < target[1]+margin:
            return False
        if not target[2]-margin < currentLoc[2] < target[2]+margin:
            return False
        return True
    
    def checkPointStart(self, target, currentLoc):
        margin = 0.05*s
        if not target[0]-margin < currentLoc[0] < target[0]+margin:
            return False
        if not target[1]-margin < currentLoc[1] < target[1]+margin:
            return False
        if not target[2]-margin < currentLoc[2] < target[2]+margin:
            return False
        return True
    
    def checkPointHuman(self, target, currentLoc):
        # The center of the gripper lies at x, y+something, z
        # The margin of y therefore needs to be larger.
        margin = 0.01*s
        marginY = 0.2*s
        if not target[0]-margin < currentLoc[0] < target[0]+margin:
            return False
        if not target[1]-marginY < currentLoc[1] < target[1]+marginY:
            return False
        if not target[2]-margin < currentLoc[2] < target[2]+margin:
            return False
        return True

    def anticipateHuman(self, can_nr):
        calculator = TrajectoryCalculator()

        if self.human_state[can_nr] == "predicted_target":
            print("-----Object state (replan): ", self.state)

            # Continue trajectory to different can
            self.new_can = random.choice(self.list_cans)
            while self.new_can == can_nr:
                self.new_can = random.choice(self.new_can_list)
            
            # CAN IS NOW THE WRONG CAN. THE CAN NEEDS TO BE CHANGED TO THE NEW_CAN CAN
            if self.new_can != can_nr:
                # Check availability
                if self.checkAvailability(self.new_can):

                    # Calculate trajectory to can
                    pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())
                
                    # Start reaching for the can
                    self.panda.realTimeSim(time_to_grab, 0, pathcan, can.getPos())
                    self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())
                    
                    while not self.robot_reached:
                        pathcan = calculator.linearTrajectory(self.panda.getEndLocation(), can.getPos())
                        self.panda.realTimeSim(time_to_correct, 0, pathcan, can.getPos())
                        self.robot_reached = self.checkPoint(can.getPos(), self.panda.getEndLocation())

                        # while reaching, check if available, otherwise return to base
                        if not self.checkAvailability(can_nr):
                            self.panda.moveToBase()
                            print(f"-----Object state {can_nr} not available (during reach): ", self.state[can_nr])
                            print(self.state)
                            break
                
                self.robot_end_time = datetime.now()

                # Put can into box
                if self.robot_reached:
                     self.putCanInBox(can, can_nr)
                    
            else:
                self.robot_end_time = datetime.now()
                self.panda.moveToBase()
                print(f"-----Object state {can_nr} not available: ", self.state[can_nr])
                print(self.state)
    
    def putCanInBox(self, can, can_nr):
        calculator = TrajectoryCalculator()

        # Grip the Can
        self.panda.closeGripper()

        # Move to Box
        h = 0.2
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

        # Check with prints if correct things happen
        if self.box.inBox(can) or self.box.fellOnFloor(can):
            print("Robot finished a can")
            # print("Human State after robot finish: ",self.human_state)

        # Return to base
        self.panda.moveToBase()
    
    def stopEnvironment(self):
        """"Stops the environment and disconnects."""
        self.stop_tracking = True
        time.sleep(0.1*SLEEP_TIME)
        p.disconnect()

    # Tracking Method (STILL TO BE WRITTEN)
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

participantID = input("Please type participant ID: ")
the_experiment = Experiment()
the_experiment.main()