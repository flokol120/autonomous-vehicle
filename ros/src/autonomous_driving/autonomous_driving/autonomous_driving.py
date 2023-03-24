from typing import List

from simple_pid import PID

from matplotlib.pyplot import sca
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Empty

from sensor_msgs.msg import Range

import time

import os

from enum import Enum

# node pub/sub queue length
QUEUE_LENGTH = 10

# [Lane Distances]
OPTIMAL_RIGHT_LANE_DISTANCE = 510.0
OPTIMAL_LEFT_LANE_DISTANCE = 700.0

# [Parking]
PARKING_SPEED = 1.5
TARGET_BOX_DISTANCE_BELOW = 0.4
PARK_STAGES = [{"steer": 0.0, "speed": -.1, "dist": .03},
               {"steer": 20.0, "speed": -.75, "dist": .72}, 
               {"steer": -20.0, "speed": -.75, "dist": .63}]
PARKING_OFFSET_TOLERANCE = 0.1
PARKING_OPTIMISE_SPEED = 0.5
PARKING_WAIT_TIME = 3
PARKING_EXIT_BACK_DIST = 0.3
PARKING_EXIT_BUFFER = 0.3
PARK_EXIT_STAGES = [{"steer": -20.0, "speed": .75, "dist": .7},
                    {"steer": 20.0, "speed": .75, "dist": .6}]

# [PID]
p, i, d = 16.4, 2/100_000, 0
laneP, laneI, laneD = 0.05, 5/1_000, 0
target = .93
speedP, speedI, speedD = 2.95, 5/10_000, 0.1
grandmaTarget = 1.0
speedPGrandma, speedIGrandma, speedDGrandma = 1.5, 5/100_000, 0
MIN_SPEED = 2.55
MAX_SPEED = 5.4

# [Overtaking]
OVERTAKE_INITIATE_DISTANCE = 1.6
SLOW_DOWN_DISTANCE = 3.0
OVERTAKE_ABORT_MAX_DISTANCE = .3
OVERTAKE_MIN_SPEED = 2.0
OVERTAKE_MAX_SPEED = 2.7
OVERTAKE_MAX_SPEED_GRANDMA = 2.4

# [States]
STATE_LIST = [1, 10, 5, 1] # always start in stopped state, press 'p' to pop

class State(Enum):
    stopped = 1 

    overtake_search_start = 2
    overtake_start = 3
    overtake_search_end = 4

    parking_search_start = 5
    parking_stages = 6
    parking_optimise = 7
    parking_prepare_exit = 8
    parking_exit = 9
    
    driving = 10
    remote = 11

class Driver(Node):
    debug = False
    mLeft = 0
    mRight = 0
    lane_dist_right = OPTIMAL_RIGHT_LANE_DISTANCE
    timestamp = 0
    isRightSide = True
    abortion = False
    ghost_driver_mode_enabled = False
    seenBoxes = 0
    seeingBox = False
    pidSetup = False
    currentParkingStage = None
    stageStartTime = None
    isSlowedDown = False
    lastState = None
    steerValue = 0.0
    tofFront = None
    tofBack = None

    def __init__(self, debug=False):
        super().__init__('autonomous_driving')

        self.debug = debug

        self.state = StateHandler(baseState=State.stopped)
        self.state.pushNumbers(STATE_LIST) # order of execution
        self.grandmaMode = False
        self.debugBegin()
        
        # heatmap data
        self.speeds = []
        self.lastSpeedChange = None
        self.distance = []
        try:
            os.remove("heatmap.csv") 
        except: pass

        # [Image Controller]
        self.create_subscription(Float64, '/image_controller/m_right', self.updateMRight, QUEUE_LENGTH)
        self.create_subscription(Float64, '/image_controller/m_right_far', self.updateMRightFar, QUEUE_LENGTH)
        self.create_subscription(Float64, '/image_controller/m_left_far', self.updateMLeftFar, QUEUE_LENGTH)
        self.create_subscription(Float64, '/image_controller/m_left', self.updateMLeft, QUEUE_LENGTH)
        self.create_subscription(Float64, '/image_controller/lane_dist_right', self.updateLaneRight, QUEUE_LENGTH)
        self.create_subscription(Float64, '/image_controller/lane_dist_left', self.updateLaneLeft, QUEUE_LENGTH)
        self.create_subscription(Empty, '/image_controller/tick', self.tickCallback, QUEUE_LENGTH)
        self.pubSwitchBaselines = self.create_publisher(Bool, '/image_controller/switch_baselines', 1)

        # [Remote Controller]
        self.create_subscription(Float64, '/remote_controller/steering', self.updateRemoteSteer, QUEUE_LENGTH)
        self.create_subscription(Float64, '/remote_controller/speed', self.updateRemoteSpeed, QUEUE_LENGTH)

        self.create_subscription(Empty, '/remote_controller/abort', self.abortCallback, QUEUE_LENGTH)
        self.create_subscription(Empty, '/remote_controller/switch_lane', self.switchLanesCallback, QUEUE_LENGTH)
        self.create_subscription(Empty, '/remote_controller/reset', self.resetCallback, QUEUE_LENGTH)
        self.create_subscription(Empty, '/remote_controller/pop_state', self.popStateCallback, QUEUE_LENGTH)
        self.create_subscription(Empty, '/remote_controller/recreate_stack', self.recreateStackCallback, QUEUE_LENGTH)
        self.create_subscription(Empty, '/remote_controller/slowmode', self.grandmaModeCallback, QUEUE_LENGTH)

        # [TOF Sensors]
        self.create_subscription(Range, '/tof_right', self.tofRightCallback, 1)
        self.create_subscription(Range, '/tof_front', self.tofFrontCallback, 1)
        self.create_subscription(Range, '/tof_back', self.tofBackCallback, 1)

        # [Driving Publishers]
        self.pubSteer = self.create_publisher(Float64, '/steering', 1)
        self.pubSpeed = self.create_publisher(Float64, '/speed', 1)

    # Setup the steering pids. Also determines whether the car should drive on the left or right side of the road. Initiates speed PID setup
    def setupPids(self):
        if 'pid' in locals() and self.pid != None:
            self.pid.reset()
        self.pid = PID(p, i, d, setpoint=0)
        self.pid.output_limits = (-45.0, 45.0)
        self.pid.sample_time = 1/30

        if 'lanePid' in locals() and self.lanePid != None:
            self.lanePid.reset()

        # set Distances. Determines whether to drive on the left or right side
        self.lanePid = PID(laneP, laneI, laneD, setpoint= OPTIMAL_RIGHT_LANE_DISTANCE if self.state.shouldSelfDrive() else OPTIMAL_LEFT_LANE_DISTANCE)
        self.lanePid.output_limits = (-45.0, 45.0)
        self.lanePid.sample_time = 1/30
        
        self.setupSpeedPid()

    # Initiates the speed pid. Also sets max/min speed according to the current mode (normal or overtake). GrandmaMode is an additional set of PID which is slightly slower
    def setupSpeedPid(self):
        if 'speedPid' in locals() and self.speedPid != None:
            self.speedPid.reset()
        self.speedPid = PID(speedPGrandma if self.grandmaMode else speedP, 
                            speedIGrandma if self.grandmaMode else speedI, 
                            speedDGrandma if self.grandmaMode else speedD, 
                            setpoint=grandmaTarget if self.grandmaMode else target)
        if self.state.isOvertaking():
            self.speedPid.output_limits = (OVERTAKE_MIN_SPEED, OVERTAKE_MAX_SPEED_GRANDMA if self.grandmaMode else OVERTAKE_MAX_SPEED)
        else:
            self.speedPid.output_limits = (MIN_SPEED, MAX_SPEED)
        self.speedPid.sample_time = 1/30
        
    # Publishes the current values to the car (speed and steering)
    def publishValues(self):
        steer = Float64()
        steer.data = self.steerValue
        speed = Float64()
        speed.data = self.speedValue
        self.pubSteer.publish(steer)
        self.pubSpeed.publish(speed)

    # Tick is 'called' from image_processing on every published frame
    # Act according to the current state
    def tickCallback(self, msg):
        autoModeEnabled = self.state.shouldSelfDrive()
        if not self.pidSetup and autoModeEnabled:
            self.setupPids()
            self.pidSetup = True
        
        # update pid modes, autoMode is true if car is self driving
        if 'pid' in locals() and self.pid != None:
            self.pid.auto_mode = autoModeEnabled
        if 'lanePid' in locals() and self.pid != None:
            self.lanePid.auto_mode = autoModeEnabled
        if 'speedPid' in locals() and self.speedPid != None:
            self.speedPid.auto_mode = autoModeEnabled
            
        # reset pids if state has been changed since last tick 
        if self.lastState != self.state.get() and not self.state.isOvertaking():
            self.setupPids()

        # initiate parking mode (this state will be replaced immediately once a second box is seen)
        if self.state.get() == State.parking_search_start:
            self.beginParking()

        # continue iteration through parking stages
        elif self.state.get() == State.parking_stages:
            if self.currentParkingStage == None: 
                self.currentParkingStage = 0

            if self.stageStartTime == None:
                self.stageStartTime = time.time()

            # determine if current parking stage is done
            elif self.calculateDistance(self.stageStartTime, PARK_STAGES[self.currentParkingStage]['speed']) >= PARK_STAGES[self.currentParkingStage]["dist"]:
                self.stageStartTime = None
                self.currentParkingStage += 1
                self.resetMovement()
                # determine if all parking stages are done
                if self.currentParkingStage >= len(PARK_STAGES):
                    self.state.pop()
                    self.currentParkingStage = None
                    self.seenBoxes = 0
                    self.seeingBox = False
            else:
                # keep driving at angle 'steer' with soeed 'speed'
                steerFloat = Float64()
                steerFloat.data = PARK_STAGES[self.currentParkingStage]['steer']
                self.pubSteer.publish(steerFloat)
                speedFloat = Float64()
                speedFloat.data = PARK_STAGES[self.currentParkingStage]['speed']
                self.pubSpeed.publish(speedFloat)

        # optimise position after parking
        elif self.state.get() == State.parking_optimise:
            if self.stageStartTime == None:
                self.stageStartTime = time.time()
                steerFloat = Float64()
                steerFloat.data = 0.0
                self.pubSteer.publish(steerFloat)
            
            currentOffset = abs(self.tofFront - self.tofBack)

            # determine if the position still needs to be optimised
            if currentOffset > PARKING_OFFSET_TOLERANCE:
                self.stageStartTime = None
                if self.tofFront > self.tofBack:
                    speedFloat = Float64()
                    speedFloat.data = PARKING_OPTIMISE_SPEED
                    self.pubSpeed.publish(speedFloat)
                else:
                    speedFloat = Float64()
                    speedFloat.data = -PARKING_OPTIMISE_SPEED
                    self.pubSpeed.publish(speedFloat)
            else:
                # optimization done, wait until parking wait time has passed and end state
                speedFloat = Float64()
                speedFloat.data = 0.0
                self.pubSpeed.publish(speedFloat)

                if time.time() - self.stageStartTime > PARKING_WAIT_TIME:
                    self.stageStartTime = None
                    self.state.pop()
                    self.resetMovement()
    
        # reverse to get ready for exiting the parking space
        elif self.state.get() == State.parking_prepare_exit:
            if self.stageStartTime == None:
                self.stageStartTime = time.time()
            
            # determine if vehicle needs to reverse more
            if self.tofBack > PARKING_EXIT_BACK_DIST:
                self.stageStartTime = None
                speedFloat = Float64()
                speedFloat.data = -PARKING_OPTIMISE_SPEED
                self.pubSpeed.publish(speedFloat)
            else:
                # reverse done, wait for buffer to end and end state
                speedFloat = Float64()
                speedFloat.data = 0.0
                self.pubSpeed.publish(speedFloat)

                if time.time() -  self.stageStartTime > PARKING_EXIT_BUFFER:   
                    self.stageStartTime = None
                    self.state.pop()
                    self.resetMovement()

        # continue iteration through parking exit stages
        elif self.state.get() == State.parking_exit:
            if self.currentParkingStage == None: 
                self.currentParkingStage = 0

            if self.stageStartTime == None:
                self.stageStartTime = time.time()

            elif self.calculateDistance(self.stageStartTime, PARK_EXIT_STAGES[self.currentParkingStage]['speed']) >= PARK_EXIT_STAGES[self.currentParkingStage]["dist"]:
                self.stageStartTime = None
                self.currentParkingStage += 1
                self.resetMovement()
                if self.currentParkingStage >= len(PARK_EXIT_STAGES):
                    self.state.pop()
                    self.currentParkingStage = None
                    self.stageStartTime = None
                    self.setupPids()
                    speedFloat = Float64()
                    speedFloat.data = 0.0
                    self.pubSpeed.publish(speedFloat)
            else:
                steerFloat = Float64()
                steerFloat.data = PARK_EXIT_STAGES[self.currentParkingStage]['steer']
                self.pubSteer.publish(steerFloat)
                speedFloat = Float64()
                speedFloat.data = PARK_EXIT_STAGES[self.currentParkingStage]['speed']
                self.pubSpeed.publish(speedFloat)

        elif self.state.get() == State.overtake_search_start:
            self.beginOvertake()

        elif self.state.get() == State.stopped:
            self.resetMovement()
        
        elif self.state.get() == State.driving:
            pass # self driving is handled seperately
        
        self.lastState = self.state.get()

        self.pdebug("Grandma mode: %s" % (str(self.grandmaMode)))

        self.debugEnd()
        self.debugBegin()
        
    # Handle the values from the right TOF Sensor
    # Parking-Mode: Count boxes and stop after finding the second box.
    def tofRightCallback(self, msg):
        self.pdebug("Received Tof Right: %f" % msg.range)

        if self.state.get() == State.parking_search_start:
            if TARGET_BOX_DISTANCE_BELOW > msg.range and not self.seeingBox:
                self.seeingBox = True
            elif TARGET_BOX_DISTANCE_BELOW <= msg.range and self.seeingBox:
                self.seeingBox = False
                self.seenBoxes += 1
            
        if self.state.get() == State.overtake_search_end:
            if not self.seeingBox and msg.range <= OVERTAKE_ABORT_MAX_DISTANCE:
                self.seeingBox = True
            elif self.seeingBox and msg.range > OVERTAKE_ABORT_MAX_DISTANCE:
                self.state.pop()
                self.state.push(State.driving)
                self.isRightSide = True
                self.seeingBox = False
                self.isSlowedDown = False
                self.setupPids()
                self.timer = self.create_timer(.1, self.switchBaselines)

    # Handle the values from the front TOF Sensor
    # if a box is within sensoring distance -> slow down
    # if the box is very clos -> initiate overtake behaviour
    def tofFrontCallback(self, msg):
        self.tofFront = msg.range
        if self.state.get() != State.driving:
            return

        self.pdebug("Received Tof Front: %f" % msg.range)
        
        if not self.isSlowedDown and msg.range <= SLOW_DOWN_DISTANCE:
            self.speedPid.output_limits = (OVERTAKE_MIN_SPEED, OVERTAKE_MAX_SPEED)
            self.speedPid.reset()
            self.isSlowedDown = True

        if msg.range <= OVERTAKE_INITIATE_DISTANCE:
            self.state.pop()
            self.state.push(State.overtake_search_end)
            self.setupPids()
            self.speedPid.output_limits = (OVERTAKE_MIN_SPEED, OVERTAKE_MAX_SPEED)
            self.isRightSide = False
            self.timer = self.create_timer(.5, self.switchBaselines)

    # Handle the values from the front TOF Sensor
    # used for centering the car between the two boxes while parking
    def tofBackCallback(self, msg):
        self.tofBack = msg.range
    
    # transition to parking states once the second box is being seen
    def beginParking(self):
        if not (self.seeingBox and self.seenBoxes == 1):
            return

        self.state.pop()
        self.state.pushList([State.parking_exit, State.parking_prepare_exit, State.parking_optimise, State.parking_stages])

        self.resetMovement()
    
    # transition to overtake_start state. Also needs to re-setup PIDs as the baselines need to change
    def beginOvertake(self):
        self.state.pop()
        self.state.push(State.overtake_start)
        self.resetMovement()
        self.setupPids()
        
    # calculates a distance relative to the startTime and the current time while respecting the current speed
    def calculateDistance(self, startTime, speed):
        currentTime = time.time()
        delta = currentTime - startTime
        self.pdebug("Delta: %f, Speed: %f, startTime %f, Distance: %f" % (delta, speed, startTime, abs(delta * speed)))
        return abs(delta * speed)
        
    # stop the car and center steerings
    def resetMovement(self):
        steer = Float64()
        steer.data = 0.0
        speed = Float64()
        speed.data = 0.0
        self.pubSteer.publish(steer)
        self.pubSpeed.publish(speed)

    # Update the received m right
    def updateMRight(self, msg):
        if msg.data == -1.0:
            return
        self.mRight = msg.data
        self.pdebug("Received m-right: %f" % self.mRight)
        if self.state.shouldSelfDrive():
            if 'timer' in locals() and self.timer != None:
                self.timer.cancel()
            (self.steerValue, self.speedValue) = self.handleDriving(self.mRight, self.lane_dist_right, OPTIMAL_RIGHT_LANE_DISTANCE)
            self.publishValues()

    # Update the received far m right
    def updateMRightFar(self, msg):
        if msg.data == -1.0:
            self.mRightFar = 0.0
            return
        self.mRightFar = msg.data
        self.pdebug("Received m-right-far: %f" % self.mRightFar)

    # Update the received far m right
    def updateMLeftFar(self, msg):
        if msg.data == -1.0:
            self.mLeftFar = 0.0
            return
        self.mLeftFar = msg.data
        self.pdebug("Received m-right-far: %f" % self.mLeftFar)
        
    # Update the received m left
    def updateMLeft(self, msg):
        if msg.data == -1.0:
            return
        self.mLeft = msg.data
        self.pdebug("Received m-left: %f" % self.mLeft)
        if self.state.get() == State.overtake_search_end:
            (self.steerValue, self.speedValue) = self.handleDriving(self.mLeft, self.lane_dist_left, OPTIMAL_LEFT_LANE_DISTANCE)
            if self.isRightSide:
                self.isRightSide = not self.isRightSide
                self.timer = self.create_timer(1.0, self.switchBaselines)
            self.publishValues()

    # switch the baselines while overtaking
    def switchBaselines(self):
        boolmsg = Bool()
        boolmsg.data = self.isRightSide
        self.pubSwitchBaselines.publish(boolmsg)
        self.setupPids()
        self.timer.cancel()
        
    # update the current right lane orientation
    def updateLaneRight(self, msg):
        self.lane_dist_right = msg.data
        self.pdebug("Received lane_dist_right: %f" % self.lane_dist_right)

    # update the current left lane orientation
    def updateLaneLeft(self, msg):
        self.lane_dist_left = msg.data
        self.pdebug("Received lane_dist_left: %f" % self.lane_dist_left)

    # Change state to manual mode on incoming signal
    def updateRemoteSteer(self, msg):
        self.pdebug("Received remote steer: %f" % msg.data)
        self.remoteSteer = msg.data
        if self.state.get() == State.remote:
            (self.steerValue, self.speedValue) = (self.remoteSteer, self.remoteSpeed)
            self.publishValues()

    # get the remote speed input
    def updateRemoteSpeed(self, msg):
        self.pdebug("Received remote speed: %f" % msg.data)
        self.remoteSpeed = msg.data
        if self.state.get() == State.remote:
            (self.steerValue, self.speedValue) = (self.remoteSteer, self.remoteSpeed)
            self.publishValues()

    # feed values to PIDs -> this is the heart of the autonomous driver
    def handleDriving(self, m, lane_dist, targetLaneDistance):
        if not self.pidSetup: return (0.0, 0.0)

        self.pdebug("Handling driving")
        # feed incline to PID
        output = self.pid(m)
        if output == None:
            output = 0
        # feed value to lane PID
        lane_output = self.lanePid(lane_dist if lane_dist > 0 else targetLaneDistance)
        if lane_output == None:
            lane_output = 0
        # incline PID + lane PID
        steerFloat = output + lane_output
        # feed values to speed PID
        speedFloat = PARKING_SPEED if self.state.isParking() else self.speedPid((abs(self.mRightFar) + abs(self.mLeftFar) + abs(self.mLeft) + abs(self.mRight)) / 4)

        self.logSpeed(speedFloat)
        
        self.pdebug("")
        self.pdebug("Received m: %f, Received lane_dist: %f" % (m, lane_dist))
        self.pdebug("Output: %f, Lane output: %f" % (output, lane_output))
        self.pdebug("Steer: %f, Speed: %f" % (steerFloat, speedFloat))
        self.pdebug("Far m: %f, Speed: %f" % (self.mRightFar, speedFloat))
        return (steerFloat, speedFloat)

    # write speed values and distance to csv file for inspection and analysis
    def logSpeed(self, speedFloat):
        if self.state.get() == State.driving or self.state.isOvertaking():
            distance = 0.0
            speed = 0.0
            if(self.lastSpeedChange != None and self.speeds[-1] > 0.0):
                lastDistance = 0.0
                if self.distance[-1] != None:
                    lastDistance = self.distance[-1]
                distance = self.calculateDistance(self.lastSpeedChange, self.speeds[-1]) + lastDistance
                speed = self.speeds[-1]
            self.distance.append(distance)
            with open("speed.csv", "a") as speedFile:
                speedFile.write("%f,%f\n" % (distance, speed))
            self.speeds.append(speedFloat)
            self.lastSpeedChange = time.time()

    # Abort all autonomous efforts if received signal
    def abortCallback(self, _):
        self.abortion = not self.abortion
        if self.abortion: 
            self.state.push(State.remote)
        elif self.state.get() == State.remote:
            self.state.pop()

    # manually initiate overtake mode
    def switchLanesCallback(self, msg):
        self.ghost_driver_mode_enabled = not self.ghost_driver_mode_enabled
        if self.ghost_driver_mode_enabled: 
            self.state.push(State.overtake_search_start)
            self.setupPids()
        elif self.state.get() == State.overtake_search_start:
            self.state.pop() # manually abort overtake

    # manually reset PIDs
    def resetCallback(self, _):
        self.setupPids()

    # manually reset PIDs
    def grandmaModeCallback(self, _):
        self.grandmaMode = not self.grandmaMode
        self.setupSpeedPid()

    # manually reset stack elements
    def recreateStackCallback(self, _):
        self.state.clear()
        self.state.pushNumbers(STATE_LIST)
        self.setupPids()

    # manually pop element from stack
    def popStateCallback(self, _):
        self.state.pop()

    # custom queued print function for debug mode
    def pdebug(self, str):
        if self.debug: self.lines.append(str)

    # start debug frame (if in debug mode). Frame will collect lines (through pdebug) as long as the frame is not ended
    def debugBegin(self):
        if not self.debug: return
        self.timestamp = time.time()
        self.lines = []
        self.pdebug("----------------------------------------------------")
        self.pdebug("Debug: %s, State: %s" % (self.debug, self.state.get()))
        self.pdebug("")

    # end debug frame (if in debug mode). Append time taken in frame, clear the terminal and print all collected lines
    def debugEnd(self):
        if not self.debug: return
        self.pdebug("")
        self.pdebug("Time taken: %f" % (time.time() - self.timestamp))
        self.pdebug("----------------------------------------------------")
        clear_terminal()
        print('\n'.join(self.lines))

class StateHandler:
    def __init__(self, baseState: State):
        self.baseState = baseState
        self.stateStack = []
    
    def push(self, state: State):
        self.stateStack.append(state)

    # append multiple states at once. Last state in the array will be on top of the stack
    def pushList(self, states: List[State]):
        self.stateStack.extend(iter(states))

    # append multiple states by ordinal
    def pushNumbers(self, states: List[int]):
        self.stateStack.extend(iter(State(x) for x in states))
        
    # remove the top state. baseState acts as a default state which cannot be popped
    def pop(self):
        if len(self.stateStack) > 0:
            return self.stateStack.pop()
        else:
            return self.baseState

    # get top state or base state if no states are stored
    def get(self):
        if len(self.stateStack) > 0:
            return self.stateStack[-1]
        else:
            return self.baseState

    def clear(self):
        self.stateStack = []

    def isParking(self):
        return self.get() in [State.parking_search_start, State.parking_stages, State.parking_optimise, State.parking_prepare_exit, State.parking_exit]

    def isOvertaking(self):
        return self.get() in [State.overtake_search_start, State.overtake_start, State.overtake_search_end]

    def shouldSelfDrive(self):
        return self.get() in [State.driving, State.overtake_search_start, State.overtake_start, State.parking_search_start]

def clear_terminal():
    # https://stackoverflow.com/a/2084628
    os.system('cls' if os.name == 'nt' else 'clear')

def default():
    run_with_args()

def debug():
    run_with_args(debug=True)

def run_with_args(debug=False):
    rclpy.init()

    driver = Driver(debug)

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    default()

if __name__ == '__main__':
    main()
