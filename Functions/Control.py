from Arm import Arm
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *

import HiwonderSDK.Board as Board

AK = ArmIK()

class Control():
    def __init__(self, arm):
        self.arm = arm
        self.servo1 = 500
        self.coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }


    def set_rgb(color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    
    def setBuzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)


# Arm movement route
def move(self):

    # CONTROL FROM HERE
    # coordinates for goal location (x, y, z) for each color
    
    while True:
        if self.arm.__isRunning:
            if self.arm.first_move and self.arm.start_pick_up: # first time an object is detected               
                self.arm.action_finish = False
                self.set_rgb(self.arm.detect_color)
                self.setBuzzer(0.1)               
                result = AK.setPitchRangeMoving((self.arm.world_X, self.arm.world_Y - 2, 5), -90, -90, 0) # operation time is not set, adapted
                if result == False:
                    self.arm.unreachable = True
                else:
                    self.arm.unreachable = False
                time.sleep(result[2]/1000) # result returns a list where the third item is an estimation of time
                self.arm.start_pick_up = False
                self.arm.first_move = False
                self.arm.action_finish = True
            elif not self.arm.first_move and not self.arm.unreachable: # not the first time object is dtected 
                self.set_rgb(self.arm.detect_color)
                if track: # if we are in the tracking state
                    if not self.arm.__isRunning: # if we stopped or exited from target detection
                        continue
                    AK.setPitchRangeMoving((self.arm.world_x, self.arm.world_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)                    
                    track = False
                if self.arm.start_pick_up: #if item hasn't moved in a while, start grasping it
                    action_finish = False
                    if not self.arm.__isRunning: # stopped or exited
                        continue
                    Board.setBusServoPulse(1, self.servo1 - 280, 500)  # opens the gripper here
                    # get_angle calculates the rotation angle of the gripper
                    servo2_angle = getAngle(self.arm.world_X, self.arm.world_Y, self.arm.rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
                    
                    if not self.arm.__isRunning:
                        continue
                    AK.setPitchRangeMoving((self.arm.world_X, self.arm.world_Y, 2), -90, -90, 0, 1000)  # lower the gripper 
                    time.sleep(2)
                    
                    if not self.arm.__isRunning:
                        continue
                    Board.setBusServoPulse(1, self.servo1, 500)  # close the gripper 
                    time.sleep(1)
                    
                    if not self.arm.__isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((self.arm.world_X, self.arm.world_Y, 12), -90, -90, 0, 1000)  # raise the arm 
                    time.sleep(1)
                    
                    if not self.arm.__isRunning:
                        continue
                    # sort different blocks by color to different location
                    result = AK.setPitchRangeMoving((self.coordinate[self.arm.detect_color][0], self.coordinate[self.arm.detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    if not self.arm.__isRunning:
                        continue
                    servo2_angle = getAngle(self.coordinate[self.arm.detect_color][0], self.coordinate[self.arm.detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not self.arm.__isRunning:
                        continue
                    AK.setPitchRangeMoving((self.coordinate[self.arm.detect_color][0], self.coordinate[self.arm.detect_color][1], self.coordinate[self.arm.detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not self.arm.__isRunning:
                        continue
                    AK.setPitchRangeMoving((self.coordinate[self.arm.detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
                    
                    if not self.arm.__isRunning:
                        continue
                    Board.setBusServoPulse(1, self.servo1 - 200, 500)  # gripper opens and drops the block 
                    time.sleep(0.8)
                    
                    if not self.arm.__isRunning:
                        continue                    
                    AK.setPitchRangeMoving((self.coordinate[self.arm.detect_color][0], self.coordinate[self.arm.detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    self.arm.initMove()  # return to starting position 
                    time.sleep(1.5)

                    self.arm.detect_color = 'None'
                    self.arm.first_move = True
                    self.arm.get_roi = False
                    self.arm.action_finish = True
                    self.arm.start_pick_up = False
                    self.set_rgb(self.arm.detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, self.servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)