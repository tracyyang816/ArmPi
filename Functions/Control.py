
#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import numpy as np
import Camera
import threading
from LABConfig import *
from ArmIK import Transform
from ArmIK import ArmMoveIK
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from Perception import Perception
from Control_Class import Control


class Control():
    def __init__(self, reset_time_ms=1500):
        self.reset_time_ms = reset_time_ms # reset speed
        self.cube_grab_from_bottom_cm = 1.0 # how far from the ground is the cube
        self.cube_height_cm = 3.0 # coordinates for stacking 

        self.offset_world_xy = np.array([1.5, 0.25]) # set an offset for the world coordinates
        self.AK = ArmMoveIK.ArmIK()
        self.servo1 = 500
        self.reset_position()
     
        self.coordinate = {
            "red":   (-14.5, 12.5, self.cube_grab_from_bottom_cm),
            "green": (-14.5, 6.5,  self.cube_grab_from_bottom_cm),
            "blue":  (-14.5, 0.5,  self.cube_grab_from_bottom_cm),
            "stack": (-14.5, -7.0, [self.cube_grab_from_bottom_cm + i*(self.cube_height_cm) for i in range(0,3)])
        }

        self.target_color = "None"
        self.stack_idx = 0
        
    

    def _reset(self):
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    def start(self):
        self._reset()
        self.reset_position()

    def reset_position(self):
        Board.setBusServoPulse(1, self.servo1 - 50, min(300, self.reset_time_ms))
        Board.setBusServoPulse(2, 500, min(500, self.reset_time_ms))
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, self.reset_time_ms)
        time.sleep(self.reset_time_ms/1000)

    def run(self, task, position_vector, speed=1.0):


        if task == "sort":
            if((self.target_color == "None") or (position_vector is None)):
                return
            # parse the position vector
            world_X, world_Y = np.array(position_vector[0]) + self.offset_world_xy
            width, height = position_vector[1]
            rotation_angle = position_vector[2]

            # get the target coodinates for three colors
            home_X, home_Y, home_Z = self.coordinate[self.target_color]
        
        elif task == "stack":
            if((self.stack_idx >= 3) or (self.stack_idx < 0) or (position_vector is None)):
                return
            # parse the position vector 
            world_X, world_Y = np.array(position_vector[0]) + self.offset_world_xy
            width, height = position_vector[1]
            rotation_angle = position_vector[2]

            # get stack placement coordinates
            home_X, home_Y, home_Z = self.coordinate["stack"]
            home_Z = home_Z[self.stack_idx]
            
  
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0, int(1500/speed))
        time.sleep(movetime_ms/1000)


        servo2_angle = Transform.getAngle(world_X, world_Y, rotation_angle) #calculate gripper angle rotation
        Board.setBusServoPulse(1, self.servo1 - 280, int(500/speed))  # open claw
        Board.setBusServoPulse(2, servo2_angle, int(500/speed))
        time.sleep(0.5/speed)

        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((world_X, world_Y, self.cube_grab_from_bottom_cm), -90, -90, 0, int(1000/speed))
        time.sleep(movetime_ms/1000)

        Board.setBusServoPulse(1, self.servo1, int(500/speed))  # close claw
        time.sleep(1.2/speed)


        Board.setBusServoPulse(2, 500, int(500/speed))
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (world_X, world_Y, 12), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(1000/speed)
        )
        time.sleep(movetime_ms/1000)

        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((home_X, home_Y, 12), -90, -90, 0, int(1500/speed))
        time.sleep(movetime_ms/1000)
        
    
        servo2_angle = Transform.getAngle(home_X, home_Y, -90)
        Board.setBusServoPulse(2, servo2_angle, int(500/speed))
        time.sleep(0.5/speed)


        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((home_X, home_Y, home_Z+3), -90, -90, 0, int(500/speed))
        time.sleep(movetime_ms/1000)
 
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((home_X, home_Y, home_Z), -90, -90, 0, int(1000/speed))
        time.sleep(movetime_ms/1000)

        Board.setBusServoPulse(1, self.servo1 - 200, int(500/speed))
        time.sleep(0.8/speed)

        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving((home_X, home_Y, 12), -90, -90, 0, int(800/speed))
        time.sleep(movetime_ms/1000)




def main(task):

    my_camera = Camera.Camera()
    my_camera.camera_open()
    perception = Perception(my_camera.frame)
    
    motion = Control(reset_time_ms=650)
    motion.start()

    while True:
        frame_labelled, position_dictionary = perception.run()
        if frame_labelled is None:
            time.sleep(0.01)
            continue

        # If there are no objects in the field, reset the robot and wait
        if len(position_dictionary) == 0:
            motion.reset_position()
            continue

        # Get the stack index from the number of objects on the field
        stack_index = 3 - len(position_dictionary)
        motion.stack_idx = stack_index

        # Get the first instance in the color-position dictionary to check the status of the field again
        color = list(position_dictionary.keys())[0]
        print(f"Getting {color}")
        motion.target_color = color
        
        # execute sort or stack task 
        motion.run_stack(task, position_dictionary[color], speed=3.5)

    