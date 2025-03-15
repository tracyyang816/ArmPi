import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from Perception import Perception
from Control import Control
from Arm import Arm


if __name__ == '__main__':
    arm = Arm()
    arm.init()
    arm.start()

    perception_mod = Perception(arm)
    control_mod = Control(arm)

    #__target_color = ('red', )

    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        print("img not none, debug")
        if img is not None:
            frame = img.copy()
            Frame = perception_mod.process_frame(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
            control_mod.move()
    my_camera.camera_close()
    cv2.destroyAllWindows()



#!/usr/bin/python3
# coding=utf8

import time

from Perception import Perception
from Motion import Motion

def main(stacking):
    # start the camera and perception
    my_camera = Camera.Camera()
    my_camera.camera_open()
    perception = Perception(my_camera.frame)
    
    # Start the motion object
    motion = Motion(reset_time_ms=650)
    motion.start()

    # Continuous loop
    while True:
        # get the labelled frame and position dictionary from running the perception object
        frame_labelled, position_dictionary = perception.run()
        # restart the loop if the frame was not captured
        if(frame_labelled is None):
            # stall a bit to not use resources
            time.sleep(0.01)
            continue
        # If there are no objects in the field, then reset the robot and wait
        if(len(position_dictionary) == 0):
            motion.reset_position()
            continue
        # code to run when stacking
        if(stacking):
            # Get the stack index from the number of objects on the field
            stack_index = 3 - len(position_dictionary)
            # Get the first instance in the color-position dictionary to check the status of the field again
            color = list(position_dictionary.keys())[0]
            # move the arm to stack the colors
            print(f"Getting {color}")
            motion.run_stack(stack_index, position_dictionary[color], speed=3.5)
        # code to run when sorting
        else:
            # Get the stack index from the number of objects on the field
            stack_index = 3 - len(position_dictionary)
            # Get the first instance in the color-position dictionary to check the status of the field again
            color = list(position_dictionary.keys())[0]
            # move the arm to stack the colors
            print(f"Getting {color}")
            motion.run_sort(color, position_dictionary[color], speed=5.0)


if(__name__ == "__main__"):
    # run main with stacking option; False=sorting
    main(stacking=True)