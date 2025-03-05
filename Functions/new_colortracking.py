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