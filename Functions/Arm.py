
import HiwonderSDK.Board as Board
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *


AK = ArmIK()

class Arm():
    def __init__(self):
        self.rect = None 
        self.track = False
        self._stop = False
        self.get_roi = False
        self.unreachable = False
        self.__isRunning = False
        self.detect_color = "None"
        self.action_finish = True
        self.rotation_angle = 0
        self.world_X = 0
        self.world_Y = 0
        self.world_x = 0
        self.world_y = 0
        self.center_list = []
        self.count = 0
        self.start_pick_up = False
        self.first_move = True
        self.start_count_t1 = True
        self.servo1 = 500

        

        self.range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
        }


    # initialize app
    def init(self):
        print("ColorTracking Init")
        self.initMove()

    # run app
    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    # stopp app
    def stop(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Stop")

    # exit app
    def exit(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Exit")

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    
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
