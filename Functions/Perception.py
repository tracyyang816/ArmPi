
import cv2 
import math 
from Arm import Arm
import numpy as np
import time

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *


class Perception():
    def __init__(self, arm, target_color='red'):
        self.camera = cv2.VideoCapture(0)  # Initialize camera
        self.target_color = target_color
        self.arm = arm
        self.size = (640, 480)

    def set_targetColor(self, target_color):
        self.target_color = target_color

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate through all contours 
            contour_area_temp = math.fabs(cv2.contourArea(c))  # calcualte the aread size
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # filter out areas smaller than 300
                    area_max_contour = c

        return area_max_contour, contour_area_max  # return the largest contour

    
    def process_frame(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not self.arm.__isRunning:
            return img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        # if an roi(region of interest) has object detected, keep monitoring it no object is detected no more
        if self.arm.get_roi and self.arm.start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, self.size)    
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # translate the image to "lab" space 
        
        area_max = 0
        areaMaxContour = 0
        if not self.arm.start_pick_up:
            for i in color_range:
                if i in self.target_color:
                    detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Perform bitwise operations on the original image and the mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the contours
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
            if area_max > 2500:  # we did find the largest contour!
                self.arm.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.arm.rect))

                roi = getROI(box) # get the region of interest (the large white box )
                self.arm.get_roi = True

                img_centerx, img_centery = getCenter(self.arm.rect, roi, self.size, square_length)  # get the centroild location of the block
                world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size) # translate/transorm it to realworld coordinates
                
                
                cv2.drawContours(img, [box], -1, self.arm.range_rgb[detect_color], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.arm.range_rgb[detect_color], 1) # draw the centroid
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) # compare it to coordinates from the last time to determine if had moved 
                last_x, last_y = world_x, world_y
                self.arm.track = True
                #print(count,distance)
                
                # count the calculations 
                if self.arm.action_finish:
                    if distance < 0.3:
                        self.arm.center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            self.arm.rotation_angle = self.arm.rect[2]
                            start_count_t1 = True
                            self.arm.world_X, self.arm.world_Y = np.mean(np.array(self.arm.center_list).reshape(count, 2), axis=0)
                            count = 0
                            self.arm.center_list = []
                            self.arm.start_pick_up = True
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        self.arm.center_list = []
        return img