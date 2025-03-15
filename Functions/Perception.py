
import sys
sys.path.append('/home/pi/ArmPi/')

import cv2 
import math 
from Arm import Arm
import numpy as np
import time
import Camera

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *


class Perception():
    def __init__(self, frame, target_color='red'):
        self.frame = frame
        self.target_color = target_color
        self.size = (640, 480)
        self.start_count_t1 = True
        self.range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
        }
        self.color_list = []

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
        print("processing")
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)   
        
        return frame_gb
    
    def get_img_color_mask(self, img, detect_color):

        frame_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)  # translate the image to "lab" space 
        frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Perform bitwise operations on the original image and the mask
        opened_mask = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算

        return closed_mask
        

    
    def get_largest_area_contour(self, mask):
        area_max = 0
        areaMaxContour = 0

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the contours
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour

        return areaMaxContour, area_max

    def get_position_vec(self, rect, roi): # get position and angle for grasp

        img_centerx, img_centery = getCenter(rect, roi, self.size, square_length)  # get the centroild location of the block
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size) # translate/transorm it to realworld coordinates

        width, height = rect[1]
        rotation_angle = rect[2]
        # Define the position vector
        position_vector = ((world_x, world_y), (width, height), rotation_angle)
        # return the position vector
        return position_vector  


    def draw_bb_on_image(self, img, box, position_vector, detect_color):
        world_x = position_vector[0][0]
        world_y = position_vector[0][1]
        cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1) # draw the centroid



    def run_color(self, img, color):
        preprocessed_img = self._preprocess_img(img.copy()) # # preproccess the image to be better suited for color detection
        img_color_mask = self._get_img_color_mask(preprocessed_img, color) # # get the mask using the specific color from the pre-processed image
        areaMaxContour, area_max = self._get_largest_area_contour(img_color_mask) # get the max contour for the mask
        
        if area_max > 2500:  # we did find the largest contour!
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(self.arm.rect))
            roi = getROI(box) # get the region of interest (the large white box )
            get_roi = True
            # compute the position vector
            position_vector = self._get_position_vector(rect, roi)
            # draw the bounding box on the image
            self.draw_bb_on_image(img, box, position_vector, color)
        
        # if contour too small, specify a Null position vector
        else:
            position_vector = None
        
        return img, position_vector


    def run(self):
        position_dict = dict()
        frame = self.frame
        # check if it is a valid frame
        if frame is not None:
            # loop through each specified color
            for color in self.color_list:
                # get the labeled frame and position of the perception code for each object
                frame, position = self.run_color(frame, color)
                if(position is not None):
                    position_dict[color] = position
        # return the labelled frame and position dictionary
        return frame, position_dict


            
                

# main function to test perception class
def main():
    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame
        perception = Perception(img) 
        perception.color_list = ['red', 'grenn', 'blue']
        frame_labelled, position_dictionary = perception.run() # restart the loop if the frame was not captured
        if frame_labelled is None:  # stall a bit to not use resources
            time.sleep(0.01)
            continue

        
        cv2.imshow('Frame', frame_labelled)
        key = cv2.waitKey(1) # press [ESC] to quit)
        if key == 27:
            break

    # Destroy the CV2 window
    cv2.destroyAllWindows()


# Specify execution of main function when file is ran directly
if __name__ == '__main__':
    main()


'''
 
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) # compare it to coordinates from the last time to determine if had moved 
                last_x, last_y = world_x, world_y
              
                
                # count the calculations 
                if self.arm.action_finish:
                    if distance < 0.3:
                        self.arm.center_list.extend((world_x, world_y))
                        count += 1
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            self.arm.rotation_angle = self.arm.rect[2]
                            self.start_count_t1 = True
                            self.arm.world_X, self.arm.world_Y = np.mean(np.array(self.arm.center_list).reshape(count, 2), axis=0)
                            count = 0
                            self.arm.center_list = []
                            self.arm.start_pick_up = True
                    else:
                        t1 = time.time()
                        self.start_count_t1 = True
                        count = 0
                        self.arm.center_list = []

        return img
        
        '''