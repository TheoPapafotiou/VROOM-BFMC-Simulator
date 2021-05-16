#!/usr/bin/python

"""
The main FSM of the system. Based on the software architecture of the team and the current conditions,
it sets the suitable behaviours of the car.

It should be used in parallel with the sign_detection_sim.py file, so the sign detection can be initiated.
"""

import cv2
import numpy as np
from enum import Enum
import imutils
import os
import rospy
import matplotlib.pyplot as plt
from time import sleep
import base64
import math

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
from bfmclib.objectDetection import ObjectDetection
from bfmclib.redisComm import RedisComm
from bfmclib.Line import Line
from bfmclib.Mask import Mask
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.LaneKeepingReloaded import LaneKeepingReloaded 
from bfmclib.PathPlanning import PathPlanning as pp
from imutils.object_detection import non_max_suppression
from bfmclib.pedestrianDetection import PedestrianDetection
from bfmclib.MainFunctions import MainFunctions
from bfmclib.vehicleHandler import VehicleHandler
from std_msgs.msg import String


import sys
import networkx as nx

rospy.init_node('main_node', anonymous=True)

pub = rospy.Publisher('frame_transfer', String, queue_size=10)

mf = MainFunctions()

car = Controller()

cam = CameraHandler()

sem = TrafficLight()

bno = BNO055()

gps = Gps()

VehicleHandler = VehicleHandler()

######### PARAMETERS #########

### Frames params ###
imgWidth = 640
imgHeight = 480
frame = np.zeros((imgWidth, imgHeight))
sign_frame = np.zeros((imgWidth, imgHeight))
frame_with_text = np.zeros((imgWidth, imgHeight))
img_steady = np.zeros((imgWidth, imgHeight))
img_behaviour = np.zeros((imgWidth, imgHeight))
masked_img = frame
finishline_detected = False
countFrames = 0
lane_lines = []
polygon_array = np.array([[0,460], [640,460], [546,260], [78, 260]])

### Plot points ###
x_points = []
y_points = []
yaw_points = []

### Lane Keeping params ###
lane_keeping = LaneKeepingReloaded(imgWidth, imgHeight)
right_cache = []
left_cache = []
cache = [None, None]
rolling_average = []
rolling_length = 1
roling_index = 0
count = 0
prev_angle = 0

### Roundabout params ###
roundabout_sequence = False
roundabout_turn_time = 4
angle_modifier = 36
roundabout = None

### Car conditions & state ### 
speed = 0.5
angle = 0.0
speed_init = speed
behaviour = 0
dist_from_vehicle = 1000.0
yaw_init = 0.0
yaw = 0.0
pitch = 0.0
roll = 0.0
x = 0.0
y = 0.0
current_state = list("" for i in range(0,6))
previous_state = list("" for i in range(0,6))
preprevious_state = list("" for i in range(0,6))

### Semifores ###
sem_list = list("" for i in range(0,4))
sem_state = list("" for i in range(0,4))
sem_positions = [[0.75, 14.7],
                 [1.75, 10.75],
                 [3.05, 11.5],
                 [3.8, 10.45]]
pos_margin = 0.25
sem_yaw = [90, 0, 90, 180]
yaw_margin = 10

### Flags ###
parking_flag_1 = False
parking_flag_2 = False
parking_type = "False"
pedestrian_detected = False
vehicle_detected = False
overtake_flag = False
stop_ready = False
stop_flag = False
sign_flag = False
crosswalk_flag = False
priority_flag = False
got_on_ramp = False
got_off_ramp = False
parking_ready = False
parking_initiated = False
flag_inter = False
pedestrian_feed_limit = 20
vehicle_feed_limit = 20

### Intersection navigation params ###
intersection_navigation = False
found_intersection = False
steer_factor = 0
start_yaw = yaw
target_node = "0"
prev_hor_distance = 0
prev_both_lines = False
exit_navigation = True
start_time = 0
turn_type = 2

### Path planning ###
source, finish = "86", "274"
#source, finish = "86", "163"
complete_path = pp.shortest_path(source, finish)
#print(complete_path)
path = pp.remove_central_nodes(complete_path)
reached_finish = False

### Signs stuff ###
sign_detected = "Something"
counter_init_priority = -50
counter_init_stop = -50
counter_init_crosswalk = -50
counter_finito = 40
count_steps = 0
max_count_steps = 0

### Parking params ###
norm_factor = 300/480

### Ramp params ###
pitch_up_limit = 0.08
pitch_down_limit = -0.01

######### BEHAVIOURS #########
behaviours = {
    0: {
        "name": "PAUSE",
        "behaviour": mf.pause()
    },
    1: {
        "name": "START",
        "behaviour": mf.start()
    },
    2: {
        "name": "LANE_KEEPING"
    },
    3: {
        "name": "PARKING"
    },
    4: {
        "name": "ROUNDABOUT_NAVIGATION"
    },
    5: {
        "name": "INTERSECTION_NAVIGATION"
    },
    6: {
        "name": "OVERTAKE"
    },
    7: {
        "name": "REVERSE"
    }
}

######### STACK IMAGES #########
def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0,rows):
            for y in range(0,cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0,0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1],imgArray[0][0].shape[0]), None, scale, scale)

                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)

        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0,0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

### Images initialization for car's states ###
path_images = 'src/startup_package/src/bfmclib/Images/'

img_go_start = cv2.imread(path_images+'start.png')

img_stop = cv2.imread(path_images+'stop.png')

img_right = cv2.imread(path_images+'inter_right.png')

img_left = cv2.imread(path_images+'inter_left.png')

img_straight = cv2.imread(path_images+'inter_straight.png')

img_parking = cv2.imread(path_images+'parking.png')

img_faster = cv2.imread(path_images+'priority.png')

img_crosswalk = cv2.imread(path_images+'crosswalk.png')

img_incline = cv2.imread(path_images+'incline.png')

img_decline = cv2.imread(path_images+'decline.png')

img_lk = cv2.imread(path_images+'lane_keeping.png')

img_round = cv2.imread(path_images+'decline.png')

images = [img_go_start, img_stop, img_right, img_left, img_straight, img_parking, img_faster, img_crosswalk, img_incline, img_decline, img_lk]

font                   = cv2.FONT_HERSHEY_SIMPLEX
textPosition1           = (20,420)
textPosition2           = (20, 460)
fontScale              = 1
fontColor              = (0,0,0)
lineType             = 2

text1 = 'Start node: ' + str(source) 
text2 = 'Finish node: ' + str(finish)

for image in images:
    cv2.putText(image, text1, 
    textPosition1, 
    font, 
    fontScale,
    fontColor,
    lineType)

    cv2.putText(image, text2, 
    textPosition2, 
    font, 
    fontScale,
    fontColor,
    lineType)

sleep(2)

while True:

    countFrames += 1
    count_steps += 1

    preprevious_state = previous_state
    previous_state = current_state

    ###### RECEIVE FRAME ######
    frame = cam.getImage()

    ###### PROCESS FRAME ######     
    img_dims = frame[:,:,0].shape
    mask = Mask(4, img_dims)
    mask.set_polygon_points(polygon_array)
    processed_img = hf.image_processing(frame)
    masked_img = mask.apply_to_img(processed_img)

    ###### PEDESTRIAN DETECTION ######
    if countFrames%pedestrian_feed_limit == 0:
        pedestrian_detected, img_ped = mf.pedestrian_detection(frame)

    ###### VEHICLE DETECTION ######
    if countFrames%vehicle_feed_limit == 1:
        vehicle_detected, veh_frame = VehicleHandler.detect_vehicle(frame)

    ###### SIGN DETECTION ######
    if behaviour != 5:
        sign_detected, distance, img_sign = mf.sign_detection(frame, countFrames)
        if sign_detected != 'Something' and distance > 0:
            img_steady = img_sign
            sign_flag = True

    ###### "STOP" SIGN DETECTION ######
    if sign_detected == 'Stop' and stop_ready is False and stop_flag is False:#and distance <= 350:
        stop_ready = True
        count_steps = 0
        max_count_steps = 55
    if count_steps == max_count_steps and stop_ready is True:
        stop_flag = True

    ###### DETECT LINES ######
    lane_lines = hf.detect_lane(masked_img)

    ###### GPS SIGNAL ######
    x, y = mf.GPS()

    ###### "PARKING" SIGN DETECTION ######
    if sign_detected == 'ParkingSpot' and parking_ready is False and parking_initiated is False:
        
        if x < 3 and y < 2.3:
            parking_type = "vertical"
        else:
            parking_type = "horizontal"

        max_count_steps = distance*norm_factor
        count_steps = 0
        parking_ready = True

    if count_steps == max_count_steps and parking_ready is True:
        yaw_init = yaw
        parking_initiated = True
        parking_ready = False

    ###### BNO YAW, PITCH, ROLL ######
    yaw = math.degrees(bno.getYaw())
    pitch = math.degrees(bno.getPitch())
    roll = math.degrees(bno.getRoll())

    ###### SEMIFORES STATES ######
    sem_list[0] = sem.getTL0State()
    sem_list[1] = sem.getTL1State()
    sem_list[2] = sem.getTL2State()
    sem_list[3] = sem.getTL3State()
    
    for semi in range (0, 4):
        if sem_list[semi] == 0:
            sem_state[semi] = "RED"
        elif sem_list[semi] == 1:
            sem_state[semi] = "YELLOW"
        else:
            sem_state[semi] = "GREEN"


    ###### PATH PLANNING ######
    path, reached_finish = pp.update_path(path, x, y, finish)

    ###### INTERSECTION DETECTION ######
    found_intersection, start_yaw, target_node, reached_finish, start_time = mf.intersection_detection(path, reached_finish, 
                                                                                            masked_img, frame, intersection_navigation, 
                                                                                            found_intersection, start_yaw, yaw, 
                                                                                            target_node)

    ###### FINISH SIMULATION ######
    if (reached_finish is True or len(path) < 2 or len(path) == 0):
        break
    
    ###### BEHAVIOURS SETTLEMENT ######
    if pedestrian_detected is False:

        ## Lane keeping                                                             
        if (                                                
            (intersection_navigation == False)           
            ):
            behaviour = 2
            intersection_navigation = False
            if len(lane_lines) == 2:
			    found_intersection = False
            steer_factor = 0
            
        ## Intersection navigation
        if (                                                       
            found_intersection is True or intersection_navigation is True
            ):
            if(intersection_navigation == False):
                intersection_navigation = True
                behaviour = 5
                found_intersection = False
            else:
                behaviour = 5
            steer_factor += 1

        ## Roundabout Navigation
        if (
            roundabout is not None
            ):
            behaviour = 4

        ## Start after stop
        if (                                               
            (speed > 0 and (behaviour == 1 or behaviour == 2) and intersection_navigation is False and found_intersection is False ) or
            (speed > 0 and behaviour == 1 and stop_ready is True)
            ):
            behaviour = 2

        ## Behaviour on semifores
        if (
            yaw > sem_yaw[0]-yaw_margin and yaw < sem_yaw[0]+yaw_margin and    
            sem_list[0] == 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[0][0]-pos_margin and x < sem_positions[0][0]+pos_margin and 
            y > sem_positions[0][1]-pos_margin and y < sem_positions[0][1]+pos_margin
            ):
            behaviour = 1
        if (
            yaw > sem_yaw[0]-yaw_margin and yaw < sem_yaw[0]+yaw_margin and          
            sem_list[0] != 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[0][0]-pos_margin and x < sem_positions[0][0]+pos_margin and 
            y > sem_positions[0][1]-pos_margin and y < sem_positions[0][1]+pos_margin
            ):
            behaviour = 0
        if (
            yaw > sem_yaw[1]-yaw_margin and yaw < sem_yaw[1]+yaw_margin and          
            sem_list[1] == 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[1][0]-pos_margin and x < sem_positions[1][0]+pos_margin and 
            y > sem_positions[1][1]-pos_margin and y < sem_positions[1][1]+pos_margin
            ):
            behaviour = 1
        if (
            yaw > sem_yaw[1]-yaw_margin and yaw < sem_yaw[1]+yaw_margin and          
            sem_list[1] != 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[1][0]-pos_margin and x < sem_positions[1][0]+pos_margin and 
            y > sem_positions[1][1]-pos_margin and y < sem_positions[1][1]+pos_margin
            ):
            behaviour = 0
        if (
            yaw > sem_yaw[2]-yaw_margin and yaw < sem_yaw[2]+yaw_margin and          
            sem_list[2] == 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[2][0]-pos_margin and x < sem_positions[2][0]+pos_margin and 
            y > sem_positions[2][1]-pos_margin and y < sem_positions[2][1]+pos_margin
            ):
            behaviour = 1
        if (
            yaw > sem_yaw[2]-yaw_margin and yaw < sem_yaw[2]+yaw_margin and          
            sem_list[2] != 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[2][0]-pos_margin and x < sem_positions[2][0]+pos_margin and 
            y > sem_positions[2][1]-pos_margin and y < sem_positions[2][1]+pos_margin
            ):
            behaviour = 0
        if (
            yaw > sem_yaw[3]-yaw_margin and yaw < -sem_yaw[3]+yaw_margin and          
            sem_list[3] == 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[3][0]-pos_margin and x < sem_positions[3][0]+pos_margin and 
            y > sem_positions[3][1]-pos_margin and y < sem_positions[3][1]+pos_margin
            ):
            behaviour = 1
        if (
            yaw > sem_yaw[3]-yaw_margin and yaw < -sem_yaw[3]+yaw_margin and          
            sem_list[3] != 2 and #sign_detected == 'TrafficLights' and
            x > sem_positions[3][0]-pos_margin and x < sem_positions[3][0]+pos_margin and 
            y > sem_positions[3][1]-pos_margin and y < sem_positions[3][1]+pos_margin
            ):
            behaviour = 0

        ## Start after "Stop" sign
        if (
            (speed == 0 and previous_state[3] is True) or
            (countFrames >= counter_init_stop+counter_finito and stop_flag is True and counter_init_stop > 0)
            ):
            behaviour = 1
            stop_flag = False
        
        ## Stop for TBC time when "Stop" sign is here
        if (
            stop_flag is True and countFrames < counter_init_stop + counter_finito
            ):
            behaviour = 0
        
        ## Parking procedure
        if (
            parking_initiated is True
            ):
            behaviour = 3

        if (
            (vehicle_detected is True and behaviour == 2 and overtake_flag is False) or
            (overtake_flag is True)
            ):
            behaviour = 6

    else:
        behaviour = 0


    ###### BEHAVIOURS TRANSLATION TO SPEED AND SPEED ######

    lane_keeping_angle, both_lanes = lane_keeping.lane_keeping_pipeline(frame)

    if behaviour == 3:
        if parking_type == 'vertical':
            speed, angle, parking_initiated = mf.parking_vertical(yaw_init, yaw, frame, parking_initiated)
        else:
            speed, angle, parking_initiated = mf.parking_horizontal(yaw_init, yaw, frame, parking_initiated)
        img_behaviour = img_parking
    
    elif behaviour == 2:
        speed, angle = mf.lane_keeping(speed, lane_keeping_angle)
        img_behaviour = img_lk

    elif behaviour == 4:
        speed, angle, roundabout = mf.roundabout_navigation(roundabout, lane_keeping_angle, yaw, speed)
        img_behaviour = img_rda

    elif behaviour == 6:
        speed, angle, overtake_flag = mf.overtake(complete_path, veh_frame, vehicle_detected,
                                                    overtake_flag, yaw, lane_keeping_angle)

    elif behaviour == 5:
        speed, angle, intersection_navigation = mf.inter_navigation(path, x, y, complete_path,
                                                                    intersection_navigation, target_node, 
                                                                    start_yaw, yaw, steer_factor, speed, start_time)
        # if turn_type == 0:
        #     img_behaviour = img_straight
        # elif turn_type == 1:
        #     img_behaviour = img_right
        # elif turn_type == -1:
        #     img_behaviour = img_left
        # else:
        #     img_behaviour = img_lk
        
    elif behaviour == 0:
        speed, angle = behaviours[behaviour]['behaviour']
        img_behaviour = img_stop    
    elif behaviour == 1:
        speed, angle = behaviours[behaviour]['behaviour']
        img_behaviour = img_go_start
    else:
        #print("Error, no behaviour found")
        cv2.destroyAllWindows()
        break 

    ###### SET IMAGES AND PLAY WITH SPEED ######

    # Priority road
    priority_flag = False
    if sign_detected == 'PriorityRoad':
        priority_flag = True
        img_behaviour = img_faster
        counter_init_priority = countFrames
    if countFrames <= counter_init_priority + counter_finito:
        priority_flag = True
        img_behaviour = img_faster
    if priority_flag is True:
        speed = speed_init*1.5

    # Crosswalk
    crosswalk_flag = False
    if sign_detected == 'Crosswalk':
        crosswalk_flag = True
        img_behaviour = img_crosswalk
        counter_init_crosswalk = countFrames
    if countFrames <= counter_init_crosswalk + counter_finito:
        crosswalk_flag = True
        img_behaviour = img_crosswalk
    if crosswalk_flag is True:
        speed = speed_init/2

    # Stop
    if stop_flag is True and counter_init_stop < 0:
        img_behaviour = img_stop
        counter_init_stop = countFrames
    
    ###### RAMP DETECTION ######
    if pitch > pitch_up_limit:
        got_off_ramp = True
        img_behaviour = img_decline
    elif pitch < pitch_down_limit:
        got_on_ramp = True
        img_behaviour = img_incline


    current_state = [speed, angle, behaviour, pedestrian_detected]

    # ###### PRINT FRAME ###### 
    # #Put text
    # font                   = cv2.FONT_HERSHEY_SIMPLEX
    # textPosition           = (10,450)
    # fontScale              = 1
    # fontColor              = (255,0,255)
    # lineType               = 2

    # # Stack images
    # if sign_flag is True:
    #     #imgStack = stackImages(0.8, ([frame_with_text, img_sign, img_behaviour]))
    #     imgStack = stackImages(0.8, ([img_steady, frame, img_behaviour]))
    #     cv2.imshow("Qualifications Rounds - Frames", imgStack)
    
    # elif sign_detected is not None:
    #     imgStack = stackImages(0.8, ([img_sign, frame, img_behaviour]))
    #     cv2.imshow("Qualifications Rounds - Frames", imgStack)

    # else:
    #     if frame is not None and img_behaviour is not None:
    #         #imgStack = stackImages(0.8, ([frame_with_text, frame, img_behaviour]))
    #         imgStack = stackImages(0.8, ([frame, frame, img_behaviour]))
    #         cv2.imshow("Qualifications Rounds - Frames", imgStack)
 

    ###### COMMAND ###### 
    car.drive(speed, angle)

    key = cv2.waitKey(1)
    if key == ord('q'):
        cv2.destroyAllWindows()
        break  
    
    ###### CLOSE SIMULATION ######
    if behaviour == 55:
        #print('See you next time!')
        sleep(2)
        cv2.destroyAllWindows()
        print('Car stopped. \n END')
        car.stop(0.0)
        break

print("Car stopped. \n END")
car.stop(0.0)
