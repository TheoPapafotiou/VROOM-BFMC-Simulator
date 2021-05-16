import os
import rospy
import cv2
from time import sleep
import numpy as np
import math

class Parking:

    """
    This class is used for the implementation of the parking maneuver of the vehicle.
    
    Both vertical and horizontal parking maneuvers have been implemented.
    """
    
    """
    TODO: Fill the counters of the correction with horizontal line detection! 
    """
    def __init__(self):
        self.theta = 30
        self.phi = -30
        self.omega = 55
        self.kappa = 55

        self.theta_ver = -20
        self.phi_ver = 30
        self.omega_ver = 44
        self.kappa_ver = 44
        self.ro_ver = 40
        
        self.margin = 3
        self.speed = 0.0
        self.angle = 0.0
        self.park_dist = 20
        self.min_speed = 0.2
        self.counter1 = 0
        self.counter2 = 0
        self.counter3 = 0
        self.part = [False, False, False, False, False, False, False, False]
        self.correction = False
        self.correction_counter = 0
        self.cor_count_limit = 10
        self.prepared = False
        self.forward_prepare = False

    def parking_horizontal(self, yaw_init, yaw, frame, flag):
        
        if self.correction_counter == self.cor_count_limit:
            self.correction = True

        ### Check the part of the parking procedure
        if yaw == yaw_init and self.part[0] is False: #Turn right and backwards
            for i in range(0, 4):
                self.part[i] = False
            self.part[0] = True

        elif yaw >= (yaw_init + self.kappa) and self.part[0] is True: #Turn left and backwards
            for i in range(0, 4):
                self.part[i] = False
            self.part[1] = True

        elif yaw <= (yaw_init + self.margin) and yaw >= (yaw_init - self.margin) and self.part[1] is True: #Correct parking
            for i in range(0, 4):
                self.part[i] = False
            self.part[2] = True

        elif yaw <= (yaw_init + self.margin) and yaw >= (yaw_init - self.margin) and self.part[2] is True and self.correction is True: #Turn left and forward
            for i in range(0, 4):
                self.part[i] = False
            self.part[3] = True
            self.correction = False
            self.correction_counter = 0
            
        elif yaw >= (yaw_init + self.omega) and self.part[3] is True: #Turn right until 
            for i in range(0, 4):
                self.part[i] = False
            self.part[4] = True

        elif yaw <= (yaw_init + self.margin) and yaw >= (yaw_init - self.margin) and self.part[4] is True: 
            for i in range(0, 4):
                self.part[i] = False
            self.part[0] = True
            flag = False   

        ### Calculate the speed and angle
        if self.part[0] is True:
            #print("Part 1")
            self.angle = self.theta
            self.speed = self.min_speed

        elif self.part[1] is True:
            #print("Part 2")
            self.angle = self.phi
            self.speed = -self.min_speed

        elif self.part[2] is True:
            #print("Part 3")
            self.angle = 0
            self.speed = 0
            self.correction_counter += 1
        
        elif self.part[3] is True:
            #print("Part 4")
            self.angle = self.phi
            self.speed = self.min_speed
        
        elif self.part[4] is True:
            #print("Part 5")
            self.angle = self.theta
            self.speed = self.min_speed

        return self.speed, self.angle, flag

    def parking_vertical(self, yaw_init, yaw, frame, flag):
        
        if self.counter1 == 3*45:
            self.correction = True

        if self.counter2 == 140:
            self.forward_prepare = True

        if self.counter3 == 3*25:
            self.prepared = True
            

        ### Check the part of the parking procedure
        if yaw <= (yaw_init + 1) and yaw >= (yaw_init - 1) and self.part[0] is False: #Turn left and forward
            for i in range(0, 7):
                self.part[i] = False
            self.part[0] = True

        elif yaw >= (yaw_init + self.kappa_ver) and self.part[0] is True: #Turn right and backwards
            for i in range(0, 7):
                self.part[i] = False
            self.part[1] = True

        elif yaw <= (yaw_init + 90 + self.margin) and yaw >= (yaw_init + 90 - self.margin) and self.part[1] is True: #Correct parking
            for i in range(0, 7):
                self.part[i] = False
            self.part[2] = True

        elif yaw <= (yaw_init + 90 + self.margin) and yaw >= (yaw_init + 90 - self.margin) and self.part[2] is True and self.correction is True: #Wait a little
            for i in range(0, 7):
                self.part[i] = False
            self.part[3] = True
            self.correction = False
            self.counter1 = 0

        elif yaw <= (yaw_init + 90 + self.margin) and yaw >= (yaw_init + 90 - self.margin) and self.part[3] is True and self.forward_prepare is True: #Forward, being prepared for going out 
            for i in range(0, 7):
                self.part[i] = False
            self.part[4] = True
            self.forward_prepare = False
            self.counter2 = 0

        elif yaw <= (yaw_init + 90 + self.margin) and yaw >= (yaw_init + 90 - self.margin) and self.part[4] is True and self.prepared is True: #Turn right and forward
            for i in range(0, 7):
                self.part[i] = False
            self.part[5] = True
            self.prepared = False
            self.counter3 = 0
            
        elif yaw <= (yaw_init + self.omega_ver) and self.part[5] is True: #Turn more right until 
            for i in range(0, 7):
                self.part[i] = False
            self.part[6] = True

        elif yaw <= (yaw_init + self.margin) and yaw >= (yaw_init - self.margin) and self.part[6] is True: 
            for i in range(0, 7):
                self.part[i] = False
            self.part[0] = True
            flag = False   

        ### Calculate the speed and angle
        if self.part[0] is True:
            #print("Part 1")
            self.angle = self.theta_ver
            self.speed = self.min_speed

        elif self.part[1] is True:
            #print("Part 2")
            self.angle = self.phi_ver
            self.speed = -self.min_speed

        elif self.part[2] is True:
            #print("Part 3")
            self.angle = 0
            self.speed = -self.min_speed
            self.counter1 += 1

        elif self.part[3] is True:
            #print("Part 4")
            self.angle = 0
            self.speed = 0
            self.counter2 += 1

        elif self.part[4] is True:
            #print("Part 5")
            self.angle = 0
            self.speed = self.min_speed
            self.counter3 += 1
        
        elif self.part[4] is True:
            #print("Part 6")
            self.angle = self.phi_ver
            self.speed = self.min_speed
        
        elif self.part[5] is True:
            #print("Part 7")
            self.angle = self.ro_ver
            self.speed = self.min_speed

        return self.speed, self.angle, flag
