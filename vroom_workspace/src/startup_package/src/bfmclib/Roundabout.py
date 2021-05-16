import cv2
import numpy as np
import math
from time import time

class Roundabout:

    """
    This class is used for the navigation of the vehicle on Roundabout. 

    It determines the speed and the steering angle to guide the vehicle in the Roundabout according to the desired path. 
    """

    def __init__(self, start_time, code=None, start=None, end=None):
        
        if code is None:
            self.code = self.findPath(start, end)
        else:
            self.code = code

        #print("Code is ",self.code)
        self.start_turn = False

        self.roundabout_turn_time = 4
        self.start_time = start_time

        self.angle_modifier = 40

        self.turn_to_three = False

        self.roundabout_sequence = True
        
        self.exitDestroy = 0
        self.exitState = 0
        
        self.twoToTwoDestroy = 0
        self.twoToTwoState = 0

    def findPath(self, start, end):

        code = 0
        start = int(start)
        end = int(end)
        if start == 230: 
            if end == 231:
                code = 11
            elif end == 272:
                code = 12
            elif end == 343:
                code = 13
        
        elif start == 301: 
            if end == 231:
                code = 21
            elif end == 272:
                code = 22
            elif end == 343:
                code = 23
        
        elif start == 342: 
            if end == 231:
                code = 31
            elif end == 272:
                code = 32
            elif end == 343:
                code = 33

        return code

    def stopEnterSequence(self):
        self.roundabout_sequence = False

    def startTurn(self):
        self.start_turn = True

    def isTurning(self):
        return self.start_turn

    def isEnteringRoundabout(self):
        return self.roundabout_sequence

    def enteringRoundabout(self, yaw):

        stop = True
        angle = 0
        second_timer = self.start_time + 3

        if time() < self.start_time + self.roundabout_turn_time:
            period = time() - self.start_time
            angle = self.angle_modifier * math.exp(period - 5) + 3
            stop = False


        if stop == True and self.code > 31 and yaw > 45 or (stop == True and self.code == 31):
            if time() < second_timer + self.roundabout_turn_time:
                period = time() - second_timer
                angle = 50 * math.exp( period - 5) + 12
                stop = False

        return angle, stop

    def roundaboutTurns(self, yaw):
        
        angle = 0
        laneKeepingFlag = False
        destroy = False
        if self.code == 11:
            angle, laneKeepingFlag, destroy = self.oneToOne(yaw)

        elif self.code == 12:
            laneKeepingFlag = True
            destroy = True
        
        elif self.code == 13:
            angle, laneKeepingFlag, destroy = self.oneToThree(yaw)
        
        elif self.code == 21:
            laneKeepingFlag = True
            destroy = True

        elif self.code == 22:
            angle, laneKeepingFlag, destroy = self.twoToTwo(yaw)
            #print("Yaw in 22",yaw)
            
        elif self.code == 23:
            angle, laneKeepingFlag, destroy = self.twoToThree(yaw)

        elif self.code == 31:
            laneKeepingFlag = True
            destroy = True

        elif self.code == 32:
            angle, laneKeepingFlag, destroy = self.threeToTwo(yaw)

        elif self.code == 33:
            angle, laneKeepingFlag, destroy = self.threeToThree(yaw)

        return angle, laneKeepingFlag, destroy

    def oneToOne(self, yaw):

        angle = 0
        laneKeepingFlag = False
        destroy = False

        if(-165 <= yaw <= -20): 
            angle = -19

            if self.exitState == 1:
                self.exitDestroy = 1

        else:
            laneKeepingFlag = True

            if self.exitState == 0:
                self.exitState = 1
            
            if self.exitDestroy == 1:
                self.exitState = 2

        if self.exitState == 2:
            destroy = True

        return angle, laneKeepingFlag, destroy


    def oneToThree(self, yaw):
        
        angle = 0
        laneKeepingFlag = False
        destroy = False

        if(-165 <= yaw <= -50 and self.turn_to_three == False):
            angle = -19
        elif(-90 <= yaw <= 0 and self.turn_to_three == False):
            self.turn_to_three = True
        elif(self.turn_to_three == False):
            laneKeepingFlag = True
            
                
        if(self.turn_to_three and yaw >= -90):
            angle = 20
        if(self.turn_to_three and yaw < -90):
            destroy = True
            
        return angle, laneKeepingFlag, destroy

    def twoToTwo(self, yaw):
        
        angle = 0
        laneKeepingFlag = False
        destroy = False

        if 25 <= yaw <= 150:
            angle = -20
            
            if self.exitState == 1:
                self.exitDestroy = 1

        else:
            laneKeepingFlag = True
            
            if self.exitState == 0:
                self.exitState = 1
                            
            if self.exitDestroy == 1:
                self.exitState = 2
            
        if self.exitState == 2:
            destroy = True

        return angle, laneKeepingFlag, destroy

    def twoToThree(self, yaw):
        
        angle = 0 
        laneKeepingFlag = False
        destroy = False

        if(yaw >= -90):
            angle = 14

        else:
            laneKeepingFlag = True
            destroy = True

        return angle, laneKeepingFlag, destroy

    def threeToTwo(self, yaw):
        
        angle = 0
        laneKeepingFlag = False
        destroy = False

        if 30 <= yaw <= 100:
            angle = -17
        else:
            laneKeepingFlag = True

            destroy = True

        return angle, laneKeepingFlag, destroy

    def threeToThree(self, yaw):

        angle = 0
        laneKeepingFlag = False
        destroy = False
        if 30 <= yaw <= 100 or -165 <= yaw <= -100:
            angle = -17

            if self.exitState == 1:
                self.exitDestroy = 1

        else:
            laneKeepingFlag = True
            if self.exitState == 0:
                self.exitState = 1
                            
            if self.exitDestroy == 1:
                self.exitState = 2
            

        if self.exitState == 2:
            destroy = True
        
        return angle, laneKeepingFlag, destroy

        