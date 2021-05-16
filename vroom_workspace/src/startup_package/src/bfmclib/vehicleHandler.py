# Reaction to static elements code

import cv2
import numpy as np
import os
import imutils
from imutils.object_detection import non_max_suppression
from time import sleep 
import math
from bfmclib.LaneKeeping import LaneKeeping as lk

"""
This class is used for the vehicle detection and the static vehicle overtake procedure.

A HAAR Classifier has been trained for the detection procedure. 

"""

class VehicleHandler:

	def __init__(self):	

		"""
		Parameters
		----------
		ccascade : 
			The xml classifier file used for the vehicle detection 
		bbox : 
			The bounding box surounding the detected vehicle
		vehicleDetected : 
			Flag that indicates the detection of a vehicle
		laneKeepingFlag : 
			Flag that indicates the beginning of lane keeping procedure
		angle : 
			The steering angle of the vehicle
		speed : 
			The speed of the vehicle
		distance :
			The distace from the foregoing vehicle
		overtakeTime : 
			The duration of the overake maneuver
		startTime : 
			The start time of the overtake maneuver calculated as ros time 
		minSpeed : 
			The vehicle's speed during the overtake maneuver 
		minDistance : 
			The distance from the foregoing vehicle threshold to indicate the start of the overtake maneuver
		laneWidth : 
			The width of the road's lane
		safeDistance : 
			The horizontal distance traveled in each part of the maneuver (i.e. Change lane, Do lane keeping)
		dafeDuration : 
			The duration of each part of the maneuver 
		currentX : 
			The vehicle's x coordinate according to the coordinate system used for the overtake formula 
		currentY : 
			The vehicle's y coordinate according to the coordinates system used for the overtake formula
		targetX : 
			The calculated x from the overtake formula
		targetY : 
			The calculated y from the overtake formula
		"""

		self.ccascade = cv2.CascadeClassifier('src/startup_package/src/bfmclib/cars.xml')
		self.bbox = (0,0,0,0)
		self.vehicleDetected = False
		self.laneKeepingFlag = False
		self.angle = 0
		self.speed = 0.4
		self.distance = 0
		self.overtakeTime = 0
		self.startTime = 0
		self.minSpeed = 0.5
		self.minDistance = 330
		self.laneWidth = 3.5
		self.safeDistance = 78.67
		self.safeDuration = 2.26
		self.currentX = 0
		self.currentY = 0
		self.targetX = 0
		self.targetY = 0

	def detect_vehicle(self, image):
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cars = self.ccascade.detectMultiScale(gray, 1.1, 5)

		for (x,y,w,h) in cars:
			detected_area =  (x+w)*(y+h)
			if x > 200 and y>=44 and  detected_area > 70000:
				self.vehicleDetected = True
				self.bbox = (x,y,w,h)

		cars = np.array([[x, y, x + w, y + h] for (x, y, w, h) in cars])
		pick = non_max_suppression(cars, probs=None, overlapThresh=0.2)

		for(xA, yA, xB, yB) in pick:
			if (xA+xB) * (yA + yB) > 70000:
				cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

		return self.vehicleDetected, image

	def distance_to_vehicle(self, frame):
		img_dims = frame[:,:,0].shape

		roi_center = (self.bbox[0] + self.bbox[2]/2, self.bbox[1] + self.bbox[3]/2) 
		vehicle_nose = (img_dims[1]/2, img_dims[0])

		cv2.line(frame, vehicle_nose, roi_center, (255,0,0), 3)

		vehicle_nose_list = [img_dims[1] / 2, img_dims[0]]
		roi_center_list = [self.bbox[0] + self.bbox[2]/2, self.bbox[1] + self.bbox[3]/2]

		self.distance = vehicle_nose_list[1] - roi_center_list[1]
		#print("Distance to vehicle = ", self.distance)

	def check_dotted_line(self, graph, source, target):
		dotted = graph[source][target]["dotted"]
		return dotted

	def make_detour(self, image, overtake_flag, cur_time):
		if (self.overtakeTime < self.safeDuration):
			#print("Part 0")

			self.overtakeTime = cur_time - self.startTime
			self.angle = self.calculate_steering_angle(self.overtakeTime)

		elif self.overtakeTime > self.safeDuration and self.overtakeTime < 2*self.safeDuration:
			#print("Part 1")

			self.overtakeTime = cur_time - self.startTime
			self.laneKeepingFlag = True
			#print("Overtake timee = ", self.overtakeTime)
		
		elif self.overtakeTime >= 2*self.safeDuration and self.overtakeTime < 3*self.safeDuration : #2.2 for straight & 2 for turns
			#print("Part 2")

			self.overtakeTime = cur_time - self.startTime
			self.angle = -1 * self.calculate_steering_angle(self.overtakeTime - 2*self.safeDuration)
			self.laneKeepingFlag = False

		else:
			#print("Part 3")
			self.startTime = 0
			self.laneKeepingFlag = True
			#self.overtakeFlag = False

		return self.speed, self.angle, overtake_flag, self.laneKeepingFlag

	def calculate_x(self, cur_time):
		x = 70*self.minSpeed * cur_time + ((70*self.minSpeed * self.safeDuration - self.safeDistance))*(10*((cur_time/self.safeDuration)**3) - 15*((cur_time/self.safeDuration)**4) + 6*((cur_time/self.safeDuration)**5))
		return x

	def calculate_y(self, cur_time):
		y = self.laneWidth + self.laneWidth*(10*((cur_time/self.safeDuration)**3) - 15*((cur_time/self.safeDuration)**4) + 6*((cur_time/self.safeDuration)**5))
		return y

	def calculate_steering_angle(self, cur_time):
		self.currentX = self.targetX
		self.currentY = self.targetY

		self.targetX = self.calculate_x(cur_time)
		self.targetY = self.calculate_y(cur_time)
		
		if(self.targetX - self.currentX == 0 ):
			return self.angle

 		return -100*(self.targetY - self.currentY) / (self.targetX - self.currentX)

	def react_to_vehicle(self, dotted):				
						
		if self.distance <= self.minDistance:
			if dotted is True:
				return self.speed, True	
			else:
				return self.speed, False
		else:
			self.speed = self.minSpeed
			return self.speed, False