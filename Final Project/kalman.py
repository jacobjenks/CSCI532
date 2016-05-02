#!/usr/bin/python
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from numpy.linalg as la 
import math


#Constants
gravity = 9.806
markers = ['o', 'v', '*', 'x'] 
startPos = 10
timeUpdate = 1#seconds between updates

class SensorLog:

	def __init__(self, name):
		self.posX = []
		self.posY = []
		self.posX.append(startPos)
		self.posY.append(0)
		self.name = name
		
	def log(self, x, y):
		self.posX.append(x)
		self.posY.append(y)
		

#Basic 2d vehicle simulation
class Vehicle:
	def __init__(self, sensorNoise=0, randomForce=0):
		#initialize vehicle position
		self.posX = startPos 
		self.posY = 0
		self.posZ = 0
		self.velX = 0
		self.velY = 0
		self.velZ = 0
		
		#noise paramters
		self.sensorNoise = sensorNoise
		self.randomForce = randomForce
		
		#Naive position and velocity estimation
		self.posXEst = self.posX
		self.posYEst = self.posY
		self.velXEst = self.velX
		self.velYEst = self.velY
		
		#Kalman stuff 
		self.x = np.matrix([[startPos,0],[0,0]])#state matrix
		self.p = math.pow(randomForce, 2) * np.matrix([[math.pow(timeUpdate, 4)/4, math.pow(timeUpdate, 3)/2],[math.pow(timeUpdate, 3)/2, math.pow(timeUpdate, 2)]])# Covariance matrix - should be initialized to match errorX if it gets updated 
		
		self.time = 0
		self.crashed = False#Whether or not vehicle has crashed yet
		
		self.logGT = SensorLog("True Position")#Ground truth position log
		self.logEst = SensorLog("Estimated")#Naive estimated position log
		self.logKal = SensorLog("Kalman")#Kalman estimated position log

		self.logGT.log(startPos, 0)
		self.logEst.log(startPos, 0)
		self.logKal.log(self.x[0,0], self.x[0,1])

		self.printValues()
		
	
	#update vehicle state by advancing time one second
	#Accel refers to intentional acceleration
	def update(self, accelX, accelY):
		#Don't update after a crash
		if self.crashed:
			return
		
		accelY -= gravity	


		#Update true position
		#True velocity is intentional accel + random force accel
		self.posX += self.velX + .5*math.pow(timeUpdate, 2)*accelX
		self.posY += self.velY + .5*math.pow(timeUpdate, 2)*accelY
		self.velX += accelX + random.gauss(0, self.randomForce)
		self.velY += accelY + random.gauss(0, self.randomForce)
		
		#update naive estimated position
		self.posXEst += self.velXEst + .5*math.pow(timeUpdate, 2)*accelX
		self.posYEst += self.velYEst + .5*math.pow(timeUpdate, 2)*accelY
		self.velXEst += accelX + random.gauss(0, self.sensorNoise)
		self.velYEst += accelY + random.gauss(0, self.sensorNoise)	

				
		#Crash if we hit the ground
		if self.posY < 0:
			#self.posY = 0
			#self.velY = 0
			#self.velX = 0
			self.crashed = True
			
		self.logGT.log(self.posX, self.posY)
		self.logEst.log(self.posXEst, self.posYEst)
		self.logKal.log(self.x[0,0], self.x[0,1])
		
		self.time += 1
		self.printValues()	
		
	def printValues(self):
		print "---Time %d---" % self.time
		print "True:      %d, %d: %d, %d" % (self.posX, self.posY, self.velX, self.velY)
		#print "Estimated: %d, %d" % (self.posXEst, self.posYEst)
		print "Kalman:    %d, %d: %d, %d" % (self.x[0,0], self.x[0,1], self.x[1,0], self.x[0,1])

	def printLogs(self, numLogs = 1):
		lineTrue = plt.plot(self.logGT.posX, self.logGT.posY, "g", label=self.logGT.name)#, c=[x for x in range(self.time)])
		lineEst = plt.plot(self.logEst.posX, self.logEst.posY, "bo", label=self.logEst.name)#, c=[x for x in range(self.time)])
		lineKal = plt.plot(self.logKal.posX, self.logKal.posY, "ro", label=self.logKal.name)#, c=[x for x in range(self.time)])
		
		plt.legend()
		plt.xlabel('X Position')
		plt.ylabel('Y Position')
		plt.ylim(ymin=0)
		plt.tight_layout()
		plt.show()

	#ax,ay,az = accell a,y,z
	def kalman3d(self, delta, ax, ay, az):
		## Variables	
		A = np.matrix([[1, delta, 0], [0, 1, delta], [0,0,1]])#State transition matrix
		B = np.matrix([[1, delta, .5*math.pow(delta, 2)], [0,1,delta],[0,0,1]])#Input control matrix
		H = np.matrix([0, 0, 1])#Measurement matrix - expected measurement given predicted state
		#errorX = math.pow(self.randomForce, 2) * np.matrix([[math.pow(timeUpdate, 4)/4, math.pow(timeUpdate, 3)/2],[math.pow(timeUpdate, 3)/2, math.pow(timeUpdate, 2)]])# process noise standard deviation converted into covariance matrix
		#errorZ = math.pow(self.sensorNoise, 2) 
		Q = B*self.randomForce*B#piecewise process noise
		R = np.matrix(self.sensorNoise*np.identity(3))#measurement noise

		## Prediction Step 
		#
		self.x = A.dot(self.x)# + B.dot(np.matrix([[0,0],[ax, ay, az]]))
		self.p = A.dot(self.p).dot(A) + Q 
		
		## Update Step 
		#
		## Kalman Gain
		K = self.p * H  / (H*self.p*H + R)

		## Update State Estimate
		self.x = self.x + K * (np.array([[ax, ay, az]]) - H*self.x)	
		self.p = (np.identity(3) - K*H)*P
		
		#####################
		# Finish updating for 3D
		# Don't think I should be using sensor measurements in prediction step...
		# Actually probably get rid of control matrix
		


		
			
#####################
#Turbulence:
#	Low: .2g
#	Moderate: .5g
#	Severe: 1.5g
#	Very severe: > 1.5g
v = Vehicle(0, 0*gravity)

#Accelerate for 10 seconds
for x in range(0, 10):
	v.update(1, 15)
	
while v.crashed is False:
	v.update(0, 0)

v.printLogs()
