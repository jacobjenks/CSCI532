#!/usr/bin/python
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from numpy import linalg as la 
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
		self.posZ = []
		self.posX.append(startPos)
		self.posY.append(0)
		self.posZ.append(0)
		self.name = name
		
	def log(self, x, y, z):
		self.posX.append(x)
		self.posY.append(y)
		self.posZ.append(z)
		

#Basic vehicle simulation
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
		self.posZEst = self.posZ
		self.velXEst = self.velX
		self.velYEst = self.velY
		self.velZEst = self.velZ
		
		#Kalman stuff 
		self.x = np.zeros(9)#State matrix
		self.p = np.identity(9)#State covariance matrix

		self.time = 0
		self.crashed = False#Whether or not vehicle has crashed yet
		
		self.logGT = SensorLog("True Position")#Ground truth position log
		self.logEst = SensorLog("Estimated")#Naive estimated position log
		self.logKal = SensorLog("Kalman")#Kalman estimated position log

		self.logGT.log(startPos, 0, 0)
		self.logEst.log(startPos, 0, 0)
		self.logKal.log(startPos, 0, 0)

		self.printValues()
		
	
	#update vehicle state by advancing time one second
	#Accel refers to intentional acceleration
	def update(self, ax, ay, az):
		#Don't update after a crash
		if self.crashed:
			return

		#apply random forces to acceleration
		ax += random.gauss(0, self.randomForce)	
		ay += random.gauss(0, self.randomForce) - gravity	
		az += random.gauss(0, self.randomForce)


		#Update true position
		#True velocity is intentional accel + random force accel
		self.posX += self.velX + .5*math.pow(timeUpdate, 2)*ax
		self.posY += self.velY + .5*math.pow(timeUpdate, 2)*ay
		self.posZ += self.velZ + .5*math.pow(timeUpdate, 2)*az
		self.velX += ax 
		self.velY += ay  
		self.velZ += az 

		#Add sensor noise
		ax += random.gauss(0, self.sensorNoise) 
		ay += random.gauss(0, self.sensorNoise) 
		az += random.gauss(0, self.sensorNoise) 
		
		#update naive estimated position
		self.posXEst += self.velXEst + .5*math.pow(timeUpdate, 2)*ax
		self.posYEst += self.velYEst + .5*math.pow(timeUpdate, 2)*ay
		self.posZEst += self.velZEst + .5*math.pow(timeUpdate, 2)*az
		self.velXEst += ax
		self.velYEst += ay
		self.velZEst += az

		self.kalman2(timeUpdate, ax, ay, az)

		#Crash if we hit the ground
		if self.posY < 0:
			self.crashed = True
			
		self.logGT.log(self.posX, self.posY, self.posZ)
		self.logEst.log(self.posXEst, self.posYEst, self.posZEst)
		self.logKal.log(self.x[0,0], self.x[0,1], self.x[0,2])
		
		self.time += 1
		self.printValues()	
		
	def printValues(self):
		test = ""
		print "---Time %d---" % self.time
		print "True:      %d, %d: %d, %d" % (self.posX, self.posY, self.velX, self.velY)
		#print "Estimated: %d, %d" % (self.posXEst, self.posYEst)
		#print "Kalman:    %d, %d: %d, %d" % (self.x[0,0], self.x[0,1], self.x[1,0], self.x[0,1])
		print "-------------"

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

	#ax,ay,az = accel a,y,z
	def kalman(self, delta, ax, ay, az):
		## Variables	
		A = np.matrix([[1, delta, .5*math.pow(delta, 2)], [0,1,delta],[0,0,1]])#State transition matrix
		B = np.matrix([[1, delta, .5*math.pow(delta, 2)], [0,1,delta],[0,0,1]])#Input control matrix
		#H = np.matrix([[0,0,0],[0,0,0],[0, 0, 1]])#Measurement matrix - expected measurement given predicted state
		H = np.identity(3)
		#errorX = math.pow(self.randomForce, 2) * np.matrix([[math.pow(timeUpdate, 4)/4, math.pow(timeUpdate, 3)/2],[math.pow(timeUpdate, 3)/2, math.pow(timeUpdate, 2)]])# process noise standard deviation converted into covariance matrix
		#errorZ = math.pow(self.sensorNoise, 2) 
		gamma = np.array(.5*math.pow(delta, 2))
		Q = gamma*self.randomForce*gamma.T#piecewise process noise
		R = self.sensorNoise*np.identity(3)#measurement noise
		
		## Prediction Step 
		#
		self.x = A.dot(self.x)# + B.dot(np.matrix([[0,0],[ax, ay, az]]))
		self.p = A*self.p*A.T + Q 
		
		## Update Step 
		#
		## Kalman Gain
		K = (self.p * H.T) / (H*self.p*H.T + R)
		#print "K: "
		#print K

		## Update State Estimate
		self.x = self.x + K * (np.array([[self.x[0,0],self.x[0,1],self.x[0,2]],[self.x[1,0],self.x[1,1],self.x[1,2]],[ax, ay, az]]) - H*self.x)	
		self.p = (np.identity(3) - K*H.T)*self.p
		
		#####################

	def kalman2(self, delta, ax, ay, az):
		## Variables	
		A = np.identity(9)
		A[0,3] = delta
		A[1,4] = delta
		A[2,5] = delta
		A[3,6] = delta
		A[4,7] = delta
		A[5,8] = delta
		A[0,6] = .5*math.pow(delta, 2)	
		A[1,7] = .5*math.pow(delta, 2)	
		A[2,8] = .5*math.pow(delta, 2)	

	
		H = np.zeros((3,9))
		H[0,6] = 1
		H[1,7] = 1
		H[2,8] = 1
		
		gamma = np.array(.5*math.pow(delta, 2))
		Q = gamma*self.randomForce*gamma.T#piecewise process noise
		R = self.sensorNoise*np.identity(3)#measurement noise
		
		## Prediction Step 
		#
		self.x = A.dot(self.x)# + B.dot(np.matrix([[0,0],[ax, ay, az]]))
		self.p = A*self.p*A.T + Q 
		
		## Update Step 
		#
		## Kalman Gain
		K = (self.p * H.T) / (H*self.p*H.T + R)

		## Update State Estimate
		z = np.zeros((3,9))
		z[0,6] = ax
		z[1,7] = ay
		z[2,8] = az

		self.x = self.x + K * (z - H*self.x)	
		self.p = (np.identity(3) - K*H.T)*self.p
		
		#####################
		
			
#####################
#Turbulence:
#	Low: .2g
#	Moderate: .5g
#	Severe: 1.5g
#	Very severe: > 1.5g
v = Vehicle(.5, 0*gravity)

#Accelerate for 10 seconds
for x in range(0, 10):
	v.update(1, 15, 0)
	
while v.crashed is False:
	v.update(0,0,0)
v.printLogs()
