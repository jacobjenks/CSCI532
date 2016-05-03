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
startPos = 0
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
		

#Basic vehicle simulation
class Vehicle:
	def __init__(self, sensorNoise=0, randomForce=0):
		#initialize vehicle position
		self.posX = startPos 
		self.posY = 0 
		self.velX = 0
		self.velY = 0
			
		#noise paramters
		self.sensorNoise = sensorNoise
		self.randomForce = randomForce
		
		#Naive position and velocity estimation
		self.posXEst = self.posX
		self.posYEst = self.posY
		
		#Kalman stuff 
		self.x = np.array([[startPos,0,1,10]]).T#State matrix
		self.p = np.identity(4)*5#State covariance matrix

		self.time = 0
		self.crashed = False#Whether or not vehicle has crashed yet
		
		self.logGT = SensorLog("True Position")#Ground truth position log
		self.logEst = SensorLog("Sensor Readings")#Naive estimated position log
		self.logKal = SensorLog("Kalman")#Kalman estimated position log

		self.printValues()
		
	
	#update vehicle state by advancing time one second
	#Accel refers to intentional acceleration
	def update(self, velX, velY):
		#Don't update after a crash
		#if self.crashed:
		#	return

		velY -= gravity

		#Update true position
		#True velocity is intentional accel + random force accel
		self.posX += self.velX + random.gauss(0, self.randomForce)
		self.posY += self.velY + random.gauss(0, self.randomForce)
		self.velX = velX
		self.velY = velY

		sensorX = self.posX + random.gauss(0,self.sensorNoise)
		sensorY = self.posY + random.gauss(0,self.sensorNoise)

		#update naive estimated position
		self.posXEst = sensorX 
		self.posYEst = sensorY

		self.kalman(timeUpdate, sensorX, sensorY)

		#Crash if we hit the ground
		if self.posY < 0:
			self.crashed = True
			
		self.logGT.log(self.posX, self.posY)
		self.logEst.log(self.posXEst, self.posYEst)
		self.logKal.log(self.x[0], self.x[1])
		
		self.time += 1
		self.printValues()	
		
	def printValues(self):
		test = ""
		print "---Time %d---" % self.time
		print "True:      %d, %d" % (self.posX, self.posY)
		print "Estimated: %d, %d" % (self.posXEst, self.posYEst)
		print "Kalman:    %d, %d" % (self.x[0], self.x[1]) 
		print "-------------"

	def printLogs(self, numLogs = 1):
		lineTrue = plt.plot(self.logGT.posX, self.logGT.posY, "g", label=self.logGT.name)#, c=[x for x in range(self.time)])
		lineEst = plt.plot(self.logEst.posX, self.logEst.posY, "bo", label=self.logEst.name)#, c=[x for x in range(self.time)])
		lineKal = plt.plot(self.logKal.posX, self.logKal.posY, "r", label=self.logKal.name)#, c=[x for x in range(self.time)])
		
		plt.legend(loc=0)
		plt.xlabel('X Position')
		plt.ylabel('Y Position')
		plt.ylim(ymin=0)
		plt.tight_layout()
		plt.show()

	#ax,ay,az = accel a,y,z
	def kalman(self, delta, x, y):
		## Variables	
		A = np.matrix([[1,0,delta,0],[0,1,0,delta],[0,0,1,0],[0,0,0,1]])#State transition matrix
		H = np.array([[1,0,0,0],[0,1,0,0]]) 
		
		gamma = np.array(.5*math.pow(delta, 2))
		#piecewise process noise
		Q = np.matrix([[.25*delta**4, .5*delta**2,0,0],[.5*delta**3, delta**2, 0, 0], [0,0,.25*delta**4, .5*delta**2],[0,0,.5*delta**3, delta**2]]) * self.randomForce 
		R = self.sensorNoise*np.identity(2)#measurement noise
		
		## Prediction Step 
		#
		self.x = A * self.x# + B.dot(np.matrix([[0,0],[ax, ay, az]]))
		self.p = A*self.p*A.T + Q 
		
		## Update Step 
		#
		## Kalman Gain
		K = (np.dot(self.p,H.T)).dot(np.linalg.inv(np.dot(np.dot(H, self.p), H.T) + R))

		## Update State Estimate

		self.x = self.x + K.dot(np.array([[x, y]]).T - H.dot(self.x))
		self.p = (np.identity(4) - K.dot(H)).dot(self.p)
	
		#####################

			
			
#####################
#Turbulence:
#	Low: .2g
#	Moderate: .5g
#	Severe: 1.5g
#	Very severe: > 1.5g
v = Vehicle(10, 10)

for x in range(0, 50):
	v.update(1, 20)


#while v.crashed is False:
#	v.update(0,0)
	
v.printLogs()
