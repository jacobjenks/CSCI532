#!/usr/bin/python
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


#Constants
gravity = 9.806
markers = ['o', 'v', '*', 'x'] 
startPos = 10

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
		
		#Kalman position and velocity estimation
		self.posXKal = self.posX
		self.posYKal = self.posY
		self.velXKal = self.velX
		self.velXKal = self.velY
		
		self.time = 0
		self.crashed = False#Whether or not vehicle has crashed yet
		
		self.logGT = SensorLog("True Position")#Ground truth position log
		self.logEst = SensorLog("Estimated")#Naive estimated position log
		self.logKal = SensorLog("Kalman")#Kalman estimated position log
		
	
	#update vehicle state by advancing time one second
	#Accel refers to intentional acceleration
	def update(self, accelX, accelY):
		#Don't update after a crash
		if self.crashed:
			return
			
		#Update true position
		#True velocity is intentional accel + random force accel
		self.velX += accelX + random.gauss(0, self.randomForce)
		self.velY += accelY + random.gauss(0, self.randomForce) - gravity
		self.posX += self.velX
		self.posY += self.velY
		
		#update naive estimated position
		self.velXEst += accelX + random.gauss(0, self.sensorNoise)
		self.velYEst += accelY + random.gauss(0, self.sensorNoise) - gravity
		self.posXEst += self.velXEst
		self.posYEst += self.velYEst
		
		#Crash if we hit the ground
		if self.posY < 0:
			self.posY = 0
			self.velY = 0
			self.velX = 0
			self.crashed = True
			
		self.logGT.log(self.posX, self.posY)
		self.logEst.log(self.posXEst, self.posYEst)
		self.logKal.log(self.posXKal, self.posYKal)
		
		self.time += 1
		
		print "---Time %d---" % self.time
		print "True pos:  %d, %d" % (self.posX, self.posY,)
		print "Estimated: %d, %d" % (self.posXEst, self.posYEst)
		print ""

	def printLogs(self, numLogs = 1):
		lineTrue = plt.plot(self.logGT.posX, self.logGT.posY, "g", label=self.logGT.name)#, c=[x for x in range(self.time)])
		lineEst = plt.plot(self.logEst.posX, self.logEst.posY, "bo", label=self.logEst.name)#, c=[x for x in range(self.time)])
		
		plt.legend()
		plt.xlabel('X Position')
		plt.ylabel('Y Position')
		plt.ylim(ymin=0)
		plt.tight_layout()
		plt.show()
			
			
#####################
#Turbulence:
#	Low: .2g
#	Moderate: .5g
#	Severe: 1.5g
#	Very severe: > 1.5g
v = Vehicle(.1, .1*gravity)

#Accelerate for 10 seconds
for x in range(0, 10):
	v.update(1, 15)
	
while v.crashed is False:
	v.update(0, 0)

v.printLogs()
