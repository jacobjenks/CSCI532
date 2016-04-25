#!/usr/bin/python
import random


class SensorLog:
	def __init__(self):
		self.posX = []
		self.posY = []
		
	def log(self, x, y):
		self.posX.append(x)
		self.posY.append(y)

#Basic 2d vehicle simulation
class Vehicle:
	def __init__(self, sensorNoise=0, randomForce=0):
		#initialize vehicle position
		self.posX = 10
		self.posY = 0
		self.velX = 0
		self.velY = 0
		
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
		
		self.logGT = SensorLog()#Ground truth position log
		self.logEst = SensorLog()#Naive estimated position log
		self.logKal = SensorLog()#Kalman estimated position log
		
	
	#update vehicle state by advancing time one second
	#Accel refers to intentional acceleration
	def update(self, accelX, accelY):
		#Don't update after a crash
		if self.crashed:
			return
			
		#Update true position
		#True velocity is intentional accel + random force accel
		self.velX += accelX + random.gauss(0, self.randomForce)
		self.velY += accelY + random.gauss(0, self.randomForce) - 9.806#gravity
		self.posX += self.velX
		self.posY += self.velY
		
		#update naive estimated position
		self.velXEst += accelX + random.gauss(0, self.sensorNoise)
		self.velYEst += accelY + random.gauss(0, self.sensorNoise) - 9.806
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
		self.logEst.log(self.posXKal, self.posYKal)
		
		self.time += 1
		
		print "---Time %d---" % self.time
		print "True pos:  %d, %d" % (self.posX, self.posY,)
		print "Estimated: %d, %d" % (self.posXEst, self.posYEst)
		print ""
			
#####################
#Turbulence:
#	Low: .2g
#	Moderate: .5g
#	Severe: 1.5g
#	Very severe: > 1.5g
v = Vehicle(.1, 0)

#Accelerate for 10 seconds
for x in range(0, 10):
	v.update(1, 15)
	
while v.crashed is False:
	v.update(0, 0)