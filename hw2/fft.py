#!/usr/bin/python
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
import random
import time

############ TEST DATA ############
testSet = []

#1
a = [0, -3]
b = [10, -1, 4]
c = [0, -30, 3, -12, 0]
testSet.append([a,b,c])

#2
a = [3, 1]
b = [2, 1]
c = [6, 5, 1]
testSet.append([a,b,c])

#3
a = [5, -9, 3]
b = [-7, 4, 2]
c = [-35, 83, -47, -6, 6]
testSet.append([a,b,c])

######## FUNCTIONS #########

# Get next power of 2
def nextPower2(x):  
    return 2**(x-1).bit_length()

# Returns whether or not number is power of 2
def isPower2(n):
	return n != 0 and ((n & (n - 1)) == 0)
	
# make array have length be a power of 2
def makePower2(a):
	n = len(a)
	#Make sure n is a power of 2
	if isPower2(n) == False:
		a.extend([0]*(nextPower2(n)-n))
	return a

# Returns complex exponential
def complexExp(u):
	return math.cos(u) +  math.sin(u)*1j

def fftPolyMult(a, b):
	# Double size
	a.extend([0]*len(a))
	b.extend([0]*len(b))
	
	# Make power of 2 and same length
	if(a > b):
		a = makePower2(a)
		b.extend([0] * (len(a) - len(b)))
	elif(b > a):
		b = makePower2(b)
		a.extend([0] * (len(b) - len(a)))
	else:
		a = makePower2(a)
		b = makePower2(b)
		
	n = len(a)
		
	pvA = recursiveFFT(a)
	pvB = recursiveFFT(b)
	pvC = [0]*len(pvA)
	
	for i in range(0, len(pvA)):
		pvC[i] = pvA[i] * pvB[i]
		
	
	V = np.zeros((n,n))
	V = V * 1j
	for i in range(0, n):
		for j in range(0, n):
			V[i][j] = complexExp(i*j*2*math.pi/n)
			
	V = np.linalg.inv(V)
	y = np.array(pvC)
	
	return np.real(np.dot(y,V))
	
def recursiveFFT(a):
	a = makePower2(a)
	n = len(a)
	nHalf = int(math.floor(n/2))
	
	if n == 1:
		return a
		
	wn = complexExp(2*math.pi/n)
	w = 1
	a0 = a[::2]#even elements
	a1 = a[1::2]#odd elements
	y0 = recursiveFFT(a0)
	y1 = recursiveFFT(a1)
	y = [0]*n
	for k in range(0, nHalf):
		y[k] = y0[k] + w*y1[k]
		y[k+nHalf] = y0[k] - w*y1[k]
		w = w * wn
	return y

def polyMultDC(a, b):
	n = len(a)
	a0 = a[1 : math.floor(n/2) - 1]
	a1 = a[math.floor(n/2) : n - 1]
	
	n = len(b)
	b0 = b[1 : math.floor(n/2) - 1]
	b1 = b[math.floor(n/2) : n - 1]
	
	y = polyMultDC()

#Default n^2 algorithm
def polyMult(a, b):

	c = [0]*(len(a)+len(b))
	
	for i in range(0, len(a)):
		for j in range(0, len(b)):
			c[i + j] += a[i] * b[j]
			
	return c
		
		
#Compare two polynomial coefficient lists, return true if they are equivalent
def equalPoly(a, b):
	if(len(a) > len(b)):
		max = len(a)
	else:
		max = len(b)
	for i in range(0, max):
		if len(a) <= i:
			if b[i] != 0:
				return False;
		elif len(b) <= i:
			if a[i] != 0:
				return False;
		else:
			if a[i] != b[i]:
				return False;
	
	return True
	
	
def testResults(testSet):
	correct = 0
	failed = 0
	for i in range(0, len(testSet)):
		result = polyMult(testSet[i][0], testSet[i][1])
		if equalPoly(result, testSet[i][2]):
			correct = correct + 1
		else:
			print("Test %i failed" % (i+1))
			print(result)
			failed = failed + 1
			
	print("%i successful tests" % correct)
	print("%i failed tests" % failed)
		

######### Testing #########
#print(recursiveFFT([1,1]))
#print(recursiveFFT(testSet[0][1]))
#print(fftPolyMult(testSet[1][0], testSet[1][1]))


#Generate random tests and compare times
defaultTime = [0]
fftTime = [0]

defaultEnabled = True
iterations = 1000

for i in range(0, iterations):
	testA = [random.random() * 10 for j in range(i)]
	testB = [random.random() * 10 for j in range(i)]
	start = time.time()
	fftPolyMult(testA, testB)
	totalFFTTime = (time.time() - start)*1000
	fftTime.append(totalFFTTime)
	print("%d: %d ms" %(i,totalFFTTime))
	
	if defaultEnabled:
		start = time.time()
		polyMult(testA, testB)
		totalTime = (time.time() - start)*1000
		defaultTime.append(totalTime)
		if totalTime > 500 and totalTime > totalFFTTime:
			defaultEnabled = False
			
	
line1 = plt.plot(defaultTime, label="Default algorithm")
line2 = plt.plot(fftTime, label="FFT algorithm")

plt.legend(loc='upper left')
plt.legend(loc='upper left')

plt.axis([0, iterations, 0, max(fftTime)])
plt.xlabel('Polynomial Degree')
plt.ylabel('Time (ms)')
plt.show()