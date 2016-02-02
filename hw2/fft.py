#!/usr/bin/python

#Default n^2 algorithm
def polyMult(a, b):

	c = [0]*(len(a)+len(b))
	
	for i in range(0, len(a)):
		for j in range(0, len(b)):
			c[i + j] += a[i] * b[j]
			
	return c
		
		
#Compare two polynomial coefficient lists, return true if they are equivalent
def testResult(a, b):
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
		

######### Testing #########
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

correct = 0
failed = 0
for i in range(0, len(testSet)):
	result = polyMult(testSet[i][0], testSet[i][1])
	if testResult(result, testSet[i][2]):
		correct = correct + 1
	else:
		print "Test %i failed" % (i+1)
		print result
		failed = failed + 1
		
print "%i successful tests" % correct
print "%i failed tests" % failed