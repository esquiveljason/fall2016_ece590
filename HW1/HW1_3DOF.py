#!/usr/bin/python
import math

# ECE 590 HW1 Jason Esquivel
# 
# function will return end-effector positions in X,Y,Z position
# when give angle for 3 degrees of freedom mechanism
# also length of the first and second arm in cm
#
# function inputs in degrees: deg1, deg2, deg3, l1, l2
# output in cm : X, Y, Z 

def threeDOF( deg1, deg2, deg3, l1, l2):
	deg2rad = math.pi/180

	X =(l1*math.cos(deg1 * deg2rad) + l2*math.cos((deg1+deg2) * deg2rad)) * math.cos(deg3 * deg2rad)
	Y =(l1*math.cos(deg1 * deg2rad) + l2*math.cos((deg1+deg2) * deg2rad)) * math.sin(deg3 * deg2rad)
	Z =(l1*math.sin(deg1 * deg2rad) + l2*math.sin((deg1+deg2) * deg2rad))
	
	return (X,Y,Z)


print threeDOF(30,10,270,4,4)
