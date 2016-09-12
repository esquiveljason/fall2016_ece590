#!/usr/bin/python

# ECE 590 HW2 
# File Name: blocking.py
# Author: Jason Esquivel
# Date Created: 9/11/2016

# Blocking Real Time

import time
from foo import *


def blockingLoop():
	loopTime = .2 # 5Hz
	
	#End loop Counter
	loopCounter = 0;
	loopFinished = 100
 
	while loopCounter < loopFinished:
		t0 = time.time()

		foo() 

		t1 = time.time()
		dt = t1-t0
		
		#blocking loop
		while (0 < (loopTime - dt)):
			dt = time.time()-t0 
		
		loopCounter += 1
		if(loopCounter % 5 == 0):
			print "dt = " + str(dt)


blockingLoop()


