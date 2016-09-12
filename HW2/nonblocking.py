#!/usr/bin/python

# ECE 590 HW2 
# File Name: nonblocking.py 
# Author: Jason Esquivel
# Date Created: 9/12/2016

# NonBlocking Real Time

import time
from foo import *


def nonBlockingLoop():
	loopTime = .2 # 5Hz
	
	#End loop Counter
	loopCounter = 0;
	loopFinished = 100
 
	while loopCounter < loopFinished:
		t0 = time.time()

		foo() 

		t1 = time.time()
		dt = t1-t0
		
		#print dt every sec		
		loopCounter += 1
		if(loopCounter % 5 == 0):
			print "dt = " + str(dt)

		#NonBlocking Call
		time.sleep(loopTime-dt)

nonBlockingLoop()


