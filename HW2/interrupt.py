#!/usr/bin/python

# ECE 590 HW2 
# File Name: interrupt.py 
# Author: Jason Esquivel
# Date Created: 9/12/2016

# Interrupt  Real Time

import signal, time
from foo import *

def interruptHandler(signum, stack):
	global interruptCounter
	t0 = time.time()
	foo()
	t1 = time.time()
	dt = t1-t0
	interruptCounter += 1
	if(interruptCounter % 5 == 0):
		print "dt = " + str(dt)

signal.signal(signal.SIGALRM, interruptHandler)

signal.setitimer(signal.ITIMER_REAL, .2, .2)

interruptCounter = 0
while(interruptCounter < 100):
	pass

signal.setitimer(signal.ITIMER_REAL, 0)
#signal.alarm(0)


