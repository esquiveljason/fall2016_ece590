#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

import math

NUM_SQUATS = 4

SQUAT_DOWN = 0
SQUAT_UP   = 1
SQUAT_MIDDLE = 2

LOWER_LEG_D1 = 300.38
UPPER_LEG_D2 = 300.03

SQUAT_DOWN_LENGTH_FROM_GROUND = 500.0
SQUAT_UP_LENGTH_FROM_GROUND = 800.0
SQUAT_MIDDLE_LENGTH_FROM_GROUND = 650.0

ANKLE_LENGTH = 94.97
HIP_TO_WAIST_LENGTH = 289.47 - 107.0

SQUAT_DOWN_Y = SQUAT_DOWN_LENGTH_FROM_GROUND - ANKLE_LENGTH - HIP_TO_WAIST_LENGTH
SQUAT_DOWN_X = 0.0

SQUAT_UP_Y   = SQUAT_UP_LENGTH_FROM_GROUND - ANKLE_LENGTH - HIP_TO_WAIST_LENGTH
SQUAT_UP_X   = 0.0


SQUAT_MIDDLE_Y = SQUAT_MIDDLE_LENGTH_FROM_GROUND - ANKLE_LENGTH - HIP_TO_WAIST_LENGTH
SQUAT_MIDDLE_X = 0.0

def inverseKinematics(x,y,d1,d2):
	theta2 = math.acos((x*x+y*y-d1*d1-d2*d2)/(2.0*d1*d2))
	theta1 = math.atan2(y*(d1+d2*math.cos(theta2)) - x*d2*math.sin(theta2) ,  x*(d1+d2*math.cos(theta2)) + y*d2*math.sin(theta2) )

	theta1 = -1.0*(math.pi/2.0 - theta1)

	print "Theta1 (RAP,LAP,RHP,LHP) = ", theta1
	print "Theta2 (RKN,LKN)         = ", theta2

	return (theta1, theta2)

def squat(ref, r, up_down):
	
	if(up_down == SQUAT_DOWN):
		x = SQUAT_DOWN_X #0
		y = SQUAT_DOWN_Y #222.5
	elif(up_down == SQUAT_UP):
		x = SQUAT_UP_X #0
		y = SQUAT_UP_Y #522.5
	elif(up_down == SQUAT_MIDDLE):
		x = SQUAT_MIDDLE_X
		y = SQUAT_MIDDLE_Y
	

	theta1, theta2 = inverseKinematics(x, y, LOWER_LEG_D1, UPPER_LEG_D2)
	

	ref.ref[ha.RKN] = theta2
	ref.ref[ha.LKN] = theta2

	ref.ref[ha.RHP] = theta1
	ref.ref[ha.LHP] = theta1

	
	ref.ref[ha.RAP] = theta1
	ref.ref[ha.LAP] = theta1 

	# Write to the feed-forward channel
	r.put(ref)
	
	return


def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return




# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel("huboFilterChan")
#r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

#while(True):
	# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)
	

for i in range(NUM_SQUATS):
	

	print "\nSquat  waist to 0.8(m)"
	squat(ref,r, SQUAT_UP)
	simSleep(.5, s, state)

	squat(ref,r, SQUAT_MIDDLE)
	simSleep(.5, s, state)
	
	print "\nSquat waist to 0.5(m)"
	squat(ref, r, SQUAT_DOWN)
	simSleep(.5, s, state)

	squat(ref, r, SQUAT_MIDDLE)
	simSleep(.5, s, state)
	

print "\nSquat waist to 0.8(m)"
squat(ref, r, SQUAT_UP)


# Close the connection to the channels
r.close()
s.close()





	

