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
import numpy as np

#Length of arm components
LENGTH_BASE_TO_SHOULDER  = 214.5 - 120.0
LENGTH_SHOULDER_TO_ELBOW = 179.14
LENGTH_ELBOW_TO_WRIST    = 181.59
LENGTH_WRIST_TO_FINGER   = 0.0

def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return


# Rotation matrices
def rX(theta)
	R = np.identity(4)
	R[1,1] = np.cos(theta)
	R[1,2] = np.sin(theta) * -1.0
	R[2,1] = np.sin(theta)
	R[2,2] = np.cos(theta)
	
	return R

def rY(theta)
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,2] = np.sin(theta)
	R[2,0] = np.sin(theta) * -1.0
	R[2,2] = np.cos(theta)

	return R

def rZ(theta)
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,1] = np.sin(theta) * -1.0
	R[1,0] = np.sin(theta)
	R[1,1] = np.cos(theta)

	return R


def getFK(thetas) #theta0, theta1, theta2, ....
	
	#shoulder Pitch
	T1 = np.identity(4)
	T1[1,3] = LENGTH_BASE_TO_SHOULDER
	Q1 = np.dot(rY(thetas[0]), T1)
	
	#shoulder Roll
	T2 = np.identity(4)
	T2 = T1
	Q2 = np.dot(rX(thetas[1]), T2)
	
	#shoulder Yaw
	T3 = np.identity(4)
	T3 = T1
	Q3 = np.dot(rZ(thetas[2]), T3)
	
	#elbow Pitch
	T4 = np.identity(4)
	T4[2,3] = LENGTH_SHOULDER_TO_ELBOW * -1.0
	Q4 = np.dot(rY(thetas[3]), T4)

	#wrist Pitch
	T5 = np.identity(4)
	T5[2,3] = (LENGTH_ELBOW_TO_WRIST + LENGTH_WRIST_FINGER) * -1.0
	Q5 = np.dot(ry(thetas[4], T5)

	Qend = np.dot(Q1, Q2)
	Qend = np.dot(Qend, Q3)
	Qend = np.dot(Qend, Q4)
	Qend = np.dot(Qend, Q5)
	
	e = np.array([round(Qend[0,3],3), round(Qend[1,3],3), round(Qend[2,3],3)])
	return e


# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
#r = ach.Channel("huboFilterChan")
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)
	
ref.ref[ha.LSP] = -1.3
ref.ref[ha.RSP] = -1.3

r.put(ref)

# Close the connection to the channels
r.close()
s.close()
	

