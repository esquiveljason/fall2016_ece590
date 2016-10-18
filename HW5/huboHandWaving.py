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
LENGTH_WRIST_TO_FINGER   = 0.0 #placeholder

#IK Constatns
DELTA_THETA = 0.01
ERROR       = 5 # PLACEHOLDER - ERROR FROM GOAL
ENDEFF_STEP = 2 # PLACEHOLDER - STEP_SIZE

#GOALS
GOAL_X_OFFSET = 10.09298
#TOPLEFT
GOAL_TOP_LEFT  = np.array([[361.73-GOAL_X_OFFSET], [34.5], [60.0]])
GOAL_BOT_LEFT  = np.array([[361.73-GOAL_X_OFFSET], [34.5], [-60.0]])
GOAL_TOP_RIGHT = np.array([[361.73-GOAL_X_OFFSET], [154.5], [60.0]])
GOAL_BOT_RIGHT = np.array([[361.73-GOAL_X_OFFSET], [154.5], [-60.0]])


def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return


# Rotation matrices
def rX(theta):
	R = np.identity(4)
	R[1,1] = np.cos(theta)
	R[1,2] = np.sin(theta) * -1.0
	R[2,1] = np.sin(theta)
	R[2,2] = np.cos(theta)
	
	return R

def rY(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,2] = np.sin(theta)
	R[2,0] = np.sin(theta) * -1.0
	R[2,2] = np.cos(theta)

	return R

def rZ(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,1] = np.sin(theta) * -1.0
	R[1,0] = np.sin(theta)
	R[1,1] = np.cos(theta)

	return R


def getFK(thetas): #theta0, theta1, theta2, ....
	
	#shoulder Pitch
	T1 = np.identity(4)
	T1[1,3] = LENGTH_BASE_TO_SHOULDER
	Q1 = np.dot(rY(thetas[0,0]), T1)
	
	#shoulder Roll
	T2 = np.identity(4)
	#T2[1,3] = LENGTH_BASE_TO_SHOULDER
	Q2 = np.dot(rX(thetas[1,0]), T2)
	
	#shoulder Yaw
	T3 = np.identity(4)
	#T3[1,3] = LENGTH_BASE_TO_SHOULDER
	Q3 = np.dot(rZ(thetas[2,0]), T3)
	
	#elbow Pitch
	T4 = np.identity(4)
	T4[2,3] = LENGTH_SHOULDER_TO_ELBOW * -1.0
	Q4 = np.dot(rY(thetas[3,0]), T4)

	#wrist Yaw
	T5 = np.identity(4)
	T5[2,3] = (LENGTH_ELBOW_TO_WRIST + LENGTH_WRIST_TO_FINGER) * -1.0
	Q5 = np.dot(rZ(thetas[4,0]), T5)

	#wrist roll
	T6 = np.identity(4)
	Q6 = np.dot(rX(thetas[5,0]), T6)
	
	#not using pitch
	
	Qend = np.dot(Q1, Q2)
	Qend = np.dot(Qend, Q3)
	Qend = np.dot(Qend, Q4)
	Qend = np.dot(Qend, Q5)
	Qend = np.dot(Qend, Q6)
	
	endEff = np.array([[round(Qend[0,3],3)], [round(Qend[1,3],3)], [round(Qend[2,3],3)]])
	
	return endEff

def getJacobian(thetas, deltaTheta):
	Jac = np.zeros((3,6))
	for i in range((np.shape(Jac))[0]): #3
		for j in range((np.shape(Jac))[1]): #6
			#print "********* ", i,j
			newThetas = np.copy(thetas)
			newThetas[j] = thetas[j] + deltaTheta		
			newEndEff = getFK(newThetas)
			#print newEndEff
			Jac[i,j] = (newEndEff[i,0] )/ deltaTheta
	return Jac

def getDistance(endEff, goal):
	m = math.sqrt(math.pow(endEff[0] - goal[0],2) + math.pow(endEff[1] - goal[1],2) + math.pow(endEff[2] - goal[2],2))
	return m

def getNext(endEff, goal, eStep, dist):
	dx = (goal[0] - endEff[0]) * eStep / dist
	dy = (goal[1] - endEff[1]) * eStep / dist
	dz = (goal[2] - endEff[2]) * eStep / dist
	
	deltaEndEff = np.array([[round(dx,3)], [round(dy,3)], [round(dz,3)]])
	
	return deltaEndEff
	
def setArmThetas(thetas, ref, r):

	ref.ref[ha.LSP] = thetas[0]
	ref.ref[ha.LSR] = thetas[1]
	ref.ref[ha.LSY] = thetas[2]
	ref.ref[ha.LEB] = thetas[3]
	ref.ref[ha.LWY] = thetas[4]
	ref.ref[ha.LWR] = thetas[5]

	#ref.ref[ha.RSP] = thetas[0]
	#ref.ref[ha.RSR] = thetas[1]
	#ref.ref[ha.RSY] = thetas[2]
	#ref.ref[ha.REB] = thetas[3]
	#ref.ref[ha.RWY] = thetas[4]
	#ref.ref[ha.RWR] = thetas[5]

	r.put(ref)

def moveArm(thetaInit, goal, deltaTheta, eStep, error, ref, r):
	print "Starting ...."
	endEff = getFK(thetaInit)
	thetas = np.copy(thetaInit)
	dist = getDistance(endEff, goal)
	orig_dist = dist
	print "Goal    Position : ", goal.transpose()
	print "Current Theta    : ", thetas.transpose()
	print "Current Position : ", endEff.transpose()
	print "Error Allowed : ", error, "Distance : ", dist
	counter = 0
	while(dist > error):
		print "\n\nGoal    Position : ", goal.transpose()
		Jac = getJacobian(thetas, deltaTheta)
		invJac = np.linalg.pinv(Jac)

		deltaEndEff = getNext(endEff, goal, eStep, orig_dist)
		print "Increm  Position : ", deltaEndEff.transpose()

		changeTheta = np.dot(invJac, deltaEndEff)
		print "Change Theta     : ", changeTheta.transpose()

		thetas = np.add(thetas, changeTheta)
		print "New Thetas       : ", thetas.transpose()

		endEff = getFK(thetas)
		print "New Position : ", endEff.transpose()	
		
		dist = getDistance(endEff, goal)		
		print "Error Allowed : ", error, "Distance : ", dist
		counter = counter + 1
	setArmThetas(thetas, ref, r)
	print "Counter : ", counter
	return thetas


	

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)
	
thetaInit = np.zeros((6,1))
goal = np.array([[361.73-10.09298], [154.5], [60.0]])


#move to TOP LEFT
newThetas = moveArm(thetaInit, GOAL_TOP_LEFT, DELTA_THETA, ENDEFF_STEP, ERROR, ref, r)
print newThetas
time.sleep(5.0)

#move to BOTTOM LEFT
newThetas = moveArm(newThetas, GOAL_BOT_LEFT, DELTA_THETA, ENDEFF_STEP, ERROR, ref, r)
print newThetas
time.sleep(5.0)

#move to BOTTOM RIGHT
newThetas = moveArm(newThetas, GOAL_BOT_RIGHT, DELTA_THETA, ENDEFF_STEP, ERROR, ref, r)
print newThetas
time.sleep(5.0)

#move to TOP RIGHT
newThetas = moveArm(newThetas, GOAL_TOP_RIGHT, DELTA_THETA, ENDEFF_STEP, ERROR, ref, r)
print newThetas
time.sleep(5.0)

#move to TOP RIGHT
newThetas = moveArm(newThetas, GOAL_TOP_LEFT, DELTA_THETA, ENDEFF_STEP, ERROR, ref, r)
print newThetas







# Close the connection to the channels
r.close()
s.close()
	

