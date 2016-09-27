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

STEP_SIZE        = 0.088  # 5 deg step size
NUM_STEPS	 = 10
FILTER_SLEEP_SEC = 0.1      # filter sleep

def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
f = ach.Channel("huboFilterChan")
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
refFilter = ha.HUBO_REF()

jointDiff = [0] * ha.HUBO_JOINT_COUNT
jointDone = [0] * ha.HUBO_JOINT_COUNT


while(True):
	# Get the current feed-forward (state) 

	[statuss, framesizes] = f.get(refFilter, wait=False, last=True)
	if(statuss == 0):
		print "*** NEW COMMANDS ***"
		jointDone = [0] * ha.HUBO_JOINT_COUNT

	while(sum(jointDone) != ha.HUBO_JOINT_COUNT):
		
		s.get(state, wait=False, last=True)
		
		for i in range(ha.HUBO_JOINT_COUNT):
			jointDiff[i] = refFilter.ref[i] - state.joint[i].pos
			if(abs(jointDiff[i] ) > STEP_SIZE):
				if(jointDiff[i] > 0):
					ref.ref[i] = state.joint[i].pos + STEP_SIZE
				else:
					ref.ref[i] = state.joint[i].pos - STEP_SIZE
			else:
				ref.ref[i] = refFilter.ref[i]
				jointDone[i] = 1		
	
		# Write to the feed-forward channel
		r.put(ref)
		simSleep(FILTER_SLEEP_SEC, s, state ) # create this function (step period of 2 seconds or slower)

		print "State time : " , state.time
		print "Joint = " , state.joint[ha.RSP].pos
		print "Joint = " , state.joint[ha.LSP].pos	

# Close the connection to the channels
r.close()
s.close()

