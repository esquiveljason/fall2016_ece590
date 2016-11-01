#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
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
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

def reSize(img, displayImage):

    resize_img = np.zeros((ny/2,nx/2,3), np.uint8)
    for i in range(ny/2):
	for j in range(nx/2):
		resize_img[i,j,0] = img[i*2,j*2,0]
		resize_img[i,j,1] = img[i*2,j*2,1]
		resize_img[i,j,2] = img[i*2,j*2,2]
    if(displayImage):
    	cv2.imshow("resized", resize_img)
    	cv2.waitKey(10)
    return resize_img

def blackWhite(img, displayImage):
    RED_WEIGHT   = np.array([0.2989])
    GREEN_WEIGHT = np.array([0.5870])
    BLUE_WEIGHT  = np.array([0.1140])

    redScaled   = np.multiply(img[:,:,2], RED_WEIGHT)
    greenScaled = np.multiply(img[:,:,1], GREEN_WEIGHT)
    blueScaled  = np.multiply(img[:,:,0], BLUE_WEIGHT)

    temp = np.add(redScaled, greenScaled)
    temp = np.add(temp, blueScaled)
    

    grayscale_img = np.zeros((ny,nx,3), np.uint8)
    grayscale_img[:,:,0] = np.copy(temp) 
    grayscale_img[:,:,1] = np.copy(temp) 
    grayscale_img[:,:,2] = np.copy(temp) 
    
    if(displayImage):
    	cv2.imshow("blackwhite", grayscale_img)
    	cv2.waitKey(10) 

    return grayscale_img

def fillWhiteCircle(img, X, Y):
	if(X < 2 or (X >= nx -2)):
		return

	img[Y-2,  X,0] = img[Y-2,  X,1] = img[Y-2,  X,2] = 255	

	img[Y-1,X-1,0] = img[Y-1,X-1,1] = img[Y-1,X-1,2] = 255	
	img[Y-1,X  ,0] = img[Y-1,X  ,1] = img[Y-1,X  ,2] = 255
	img[Y-1,X+1,0] = img[Y-1,X+1,1] = img[Y-1,X+1,2] = 255
	
	img[Y,X-2,0] = img[Y,X-2,1] = img[Y,X-2,2] = 255	
	img[Y,X-1,0] = img[Y,X-1,1] = img[Y,X-1,2] = 255	
	img[Y,X  ,0] = img[Y,X  ,1] = img[Y,X  ,2] = 255	
	img[Y,X+1,0] = img[Y,X+1,1] = img[Y,X+1,2] = 255	
	img[Y,X+2,0] = img[Y,X+2,1] = img[Y,X+2,2] = 255
	
	img[Y+1,X-1,0] = img[Y+1,X-1,1] = img[Y+1,X-1,2] = 255	
	img[Y+1,X  ,0] = img[Y+1,X  ,1] = img[Y+1,X  ,2] = 255	
	img[Y+1,X+1,0] = img[Y+1,X+1,1] = img[Y+1,X+1,2] = 255	

	img[Y+2,X  ,0] = img[Y+2,X  ,1] = img[Y+2,X  ,2] = 255	
	
	return img


RED_R_LIMIT = np.array([100, 255])
RED_G_LIMIT = np.array([0, 30])
RED_B_LIMIT = np.array([0, 30])

GREEN_R_LIMIT = np.array([0, 30])
GREEN_G_LIMIT = np.array([100, 255])
GREEN_B_LIMIT = np.array([0, 30])

BLUE_R_LIMIT = np.array([0, 30])
BLUE_G_LIMIT = np.array([0, 30])
BLUE_B_LIMIT = np.array([100, 255])

def findColor(img, rLimit, gLimit, bLimit):
	
	found = False
	totalCount = 0
	X = 0
	Y = 0
	newimg = np.copy(img)
	redmask   = np.logical_and(newimg[:,:,2] >= rLimit[0], newimg[:,:,2] <= rLimit[1])
	greenmask = np.logical_and(newimg[:,:,1] >= gLimit[0], newimg[:,:,1] <= gLimit[1])
	bluemask  = np.logical_and(newimg[:,:,0] >= bLimit[0], newimg[:,:,0] <= bLimit[1])
	
	finalmask = np.logical_and(redmask, greenmask)
	finalmask = np.logical_and(finalmask,bluemask)
	
	
	for j in range(ny):
		for i in range(nx):
			if(finalmask[j,i]):
				totalCount = totalCount + 1					
				X = X + i
				Y = Y + j		
	if(totalCount > 0):	
		X = X / totalCount
		Y = Y / totalCount
		newimg = fillWhiteCircle(img, X, Y)
		found = True
	return newimg, X, Y, found
	
def findBalls(img):
	
	newimg = np.copy(img)
	
	[trackImg, xR, yR, rFound] = findColor( newimg,   RED_R_LIMIT,   RED_G_LIMIT,   RED_B_LIMIT)
	[trackImg, xG, yG, gFound] = findColor( newimg, GREEN_R_LIMIT, GREEN_G_LIMIT, GREEN_B_LIMIT)	
	[trackImg, xB, yB, bFound] = findColor( newimg,  BLUE_R_LIMIT,  BLUE_G_LIMIT,  BLUE_B_LIMIT)
	
	if(rFound):
		print 'Red   Center - (%d, %d)' % (xR, yR) 
	if(gFound):
		print 'Green Center - (%d, %d)' % (xG, yG) 
	if(bFound):
		print 'Blue  Center - (%d, %d)' % (xB, yB) 
	cv2.imshow("Colors", trackImg)
        cv2.waitKey(10)		
	 
def erosionDilation(img):
	#assuming img is in grayscale
	#using 11x11 structure element
	dim = img.shape
	ny1 = dim[0]
	nx1 = dim[1]

	eroImg = np.zeros((ny1,nx1), np.uint8)
	diaImg = np.zeros((ny1,nx1), np.uint8)

	finalEro = np.zeros((ny1,nx1,3), np.uint8)
	finalDia = np.zeros((ny1,nx1,3), np.uint8)

	for j in range(5,ny1-5):
		for i in range(5,nx1-5):
			subset = np.copy(img[j-5:j+5, i-5:i+5, 0])
			eroImg[j,i] = np.amin(subset)
			diaImg[j,i] = np.amax(subset)
	
	finalEro[:,:,0] = np.copy(eroImg) 
    	finalEro[:,:,1] = np.copy(eroImg) 
    	finalEro[:,:,2] = np.copy(eroImg) 

	finalDia[:,:,0] = np.copy(diaImg) 
    	finalDia[:,:,1] = np.copy(diaImg) 
    	finalDia[:,:,2] = np.copy(diaImg) 

	cv2.imshow("Erosion", finalEro)
	cv2.imshow("Dilation", finalDia)
        cv2.waitKey(1)	
	
	return
count = 0

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format
    if(img.ndim != 0):
	    if(count == 100):
		cv2.destroyWindow("Colors")
		cv2.destroyWindow("blackwhite")
	    if(count < 100):
	    	reSizeImg = reSize(img, True)
	    	findBalls(img)
	    	grayScaleImg = blackWhite(img, True)
	    else:
		grayScaleImg = blackWhite(img, False)
		reSizeImg = reSize(grayScaleImg, True)
		erosionDilation(reSizeImg)

	    ref.ref[0] = -0.5
	    ref.ref[1] = 0.5

	    #print 'Sim Time = ', tim.sim[0]
	    
	    # Sets reference to robot
	    r.put(ref);

	    # Sleeps
	    time.sleep(0.1)   
	    count = count + 1
	    print count
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
