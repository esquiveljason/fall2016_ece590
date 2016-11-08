import time
import serial
import signal

import hubo_ach as ha
import ach
from ctypes import *

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=1000000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.open()
ser.isOpen()

MINDEGINPUT = -150.0
MAXDEGINPUT = 150.0
POSOFFSET = 150.0
MAXDEG = 300.0

MINVEL  = 0.0
MAXVEL = 114.0
MINTOR  = 0.0
MAXTOR = 16.5

MAXBITS = 1023.0

LENPOS = 0x05
LENPOSVEL = 0x07
LENPOSVELTOR = 0x09

DYNDATAUPDATE	= 100
CMDPOS 			= 101
CMDPOSVEL 		= 102
CMDPOSVELTOR	= 103

IDS = [3, 2] # Valid Actuator IDS

SENDDATATIMESTEP = 0.1

class DYNDATA(Structure):
    _pack_ = 1
    _fields_ = [("err"	, c_int8),
				("cmd"	, c_int8),
				("id"	, c_int8),
				("pos"	, c_double),
				("vel"	, c_double),
				("tor"	, c_double)
				 ]

def degToDynPos(deg):
	#input deg between -150 deg to 150 deg
	dynPosBits = int(( (deg + POSOFFSET)*MAXBITS )/MAXDEG)
	dynPosLow = dynPosBits & 0xff
	dynPosHigh = (dynPosBits >> 8) & 0xff
	return [dynPosHigh, dynPosLow]

def dynPosToDeg(inPos):
	return (inPos*MAXDEG/MAXBITS) - POSOFFSET
	
def rpmToDynVel(vel):
	#input rpm from 0 to 114
	dynVelBits = int((vel * MAXBITS)/MAXVEL)
	dynVelLow = dynVelBits & 0xff
	dynVelHigh = (dynVelBits >> 8) & 0xff
	return [dynVelHigh, dynVelLow]
	
def dynVelToRPM(inVel):
	return (inVel*MAXVEL/MAXBITS)
	
def kgfcmToDynTorque(torque):
	#input torque from 0 to 16.5 
	dynTorBits = int((torque * MAXBITS)/ MAXTOR)
	dynTorLow = dynTorBits & 0xff
	dynTorHigh = (dynTorBits >> 8) & 0xff
	return [dynTorHigh, dynTorLow]
	
def dynTorqueToKGFCM(inTorque):
	return (inTorque*MAXTOR/MAXBITS)
	
def fromDyn(dynId):
	
	a_head 		= [0xff, 0xff]
	a_id 		= [dynId & 0xff] #[0x03]
	a_len 		= [0x04]
	a_cmd 		= [0x02] # Read Data
	a_address  	= [0x24] # Starting Pos
	a_read_l	= [0x06] # Read Length
	a_sum		= [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_read_l[0] ) & 0xff]  # get checksum
	the_out_list= a_head + a_id + a_len + a_cmd + a_address + a_read_l + a_sum

	the_out 	= bytearray(the_out_list)
	ser.write(the_out)
	
	inList = bytearray(12)
	inList2 = [0,0,0,0,0,0,0,0,0,0,0,0]
	
	for i in range(12):
		inList[i] = ser.read(1)
		inList2[i] =  int(inList[i])
		
	print "New Cmd Pack from Actuator", inList2

	######################
	inHead		= [inList2[0], inList2[1]]
	inId		= inList2[2] 
	inLen		= inList2[3]
	inError		= inList2[4]
	
	inPos		=  ((inList2[6] & 0x03) << 8) + inList2[5]
	
	inVelDirection = (inList2[8] >> 2)
	inVel		=  ((inList2[8] & 0x03) << 8) + inList2[7]
	
	inTorDirection = (inList2[10] >> 2)
	inTor		= ((inList2[10] & 0x03) << 8) + inList2[9]
	
	inSum		= inList2[11]
	
	if(inVelDirection):
		inVel = inVel * -1.0
	if(inTorDirection):
		inTor = inTor * -1.0
	
	cs = checkSum(inList2)
	pos = -1
	vel = -1
	tor = -1
	
	if(cs):
		pos = dynPosToDeg(inPos)
		vel = dynVelToRPM(inVel)
		tor = dynTorqueToKGFCM(inTor)
	
	return [inError, inId, pos, vel, tor]

def checkSum(inList2):
	s = 0x00
	for i in range(2,11):
		s = s + inList2[i]
	snot = s & 0xff
	snot = (~snot & 0xFF)
	if(snot == inList2[11]):
		return True
	return False
	
	
def toDyn(dynId, deg, vel = None , torque = None):
	
	dynVelHigh = 0x00
	dynVelLow  = 0xff
	dynTorHigh = 0x00
	dynTorLow  = 0xff
	
	if(deg < MINDEGINPUT or deg > MAXDEGINPUT):
		print "Deg out of range [", MINDEGINPUT, " ", MAXDEGINPUT , " !!!!\n"
		return
	
	[dynPosHigh, dynPosLow] = degToDynPos(deg)
		
	if(vel):
		if(vel < MINVEL or vel > MAXVEL):
			print "Vel out of range [", MINVEL, " ", MAXVEL , " !!!!\n"
			return
			
		[dynVelHigh, dynVelLow] = rpmToDynVel(vel)
		
	if(torque):
		if(torque < MINTOR or torque > MAXTOR):
			print "Torque out of range [", MINTOR, " ", MAXTOR , " !!!!\n"
			return
		if(vel < MINVEL or vel > MAXVEL):
			print "Vel out of range [", MINVEL, " ", MAXVEL , " !!!!\n"
			return
			
		[dynVelHigh, dynVelLow] = rpmToDynVel(vel)
		[dynTorHigh, dynTorLow] = kgfcmToDynTorque(torque)
		
	
	a_head 		= [0xff, 0xff]
	a_id 		= [dynId & 0xff] #[0x03]
	a_len 		= [LENPOS]
	a_cmd 		= [0x03] # Write Data
	a_address  	= [0x1e] # Goal Pos
	a_goal_l	= [dynPosLow] # Goal low byte
	a_goal_h	= [dynPosHigh] # Goal high byte
	a_vel_l		= [dynVelLow] 
	a_vel_h		= [dynVelHigh]
	a_tor_l		= [dynTorLow]
	a_tor_h		= [dynTorHigh]
	
	#Just Position
	a_sum		= [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0]) & 0xff]  # get checksum
	the_out_list 	= a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_sum

	if(torque):
		a_len = [LENPOSVELTOR]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0] + a_tor_l[0] + a_tor_h[0]) & 0xff]  # get checksum
		the_out_list 	= a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_tor_l + a_tor_h + a_sum
		
	elif(vel):
		a_len = [LENPOSVEL]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0]) & 0xff]  # get checksum
		the_out_list 	= a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_sum
		
	the_out 	= bytearray(the_out_list)
	print "CMD Packet to Actuator : " , the_out_list
	ser.write(the_out)
	
	return_list = ser.read(6);
	


def fromDynInterruptHandler(signum, stack):
	for i in range(len(IDS)):
		[dataOut.err, dataOut.id, dataOut.pos, dataOut.vel, dataOut.tor] = fromDyn(IDS[i])
		dataOut.cmd = DYNDATAUPDATE
		dout.put(dataOut)
		print "Sent Current Pos, Vel, Torque"
		
	

	
counter = 0	
din = ach.Channel('toDynCtrlProcess')
dout = ach.Channel('toTestProcess')

dataIn = DYNDATA()
dataOut = DYNDATA()

signal.signal(signal.SIGALRM, fromDynInterruptHandler)
signal.setitimer(signal.ITIMER_REAL, SENDDATATIMESTEP, SENDDATATIMESTEP)

din.flush()

while 1 :
	[statuss, framesizes] = din.get(dataIn, wait=False, last=False)
	
	if(dataIn.cmd == CMDPOS):
		print "\n**************\nNew Position Command Received\n**************"
		print "Moving Actuator ", dataIn.id, " to Pos : ", dataIn.pos
		toDyn(dataIn.id, dataIn.pos)
		print ""
		dataIn.cmd = 0
	elif(dataIn.cmd == CMDPOSVEL):
		print "\n**************\nNew Position, Velocity Command Received\n**************\n"
		print "Moving Actuator ", dataIn.id, " to Pos : ", dataIn.pos , " Vel : ", dataIn.vel
		toDyn(dataIn.id, dataIn.pos, dataIn.vel)
		print ""
		dataIn.cmd = 0
	elif(dataIn.cmd == CMDPOSVELTOR):
		print "\n**************\nNew Position, Velocity, Torque Command Received\n**************\n"
		print "Moving Actuator ", dataIn.id, " to Pos : ", dataIn.pos , " Vel : ", dataIn.vel, " Tor : ", dataIn.tor
		toDyn(dataIn.id, dataIn.pos, dataIn.vel, dataIn.tor)
		print ""
		dataIn.cmd = 0
		
	#t1 = time.time()
	#dt = t1-t0
	#if(dt > SENDDATATIMESTEP):
		#[dataOut.err, dataOut.id, dataOut.pos, dataOut.vel, dataOut.tor] = fromDyn(IDS[0])
		#dataOut.cmd = DYNDATAUPDATE
		#dout.put(dataOut)
		#print "Sent Data"
		#t0 = time.time()
		
    ## get keyboard input
	#input = raw_input("1) input pos\n2) input pos,vel\n3) input pos,vel,torque\n4) Get Data\n5) exit\n>> ")
	#if input == '5':
		#ser.close()
		#exit()
	#elif input == '1':
		#inputId = int(raw_input("Enter Id >> "))
		#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
		
		#toDyn(inputId, inputPos)
	#elif input == '2':
		#inputId = int(raw_input("Enter Id >> "))
		#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
		#inputVel = float(raw_input("Enter Vel in rpm (0 to 114) >> "))

		#toDyn(inputId, inputPos, inputVel)
	#elif input == '3':
		#inputId = int(raw_input("Enter Id >> "))
		#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
		#inputVel = float(raw_input("Enter Vel in rpm (0 to 114) >> "))
		#inputTorque = float(raw_input("Enter Max Torque in kgf/cm (0 to 16.5) >> "))

		#toDyn(inputId, inputPos, inputVel, inputTorque )
	
	#elif input == '4':
		#inputId = int(raw_input("Get Info from Id >> "))
		#print fromDyn(inputId)
	
