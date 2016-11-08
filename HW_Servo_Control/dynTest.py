import hubo_ach as ha
import ach
from ctypes import *

DYNDATAUPDATE	= 100
CMDPOS 			= 101
CMDPOSVEL 		= 102
CMDPOSVELTOR	= 103

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


dout = ach.Channel('toDynCtrlProcess')
din  = ach.Channel('toTestProcess')

dataIn = DYNDATA()
dataOut = DYNDATA()

din.flush()

counter = 0

while 1 :

	[statuss, framesizes] = din.get(dataIn, wait=True, last=True)
	if(dataIn.cmd == DYNDATAUPDATE):
		print '%03d ID : %d Pos : %07.3f Vel : %07.3f Tor : %07.3f' % (counter, dataIn.id, dataIn.pos, dataIn.vel, dataIn.tor)
		counter = counter + 1
	# Test Pos 
	if(counter == 10):
		dataOut.cmd = CMDPOSVELTOR
		dataOut.id  = 3
		dataOut.pos = 90.0
		dataOut.vel = 75.0
		dataOut.tor = 16.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel, Torque Command to Actuator"
		print "Id : ", dataOut.id , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel, " Tor : ", dataOut.tor
		
	if(counter == 40):
		dataOut.cmd = CMDPOS
		dataOut.id  = 2
		dataOut.pos = 30.0
		dout.put(dataOut)
		print "\nSending new Pos Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos
	if(counter == 70):
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 3
		dataOut.pos = -90.0
		dataOut.vel = 30.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
		
	if(counter == 100):
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 3
		dataOut.pos = 90.0
		dataOut.vel = 5.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
		
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 2
		dataOut.pos = 0.0
		dataOut.vel = 5.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
		
	if(counter == 250):
		dataOut.cmd = CMDPOSVELTOR
		dataOut.id = 3
		dataOut.pos = 0.0
		dataOut.vel = 100.0
		dataOut.tor = 10.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel, Tor Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel, " Tor : ", dataOut.tor
	
	if(counter == 280):
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 3
		dataOut.pos = -90.0
		dataOut.vel = 30.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
	if(counter == 320):
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 3
		dataOut.pos = 90.0
		dataOut.vel = 10.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
	if(counter == 450):
		dataOut.cmd = CMDPOSVEL
		dataOut.id = 3
		dataOut.pos = -90.0
		dataOut.vel = 100.0
		dout.put(dataOut)
		print "\nSending new Pos, Vel Command to Actuator"
		print "Id : ", dataOut.id  , " Pos : " , dataOut.pos, " Vel : ", dataOut.vel
	# Test Pos Vel
	
	# Test Pos Vel Torque
	
	
	
	
    ## get keyboard input
	#input = raw_input(">> ")
	#if input == 'exit':
		#exit()
	#else:
		#input = raw_input("1) input pos\n2) input pos,vel\n3) input pos,vel,torque\n>> ")

		#if input == '1':
			#inputId = int(raw_input("Enter Id >> "))
			#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
			
			#dataOut.cmd = CMDPOS
			#dataOut.id = inputId
			#dataOut.pos = inputPos
			
			##toDyn(inputId, inputPos)
		#elif input == '2':
			#inputId = int(raw_input("Enter Id >> "))
			#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
			#inputVel = float(raw_input("Enter Vel in rpm (0 to 114) >> "))
			
			#dataOut.cmd = CMDPOSVEL
			#dataOut.id = inputId
			#dataOut.pos = inputPos
			#dataOut.vel = inputVel
			
			##toDyn(inputId, inputPos, inputVel)
		#elif input == '3':
			#inputId = int(raw_input("Enter Id >> "))
			#inputPos = float(raw_input("Enter Position in Degs (-150 to 150) >> "))
			#inputVel = float(raw_input("Enter Vel in rpm (0 to 114) >> "))
			#inputTorque = float(raw_input("Enter Max Torque in kgf/cm (0 to 16.5) >> "))
			
			#dataOut.cmd = CMDPOSVELTOR
			#dataOut.id = inputId
			#dataOut.pos = inputPos
			#dataOut.vel = inputVel
			#dataOut.tor	= inputTorque
		
		#d.put(dataOut)
			##toDyn(inputId, inputPos, inputVel, inputTorque )
	

