#!/usr/bin/env python

import rospy
import math
import time
import numpy as numpy
from numpy.linalg import inv
from std_msgs.msg import Float64
from std_msgs.msg import String

import roslib
import actionlib
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from dynamixel_controllers.srv import *

import threading

def dhmatrix(a, twist, offset, deg):
	dh = numpy.zeros((4,4))
	dh[0][0] = math.cos(deg);
	dh[0][1] = -1*math.sin(deg)*math.cos(twist)
	dh[0][2] = math.sin(deg)*math.sin(twist)
	dh[0][3] = offset*math.cos(deg)
	
	dh[1][0] = math.sin(deg)
	dh[1][1] = math.cos(deg)*math.cos(twist)
	dh[1][2] = -1*math.cos(deg)*math.sin(twist)
	dh[1][3] = offset*math.sin(deg)
	
	dh[2][1] = math.sin(twist)
	dh[2][2] = math.cos(twist)
	dh[2][3] = a
	
	dh[3][3] = 1
	
	return dh
	
def inverseK(x0, y0, z0, a20, a30, d10, deg1c, deg3c):
	x = float(x0)
	y = float(y0)
	z = float(z0)
	a2 = float(a20)
	a3 = float(a30)
	d1 = float(d10)
	r = math.sqrt(pow(x,2) + pow(y,2))
	s = z - d1
	
	if (r>(a2+a3)):
		print "Error, out of reach of configuration"
	else:
		R = math.sqrt(pow(r,2) + pow((z-d1),2))
		
		#DETERMINING DEG1
		if x==0 and y>0:
			deg1 = PI/2
		elif x==0 and y<0:
			deg1 = -PI/2
		elif x<0 and y==0:
			deg1 = -PI
		elif x>0 and y==0:
			deg1 = 0
		elif x<0 and y<0:
			deg1 = math.atan(y/x) - deg1c+ PI
		elif x==0 and y==0:
			deg1 = 0
		else:
			deg1 = math.atan(y/x) - deg1c
			
		#DETERMINING DEG 2 AND DEG 3
		d = (pow(r,2)+pow((s),2)- pow(a2,2) - pow(a3,2))/(2*a2*a3)

		if d<=1 and d>0:
			#~ print "CASE 1"
			deg3 = math.atan(-1*math.sqrt(1-pow(d,2))/d)
		elif d==0:
			#~ print "CASE 2"
			deg3 = - PI/2
		elif d>=-1 and d<0:
			#~ print "CASE 3"
			deg3 = math.atan(-1*math.sqrt(1-pow(d,2))/d) - PI
		
		if r==0 and s>0:
			deg2 = PI/2
		else:
			deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
				(a2+a3*math.cos(-deg3)))	
		
		output = numpy.zeros((3))
		output[0] = deg1*180/PI
		output[1] = deg2*180/PI
		output[2] = deg3*180/PI + deg3c
		#~ print "s: %.2f, d: %.2f, R: %.2f" % (s,d,R)
		return output
		
def linearTrajectory(cord1, cord2, points, duration, constants):
	#cord1 and cord2 is a list of size 3 which contain coordinates in 3D space
	#points indicate the number of divisions in the trajectory
	#duration is the time required for the robot to move from point1 to point2
	#constants is a list of size 5
	
	d = 3
	current = numpy.zeros((d))
	increment = numpy.zeros((d))
	positions = numpy.zeros((points+1,3))
	timeStamp = numpy.zeros(points+1)
	tIncrement = float(duration)/points
	currentTime = 0
	
	for i in range(d):
		current[i] = cord1[i]
		increment[i] = float(cord2[i]-cord1[i])/points
		
	for i in range(points+1):
		
		inverseAngles = inverseK(current[0], current[1], current[2], constants[0], constants[1], constants[2], constants[3], constants[4])
		inverseAngles = degToRad(inverseAngles)
		
		for j in range(d):
			positions[i][j] = inverseAngles[j]
			current[j] = current[j] + increment[j]
		
		timeStamp[i] = currentTime
		currentTime = currentTime + tIncrement
		#~ print "Time Stamp: %.2f, Positions: %.2f, %.2f, %.2f" % (timeStamp[i], positions[i][0], positions[i][1], positions[i][2])
	
	return timeStamp, positions

def publishTrajectory(positions, timeStamps):
	s = (positions.size)/3
	#~ print s
	goal = FollowJointTrajectoryGoal()                  
	goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3']
	
	p = []
	
	for i in range(s):
		
		p.append(JointTrajectoryPoint())
		p[i].positions = [positions[i][0], positions[i][1], positions[i][2]]
		goal.trajectory.points.append(p[i])
		goal.trajectory.points[i].time_from_start = rospy.Duration(timeStamps[i])

	#~ print goal

	client.send_goal_and_wait(goal)
	client.wait_for_result()

#Get Trajectory Constants for all three joints
def viaPointTrajectoryCalculation(startPoint, endPoint, duration, constants, viaPointsNumber):
	
	#Load Cartesian Coordinate Positions
	P = numpy.zeros((viaPointsNumber+2, 3))
	
	n = 0
	P[n][0] = startPoint[0]
	P[n][1] = startPoint[1] 
	P[n][2] = startPoint[2] 
	
	step = numpy.zeros((3))
	for i in range(3):
		step[i] = (endPoint[i] - startPoint[i]) / (viaPointsNumber + 1)
		
	for i in range(viaPointsNumber):
		
		for j in range(3):
			P[i+1][j] = P[i][j] + step[j]

	
	n = viaPointsNumber + 1
	P[n][0] = endPoint[0]
	P[n][1] = endPoint[1] 
	P[n][2] = endPoint[2] 
	
	#~ print "Coordinate Positions:"
	#~ print P
	
	#Load Joint Space Positions
	Ik = numpy.zeros((viaPointsNumber+2, 3))
	for i in range(viaPointsNumber + 2):
		Ik[i] = inverseK(P[i][0], P[i][1], P[i][2], constants[0], constants[1], constants[2], constants[3], constants[4])
		Ik[i] = degToRad(Ik[i])

	#~ print "Inverse Angles:"
	#~ print Ik
	
	t = numpy.zeros((viaPointsNumber + 2))
	timeStep = float(duration) / (viaPointsNumber + 1)
	t[0] = 0 
	for i in range(viaPointsNumber + 1):
		t[i+1] = t[i] + timeStep
	
	#~ print "Duration %.2f" % duration
	#~ print "Time Stamps:"
	#~ print t
	
	#Load Matrix B
	B = numpy.zeros((viaPointsNumber + 2,viaPointsNumber + 6))
	for i in range(viaPointsNumber + 2):
		for j in range(3):
			B[j][i] = Ik[i][j]
	
	#~ print "Matrix B:"
	#~ print B

	#Load Matrix A: Joint Position Rows
	A1 = numpy.zeros((6+viaPointsNumber,6+viaPointsNumber))
	
	for i in range (2 + viaPointsNumber):
		
		for j in range (6+viaPointsNumber):
			
			A1[i][j] = pow(t[i],5+viaPointsNumber-j)
		
		A1[i][5+viaPointsNumber] = 1
	
	#Load Matrix A: Loading the last 4 rows
	n = 2 + viaPointsNumber
	
	tSkip = numpy.zeros((2))
	tSkip[1] = viaPointsNumber + 1
	
	for i in range (2):
		
		for j in range(5+viaPointsNumber):
			
			A1[n+i][j] = (5+viaPointsNumber-j)*pow(t[tSkip[i]],viaPointsNumber+4-j)
		
		A1[n+i][4+viaPointsNumber] = 1
	
	n = 4 + viaPointsNumber
	for i in range (2):
		
		for j in range(4+viaPointsNumber):
			
			A1[n+i][j] = (4+viaPointsNumber-j)*(5+viaPointsNumber-j)*pow(t[tSkip[i]],viaPointsNumber+3-j)
		
		A1[n+i][3+viaPointsNumber] = 2
		
	
	#~ print "A1:"
	#~ print A1
	
	#Inverse
	A2 = inv(A1)
	
	#Determining the trajectory coefficients
	C = numpy.zeros((viaPointsNumber + 2,viaPointsNumber + 6))
	for i in range (viaPointsNumber + 2):
		C[i] = numpy.dot(A2 , B[i])
		
	#~ print "C:"
	#~ print C
	
	return C

#Ensures fork is parallel with the ground	
def publishViaPointTrajectory(c, duration, divisions, viaPointsNumber):
	
	goal = FollowJointTrajectoryGoal()                  
	goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3', 'tilt_joint4']
	duration = float(duration)
	currentTime = 0
	timeStep = float(duration/divisions)
	
	positions = numpy.zeros((divisions+1,4))
	velocities = numpy.zeros((divisions+1,4))
	accelerations = numpy.zeros((divisions+1,4))
	
	#~ print timeStep
	
	#~ print "Duration: %.2f, Time Step: %.2f" % (duration, timeStep)
	
	#Calculate Position and Velocity Values
	for i in range(divisions+1):
		
		for n in range(3):
			
			#Loading Positions
			for j in range(viaPointsNumber + 5):
				positions[i][n] = positions[i][n] + c[n][j]*pow(currentTime,viaPointsNumber+5-j)
				
			positions[i][n] = positions[i][n] + c[n][viaPointsNumber + 5]
			
			#Loading Velocities
			for j in range(viaPointsNumber + 4):
				velocities[i][n] = velocities[i][n] + (viaPointsNumber + 5-j)*c[n][j]*pow(currentTime, viaPointsNumber + 4 - j)
		
			velocities[i][n] = velocities[i][n] + c[n][viaPointsNumber + 4]
			
			#Loading Accelerations
			for j in range(viaPointsNumber + 3):
				accelerations[i][n] = accelerations[i][n] + (viaPointsNumber + 4 - j)*(viaPointsNumber + 5-j)*c[n][j]*pow(currentTime, viaPointsNumber + 3 - j)
		
			accelerations[i][n] = accelerations[i][n] + c[n][viaPointsNumber + 3]
			
		positions[i][3] = -(positions[i][1] + positions[i][2])
		
		currentTime = timeStep + currentTime	
	
	#~ print "Positions"
	#~ print positions
	#~ print "Velocities"
	#~ print velocities
	#~ currentTime = 0
	
	#Loading Goal object
	p = []
	v = []
	
	for i in range(divisions+1):
		
		p.append(JointTrajectoryPoint())
		p[i].positions = [positions[i][0], positions[i][1], positions[i][2], positions[i][3]]
		p[i].velocities = [velocities[i][0], velocities[i][1], velocities[i][2], 0]
		p[i].accelerations = [accelerations[i][0], accelerations[i][1], accelerations[i][2], 0]
		goal.trajectory.points.append(p[i])
		goal.trajectory.points[i].time_from_start = rospy.Duration(currentTime)
		currentTime = timeStep + currentTime
	
	#Print start, mid and end point for reference
	#~ print goal.trajectory.points[0]
	#~ print goal.trajectory.points[divisions/2]
	#~ print goal.trajectory.points[divisions]
	
	#~ #Executing Motion				
	client.send_goal_and_wait(goal)
	client.wait_for_result()

#Fork not parallel
def publishViaPointTrajectory2(c, duration, divisions, viaPointsNumber):
	
	goal = FollowJointTrajectoryGoal()                  
	goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3', 'tilt_joint4']
	duration = float(duration)
	currentTime = 0
	timeStep = float(duration/divisions)
	
	positions = numpy.zeros((divisions+1,4))
	velocities = numpy.zeros((divisions+1,4))
	accelerations = numpy.zeros((divisions+1,4))
	
	#~ print timeStep
	
	#~ print "Duration: %.2f, Time Step: %.2f" % (duration, timeStep)
	
	#Calculate Position and Velocity Values
	for i in range(divisions+1):
		
		for n in range(3):
			
			#Loading Positions
			for j in range(viaPointsNumber + 5):
				positions[i][n] = positions[i][n] + c[n][j]*pow(currentTime,viaPointsNumber+5-j)
				
			positions[i][n] = positions[i][n] + c[n][viaPointsNumber + 5]
			
			#Loading Velocities
			for j in range(viaPointsNumber + 4):
				velocities[i][n] = velocities[i][n] + (viaPointsNumber + 5-j)*c[n][j]*pow(currentTime, viaPointsNumber + 4 - j)
		
			velocities[i][n] = velocities[i][n] + c[n][viaPointsNumber + 4]
			
			#Loading Accelerations
			for j in range(viaPointsNumber + 3):
				accelerations[i][n] = accelerations[i][n] + (viaPointsNumber + 4 - j)*(viaPointsNumber + 5-j)*c[n][j]*pow(currentTime, viaPointsNumber + 3 - j)
		
			accelerations[i][n] = accelerations[i][n] + c[n][viaPointsNumber + 3]
			
		positions[i][3] = 0
		
		currentTime = timeStep + currentTime	
	
	#~ print "Positions"
	#~ print positions
	#~ print "Velocities"
	#~ print velocities
	currentTime = 0
	
	#Loading Goal object
	p = []
	v = []
	
	for i in range(divisions+1):
		
		p.append(JointTrajectoryPoint())
		p[i].positions = [positions[i][0], positions[i][1], positions[i][2], positions[i][3]]
		p[i].velocities = [velocities[i][0], velocities[i][1], velocities[i][2], 0]
		p[i].accelerations = [accelerations[i][0], accelerations[i][1], accelerations[i][2], 0]
		goal.trajectory.points.append(p[i])
		goal.trajectory.points[i].time_from_start = rospy.Duration(currentTime)
		currentTime = timeStep + currentTime
	
	#Print start, mid and end point for reference
	#~ print goal.trajectory.points[0]
	#~ print goal.trajectory.points[divisions/2]
	#~ print goal.trajectory.points[divisions]
	
	#~ #Executing Motion
	client.send_goal_and_wait(goal)
	client.wait_for_result()
				
def CloseClaw():
	d= 0.55
	rospy.loginfo(numpy.float64(d))
	pub4.publish(numpy.float64(d))

def OpenClaw():
	d= -0.175
	rospy.loginfo(numpy.float64(d))
	pub4.publish(numpy.float64(d))

def Turn1(deg):
	rospy.loginfo(numpy.float64(deg))
	pub1.publish(numpy.float64(deg))
	
def Turn2(deg2):
	rospy.loginfo(numpy.float64(deg2))
	pub2.publish(numpy.float64(deg2))
	
def Turn3(deg3):
	rospy.loginfo(numpy.float64(deg3))
	pub3.publish(numpy.float64(deg3))
	
def Turn4(deg4):
	rospy.loginfo(numpy.float64(deg4))
	pub4.publish(numpy.float64(deg4))
	
def Turn5(deg5):
	rospy.loginfo(numpy.float64(deg5))
	pub5.publish(numpy.float64(deg5))
	
def publishAngles(rads):
	#Receives the angles in Radians
	#degs is a list of size 3 containing the joint positions
	Turn1(rads[0])
	Turn2(rads[1])
	Turn3(rads[2])
	
def degToRad(degs):
	for i in range(3):
		degs[i] = degs[i]*PI/180
	print degs
	return degs
	
def defaultSpeed():
	rospy.wait_for_service('/tilt_controller2/set_speed')
	set_speed1 = rospy.ServiceProxy('/tilt_controller1/set_speed', SetSpeed)
	set_speed2 = rospy.ServiceProxy('/tilt_controller2/set_speed', SetSpeed)
	set_speed3 = rospy.ServiceProxy('/tilt_controller3/set_speed', SetSpeed)
	set_speed4 = rospy.ServiceProxy('/tilt_controller4/set_speed', SetSpeed)
	try:
		set_speed1(1.17)
		set_speed2(1.17)
		set_speed3(1.17)
		set_speed4(1.17)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

#Configured for Ik2
def movePoint(Point1, constants):
	
	print "Target Point: %.2f, %.2f, %.2f" % (Point1[0], Point1[1], Point1[2])
	
	Ik = inverseK(Point1[0], Point1[1], Point1[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (Ik[0], Ik[1], Ik[2])
	Ik = degToRad(Ik)
	
	confirm = "y"
	#~ confirm = raw_input('Move to start point? (y/n):')			
	if (confirm=="y"):
		
		Turn1(Ik[0])
		Turn2(Ik[1])
		Turn3(Ik[2])	
		deg4 = -(Ik[1] + Ik[2])
		Turn4(deg4)
		
		return True
	
	else:
		
		return False
		
#None Parallel Plate
def movePoint2(Point1, constants):
	
	print "Target Point: %.2f, %.2f, %.2f" % (Point1[0], Point1[1], Point1[2])
	confirm ="y"
	#~ confirm = raw_input('Move to start point? (y/n):')			
	if (confirm=="y"):
					
		Ik = inverseK(Point1[0], Point1[1], Point1[2], constants[0], constants[1], constants[2], constants[3], constants[4])
		Ik = degToRad(Ik)
		
		Turn1(Ik[0])
		Turn2(Ik[1])
		Turn3(Ik[2])	
		Turn4(0)
		
		return True
	
	else:
		
		return False
					
def movePick(P, fixedVariables2, probeInsert, probeInsertDuration, probeLift, probeLiftDuration):
	#Move to start point
	xstart = P[0]
	ystart = P[1]
	zstart = P[2]
	
	move = startPickPosition(P, fixedVariables2)
	confirm = "y"
	wait = 0.3
	
	if (move==True):
		
		#~ confirm = raw_input('Begin pick motion? (y/n):')		
		time.sleep(1.5)	
		if (confirm=="y"):
		
			#Probe Insert Action
			xend = xstart + probeInsert	
			yend = ystart			
			zend = zstart
			duration = probeInsertDuration
			
			startPoint = [xstart, ystart, zstart, 0]
			endPoint = [xend, yend, zend, duration]
			
			viaPoints = 1
			divisions = 5
			
			c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

			publishViaPointTrajectory(c,duration, divisions, viaPoints)
			
			#~ confirm = raw_input('Begin lift motion? (y/n):')		
			time.sleep(wait)
				
			if (confirm=="y"):
			
				#Probe Lift Action
				xstart = xend
				ystart = yend
				zstart = zend
				zend = zstart + probeLift
				duration = probeLiftDuration
				
				startPoint = [xstart, ystart, zstart, 0]
				endPoint = [xend, yend, zend, duration]
								
				viaPoints = 1
				divisions = 5
				
				c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

				publishViaPointTrajectory(c,duration, divisions, viaPoints)
				
				#~ time.sleep(1)
				defaultSpeed()
				
				#~ confirm = raw_input('Begin Withdraw Motion? (y/n):')	
				time.sleep(wait)
						
				if (confirm=="y"):
					
					#Probe Withdraw Action
					xstart = xend
					xend = xstart - probeInsert - 20
					ystart = yend
					yend = ystart
					zstart = zend
					duration = probeLiftDuration
					
					startPoint = [xstart, ystart, zstart, 0]
					endPoint = [xend, yend, zend, duration+1.0]
									
					viaPoints = 1
					divisions = 5
					
					c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

					publishViaPointTrajectory(c,duration, divisions, viaPoints)
					
					defaultSpeed()
				
					set_speed2 = rospy.ServiceProxy('/tilt_controller2/set_speed', SetSpeed)
					set_speed3 = rospy.ServiceProxy('/tilt_controller3/set_speed', SetSpeed)
					
					try:
						set_speed2(0.5)
						set_speed3(0.5)
					except rospy.ServiceException as exc:
						print("Service did not process request: " + str(exc))

					move = movePoint2([70,0,200], fixedVariables2)
					
					if (move==True):
						CloseClaw()
						
						time.sleep(2)
						defaultSpeed()

def startPickPosition(Point1, constants):
	#zero
	publishAngles([0,PI/2,0])
	
	print "Target Point: %.2f, %.2f, %.2f" % (Point1[0], Point1[1], Point1[2])
	
	Ik = inverseK(Point1[0], Point1[1], Point1[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (Ik[0], Ik[1], Ik[2])
	Ik = degToRad(Ik)
	confirm = raw_input('Move to start point? (y/n):')			
	if (confirm=="y"):
		
		Turn1(Ik[0])
		Turn3(Ik[2])	
		time.sleep(1.5)
		Turn2(Ik[1])			
		deg4 = -(Ik[1] + Ik[2])
		Turn4(deg4)
		
		return True
	
	else:
		
		return False
		
def moveInsert(P, fixedVariables2, probeInsert, probeInsertDuration, probeWithdraw, probeWithdrawHeight, probeWithdrawDuration):
	#Move to start point
	xstart = P[0]
	ystart = P[1]
	zstart = P[2]
		
	move = startInsertPosition(P, fixedVariables2)
	
	if (move==True):
		
		set_speed2 = rospy.ServiceProxy('/tilt_controller2/set_speed', SetSpeed)
		set_speed3 = rospy.ServiceProxy('/tilt_controller3/set_speed', SetSpeed)
		
		try:
			set_speed2(0.5)
			set_speed3(0.5)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
					
		confirm = raw_input('Begin Insert motion? (y/n):')			
		if (confirm=="y"):
		
			#Insert Motion
			xend = xstart 
			yend = ystart			
			zend = zstart + probeInsert	
			duration = probeInsertDuration
			
			startPoint = [xstart, ystart, zstart, 0]
			endPoint = [xend, yend, zend, duration]
			
			viaPoints = 1
			divisions = 5
			
			c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

			publishViaPointTrajectory(c,duration, divisions, viaPoints)
			
			confirm = raw_input('Begin Withdraw motion? (y/n):')			
			if (confirm=="y"):
			
				#Withdraw Motion
				defaultSpeed()
				Turn4(0)
				time.sleep(0.2)
				xstart = xend
				ystart = yend
				yend = ystart + probeWithdraw
				zstart = zend
				zend = zstart + probeWithdrawHeight
				duration = probeWithdrawDuration
				
				startPoint = [xstart, ystart, zstart, 0]
				endPoint = [xend, yend, zend, duration]
								
				viaPoints = 1
				divisions = 5
				
				c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

				publishViaPointTrajectory2(c,duration, divisions, viaPoints)
				
				time.sleep(1)
				defaultSpeed()
				
				confirm = raw_input('Zero? (y/n):')			
				if (confirm=="y"):
						
					publishAngles([0,PI/2,0])

def startInsertPosition(Point1, constants):
	
	print "Target Point: %.2f, %.2f, %.2f" % (Point1[0], Point1[1], Point1[2])
	
	Ik = inverseK(Point1[0], Point1[1], Point1[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (Ik[0], Ik[1], Ik[2])
	Ik = degToRad(Ik)
	confirm = raw_input('Move to start point? (y/n):')			
	if (confirm=="y"):
		
		Turn1(Ik[0])
		time.sleep(1.5)
		
		Turn3(Ik[2])	
		Turn2(Ik[1])			
		deg4 = -(Ik[1] + Ik[2])
		Turn4(deg4)
		
		return True
	
	else:
		
		return False

def callback(data):
	global publicX
	global publicY
	global publicZ
	global publicDuration
	
	global teleStepX
	global teleStepY
	global teleStepZ
	
	global constantCheck
	global threadContinue
	
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	s = str(data.data)
	print "Data converted to string: %s" % s
	#Forward
	if (s=="3"):
		
		teleStepX = 2.0
		threadContinue = True
		threads = []
		t = threading.Thread(target=beginThread)
		threads.append(t)
		t.start()
    
	#Backward
	elif (s=="4"):
						
		teleStepX = -2.0
		threadContinue = True
		threads = []
		t = threading.Thread(target=beginThread)
		threads.append(t)
		t.start()
		
	#Left
	elif (s=="2"):
				
		teleStepY = 2.0
		threadContinue = True
		threads = []
		t = threading.Thread(target=beginThread)
		threads.append(t)
		t.start()
		
	#Right
	elif s=="1":
		
		teleStepY = -2.0
		threadContinue = True
		threads = []
		t = threading.Thread(target=beginThread)
		threads.append(t)
		t.start()
		
	elif s=="8":
		constantCheck = 0
		threadContinue = False
		teleStepX = 0.0
		teleStepY = 0.0
		teleStepZ = 0.0
		
		
	
def teleOp():
	rospy.Subscriber("chatter", String, callback)
	rospy.spin()
	constantCheck=constantCheck+1
	rospy.loginfo(constantCheck)

def beginThread():
	
	global constantCheck
	global publicX
	global publicY
	global publicZ
	global publicDuration
	
	global teleStepX
	global teleStepY
	global teleStepZ
	
	threads2 = []
	
	while (threadContinue==True):
		publicX = publicX + teleStepX
		publicY = publicY + teleStepY
		publicZ = publicZ + teleStepZ
		
		t2 = threading.Thread(target=followCommand)
		threads2.append(t2)
		t2.start()
		time.sleep(0.1)
		constantCheck = constantCheck + 1
	
	
def followCommand():
	global publicX
	global publicY
	global publicZ
	global fixedVariables2
	
	move = movePoint([publicX, publicY, publicZ], fixedVariables2)
	print constantCheck
	
		
if __name__== '__main__':
	try:
		PI = math.pi
		pub4 = rospy.Publisher('/tilt_controller4/command', Float64, queue_size=10)
		pub1 = rospy.Publisher('/tilt_controller1/command', Float64, queue_size=10)
		pub2 = rospy.Publisher('/tilt_controller2/command', Float64, queue_size=10)
		pub3 = rospy.Publisher('/tilt_controller3/command', Float64, queue_size=10)
		pub5 = rospy.Publisher('/tilt_controller5/command', Float64, queue_size=10)
		rospy.init_node('TurnServo', anonymous=True)
		
		client  = actionlib.SimpleActionClient('/f_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		client.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')
		
		#Fixed
		
		#Variable set one. To do inverse kinematic for grasper tip.
		deg0 = 0
		l1 = 89
		l2 = 0
		l3 = 67.5
		l4 = 103.1
		deg3c = 18.89
		
		fixedVariables = numpy.zeros(5)
		fixedVariables[0] = l3
		fixedVariables[1] = l4
		fixedVariables[2] = l1+l2
		fixedVariables[3] = 0
		fixedVariables[4] = deg3c
		
		#Variable set two. To do inverse kinematic to the middle of the last servo.
		deg0 = 0
		l1 = 89
		l2 = 0
		l3 = 67.5
		l4 = 67.5
		deg3c = 0
		
		fixedVariables2 = numpy.zeros(5)
		fixedVariables2[0] = l3
		fixedVariables2[1] = l4
		fixedVariables2[2] = l1+l2
		fixedVariables2[3] = 0
		fixedVariables2[4] = deg3c
		
		#Teleop Start
		publicX = 100.0
		publicY = 0.0
		publicZ = 100.0
		publicDuration = 0.4
		constantCheck = 0
		threadContinue = False
		moveForward = False
		
		teleStepX = 0.0
		teleStepY = 0.0
		teleStepZ = 0.0
				
				
		while not rospy.is_shutdown():
			
			#All user inputs to be non-capitalised
			s = raw_input('--> ')
			if s=="close":
				CloseClaw()
				
			elif s=="open":
				OpenClaw()
				
			elif s=="default":				
				defaultSpeed()
				
			elif s=="t1":
				deg = float(raw_input("Enter deg:"))
				deg1 = deg*PI/180
				Turn1(deg1)
				
			elif s=="t2":
				deg = float(raw_input("Enter deg:"))
				deg2 = deg*PI/180
				Turn2(deg2)
				
			elif s=="t3":
				deg = float(raw_input("Enter deg:"))
				deg3 = deg*PI/180
				Turn3(deg3)
				
			elif s=="t4":
				deg = float(raw_input("Enter deg:"))
				deg4= deg*PI/180
				Turn4(deg4)
				
			elif s=="t5":
				deg = float(raw_input("Enter deg:"))
				deg5= deg*PI/180
				Turn5(deg5)
				
			elif s=="zero":
				publishAngles([0,PI/2,0])
			
			#Determines various joint configuration for a straight line movement - and publishes at a fixed rate. No velocity control							
			elif s=="move":						

				#Slot 1
				xstart = 150.0
				ystart = 0.0
				zstart = 120.0
				
				xend = 150.0
				yend = 0.0
				zend = 70.0
				
				delay = 0.029
							
				p1 = [xstart, ystart, zstart]
				p2 = [xend, yend, zend]
				
				divisions = 50
				duration = 2
				
				#Planning the trajectory
				timeStamp, jPositions = linearTrajectory(p1,p2,divisions,duration,fixedVariables)
				
				print jPositions[0]
				
				confirm = raw_input('Confirm move servos (y/n):')			
				
				if (confirm=="y"):
					#Moves to start position
					publishAngles(jPositions[0])
					time.sleep(2)
					
					confirm = raw_input('Confirm move servos (y/n):')			
				
					if (confirm=="y"):
				
						#Begin the linear motion
						for n in range(1,divisions):
							publishAngles(jPositions[n])
							time.sleep(delay)
			
			#Sets tip of the robot to a specified point
			elif s=="set":
				d = raw_input('Enter point values:')
				n=0
				deg =""
				out = []
				for letter in d:
					
					if letter == ",":
						out.append(float(deg))
						print out[n]
						n=n+1
						deg = ""
					else:
						deg = deg + letter	
				
				#START OF CODE
				PI = math.pi

				link_length = numpy.zeros((5))
				link_twist = numpy.zeros((5))
				link_offset = numpy.zeros((5))
				link_deg = numpy.zeros((5))
				dh = numpy.zeros((5,4,4))
				point = numpy.zeros((4))

				n_links = 4
			
				point[0] = 0
				point[1] = 0
				point[2] = 0
				point[3] = 1

				#Get inverse
				inverseAngles = inverseK(out[0],out[1],out[2],fixedVariables[0],fixedVariables[1],fixedVariables[2], fixedVariables[3], fixedVariables[4])
				print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (inverseAngles[0], inverseAngles[1], inverseAngles[2])

				#Input variables for Forward Kinematics
				deg1 = inverseAngles[0]
				deg2 = inverseAngles[1]
				deg3 = inverseAngles[2]

				#Input fixed values for Forward Kinematics
				link_length[0] = l1
				link_twist[0] = 0
				link_offset[0] = 0
				link_deg[0] = 0

				link_length[1] = l2
				link_twist[1] = PI/2	
				link_offset[1] = 0
				link_deg[1] = deg1*PI/180

				link_length[2] = 0
				link_twist[2] = 0
				link_offset[2] = l3
				link_deg[2] = (deg2)*PI/180

				link_length[3] = 0
				link_twist[3] = 0
				link_offset[3] = l4
				link_deg[3] = (deg3-deg3c)*PI/180

				#~ link_length[4] = 0
				#~ link_twist[4] = 0
				#~ link_offset[4] = 0
				#~ link_deg[4] = 0

				#Calculate Rotation Matrix
				for x in range(0,n_links):
					dh[x] = dhmatrix(link_length[x],link_twist[x],link_offset[x],link_deg[x])
					#~ print "a %d" % x
					#~ print dh[x]

				#Calculate point w.r.t to world coordinate	
				result=numpy.dot(dh[n_links-1],point)

				for x in range(1,n_links):
					result=numpy.dot(dh[n_links-1-x],result)

				#~ print "Deg1: %.2f, Deg2: %.2f, Deg3: %.2f" % (deg1,deg2,deg3)
				print "Resulting point: x: %.2f, y: %.2f, z: %.2f" % (result[0], result[1], result[2])
				
				confirm = raw_input('Confirm move servos (y/n):')
				
				if (confirm=="y"):
					Turn1(deg1*PI/180)
					Turn2(deg2*PI/180)
					Turn3(deg3*PI/180)		
			
			#Sets Joint 3 to a specified point with parallel claw
			elif s=="set2":
				d = raw_input('Enter point values:')
				n=0
				deg =""
				out = []
				for letter in d:
					
					if letter == ",":
						out.append(float(deg))
						print out[n]
						n=n+1
						deg = ""
					else:
						deg = deg + letter	
				
				#START OF CODE
				PI = math.pi

				link_length = numpy.zeros((5))
				link_twist = numpy.zeros((5))
				link_offset = numpy.zeros((5))
				link_deg = numpy.zeros((5))
				dh = numpy.zeros((5,4,4))
				point = numpy.zeros((4))

				n_links = 4
			
				point[0] = 0
				point[1] = 0
				point[2] = 0
				point[3] = 1

				#Get inverse
				inverseAngles = inverseK(out[0],out[1],out[2],fixedVariables2[0],fixedVariables2[1],fixedVariables2[2], fixedVariables2[3], fixedVariables2[4])
				#Input variables for Forward Kinematics
				deg1 = inverseAngles[0]
				deg2 = inverseAngles[1]
				deg3 = inverseAngles[2]
				deg4 = -(deg2 + deg3)
				
				print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f, deg4: %.2f" % (inverseAngles[0], inverseAngles[1], inverseAngles[2], deg4)

				#Input fixed values for Forward Kinematics
				link_length[0] = l1
				link_twist[0] = 0
				link_offset[0] = 0
				link_deg[0] = 0

				link_length[1] = l2
				link_twist[1] = PI/2	
				link_offset[1] = 0
				link_deg[1] = deg1*PI/180

				link_length[2] = 0
				link_twist[2] = 0
				link_offset[2] = l3
				link_deg[2] = (deg2)*PI/180

				link_length[3] = 0
				link_twist[3] = 0
				link_offset[3] = l4
				link_deg[3] = (deg3-deg3c)*PI/180

				#Calculate Rotation Matrix
				for x in range(0,n_links):
					dh[x] = dhmatrix(link_length[x],link_twist[x],link_offset[x],link_deg[x])
					#~ print "a %d" % x
					#~ print dh[x]

				#Calculate point w.r.t to world coordinate	
				result=numpy.dot(dh[n_links-1],point)

				for x in range(1,n_links):
					result=numpy.dot(dh[n_links-1-x],result)
				
				print "Resulting point: x: %.2f, y: %.2f, z: %.2f" % (result[0], result[1], result[2])
				
				confirm = raw_input('Confirm move servos (y/n):')
				
				if (confirm=="y"):
					Turn1(deg1*PI/180)
					Turn2(deg2*PI/180)
					Turn3(deg3*PI/180)						
					Turn4(deg4*PI/180)
					
			elif s=="pick":
				
					
				#Motion Constants
				probeInsert = 15
				probeInsertDuration = 0.5
				probeLift = 30
				probeLiftDuration = 0.5
				
				panel = raw_input('Panel number:')
				
				height = 90
				if panel=="1":
					#Move to start point
					xstart = 100.0
					ystart = 0.0
					zstart = height
				elif panel=="2":
					#Move to start point
					xstart = 115.0
					ystart = 0.0
					zstart = height
				elif panel=="3":
					#Move to start point
					xstart = 110.0
					ystart = 0.0
					zstart = height
				elif panel=="4":
					#Move to start point
					xstart = 115.0
					ystart = 0.0
					zstart = height
			
					
				P = numpy.zeros((3))
				P[0] = xstart
				P[1] = ystart
				P[2] = zstart
				
				movePick(P, fixedVariables2, probeInsert, probeInsertDuration, probeLift, probeLiftDuration)

			elif s=="insert":
				
				#Motion Constants
				probeInsert = -40
				probeInsertDuration = 1.2
				probeWithdraw = 30
				probeWithdrawHeight = 10
				probeWithdrawDuration = 0.5
				
				panel = raw_input('Panel number:')

				if panel=="1":
					#Move to start point
					Turn5(90*PI/180)
					xstart = 0
					ystart = -120.0
					zstart = 150
				elif panel=="2":
					#Move to start point
					Turn5(0)
					xstart = 0.0
					ystart = -122.0
					zstart = 145.0
				elif panel=="3":
					#Move to start point
					xstart = 110.0
					ystart = 0.0
					zstart = height
				elif panel=="4":
					#Move to start point
					xstart = 115.0
					ystart = 0.0
					zstart = height
			
					
				P = numpy.zeros((3))
				P[0] = xstart
				P[1] = ystart
				P[2] = zstart
				
				moveInsert(P, fixedVariables2, probeInsert, probeInsertDuration, probeWithdraw, probeWithdrawHeight, probeWithdrawDuration)
				
			elif s=="demo":

				#Move to start point
				
				xstart = 0.0
				ystart = -115.0
				zstart = 150.0
				
				xend = 0.0
				yend = -115.0
				zend = 100.0
				
				duration = 1.0
				viaPoints = 2
				divisions = 10

				P = numpy.zeros((3))
				P[0] = xstart
				P[1] = ystart
				P[2] = zstart
				
				move = movePoint(P, fixedVariables2)
				move = True
				if (move==True):
					
					confirm = raw_input('Start Motion? (y/n):')			
					if (confirm=="y"):
						
						startPoint = [xstart, ystart, zstart, 0]
						endPoint = [xend, yend, zend, duration]

						c = viaPointTrajectoryCalculation(startPoint,endPoint,duration,fixedVariables2,viaPoints)

						publishViaPointTrajectory(c,duration, divisions, viaPoints)
						
						time.sleep(1)
						
						defaultSpeed()
			
			elif s=="teleop":
				move = movePoint([publicX, publicY, publicZ], fixedVariables2)
				teleOp()
			
			
				
				
	except rospy.ROSInterruptException:
		pass
		
	
