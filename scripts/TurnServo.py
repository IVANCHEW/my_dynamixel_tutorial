#!/usr/bin/env python

import rospy
import math
import time
import numpy as numpy
from numpy.linalg import inv
from std_msgs.msg import Float64

import roslib
import actionlib
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

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
		print "Time Stamp: %.2f, Positions: %.2f, %.2f, %.2f" % (timeStamp[i], positions[i][0], positions[i][1], positions[i][2])
	
	return timeStamp, positions

def publishTrajectory(positions, timeStamps):
	s = (positions.size)/3
	print s
	goal = FollowJointTrajectoryGoal()                  
	goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3']
	
	p = []
	
	for i in range(s):
		
		p.append(JointTrajectoryPoint())
		p[i].positions = [positions[i][0], positions[i][1], positions[i][2]]
		goal.trajectory.points.append(p[i])
		goal.trajectory.points[i].time_from_start = rospy.Duration(timeStamps[i])

	print goal

	client.send_goal_and_wait(goal)
	client.wait_for_result()

#Get Trajectory Constants for all three joints
def viaPointTrajectoryCalculation(Point1, Point2, Point3, constants):
	
	Ik1 = inverseK(Point1[0], Point1[1], Point1[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	Ik2 = inverseK(Point2[0], Point2[1], Point2[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	Ik3 = inverseK(Point3[0], Point3[1], Point3[2], constants[0], constants[1], constants[2], constants[3], constants[4])
	
	Ik1 = degToRad(Ik1)
	Ik2 = degToRad(Ik2)
	Ik3 = degToRad(Ik3)
	print "Inverse Angles, starting point:"
	print Ik1
	print "Inverse Angles, via point:"
	print Ik2
	print "Inverse Angles, Final point:"
	print Ik3
	
	t = numpy.zeros((3))
	t[0] = 0 
	t[1] = Point2[3]
	t[2] = Point3[3]
	
	P1 = numpy.zeros((7))
	P2 = numpy.zeros((7))
	P3 = numpy.zeros((7))
	
	P1[0] = Ik1[0]
	P1[1] = Ik2[0]
	P1[2] = Ik3[0]
	
	P2[0] = Ik1[1]
	P2[1] = Ik2[1]
	P2[2] = Ik3[1]
	
	P3[0] = Ik1[2]
	P3[1] = Ik2[2]
	P3[2] = Ik3[2]	
	
	A = numpy.zeros((7,7))
	A[0][0] = pow(t[0],6)
	A[0][1] = pow(t[0],5)
	A[0][2] = pow(t[0],4)
	A[0][3] = pow(t[0],3)
	A[0][4] = pow(t[0],2)
	A[0][5] = t[0]
	A[0][6] = 1
	
	A[1][0] = pow(t[1],6)
	A[1][1] = pow(t[1],5)
	A[1][2] = pow(t[1],4)
	A[1][3] = pow(t[1],3)
	A[1][4] = pow(t[1],2)
	A[1][5] = t[1]
	A[1][6] = 1
	
	A[2][0] = pow(t[2],6)
	A[2][1] = pow(t[2],5)
	A[2][2] = pow(t[2],4)
	A[2][3] = pow(t[2],3)
	A[2][4] = pow(t[2],2)
	A[2][5] = t[2]
	A[2][6] = 1
	
	A[3][0] = 6*pow(t[0],5)
	A[3][1] = 5*pow(t[0],4)
	A[3][2] = 4*pow(t[0],3)
	A[3][3] = 3*pow(t[0],2)
	A[3][4] = 2*t[0]
	A[3][5] = 1
	A[3][6] = 0
	
	A[4][0] = 6*pow(t[2],5)
	A[4][1] = 5*pow(t[2],4)
	A[4][2] = 4*pow(t[2],3)
	A[4][3] = 3*pow(t[2],2)
	A[4][4] = 2*t[2]
	A[4][5] = 1
	A[4][6] = 0
	
	A[5][0] = 30*pow(t[0],4)
	A[5][1] = 20*pow(t[0],3)
	A[5][2] = 12*pow(t[0],2)
	A[5][3] = 6*t[0]
	A[5][4] = 2
	A[5][5] = 0
	A[5][6] = 0
	
	A[6][0] = 30*pow(t[2],4)
	A[6][1] = 20*pow(t[2],3)
	A[6][2] = 12*pow(t[2],2)
	A[6][3] = 6*t[2]
	A[6][4] = 2
	A[6][5] = 0
	A[6][6] = 0
	
	Ainv = inv(A)
	
	print "Inverse Matrix:"
	print Ainv
	
	C1 = numpy.dot(Ainv, P1)
	C2 = numpy.dot(Ainv, P2)
	C3 = numpy.dot(Ainv, P3)
	
	print "P2:"
	print P2
	print "Coefficient for joint 2:"
	print C2
	
	return C1, C2, C3

#Ensures fork is parallel with the ground	
def testTrajectory(c, duration):
	
	goal = FollowJointTrajectoryGoal()                  
	goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3', 'tilt_joint4']
	#duration = 2.0
	duration = float(duration)
	divisions = 100
	currentTime = 0
	timeStep = float(duration/divisions)
	p = []
	v = []
	positions = numpy.zeros((100,4))
	velocities = numpy.zeros((100,4))
	print timeStep
	
	print "Constants"
	print c
	
	print "Duration: %.2f, Time Step: %.2f" % (duration, timeStep)
	
	for i in range(divisions):
		n=0
		positions[i][0] = c[n][0]*pow(currentTime,6) + c[n][1]*pow(currentTime,5) + c[n][2]*pow(currentTime,4) + c[n][3]*pow(currentTime,3) + c[n][4]*pow(currentTime,2) + c[n][5]*currentTime + c[n][6]
		n=1
		positions[i][1] = c[n][0]*pow(currentTime,6) + c[n][1]*pow(currentTime,5) + c[n][2]*pow(currentTime,4) + c[n][3]*pow(currentTime,3) + c[n][4]*pow(currentTime,2) + c[n][5]*currentTime + c[n][6]
		n=2
		positions[i][2] = c[n][0]*pow(currentTime,6) + c[n][1]*pow(currentTime,5) + c[n][2]*pow(currentTime,4) + c[n][3]*pow(currentTime,3) + c[n][4]*pow(currentTime,2) + c[n][5]*currentTime + c[n][6]
		n=3
		positions[i][3] = positions[i][1] + positions[i][2]
		n=0
		velocities[i][0] = 6*c[n][0]*pow(currentTime,5) + 5*c[n][1]*pow(currentTime,4) + 4*c[n][2]*pow(currentTime,3) + 3*c[n][3]*pow(currentTime,2) + 2*c[n][4]*currentTime + c[n][5]
		n=1
		velocities[i][1] = 6*c[n][0]*pow(currentTime,5) + 5*c[n][1]*pow(currentTime,4) + 4*c[n][2]*pow(currentTime,3) + 3*c[n][3]*pow(currentTime,2) + 2*c[n][4]*currentTime + c[n][5]
		n=2
		velocities[i][2] = 6*c[n][0]*pow(currentTime,5) + 5*c[n][1]*pow(currentTime,4) + 4*c[n][2]*pow(currentTime,3) + 3*c[n][3]*pow(currentTime,2) + 2*c[n][4]*currentTime + c[n][5]
		
		currentTime = timeStep + currentTime		
		
		#~ print "Current Time: %.2f" % currentTime
		
	print positions
	#~ print velocities
	
	currentTime = 0
	
	for i in range(divisions):
		
		#~ j1 = 0.3540*pow(currentTime,6) - 2.12215*pow(currentTime,5) + 4.2355*pow(currentTime,4) - 2.8152*pow(currentTime,3) + 1.1271
		#~ j2 = 0
		#~ w1 = 2.124*pow(currentTime,5) - 10.6075*pow(currentTime,4) + 16.942*pow(currentTime,3) - 8.4456*pow(currentTime,2)
		#~ w1 = 0
		p.append(JointTrajectoryPoint())
		p[i].positions = [0, positions[i][1], positions[i][2], positions[i][3]]
		p[i].velocities = [0, velocities[i][1], velocities[i][2], 0]
		goal.trajectory.points.append(p[i])
		goal.trajectory.points[i].time_from_start = rospy.Duration(currentTime)
		currentTime = timeStep + currentTime
		
	print goal.trajectory.points[0]
	print goal.trajectory.points[50]
	print goal.trajectory.points[99]
	
	client.send_goal_and_wait(goal)
	client.wait_for_result()
		

def CloseClaw():
	d= -0.03
	rospy.loginfo(numpy.float64(d))
	pub4.publish(numpy.float64(d))

def OpenClaw():
	d= 0.5
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
	
def moveTraj(p1,p2, duration, delay):
		
	divisions = 50
	
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
				
			return True
		
		else:
			
			return False
	else:
		
		return False
		
def moveTraj2(p1,p2, duration, delay):
		
	divisions = 50
	
	#Planning the trajectory
	timeStamp, jPositions = linearTrajectory(p1,p2,divisions,duration,fixedVariables)
	
	print jPositions[0]
	
	#Moves to start position
	publishAngles(jPositions[0])
	time.sleep(2)	
	
	#Begin the linear motion
	for n in range(1,divisions):
		publishAngles(jPositions[n])
		time.sleep(delay)
		
	return True

	
if __name__== '__main__':
	try:
		PI = math.pi
		pub4 = rospy.Publisher('/tilt_controller4/command', Float64, queue_size=10)
		pub1 = rospy.Publisher('/tilt_controller1/command', Float64, queue_size=10)
		pub2 = rospy.Publisher('/tilt_controller2/command', Float64, queue_size=10)
		pub3 = rospy.Publisher('/tilt_controller3/command', Float64, queue_size=10)
		rospy.init_node('TurnServo', anonymous=True)
		
		client  = actionlib.SimpleActionClient('/f_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		client.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')
		
		#Fixed
		#~ deg0 = 0
		#~ l1 = 89
		#~ l2 = 0
		#~ l3 = 67.5
		#~ l4 = 100.552
		#~ deg3c = 11.5949
		
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
		
		deg0 = 0
		l1 = 89
		l2 = 0
		l3 = 67.5
		l4 = 45
		deg3c = 0
		
		fixedVariables2 = numpy.zeros(5)
		fixedVariables2[0] = l3
		fixedVariables2[1] = l4
		fixedVariables2[2] = l1+l2
		fixedVariables2[3] = 0
		fixedVariables2[4] = deg3c
				
				
		while not rospy.is_shutdown():
			
			#All user inputs to be non-capitalised
			s = raw_input('--> ')
			if s=="close":
				CloseClaw()
				
			elif s=="open":
				OpenClaw()
				
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
				
			elif s=="zero":
				publishAngles([0,PI/2,0])
				
			elif s=="start1":				
				
				ystart = 80.0
				yend = -80.0
				xconstant = 70.0
				zconstant = 5.0
				delay = 0.029
				
				inverseAngles = inverseK(xconstant,ystart,5,l3,l4,(l1+l2), 0, deg3c)
				inverseAngles = degToRad(inverseAngles)
				publishAngles(inverseAngles)
				print "Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (inverseAngles[0], inverseAngles[1], inverseAngles[2])
			
			elif s=="smooth":
				
				xstart = 100.0
				xend = 160.0
				ystart = 0.0
				yend = 0.0
				zstart = 80.0
				zend = 80.0
				duration = 2
				delay = 0.029
				
				p1 = [xstart, ystart, zstart, 0]
				p2 = [130, 0, 80, duration/2]
				p3 = [xend, yend, zend, duration]
				
				
				c1, c2, c3 = viaPointTrajectoryCalculation(p1,p2,p3,fixedVariables)
				
				c = []
				c.append(c1)
				c.append(c2)
				c.append(c3)
				
				testTrajectory(c,duration)
				
			elif s=="picktest":
				
				OpenClaw()
				
				#~ xstart = 100.0
				#~ xend = 160.0
				
				#Panel 1
				xstart = 130
				xend = 140
				
				ystart = 0.0
				yend = 0.0
				zstart = 77.0
				zend = 77.0
				
				#Panel 2
				#~ xstart = 150
				#~ xend = 160
				
				#~ ystart = 0.0
				#~ yend = 0.0
				#~ zstart = 75.0
				#~ zend = 75.0
				
				delay = 0.029
				
				p1 = [xstart, ystart, zstart]
				p2 = [xend, yend, zend]
				
				#Move1: Move in to pick
				checkMove = moveTraj(p1,p2, 2.0, delay)
				
				if checkMove == True:
					
					confirm = raw_input('Close Claw (y/n):')			
					
					if (confirm=="y"):
						
						#Move 2: Close claw
						CloseClaw()
						
						p1 = p2
						
						#~ xend = 153.0
						
						#Panel 1
						xend = 140
						
						#Panel 2
						#~ xend = 160
						
						yend = 0.0
						zend = 110.0
						delay = 0.029
						
						p2 = [xend, yend, zend]
						
						#Move 3: Move up to separate from base
						checkMove = moveTraj(p1,p2, 0.5, delay)
						
						if checkMove == True:
							
							p1 = p2
							
							xend = 90.0
							yend = 0.0
							zend = 140.0
							
							p2 = [xend, yend, zend]
							
							#Move 4: Move to reposition
							moveTraj(p1,p2, 0.5, delay)
				
			elif s=="pick":
				
				OpenClaw()
				
				xstart = 100.0
				xend = 160.0
				ystart = 0.0
				yend = 0.0
				zstart = 80.0
				zend = 80.0
				delay = 0.029
				
				p1 = [xstart, ystart, zstart]
				p2 = [xend, yend, zend]
				
				#Move1: Move in to pick
				checkMove = moveTraj2(p1,p2, 2.0, delay)
								
				time.sleep(2)
				
				#Move 2: Close claw
				CloseClaw()
				
				time.sleep(2)
				
				p1 = p2
				
				xend = 160.0
				yend = 0.0
				zend = 110.0
				delay = 0.029
				
				p2 = [xend, yend, zend]
				
				#Move 3: Move up to separate from base
				checkMove = moveTraj2(p1,p2, 0.5, delay)

				p1 = p2
				
				xend = 90.0
				yend = 0.0
				zend = 140.0
				
				p2 = [xend, yend, zend]
				
				#Move 4: Move to reposition
				moveTraj2(p1,p2, 0.5, delay)
				
			elif s=="move":
				
				#~ ystart = 80.0
				#~ yend = -80.0
				#~ xconstant = 70.0
				#~ zconstant = 5.0
				#~ delay = 0.029
		
				#~ p1 = [xconstant, ystart, zconstant]
				#~ p2 = [xconstant, yend, zconstant]
				
				#Pick Motion
				#~ xstart = 100.0
				#~ xend = 160.0
				#~ ystart = 0.0
				#~ yend = 0.0
				#~ zstart = 80.0
				#~ zend = 80.0
				#~ delay = 0.029
				
				#Slot 1
				xstart = 150.0
				xend = 150.0
				ystart = 0.0
				yend = 0.0
				zstart = 120.0
				zend = 70.0
				delay = 0.029
				
				#Slot 2
				#~ xstart = 147.0
				#~ xend = 147.0
				#~ ystart = 0.0
				#~ yend = 0.0
				#~ zstart = 120.0
				#~ zend = 70.0
				#~ delay = 0.029
				
				
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
				
			elif s=="traj":
				
				ystart = -80.0
				yend = 80.0
				xstart = 80.0
				xend = 80.0
				zstart = 80.0
				zend = 80.0
				
				p1 = [xstart, ystart, zstart]
				p2 = [xend, yend, zend]
				divisions = 100
				duration = 1
				timeStamp, jPositions = linearTrajectory(p1,p2,divisions,duration,fixedVariables)
				confirm = raw_input('Confirm move servos (y/n):')			
				if (confirm=="y"):
					publishTrajectory(jPositions, timeStamp)
			
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
					deg4 = (deg2 + deg3)*PI/180
					Turn4(deg4)
					
			elif s=="parallel":
				#~ xstart = 90.0
				#~ xend = 110.0
				#~ ystart = 0.0
				#~ yend = 0.0
				#~ zstart = 80.0
				#~ zend = 80.0
				#~ duration = 2
				#~ delay = 0.029
				
				#~ p1 = [xstart, ystart, zstart, 0]
				#~ p2 = [100, 0, 80, duration/2]
				#~ p3 = [xend, yend, zend, duration]
				
				xstart = 110.0
				xend = 70.0
				ystart = 0.0
				yend = 0.0
				zstart = 80.0
				zend = 150.0
				duration = 2
				delay = 0.029
				
				p1 = [xstart, ystart, zstart, 0]
				p2 = [110, 0, 90, duration/2]
				p3 = [xend, yend, zend, duration]
				
				
				#~ xstart = 110,0
				#~ xend = 110.0
				#~ ystart = 0.0
				#~ yend = 0.0
				#~ zstart = 80.0
				#~ zend = 100.0
				#~ duration = 2
				#~ delay = 0.029
				
				#~ xmid = (xstart - xend)/2
				#~ ymid = (ystart - yend)/2
				#~ zmid = (zstart - zend)/2
				
				#~ p1 = [xstart, ystart, zstart, 0]
				#~ p2 = [xmid, ymid, zmid, duration/2]
				#~ p2 = [110,0,90,duration/2]
				#~ p3 = [xend, yend, zend, duration]
				
				
				c1, c2, c3 = viaPointTrajectoryCalculation(p1,p2,p3,fixedVariables2)
				
				c = []
				c.append(c1)
				c.append(c2)
				c.append(c3)
				
				testTrajectory(c,duration)
			
			
	except rospy.ROSInterruptException:
		pass
		
	
