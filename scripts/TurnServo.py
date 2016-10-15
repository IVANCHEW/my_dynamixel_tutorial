#!/usr/bin/env python

import rospy
import math
import time
import numpy as numpy
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


def CloseClaw():
	d= -1.5	
	rospy.loginfo(numpy.float64(d))
	pub4.publish(numpy.float64(d))

def OpenClaw():
	d= 0
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
		deg0 = 0
		l1 = 89
		l2 = 0
		l3 = 67.5
		l4 = 100.552
		deg3c = 11.5949
		
		fixedVariables = numpy.zeros(5)
		fixedVariables[0] = l3
		fixedVariables[1] = l4
		fixedVariables[2] = l1+l2
		fixedVariables[3] = 0
		fixedVariables[4] = deg3c
				
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
				
			elif s=="move":
				
				ystart = 80.0
				yend = -80.0
				xconstant = 70.0
				zconstant = 5.0
				delay = 0.029
		
				p1 = [xconstant, ystart, zconstant]
				p2 = [xconstant, yend, zconstant]
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
				
				ystart = 80.0
				yend = -80.0
				xconstant = 70.0
				zconstant = 5.0
				
				p1 = [xconstant, ystart, zconstant]
				p2 = [xconstant, yend, zconstant]
				divisions = 100
				duration = 1
				timeStamp, jPositions = linearTrajectory(p1,p2,divisions,duration,fixedVariables)
				confirm = raw_input('Confirm move servos (y/n):')			
				if (confirm=="y"):
					publishTrajectory(jPositions, timeStamp)
				
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
				inverseAngles = inverseK(out[0],out[1],out[2],l3,l4,(l1+l2), 0, deg3c)
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
				
			
	except rospy.ROSInterruptException:
		pass
		
	
