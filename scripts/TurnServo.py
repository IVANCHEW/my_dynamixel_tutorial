#!/usr/bin/env python

import rospy
import math
import time
import numpy as numpy
from std_msgs.msg import Float64

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
		
		#~ if s<0:
			#~ print "CASE 4"
			#~ deg3 = -math.acos(d)
				
			#~ if s>0 and r<>0:
				#~ deg2 = (math.atan((s)/r) - math.atan((a3*math.sin(deg3))/
				#~ (a2+a3*math.cos(deg3))))
			#~ else:
				#~ deg2 = -(math.atan((s)/r) + math.atan((a3*math.sin(deg3))/
				#~ (a2+a3*math.cos(deg3))))
				
		if d<=1 and d>0:
			print "CASE 1"
			deg3 = math.atan(-1*math.sqrt(1-pow(d,2))/d)
			
			if s>0 and r<>0:
				#~ print "CASE s>0"
				deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
				(a2+a3*math.cos(-deg3)))
			elif r==0 and s>0:
				deg2 = PI/2
				#~ print "CASE r=0"
			else:
				#~ print "Case ELSE"
				#~ deg2 = -math.atan((a3*math.sin(-deg3))/
				#~ (a2+a3*math.cos(-deg3)))
				deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
				(a2+a3*math.cos(-deg3)))
				
			
		elif d==0:
			print "CASE 2"
			deg3 = - PI/2
			
			deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
			(a2+a3*math.cos(-deg3)))
			
		elif d>=-1 and d<0:
			print "CASE 3"
			deg3 = math.atan(-1*math.sqrt(1-pow(d,2))/d) - PI
			
			if s>0 and r<>0:
				#~ deg2 = (math.atan((s)/r) - math.atan((a3*math.sin(deg3))/
				#~ (a2+a3*math.cos(deg3))))
				deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
				(a2+a3*math.cos(-deg3)))
			else:
				#~ deg2 = -(math.atan((s)/r) + math.atan((a3*math.sin(deg3))/
				#~ (a2+a3*math.cos(deg3))))
				deg2 = math.atan((s)/r) + math.atan((a3*math.sin(-deg3))/
				(a2+a3*math.cos(-deg3)))
		
		
		
		output = numpy.zeros((3))
		output[0] = deg1*180/PI
		output[1] = deg2*180/PI
		output[2] = deg3*180/PI + deg3c
		#~ print "s: %.2f, d: %.2f, R: %.2f" % (s,d,R)
		return output
		
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
		
if __name__== '__main__':
	try:
		PI = math.pi
		pub4 = rospy.Publisher('/tilt_controller4/command', Float64, queue_size=10)
		pub1 = rospy.Publisher('/tilt_controller1/command', Float64, queue_size=10)
		pub2 = rospy.Publisher('/tilt_controller2/command', Float64, queue_size=10)
		pub3 = rospy.Publisher('/tilt_controller3/command', Float64, queue_size=10)
		rospy.init_node('TurnServo', anonymous=True)
		
		#Fixed
		deg0 = 0

		l1 = 89
		l2 = 0
		l3 = 67.5
		l4 = 100.552

		deg3c = 11.5949
		
		ystart = 80.0
		yend = -80.0
		xconstant = 70.0
		divisions = 10
				
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
				Turn1(0)
				Turn2(PI/2)
				Turn3(0)
				
			elif s=="start1":
				inverseAngles = inverseK(xconstant,ystart,0,l3,l4,(l1+l2), 0, deg3c)
				Turn1(inverseAngles[0]*PI/180)
				Turn2(inverseAngles[1]*PI/180)
				Turn3(inverseAngles[2]*PI/180)
				
			elif s=="move":

				#start of movement
				currenty = ystart				
				increment = float((yend - ystart)/divisions)
				
				for n in range(1,divisions):
					inverseAngles = inverseK(xconstant,currenty,0,l3,l4,(l1+l2), 0, deg3c)
					Turn1(inverseAngles[0]*PI/180)
					Turn2(inverseAngles[1]*PI/180)
					Turn3(inverseAngles[2]*PI/180)	
					print "%d Inverse Angles: deg1: %.2f, deg2: %.2f, deg3: %.2f" % (n, inverseAngles[0], inverseAngles[1], inverseAngles[2])
					currenty = currenty + increment
					time.sleep(0.0005)
							
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
		
	
