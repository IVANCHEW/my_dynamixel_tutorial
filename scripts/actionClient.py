#!/usr/bin/env python
import roslib
roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import math


class Joint:
	def __init__(self, motor_name):
		#arm_name should be b_arm or f_arm
		self.name = motor_name           
		self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')

		
	def move_joint(self, angles):
		goal = FollowJointTrajectoryGoal()                  
		char = self.name[0] #either 'f' or 'b'
		goal.trajectory.joint_names = ['tilt_joint1', 'tilt_joint2','tilt_joint3']
		point1 = JointTrajectoryPoint()
		point2 = JointTrajectoryPoint()
		point3 = JointTrajectoryPoint()
		
		point1.positions = [0.,0.3059,-1.518]
		point2.positions = [0.,PI/2,0.]
		point3.positions = [0.85196,0.1267,-1.093]
		
		goal.trajectory.points = [point1, point2, point3]
		goal.trajectory.points[0].time_from_start = rospy.Duration(1.0)
		goal.trajectory.points[1].time_from_start = rospy.Duration(3.0)
		goal.trajectory.points[2].time_from_start = rospy.Duration(5.0)
		self.jta.send_goal_and_wait(goal)
		
		#~ goal.trajectory.points[0].time_from_start = rospy.Duration(2.0)
        #~ goal.trajectory.points[1].time_from_start = rospy.Duration(4.0)

        #~ goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
 
		#~ point.time_from_start = rospy.Duration(2)                   
		#~ goal.trajectory.points.append(point)
		
              

def main():
	arm = Joint('f_arm')
	arm.move_joint([0.,0.3059,-1.518])
	#~ arm.move_joint([0.,PI/2,0.])

                        
if __name__ == '__main__':
	PI = math.pi
	rospy.init_node('joint_position_tester')
	main()
