#!/usr/bin/env python

#import roslib; roslib.load_manifest('numpy_tutorials') #not sure why I need this
import rospy
from std_msgs.msg import String
import serial

ser = serial.Serial('/dev/ttyACM1', 9600)

def talker():
 while not rospy.is_shutdown():
   data= ser.read() # I have "hi" coming from the arduino as a test run over the serial port
   rospy.loginfo(data)
   pub.publish(String(data))


if __name__ == '__main__':
  try:
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    talker()
  except rospy.ROSInterruptException:
    pass
