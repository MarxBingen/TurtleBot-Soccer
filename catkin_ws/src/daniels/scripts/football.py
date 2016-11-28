#!/usr/bin/env python

import rospy
import os
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

global joytwist

def start():
	global joytwist
	pub = rospy.Publisher('HumanTwistMsg', Twist, queue_size=5)
	rospy.Subscriber("joy", Joy, callback)
	rate=rospy.Rate(10)
	joytwist=Twist()
	while not rospy.is_shutdown():
		pub.publish(joytwist)
		rate.sleep()


def callback(data):
	global joytwist
	joytwist.linear.x = data.axes[1]
	joytwist.angular.z = data.axes[0]

if __name__ == '__main__':
	rospy.init_node('football', anonymous=True)
	start()
