#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def joystick_callback(data):
    twist = Twist()
    twist.linear.x = data.linear.x
    twist.angular.z = data.angular.z
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('laptop')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/joystick', Twist, joystick_callback)
    rospy.spin()
