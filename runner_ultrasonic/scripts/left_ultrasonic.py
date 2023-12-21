#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def left_ultra_callback(data):
    # Process data from left_ultra sensor if needed
    rospy.loginfo("left ultrasonic Data: %f", data.data)

def left_ultra_node():
    rospy.init_node('left_ultra_node', anonymous=True)
    rospy.Subscriber('left_topic', Float32, left_ultra_callback)
    rospy.spin()

if __name__ == '__main__':
    left_ultra_node()

