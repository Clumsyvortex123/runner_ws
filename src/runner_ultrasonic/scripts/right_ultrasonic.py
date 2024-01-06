#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def right_ultra_callback(data):
    # Process data from right_ultra sensor if needed
    rospy.loginfo("right ultrasonic Data: %f", data.data)

def right_ultra_node():
    rospy.init_node('right_ultra_node', anonymous=True)
    rospy.Subscriber('right_topic', Float32, right_ultra_callback)
    rospy.spin()

if __name__ == '__main__':
    right_ultra_node()

