#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import serial

def serial_data_callback(data, pub):
    # Process the serial data received from Arduino
    # In this example, we assume the data is an integer value
    parsed_data = int(data)
    
    # Publish the parsed data to the "runner_limit_value" topic
    pub.publish(parsed_data)

def runner_limit_top():
    # Initialize ROS node
    rospy.init_node('runner_limit_top', anonymous=True)
    
    # Define the serial port settings (update the port accordingly)
    serial_port = '/dev/ttyUSB0'  # Update with the correct serial port
    baud_rate = 9600
    
    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    
    # Create a publisher for the "runner_limit_value" topic
    pub = rospy.Publisher('runner_limit_value', Int32, queue_size=10)
    
    # Set up a callback function for processing serial data
    serial_data_callback_partial = lambda data: serial_data_callback(data, pub)
    
    # Create a subscriber for the serial data
    rospy.Subscriber('serial_data', Int32, serial_data_callback_partial)
    
    # Spin the node to keep it active
    rospy.spin()

if __name__ == '__main__':
    try:
        runner_limit_top()
    except rospy.ROSInterruptException:
        pass
