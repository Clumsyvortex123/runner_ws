#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
import serial
import time

class StepperMotorController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        rospy.init_node('move_base_motor_controller', anonymous=True)
        self.rate = rospy.Rate(10)
        self.is_execution_needed = False
        self.warning_log_flag = False
        rospy.Subscriber('/warning_log', Bool, self.warning_log_callback)
        rospy.Subscriber('/floater_up_flag', Bool, self.flag_callback)
        self.acknowledge_pub = rospy.Publisher('/floater_motor_acknowledge', Bool, queue_size=1)

    def warning_log_callback(self, data):
        self.warning_log_flag = data.data

    def flag_callback(self, data):
        if data.data and not self.warning_log_flag and not self.is_execution_needed:
            self.send_rotations(-5000)
            rospy.loginfo("Executed process.")
            self.is_execution_needed = True
            self.publish_acknowledge(True)

    def send_rotations(self, rotations):
        self.serial_port.write(rotations)

    def publish_acknowledge(self, value):
        self.acknowledge_pub.publish(value)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = StepperMotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
