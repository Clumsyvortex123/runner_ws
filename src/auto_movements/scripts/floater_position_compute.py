#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Float32

class FloaterPositionPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('floater_position_node', anonymous=True)

        # Create a publisher for the floater position
        self.floater_pub = rospy.Publisher('floater_position_topic', String, queue_size=10)

        # Variables to store the latest values using a dictionary
        self.values = {'y_down_value': 0.0, 'runner_limit_value': False}
        self.last_published_position = "default"

        # Subscribe to the runner_limit_topic and y_down_topic
        rospy.Subscriber('runner_limit_topic', Bool, self.runner_limit_callback)
        rospy.Subscriber('y_down_topic', Float32, self.y_down_callback)

        # Set the publishing rate (e.g., 1 Hz)
        self.rate = rospy.Rate(1)

    # Callback function to handle the received boolean value
    def runner_limit_callback(self, data):
        self.values['runner_limit_value'] = data.data

    # Callback function to handle the received float value
    def y_down_callback(self, data):
        self.values['y_down_value'] = data.data

    # Function to calculate floater position and publish it
    def calculate_and_publish_floater_position(self):
        y_down_value = self.values['y_down_value']
        runner_limit_value = self.values['runner_limit_value']

        if y_down_value < 70 and not runner_limit_value:
            self.last_published_position = "bottom"
        elif y_down_value > 70 and runner_limit_value:
            self.last_published_position = "top"
        else:
            self.last_published_position = "mid"

        # Publish the calculated floater position
        self.floater_pub.publish(self.last_published_position)
        rospy.loginfo("Floater Position: %s", self.last_published_position)

if __name__ == '__main__':
    try:
        floater_publisher = FloaterPositionPublisher()
        while not rospy.is_shutdown():
            floater_publisher.calculate_and_publish_floater_position()
            floater_publisher.rate.sleep()
    except rospy.ROSInterruptException:
        pass
