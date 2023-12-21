#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

def warning_log_callback(data):
    # Callback function for the "warning_log" topic
    if data.data:
        rospy.logwarn("Received warning_log: True. Aborting process.")
    else:
        rospy.loginfo("Received warning_log: False. Proceeding to next condition.")

def y_down_flag_callback(data):
    # Callback function for the "y_down_flag" topic
    if data.data:
        rospy.logwarn("Received y_down_flag: True. Aborting process.")
    else:
        rospy.loginfo("Received y_down_flag: False. Executing process.")
        move_flag = Bool()
        move_flag.data = True
        pub.publish(move_flag)

def base_move_acknowledge_callback(data):
    # Callback function for the "base_move_acknowledge" topic
    if data.data:
        rospy.loginfo("Received base_move_acknowledge: True. Publishing to base_move_completed.")
        move_completed_flag = Bool()
        move_completed_flag.data = True
        pub_completed.publish(move_completed_flag)
    else:
        rospy.logwarn("Received base_move_acknowledge: False. Setting warning_log and error_log.")
        warning_log_msg = Bool()
        warning_log_msg.data = True
        pub_warning_log.publish(warning_log_msg)

        error_log_msg = String()
        error_log_msg.data = "base_motor acknowledge not received"
        pub_error_log.publish(error_log_msg)

def publish_base_move_flag():
    # Check conditions before publishing
    if not rospy.get_param('/warning_log', False) and not rospy.get_param('/y_down_flag', False):
        # Create a boolean message
        move_flag = Bool()
        move_flag.data = True  # Set the boolean value (change as needed)

        # Publish the boolean message
        pub.publish(move_flag)
        rospy.loginfo("Published to base_move_flag.")
    else:
        rospy.logwarn("Conditions not met. Skipping publishing.")

def main():
    # Initialize the ROS node
    rospy.init_node('base_move_publisher', anonymous=True)

    # Create a publisher for the "base_move_flag" topic
    global pub
    pub = rospy.Publisher('base_move_flag', Bool, queue_size=10)

    # Create publishers for "warning_log," "error_log," and "base_move_completed" topics
    global pub_warning_log
    pub_warning_log = rospy.Publisher('warning_log', Bool, queue_size=10)

    global pub_error_log
    pub_error_log = rospy.Publisher('error_log', String, queue_size=10)

    global pub_completed
    pub_completed = rospy.Publisher('base_move_completed', Bool, queue_size=10)

    # Subscribe to the topics
    rospy.Subscriber('warning_log', Bool, warning_log_callback)
    rospy.Subscriber('y_down_flag', Bool, y_down_flag_callback)
    rospy.Subscriber('base_move_acknowledge', Bool, base_move_acknowledge_callback)

    # Start the main loop
    rospy.spin()

if __name__ == '__main__':
    main()
