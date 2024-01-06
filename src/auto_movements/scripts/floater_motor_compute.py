#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool

def check_base(msg):
    global base_move_flag
    base_move_flag = msg.data

def check_floater_position(msg):
    global floater_position
    floater_position = msg.data

def acknowledge_callback(msg):
    global acknowledge_received
    acknowledge_received = msg.data

def check_warn(msg):
    if msg.data:
        rospy.logwarn("Warning detected, aborting")
        return
    else:
        # Your existing code for the main process
        base_move_sub = rospy.Subscriber('base_move_completed', Bool, check_base)
        rospy.loginfo('Waiting for base_move_completed')

        while not rospy.is_shutdown() and not base_move_flag:
            rospy.loginfo('Base move not over. Waiting...')
            rospy.sleep(1)

        if rospy.is_shutdown():
            return

        rospy.loginfo('Base move over -- setting floater_down_flag')
        y_down_pub = rospy.Publisher('floater_down_flag', Bool, queue_size=10)
        msg = Bool()
        msg.data = True
        y_down_pub.publish(msg)

        floater_position_sub = rospy.Subscriber('floater_position_topic', String, check_floater_position)
        rospy.loginfo('Waiting for floater_position_topic')

        while not rospy.is_shutdown() and not floater_position:
            rospy.loginfo('Waiting for floater position...')
            rospy.sleep(1)

        if rospy.is_shutdown():
            return

        if floater_position == 'top':
            rospy.loginfo('Floater is at the top -- setting floater_down_flag as TRUE')
            floater_down_pub = rospy.Publisher('floater_down_flag', Bool, queue_size=10)
            msg = Bool()
            msg.data = True
            floater_down_pub.publish(msg)
        elif floater_position == 'bottom':
            rospy.loginfo('Floater is at the bottom -- setting floater_up_flag as TRUE')
            floater_up_pub = rospy.Publisher('floater_up_flag', Bool, queue_size=10)
            msg = Bool()
            msg.data = True
            floater_up_pub.publish(msg)
        elif floater_position == 'mid':
            rospy.logerr("Error: Floater position is mid")
            warn_pub = rospy.Publisher('warning_log', Bool, queue_size=10)
            msg = Bool()
            msg.data = True
            warn_pub.publish(msg)
        else:
            rospy.logwarn('Unknown floater position')

        # Subscribe to 'floater_move_acknowledge' topic
        acknowledge_sub = rospy.Subscriber('floater_move_acknowledge', Bool, acknowledge_callback)
        rospy.loginfo('Waiting for acknowledgement...')

        # Wait for acknowledgement for 1 minute
        timeout = rospy.Time.now() + rospy.Duration(60)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if acknowledge_received:
                rospy.loginfo("Acknowledgment received. Floater motor completed.")
                floater_completed_pub = rospy.Publisher('floater_motor_completed', Bool, queue_size=10)
                msg = Bool()
                msg.data = True
                floater_completed_pub.publish(msg)

                # Restart the process
                restart_process()

            rospy.sleep(1)

        if not rospy.is_shutdown() and not acknowledge_received:
            rospy.logwarn("No acknowledgement received within 1 minute. Raise a warning.")
            warn_pub = rospy.Publisher('warning_log', Bool, queue_size=10)
            msg = Bool()
            msg.data = True
            warn_pub.publish(msg)

            # Restart the process
            restart_process()

def restart_process():
    # Perform any cleanup or reset necessary for restarting the process

    # Delay for a while before restarting to avoid immediate restart
    rospy.sleep(5)

    # Restart the main process
    main_process()

def main_process():
    global base_move_flag, floater_position, acknowledge_received

    base_move_flag = False
    floater_position = ''
    acknowledge_received = False

    # Your existing code for the main process

def main():
    rospy.init_node('floater_motor_compute', anonymous=True)
    sub = rospy.Subscriber('warning_log', Bool, check_warn)
    main_process()
    rospy.spin()

if __name__ == '__main__':
    try:
        base_move_flag = False
        floater_position = ''
        acknowledge_received = False
        main()
    except rospy.ROSInterruptException:
        pass
