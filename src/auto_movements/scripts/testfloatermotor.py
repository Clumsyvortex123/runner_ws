#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String

def test_floater_node():
    rospy.init_node('test_floater_node', anonymous=True)

    # Simulate warning
    warn_pub = rospy.Publisher('warning_log', Bool, queue_size=10)
    warn_msg = Bool()
    warn_msg.data = True
    rospy.loginfo('Publishing warning message...')
    warn_pub.publish(warn_msg)
    rospy.sleep(1)

    # Simulate base move completed
    base_move_pub = rospy.Publisher('base_move_completed', Bool, queue_size=10)
    base_move_msg = Bool()
    base_move_msg.data = True
    rospy.loginfo('Publishing base move completed message...')
    base_move_pub.publish(base_move_msg)
    rospy.sleep(1)

    # Simulate floater position
    floater_position_pub = rospy.Publisher('floater_position_topic', String, queue_size=10)
    floater_position_msg = String()
    
    # Test 1: Top position
    floater_position_msg.data = 'top'
    rospy.loginfo('Publishing floater position (top) message...')
    floater_position_pub.publish(floater_position_msg)
    rospy.sleep(1)

    # Test 2: Bottom position
    floater_position_msg.data = 'bottom'
    rospy.loginfo('Publishing floater position (bottom) message...')
    floater_position_pub.publish(floater_position_msg)
    rospy.sleep(1)

    # Test 3: Mid position (should trigger warning)
    floater_position_msg.data = 'mid'
    rospy.loginfo('Publishing floater position (mid) message...')
    floater_position_pub.publish(floater_position_msg)
    rospy.sleep(1)

    # Test 4: Unknown position (should log warning)
    floater_position_msg.data = 'unknown'
    rospy.loginfo('Publishing unknown floater position message...')
    floater_position_pub.publish(floater_position_msg)
    rospy.sleep(1)

    rospy.loginfo('Tests completed. Shutting down.')

if __name__ == '__main__':
    try:
        test_floater_node()
    except rospy.ROSInterruptException:
        pass
