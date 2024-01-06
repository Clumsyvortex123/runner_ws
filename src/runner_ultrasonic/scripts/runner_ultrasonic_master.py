
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import serial
import time

# Set the correct serial port and baudrate for your Arduino
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to your Arduino's serial port
BAUDRATE = 9600

def runner_ultraosnic_master():
    rospy.init_node('runner_ultraosnic_master', anonymous=True)

    # Create publishers for all three topics
    left_pub = rospy.Publisher('left_topic', Float32, queue_size=10)
    right_pub = rospy.Publisher('right_topic', Float32, queue_size=10)


    # Open serial connection to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        
    except serial.SerialException as e:
        return

    rate = rospy.Rate(1)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # Read data from Arduino
        try:
            data = ser.readline().decode('utf-8').rstrip()
            
            values = [float(val) for val in data.split()]
            
            # Publish data to the respective topics
            if len(values) == 3:
                left_pub.publish(values[0])
                right_pub.publish(values[1])
              
            else:
                rospy.logwarn("Invalid data received from Arduino.")

        except serial.SerialException as e:
            rospy.logerr(f"Error reading data from Arduino: {e}")

        rate.sleep()

    # Close serial connection before exiting
    ser.close()

if __name__ == '__main__':
    try:
        runner_ultraosnic_master()
    except rospy.ROSInterruptException:
        pass

