#!/usr/bin/env python3
import rospy
import time  # Import the time module for sleep
from std_msgs.msg import String
from std_msgs.msg import UInt32

on = False

def motor_subscriber():
    global pub1
    global on
    rospy.init_node('motor_subscriber', anonymous=True)
    pub1 = rospy.Publisher('motor_steps', UInt32, queue_size=5)
    
    # Sleep for 2 seconds
    time.sleep(2)
    
    # After 2 seconds, set 'on' to True and log a message
    on = True
    rospy.loginfo("Motor is now on")

    rospy.spin()

if __name__ == '__main__':
    motor_subscriber()