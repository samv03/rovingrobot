#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8

def callback(data):
    joystick = data.data
    x = joystick & 0b00001111
    y = (joystick & 0b11110000) >> 4
    rospy.loginfo(rospy.get_caller_id() + "X: %s", x)
    rospy.loginfo(rospy.get_caller_id() + "Y: %s", y)

    

def listener():
    rospy.init_node('motorcontrol', anonymous=True)
    rospy.Subscriber('controller_input2', UInt8, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
