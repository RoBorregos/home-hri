#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/colorInstruction', String, queue_size=10)
    rospy.init_node('colorTalker', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        instruction = input("Enter instruction (RED, GREEN, BLUE, ON, OFF, WHITE, RGB): ").strip().upper()
        color_str = instruction
        rospy.loginfo(color_str)
        pub.publish(color_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    