#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('test/chatter', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    num=0
    while not rospy.is_shutdown():
        num= num + 1.5
        rospy.loginfo(num) # out screeen
        pub.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
