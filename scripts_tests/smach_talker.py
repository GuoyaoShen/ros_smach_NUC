#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('vector', Vector3, queue_size=10)
    rospy.init_node('smach_talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    count = 0
    while not rospy.is_shutdown():
        vector = Vector3()
        if count%2 == 0:
            vector.x = 1
            vector.y = 2
            vector.z = 3
        else:
            vector.x = 4
            vector.y = 5
            vector.z = 6
        count += 1
        rospy.loginfo(vector)
        pub.publish(vector)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
