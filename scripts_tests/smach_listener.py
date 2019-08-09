#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


# define state Listener
class Listener(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['listener_outcome1'],
                             input_keys=['vector3'])
                             # input_keys=['x', 'y', 'z'])
        
    def execute(self, userdata):
        rate = rospy.Rate(2)
        rospy.loginfo('Executing state LISTENER')
        # rospy.loginfo('X: %s', userdata.x)
        # rospy.loginfo('Y: %s', userdata.y)
        # rospy.loginfo('Z: %s', userdata.z)
        rospy.loginfo('VECTOR3: %s', userdata.vector3)
        rate.sleep()
        return 'listener_outcome1'


def main():
    def subscriber_cb(data_raw, smdata):
        # smdata.x = data_raw.x
        # smdata.y = data_raw.y
        # smdata.z = data_raw.z
        smdata.vector3 = data_raw

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    # sm = smach.StateMachine(input_keys=['vector3'], outcomes=['sm_outcome'])  # NO need to pass in input_keys first!
    sm = smach.StateMachine(outcomes=['sm_outcome'])
    rospy.Subscriber('vector', Vector3, subscriber_cb, callback_args=sm.userdata)

    # MUST NEED to first init the data you want to receive from other nodes
    sm.userdata.vector3 = Vector3()  # all initiate as 0 automatically
    # sm.userdata.x = 0
    # sm.userdata.y = 0
    # sm.userdata.z = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LISTENER', Listener(), 
                               transitions={'listener_outcome1':'LISTENER'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm, 'INTROSPECTION SERVER')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
