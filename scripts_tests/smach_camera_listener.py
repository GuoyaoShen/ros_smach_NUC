#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from my_folder_msgs.msg import human_joint, humans


# define state Listener
class Listener(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['listener_outcome1'],
                             input_keys=['humans'])
                             # input_keys=['x', 'y', 'z'])
        
    def execute(self, userdata):
        rate = rospy.Rate(2)
        rospy.loginfo('Executing state LISTENER')
        rospy.loginfo('VECTOR3: %s', userdata.humans)

        '''
        Here to add functions for different triggers and return with
        corresponding outcome. In the outcome States publish corresponding
        topics.
        '''


        # Publish topics
        pub = rospy.Publisher('humans_sm', humans, queue_size=10)
        pub.publish(userdata.humans)
        rospy.loginfo('NEW topic published')

        rate.sleep()
        return 'listener_outcome1'


def main():
    def subscriber_cb(data_raw, smdata):
        smdata.vector3 = data_raw

    rospy.init_node('smach_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_outcome'])
    rospy.Subscriber('joints_info', humans, subscriber_cb, callback_args=sm.userdata)

    # MUST NEED to first init the data you want to receive from other nodes
    sm.userdata.humans = humans()  # all initiate as 0 automatically

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
