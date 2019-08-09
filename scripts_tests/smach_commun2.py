#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String

# define state Talker
class Talker(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['talker_outcome1', 'talker_outcome2'],
                             input_keys = ['count_in'],
                             output_keys=['talker_msg_out', 'count_out'])
        #pub = rospy.Publisher('greeting', String, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        #rate = rospy.Rate(2)

    def execute(self, userdata):
        rospy.loginfo('Executing state TALKER')
        rate = rospy.Rate(2)
        userdata.count_out = userdata.count_in + 1

        str_greeting = "Hi there %s" % rospy.get_time()
        userdata.talker_msg_out = str_greeting
        rate.sleep()
        return 'talker_outcome1'

        # if userdata.count_in%2 == 0:
        #     str_greeting = "Hi there %s" % rospy.get_time()
        #     userdata.talker_msg_out = str_greeting
        #     rate.sleep()
        #     return 'talker_outcome1'
        # else:
        #     return 'talker_outcome2'


# define state Listener
class Listener(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['listener_outcome1'],
                             input_keys=['listener_msg_in'])
        #rospy.init_node('listener', anonymous=True)
        #rospy.Subscriber('greeting', String, queue_size=10)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state LISTENER')
        rospy.loginfo('I Heard: %s' % userdata.listener_msg_in)        
        return 'listener_outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_outcome'])
    sm.userdata.sm_msg = ''
    sm.userdata.count = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TALKER', Talker(), 
                               transitions={'talker_outcome1':'LISTENER',
                                            'talker_outcome2':'TALKER'},
                               remapping={'talker_msg_out':'sm_msg',
                                          'count_in':'count', 'count_out':'count'})
        smach.StateMachine.add('LISTENER', Listener(), 
                               transitions={'listener_outcome1':'TALKER'},
                               remapping={'listener_msg_in':'sm_msg'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm, 'INTROSPECTION SERVER')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
