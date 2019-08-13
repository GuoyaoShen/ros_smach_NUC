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
                             outcomes=['listener_outcome1'],)
                             # input_keys=['vector3'],
                             # output_keys=['vector3'])
        # input_keys=['x', 'y', 'z'])
        self._vector3 = Vector3()

    def execute(self, userdata):
        def subscriber_cb(data_raw):
            self._vector3 = data_raw

        rate = rospy.Rate(2)
        rospy.loginfo('Executing state LISTENER')
        # rospy.Subscriber('vector', Vector3, subscriber_cb, callback_args=self)
        rospy.Subscriber('vector', Vector3, subscriber_cb)

        rospy.loginfo('VECTOR3: %s', self._vector3)

        # Publish topics
        pub = rospy.Publisher('vector_sm', Vector3, queue_size=10)
        pub.publish(self._vector3)
        rospy.loginfo('NEW topic published')

        rate.sleep()
        return 'listener_outcome1'


def main():
    # def subscriber_cb(data_raw, smdata):
    #     smdata.vector3 = data_raw

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_outcome'])
    # rospy.Subscriber('vector', Vector3, subscriber_cb, callback_args=sm.userdata)

    # MUST NEED to first init the data you want to receive from other nodes
    # sm.userdata.vector3 = Vector3()  # all initiate as 0 automatically

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LISTENER', Listener(),
                               transitions={'listener_outcome1': 'LISTENER'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm, 'INTROSPECTION SERVER')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
