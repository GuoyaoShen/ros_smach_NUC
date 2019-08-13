#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


# define state Listener
class Listen1(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['right', 'wrong'],)
                             # input_keys=['x'])
        self._x = 0

    def execute(self, userdata):
        def subscriber_cb(data_raw):
            self._x = data_raw.x
        rate = rospy.Rate(2)
        rospy.Subscriber('vector', Vector3, subscriber_cb)
        rospy.loginfo('LISTENER 1 X: %s' % self._x)
        rate.sleep()
        if self._x == 1:
            rospy.loginfo('LISTENER 1 RIGHT')
            return 'right'
        else:
            rospy.loginfo('LISTENER 1 WRONG')
            return 'wrong'

class Listen2(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['right', 'wrong'], )
        # input_keys=['x'])
        self._y = 0

    def execute(self, userdata):
        def subscriber_cb(data_raw):
            self._y = data_raw.y

        rate = rospy.Rate(2)
        rospy.Subscriber('vector', Vector3, subscriber_cb)
        rospy.loginfo('LISTENER 2 Y: %s' % self._y)
        rate.sleep()
        if self._y == 4:
            rospy.loginfo('LISTENER 2 RIGHT')
            return 'right'
        else:
            rospy.loginfo('LISTENER 2 WRONG')
            return 'wrong'

class Result(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['result'], )

    def execute(self, userdata):
        rospy.loginfo('Get Result Succeed!!!')
        return 'result'



def main():
    def subscriber_cb(data_raw, smdata):
        rospy.loginfo('SUBSCRIBE VECTOR3 %s' % data_raw)
        smdata.x = data_raw.x
        smdata.y = data_raw.y
        smdata.z = data_raw.z
        # smdata.vector3 = data_raw

    rospy.init_node('smach_example_state_machine')

    sm_top = smach.StateMachine(outcomes=['out1'])
    with sm_top:
        sm = smach.Concurrence(outcomes=['succeed', 'failed'],
                               # input_keys=['x'],
                               # output_keys=['x', 'y', 'z'],
                               default_outcome='failed',
                               outcome_map={'succeed': {'LISTENER1': 'right', 'LISTENER2': 'right'}})
        with sm:
            # Add states to the container
            smach.Concurrence.add('LISTENER1', Listen1(),)
                                  # remapping={'x': 'x'})
            smach.Concurrence.add('LISTENER2', Listen2(),)
                                  # remapping={'x': 'x'})

        smach.StateMachine.add('SM', sm, transitions={'succeed': 'RESULT', 'failed': 'SM'})
        smach.StateMachine.add('RESULT', Result(), transitions={'result': 'SM'})

    # sis = smach_ros.IntrospectionServer('introspection_server', sm_mother, 'INTROSPECTION SERVER')
    # sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    main()
