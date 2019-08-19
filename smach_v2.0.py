#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3

## ===================================================================== ##
# import custom msg
from body_msgs.msg import Human_joints, Humans
## ===================================================================== ##


class InitCon(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['exist', 'wakeup', 'noone', 'sleep'])

        self._time_last = rospy.get_time()
        self._flag_human = 0
        self._flag_sleep = False

        # init subscriber
        rospy.Subscriber('/quori/PMA/Human_Num', Int32, self.init_cb)

    def init_cb(self, data_raw):
        self._flag_human = data_raw.data

    def execute(self, userdata):
        self._time = rospy.get_time()

        if self._flag_human == 1:
            if self._flag_sleep:
                self._flag_sleep = False
                rospy.loginfo('InitCon: HUMAN EXIST, WAKE UP...')
                return 'wakeup'
            else:
                rospy.loginfo('InitCon: HUMAN EXIST')
                return 'exist'
        else:
            if self._time - self._time_last > 3:
                self._time_last = self._time
                self._flag_sleep = True
                return 'sleep'
            elif self._time - self._time_last < 0.5:
                rospy.loginfo('InitCon: NO HUMAN')
                return 'noone'


# ====== define states for concurrence container ======
class HumanDistance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'close', 'far'])

        # init param
        self.closeDeterm = 20000
        self.farDeterm = 25000
        self.depth = -1.0
        self._humans = Humans()

        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):

        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self.depth = self._closest_humanbody.depth
            print("depth =", self.depth)

            if (self.depth < self.closeDeterm) and (self.depth > 0.0):
                rospy.loginfo('HumanDistance CLOSE')

                return 'close'
            elif (self.depth > self.farDeterm) and (self.depth > 0.0):
                rospy.loginfo('HumanDistance FAR')
                return 'far'
            else:
                rospy.loginfo('HumanDistance DEFAULT (MID)')
                return 'default'
        else:
            rospy.loginfo('INIT HUMAN DISTANCE STATES')
            return 'default'


class HumanPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'hello'])
        self._humans = Humans()

        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._part_ls = self._closest_humanbody.part_id

            # determine hello
            if (1 in self._part_ls and 4 in self._part_ls):
                neck_ind = self._part_ls.index(1)
                hand_ind = self._part_ls.index(4)

                neck = [self._closest_humanbody.joint_x[neck_ind], self._closest_humanbody.joint_y[neck_ind]]
                hand = [self._closest_humanbody.joint_x[hand_ind], self._closest_humanbody.joint_y[hand_ind]]

                if neck[1] > hand[1]:
                    rospy.loginfo('HumanDistance HELLO')
                    return 'hello'
                else:
                    return 'default'
            else:
                return 'default'
        else:
            rospy.loginfo('INIT HUMAN POSE STATES')
            return 'default'


class HumanNum(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'many', 'noone'])
        self._humans = Humans()

        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._numHuman = self._humans.human_number
            if self._numHuman > 2:
                rospy.loginfo('HumanNum MANY')
                return 'many'
            elif self._numHuman == 0:
                rospy.loginfo('HumanNum NOONE')
                return 'noone'
            else:
                rospy.loginfo('HumanNum DEFAULT (MID)')
                return 'default'
        else:
            rospy.loginfo('INIT HUMAN NUM STATES')
            return 'default'


# ====== define states for Lower level Behavior Functions ======
class LF_Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_LF_Sleep'], )

        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish('sleep')
        rospy.loginfo('TOPICS_SLEEP published')
        return 'sent_LF_Sleep'

class LF_Wakeup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_LF_Wakeup'], )

        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish('wakeup')
        rospy.loginfo('TOPICS_WAKEUP published')
        return 'sent_LF_Wakeup'


# ====== define states for Upper level Behavior Functions ======
class UF_PoseMimic(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_PoseMimic'], )

        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish('mimic')
        rospy.loginfo('TOPICS_POSE_MIMIC published')
        return 'sent_UF_PoseMimic'


class UF_SayHello(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_SayHello'], )
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("hello")
        rospy.loginfo('TOPICS_SAY_HELLO published')
        return 'sent_UF_SayHello'


class UF_Dance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_Dance'], )
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish('dance')
        rospy.loginfo('TOPICS_DANCE published')
        return 'sent_UF_Dance'


def main():
    # init node
    rospy.init_node('smach_state_machine')
    print('HELLO')

    # construct state machine
    sm_top = smach.StateMachine(outcomes=['out1'])

    with sm_top:
        smach.StateMachine.add('INITCON', InitCon(), transitions={'exist': 'SM_CON',
                                                                  'wakeup': 'LF_WAKEUP',
                                                                  'noone': 'INITCON',
                                                                  'sleep': 'LF_SLEEP'})



        # add concurrence container
        sm_con = smach.Concurrence(outcomes=['default', 'mimic', 'say_hello', 'dance'],
                                   # input_keys = ['humans'],
                                   default_outcome='default',
                                   outcome_map={'mimic': {'HUMAN_DISTANCE': 'close', 'HUMAN_POSE': 'default'},
                                                'say_hello': {'HUMAN_DISTANCE': 'far', 'HUMAN_POSE': 'hello'},
                                                'dance': {'HUMAN_NUM': 'many'}})

        with sm_con:
            smach.Concurrence.add('HUMAN_DISTANCE', HumanDistance(), )
            smach.Concurrence.add('HUMAN_POSE', HumanPose(), )
            smach.Concurrence.add('HUMAN_NUM', HumanNum(), )

        smach.StateMachine.add('SM_CON', sm_con, transitions={'default': 'INITCON',
                                                              'mimic': 'UF_POSE_MIMIC',
                                                              'say_hello': 'UF_SAY_HELLO',
                                                              'dance': 'UF_DANCE'})

        # add lower level function states
        smach.StateMachine.add('LF_SLEEP', LF_Sleep(), transitions={'sent_LF_Sleep': 'INITCON'})
        smach.StateMachine.add('LF_WAKEUP', LF_Wakeup(), transitions={'sent_LF_Wakeup': 'SM_CON'})

        # add upper level function states
        smach.StateMachine.add('UF_POSE_MIMIC', UF_PoseMimic(), transitions={'sent_UF_PoseMimic': 'INITCON'})
        smach.StateMachine.add('UF_SAY_HELLO', UF_SayHello(), transitions={'sent_UF_SayHello': 'INITCON'})
        smach.StateMachine.add('UF_DANCE', UF_Dance(), transitions={'sent_UF_Dance': 'INITCON'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.Rate(10).sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
