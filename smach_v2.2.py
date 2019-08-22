#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import itertools
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
            else:
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
                             outcomes=['default', 'hello', 'turn_away'])
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


class HumanWalking(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'walking_close', 'walking_close_fast', 'walking_away'])
        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

        self._humans = Humans()
        self._depthHistory = []

    # define callback function
    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._depth = self._closest_humanbody.depth

            if len(self._depthHistory) > 15:
                self._depthHistory.pop(0)
                self._depthHistory.append(self._depth)

                self._depthHistory = [k for k,g in itertools.groupby(self._depthHistory)]

                if len(self._depthHistory) > 1:
                    if all([self._depthHistory[i] < self._depthHistory[i+1] for i in range(len(self._depthHistory)-1)]):
                        rospy.loginfo('Human Move FURTHER')
                        return 'walking_away'
                    elif all([self._depthHistory[i] > self._depthHistory[i+1] for i in range(len(self._depthHistory)-1)]):
                        if (self._depthHistory[0]-self._depthHistory[-1]) > 10000:
                            rospy.loginfo('Human Move ClOSER FASTLY')
                            return 'walking_close_fast'
                        else:
                            rospy.loginfo('Human Move ClOSER')
                            return 'walking_close'
                    else:
                        rospy.loginfo("self._depthHistory = %s" %self._depthHistory)
                        rospy.loginfo('Human Not Closer or Further')
                        return 'default'
                else:
                    return 'default'
            else:
                self._depthHistory.append(self._depth)
                rospy.loginfo('Human Depth list not enough')
                return 'default'
        else:
            rospy.loginfo('INIT HUMAN MOVE STATE')
            return 'default'


class HumanHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'short', 'tall'])
        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

        self._humans = Humans()

    # define callback function
    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._depth = self._closest_humanbody.depth
            self._part_ls = self._closest_humanbody.part_id

            if  self._depth < 10000 and 0 in self._part_ls:
                nose_ind = self._part_ls.index(0)

                if self._closest_humanbody.joint_y[nose_ind] > 240:
                    return 'short'
                else:
                    return 'tall'
        return 'default'


class FacePose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'straight'])
        # init subscriber
        self._humans = Humans()
        self._depthHistory = []
        self._squareDet = False

        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

    # define callback function
    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._part_ls = self._closest_humanbody.part_id
            
            if (16 in self._part_ls and 17 in self._part_ls and 14 in self._part_ls and 15 in self._part_ls):
                rospy.loginfo('Human Face Squarely')
                return 'straight'
            else:
                rospy.loginfo('Human Face NOT Squarely')
                return 'default'

        else:
            rospy.loginfo('INIT FACE SQUARELY STATE')
            return 'default'


'''
may need more emotion results, if so, add them to outcome in init function and return in execute function
'''
class FaceEmotion(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'happy', 'sad'])
        # init subscriber

    # define callback function

    def execute(self, userdata):

        return 'default'


class FaceMimicSub(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['default', 'tilt', 'move_arm', 'move_close', 'move_far'])

        # init subscriber
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.cb_sub_humans)

        self._humans = Humans()
        self._depthHistory = []
    # define callback function
    def cb_sub_humans(self, data_raw):
        self._humans = data_raw

    def execute(self, userdata):
        if self._humans.human_body != []:
            self._closest_humanbody = self._humans.human_body[0]
            self._depth = self._closest_humanbody.depth
            self._part_ls = self._closest_humanbody.part_id

            # Trigger for close or far
            if len(self._depthHistory) > 15:
                self._depthHistory.pop(0)
                self._depthHistory.append(self._depth)

                self._depthHistory = [k for k,g in itertools.groupby(self._depthHistory)]

                if len(self._depthHistory) > 1:
                    if all([self._depthHistory[i] < self._depthHistory[i+1] for i in range(len(self._depthHistory)-1)]):
                        rospy.loginfo('Human Move FURTHER')
                        return 'move_far'
                    elif all([self._depthHistory[i] > self._depthHistory[i+1] for i in range(len(self._depthHistory)-1)]):
                        rospy.loginfo('Human Move ClOSER')
                        return 'move_close'
            else:
                self._depthHistory.append(self._depth)
                rospy.loginfo('Human Depth list not enough')
            
            # Trigger to check face tilt
            if (16 in self._part_ls and 17 in self._part_ls):
                Lear = [self._closest_humanbody.joint_x[self._part_ls.index(17)],
                        self._closest_humanbody.joint_y[self._part_ls.index(17)]]
                Rear = [self._closest_humanbody.joint_x[self._part_ls.index(16)],
                        self._closest_humanbody.joint_y[self._part_ls.index(16)]]

                tilt_angle = math.atan2(Lear[1]-Rear[1],Lear[0]-Rear[0])

                if abs(tilt_angle) > 0.2 and self._depth < 10000:
                    rospy.loginfo('Human Face Tilt')
                    return 'tilt'

            # Trigger for arm mimic
            if 2 in self._part_ls and 3 in self._part_ls:
                shoulder_ind = self._part_ls.index(2)
                elbow_ind = self._part_ls.index(3)

                shoulder = [self._closest_humanbody.joint_x[shoulder_ind], self._closest_humanbody.joint_y[shoulder_ind]]
                elbow = [self._closest_humanbody.joint_x[elbow_ind], self._closest_humanbody.joint_y[elbow_ind]]

                if (elbow[1] - shoulder[1]) < 0:
                    joint2_left = 3.14 + math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0]))

                else:
                    joint2_left = -(3.14 - math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0])))

                if joint2_left > -1.1:
                    rospy.loginfo("joint2_left %s" %joint2_left)
                    return 'move_arm'

            if 5 in self._part_ls and 6 in self._part_ls:
                shoulder_ind_r = self._part_ls.index(5)
                elbow_ind_r = self._part_ls.index(6)

                shoulder_r = [self._closest_humanbody.joint_x[shoulder_ind_r], self._closest_humanbody.joint_y[shoulder_ind_r]]
                elbow_r = [self._closest_humanbody.joint_x[elbow_ind_r], self._closest_humanbody.joint_y[elbow_ind_r]]
 
                joint2_right = math.atan2((elbow_r[1] - shoulder_r[1]),(elbow_r[0] - shoulder_r[0]))

                if joint2_right < 1.1:
                    rospy.loginfo("joint2_right %s" % joint2_right)
                    return 'move_arm'

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


class UF_Greeting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_Greeting'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)
    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("greeting")
        rospy.loginfo('TOPICS_GREETING published')
        return 'sent_UF_Greeting'       


class UF_GreetingChild(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_GreetingChild'], )
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("greeting_child")
        rospy.loginfo('TOPICS_SAY_HELLO published')
        return 'sent_UF_GreetingChild'


class UF_SayBye(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_SayBye'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)
    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("say_bye")
        rospy.loginfo('TOPICS_SAY_HELLO published')
        return 'sent_UF_GreetingChild'


class UF_MimicFace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_MimicFace'], )
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("mimic_face")
        rospy.loginfo('TOPICS_SAY_HELLO published')
        return 'sent_UF_MimicFace'


class UF_MimicFaceTilt(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_MimicFaceTilt'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("mimic_face_tilt")
        rospy.loginfo('TOPICS_MIMIC_FACE_TILT published')
        return 'sent_UF_MimicFaceTilt'
        

class UF_MimicArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_MimicArm'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("mimic_arm")
        rospy.loginfo('TOPICS_MIMIC_ARM published')
        return 'sent_UF_MimicArm'


class UF_MimicWaistClose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_MimicWaistClose'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("mimic_waistClose")
        rospy.loginfo('TOPICS_MIMIC_WAIST_CLOSE published')
        return 'sent_UF_MimicWaistClose'


class UF_MimicWaistFar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_MimicWaistFar'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("mimic_waistFar")
        rospy.loginfo('TOPICS_MIMIC_WAIST_FAR published')
        return 'sent_UF_MimicWaistFar'


class UF_Surprise(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_Surprise'], )
        # init publisher
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

    def execute(self, userdata):
        rate = rospy.Rate(20)
        self.pub.publish("surprise")
        rospy.loginfo('SURPRISE published')
        return 'sent_UF_Surprise'


class UF_Dance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sent_UF_Dance'], )
        
        self.pub = rospy.Publisher('/quori/PMA/Trigger', String, queue_size=1)

        # init param
        self.time_init = rospy.get_time()

    def execute(self, userdata):
        rate = rospy.Rate(20)
        time = rospy.get_time()

        if time - self.time_init > 60:
            self.time_init = time
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
        sm_con = smach.Concurrence(outcomes=['default', 'say_hello', 'greeting', 'greeting_child', 'say_bye',
                                             'mimic_face', 'surprise', 'dance'],
                                   default_outcome='default',
                                   outcome_map={'say_hello': {'HUMAN_DISTANCE': 'far', 'FACE_POSE': 'straight'},
                                                'greeting': {'HUMAN_DISTANCE': 'far', 'HUMAN_WALKING': 'walking_close'},
                                                'greeting_child': {'HUMAN_DISTANCE': 'close', 'FACE_POSE': 'straight',
                                                                   'HUMAN_HEIGHT': 'short'},
                                                'say_bye': {'HUMAN_DISTANCE': 'close', 'HUMAN_POSE': 'turn_away'},

                                                'mimic_face': {'HUMAN_DISTANCE': 'close', 'FACE_POSE': 'straight',
                                                               'HUMAN_HEIGHT': 'tall'},
# 
                                                'surprise': {'HUMAN_WALKING': 'walking_close_fast'},
                                                'dance': {'HUMAN_NUM': 'many'}})

        with sm_con:
            smach.Concurrence.add('HUMAN_DISTANCE', HumanDistance(), )
            smach.Concurrence.add('HUMAN_NUM', HumanNum(), )
            smach.Concurrence.add('HUMAN_WALKING', HumanWalking(), )
            smach.Concurrence.add('HUMAN_POSE', HumanPose(), )
            smach.Concurrence.add('HUMAN_HEIGHT', HumanHeight(), )
            smach.Concurrence.add('FACE_POSE', FacePose(), )
            smach.Concurrence.add('FACE_EMOTION', FaceEmotion(), )


        smach.StateMachine.add('SM_CON', sm_con, transitions={'default': 'INITCON',
                                                              'say_hello': 'UF_SAY_HELLO',
                                                              'greeting': 'UF_GREETING',
                                                              'greeting_child': 'UF_GREETING_CHILD',
                                                              'say_bye': 'UF_SAY_BYE',

                                                              'mimic_face': 'FACE_MIMIC_SUB',

                                                              'surprise': 'UF_SURPRISE',
                                                              'dance': 'UF_DANCE'})


        # add sub state for face mimic
        smach.StateMachine.add('FACE_MIMIC_SUB', FaceMimicSub(), transitions={'default': 'UF_MIMIC_FACE',
                                                                             'tilt': 'UF_MIMIC_FACE_TILT',
                                                                             'move_arm': 'UF_MIMIC_ARM',
                                                                             'move_close': 'UF_MIMIC_WAIST_CLOSE',
                                                                             'move_far': 'UF_MIMIC_WAIST_FAR'})

        # add lower level function states
        smach.StateMachine.add('LF_SLEEP', LF_Sleep(), transitions={'sent_LF_Sleep': 'INITCON'})
        smach.StateMachine.add('LF_WAKEUP', LF_Wakeup(), transitions={'sent_LF_Wakeup': 'SM_CON'})

        # add upper level function states
        smach.StateMachine.add('UF_SAY_HELLO', UF_SayHello(), transitions={'sent_UF_SayHello': 'INITCON'})
        smach.StateMachine.add('UF_GREETING', UF_Greeting(), transitions={'sent_UF_Greeting': 'INITCON'})
        smach.StateMachine.add('UF_GREETING_CHILD', UF_GreetingChild(), transitions={'sent_UF_GreetingChild': 'INITCON'})
        smach.StateMachine.add('UF_SAY_BYE', UF_SayBye(), transitions={'sent_UF_SayBye': 'INITCON'})

        smach.StateMachine.add('UF_MIMIC_FACE', UF_MimicFace(), transitions={'sent_UF_MimicFace': 'INITCON'})
        smach.StateMachine.add('UF_MIMIC_FACE_TILT', UF_MimicFaceTilt(), transitions={'sent_UF_MimicFaceTilt': 'INITCON'})
        smach.StateMachine.add('UF_MIMIC_ARM', UF_MimicArm(), transitions={'sent_UF_MimicArm': 'INITCON'})
        smach.StateMachine.add('UF_MIMIC_WAIST_CLOSE', UF_MimicWaistClose(), transitions={'sent_UF_MimicWaistClose': 'INITCON'})
        smach.StateMachine.add('UF_MIMIC_WAIST_FAR', UF_MimicWaistFar(), transitions={'sent_UF_MimicWaistFar': 'INITCON'})

        smach.StateMachine.add('UF_SURPRISE', UF_Surprise(), transitions={'sent_UF_Surprise': 'INITCON'})
        smach.StateMachine.add('UF_DANCE', UF_Dance(), transitions={'sent_UF_Dance': 'INITCON'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.Rate(10).sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
