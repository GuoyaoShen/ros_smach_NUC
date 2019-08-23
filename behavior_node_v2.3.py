#!/usr/bin/env python
import pdb
import rospy
import math
from sensor_msgs.msg import *
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Empty
#from std_msgs.msg import uint32

from geometry_msgs.msg import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import numpy as np
from numpy.linalg import inv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from body_msgs.msg import Human_joints, Humans
import cv2
import subprocess

#TODO:
#- add right base testing capability.
#- make sure things work
#- add a non syncing raw motor command test if the sensors are not mounted.

class test():

    def __init__(self):
        self.Hz = 100.0/2
        self.rate = rospy.Rate(self.Hz)

        #Publisher
        self.pubCmd_LArm = rospy.Publisher("/quori/arm_left/raw_joint_goal", Vector3, queue_size=1)
        self.pubCmd_RArm = rospy.Publisher("/quori/arm_right/raw_joint_goal", Vector3, queue_size=1) 
        self.pubCmd_Turret = rospy.Publisher("/quori/base/cmd_holo", Vector3, queue_size=3)
        self.pubCmd_Waist = rospy.Publisher("/quori/waist/joint_goal", Vector3, queue_size=3)


        #Subsriber
        #rospy.Subscriber('/joy',Joy,self.joy_callback)
        rospy.Subscriber('/quori/base/pos_status',Float32, self.basePose_callback)
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.Joint_callback)
        rospy.Subscriber('/quori/PMA/Trigger', String, self.Trigger_callback)
        rospy.Subscriber('/quori/waist/angle',Vector3,self.waist_angle_callback)
        # rospy.Subscriber('hello_msgs', String, self.Hello_callback)
        # rospy.Subscriber('dance_msgs', String, self.Dance_callback)

        self.turret = 0
        self.closest_humanbody = Human_joints()
        self.turret_goal = 0
        self.center_x =0
        self.part_ls = []
        self.left_goal = 0
        self.right_goal =0
        self.depth = 0
        self.rangeDeterm = 30000
        self.human_number = 0
        self.mimic_det = 0
        self.dance_det = 0
        self.hello_det = 0
        self.waist_ang = 0

    def waist_angle_callback(self, msg):
        self.waist_ang = msg.data

    def pub_waist_goal(self,goal):
        itr = (goal - self.waist_ang)/200;
        print('trouble')
        for gg in range(200):

            self.pubCmd_Waist.publish(Vector3(self.waist_ang+gg*itr, 0, 1))


    def Trigger_callback(self, msg):
        # self.person_follow()
        # self.pubCmd_Turret.publish(Vector3(0, 0, self.turret_goal))
        self.pubCmd_Waist.publish(Vector3(0, 0, 1))
        if msg.data == 'dance':
            #self.pub_waist_goal(-0.1)
            self.Dance()
        elif msg.data == 'hello':
            #self.pub_waist_goal(-0.1)
            self.SayHi()
        elif msg.data == 'mimic_face' or msg.data == 'mimic_face_tilt':
            #self.pub_waist_goal(-0.1)
            print('face')
        elif msg.data == 'mimic_arm':
            print('mimic')
            self.micmic()
        elif msg.data == 'greeting_child':
            #self.pub_waist_goal(0.3)
            #self.pub_waist_goal(-0.1)
            print('no surprise')
        elif msg.data == 'surprise':
            #self.pub_waist_goal(-0.1)
            self.pubCmd_Waist.publish(Vector3(-0.15,0,0))
        elif msg.data == 'sleep':
            print('wewewewewewew')
            #self.pub_waist_goal(-0.1)
            self.pubCmd_LArm.publish(Vector3(0.2, -1.3, 0))
            self.pubCmd_RArm.publish(Vector3(-0.2, 1.3, 0))
            #self.pubCmd_Waist.publish(Vector3(0, 0, 1))
        # print(msg.data)


    def basePose_callback(self,msg):
        self.turret=msg.data


    def Joint_callback(self,msg):
        self.closest_humanbody = msg.human_body[0]
        self.human_number = msg.human_number
        self.part_ls = self.closest_humanbody.part_id
        self.depth = self.closest_humanbody.depth
        # print(self.depth)


        if 2 in self.part_ls and 5 in self.part_ls:
            Rshoulder_ind = self.part_ls.index(2)
            Lshoulder_ind = self.part_ls.index(5)
            Rshoulder = [self.closest_humanbody.joint_x[Rshoulder_ind], self.closest_humanbody.joint_y[Rshoulder_ind]]
            Lshoulder = [self.closest_humanbody.joint_x[Lshoulder_ind], self.closest_humanbody.joint_y[Lshoulder_ind]]            
    
            bodyCenter_x = (Rshoulder[0]+Lshoulder[0])/2
            bodyCenter_y = (Rshoulder[1]+Lshoulder[1])/2
            self.center_x = bodyCenter_x
        #print(self.sensor_left_vel)

    def person_follow(self):
        if self.center_x > 400 and self.turret >0.19:
            self.turret_goal = -0.15
            #print("right")
        elif self.center_x < 260 and self.turret <0.68:
            self.turret_goal = 0.185
        #   print("left")
        else:
            self.turret_goal = 0
        #   print("stay")

    def micmic(self):
        #print("part_ls = ", self.part_ls )
        # print("here")
        if 2 in self.part_ls and 3 in self.part_ls:
            # print("here")
            shoulder_ind = self.part_ls.index(2)
            elbow_ind = self.part_ls.index(3)
    
            shoulder = [self.closest_humanbody.joint_x[shoulder_ind], self.closest_humanbody.joint_y[shoulder_ind]]
            elbow = [self.closest_humanbody.joint_x[elbow_ind], self.closest_humanbody.joint_y[elbow_ind]]
            
            #shoulder_d = self.closest_humanbody.joint_d[shoulder_ind]
            elbow_d = self.closest_humanbody.joint_d[elbow_ind]            

            if (elbow[1] - shoulder[1]) < 0:
                joint2_left = 3.14 + math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0]))
        
            else:
                joint2_left = -(3.14 - math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0])))   

            #print("joint2_left = ", joint2_left)
            
            self.left_goal = joint2_left
            
            
            if self.left_goal > 1.1:
                self.left_goal = 1.1
            
            if self.left_goal < -1.1:
                self.left_goal= -1.1

        if 5 in self.part_ls and 6 in self.part_ls:
            shoulder_ind_r = self.part_ls.index(5)
            elbow_ind_r = self.part_ls.index(6)
            
            shoulder_r = [self.closest_humanbody.joint_x[shoulder_ind_r], self.closest_humanbody.joint_y[shoulder_ind_r]]
            elbow_r = [self.closest_humanbody.joint_x[elbow_ind_r], self.closest_humanbody.joint_y[elbow_ind_r]]
 
            joint2_right =  math.atan2((elbow_r[1] - shoulder_r[1]),(elbow_r[0] - shoulder_r[0]))
            
            self.right_goal = joint2_right  
            
            if self.right_goal > 1.1:
                self.right_goal = 1.1
            
            if self.right_goal < -1.1:
                self.right_goal = -1.1

        self.pubCmd_RArm.publish(Vector3(0,-test_space.left_goal,0))
        self.pubCmd_LArm.publish(Vector3(0,-test_space.right_goal,0))

    def SayHi(self):
        bag_path = '~/catkin_ws/src/quori_ros/quori_bringup/quori_bag/recorded_motions/'
        bag_file = bag_path + 'wave_ggg.bag'

        play_cmd = 'rosbag play '
        sub_output = subprocess.call(play_cmd + bag_file, shell=True)

    def Dance(self):
        bag_path = '~/catkin_ws/src/quori_ros/quori_bringup/quori_bag/recorded_motions/'
        bag_file = bag_path + 'dance_1.bag'

        play_cmd = 'rosbag play '
        sub_output = subprocess.call(play_cmd + bag_file, shell=True)   


if __name__ == '__main__':
    #pdb.set_trace()
    rospy.init_node('mimic_node')
    rospy.loginfo('body listen node started')

    test_space = test()
    test_space.rate.sleep()
    rospy.spin()

    # while not rospy.is_shutdown():
        



    #   test_space.rate.sleep()
    #   rospy.spin()

