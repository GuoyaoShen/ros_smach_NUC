#!/usr/bin/env python
import pdb
import rospy
import math
import random
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
import time

from body_msgs.msg import Human_joints, Humans
import subprocess

import acceleration_filter as f

import smooth_mover as sm


#TODO:
#- add right base testing capability.
#- make sure things work
#- add a non syncing raw motor command test if the sensors are not mounted.

class test():

    def __init__(self):
        self.Hz = 100.0/2
        self.rate = rospy.Rate(self.Hz)
        self.dt = 1.0/self.Hz
        self.turret_acc_lim = 360 #degrees per second square
        self.turret_filter = f.acc_filter(self.turret_acc_lim)

        #Publisher
        self.pubCmd_LArm = rospy.Publisher("/quori/arm_left/raw_joint_goal", Vector3, queue_size=1)
        self.pubCmd_RArm = rospy.Publisher("/quori/arm_right/raw_joint_goal", Vector3, queue_size=1) 
        self.pubCmd_Turret = rospy.Publisher("/quori/base/cmd_diff", Vector3, queue_size=3)
        self.pubCmd_Waist = rospy.Publisher("/quori/waist/mode", Vector3, queue_size=3)

        #Subsriber
        rospy.Subscriber('/quori/base/pos_status',Float32, self.basePose_callback)
        rospy.Subscriber('/quori/base/vel_status',Vector3,self.sensor_vel_callback)
        rospy.Subscriber('/quori/PMA/Camera_info', Humans, self.Joint_callback)
        rospy.Subscriber('/quori/PMA/Trigger', String, self.Trigger_callback)
        rospy.Subscriber('/quori/waist/angle',Vector3,self.waist_angle_callback)

        # Initialize smooth mover functions for transitions
        self.left_arm_mover = sm.Smooth_Mover('arm_left')
        self.right_arm_mover = sm.Smooth_Mover('arm_right')

        self.sensor_left_vel=0
        self.sensor_right_vel=0
        self.sensor_turret_vel=0

        self.turret = 0
        self.closest_humanbody = Human_joints()
        self.turret_vel = 0  # target speed for turret
        self.center_x =0
        self.part_ls = []
        self.left_goal = 0
        self.right_goal =0
        self.depth = 0
        self.rangeDeterm = 30000
        self.human_number = 0
        self.waist_ang = 0
        self.first_flag = 0
        self.first_flag1 = 0
        self.turret_flag = 0
        self.elbow = [0,0]
        self.elbow_r = [0,0]
        self.Trigger_msg = String() 

        # For determining whether special versions of actions should be performed
        self.special_roll = 0
        # Number that roll  must equal or exceed to do special action
        self.min_special = 5


    def waist_angle_callback(self, msg):  # should delete in newer version
        self.waist_ang = msg.data


    def Trigger_callback(self, msg):
    	self.Trigger_msg = msg.data

        if msg.data == 'dance':
            self.Dance()

        # elif msg.data == 'shy':
        #     if self.turret > 0.5:
        #         while(self.turret < 0.9):
        #             v_goal = 40
        #             self.turret_vel = self.turret_filter.FilterCompute(v_goal,self.sensor_turret_vel,self.dt)

        #     else:
        #         while(self.turret > 0.1):
        #             v_goal = -40
        #             self.turret_vel = self.turret_filter.FilterCompute(v_goal,self.sensor_turret_vel,self.dt)


        elif msg.data == 'hello':
            self.SayHi()
        elif msg.data == 'mimic_face' or msg.data == 'mimic_face_tilt':
            print('face')
        elif msg.data == 'mimic_arm':
            print('mimic')
            self.micmic()
        elif msg.data == 'greeting_child':
            self.pubCmd_Waist.publish(Vector3(1.5, .2, 1))
            self.pubCmd_Waist.publish(Vector3(0, .2, 3))
            print('Greet child')
        elif msg.data == 'surprise':
            self.Surprise()
            print('surprise')
            # self.pubCmd_Waist.publish(Vector3(-0.15,0,0))
        elif msg.data == 'sleep':
            # print('Sleeping')
            time = rospy.get_time()
            self.pubCmd_Turret.publish(Vector3(0, 0, self.turret_vel))
            self.pubCmd_LArm.publish(Vector3(0.2, -1.3, 0))
            self.pubCmd_RArm.publish(Vector3(-0.2, 1.3, 0))
            self.pubCmd_Waist.publish(Vector3(1.5, .2, 1))
        elif msg.data == 'wakeup':
            self.pubCmd_Waist.publish(Vector3(0.0, .2, 1))



    def basePose_callback(self,msg):
        self.turret=msg.data
        print(self.Trigger_msg)
        if self.Trigger_msg == 'sleep':
        	self.turret_back()



    def sensor_vel_callback(self,msg):
        self.sensor_left_vel=msg.x
        self.sensor_right_vel=msg.y
        self.sensor_turret_vel=msg.z

    def Joint_callback(self,msg):
        self.closest_humanbody = msg.human_body[0]
        self.human_number = msg.human_number
        self.part_ls = self.closest_humanbody.part_id
        self.depth = self.closest_humanbody.depth

        if 2 in self.part_ls and 5 in self.part_ls:
            Rshoulder_ind = self.part_ls.index(2)
            Lshoulder_ind = self.part_ls.index(5)
            Rshoulder = [self.closest_humanbody.joint_x[Rshoulder_ind], self.closest_humanbody.joint_y[Rshoulder_ind]]
            Lshoulder = [self.closest_humanbody.joint_x[Lshoulder_ind], self.closest_humanbody.joint_y[Lshoulder_ind]]            
    
            bodyCenter_x = (Rshoulder[0]+Lshoulder[0])/2
            bodyCenter_y = (Rshoulder[1]+Lshoulder[1])/2
            self.center_x = bodyCenter_x

            self.person_follow()
            self.pubCmd_Turret.publish(Vector3(0, 0, self.turret_vel))


    def person_follow(self):
        if self.center_x > 400 and self.turret <0.6 and self.turret > 0.4:
            self.turret_flag = self.turret_flag +3
            self.turret_vel = 2*self.turret_flag
            if self.turret_vel>20:
                self.turret_flag = 10
                self.turret_vel = 20
        elif self.center_x < 260 and self.turret > 0.4 and self.turret <0.6:
            self.turret_flag = self.turret_flag -3
            self.turret_vel = 2*self.turret_flag
            if self.turret_vel<-20:
                self.turret_flag = -10
                self.turret_vel = -20
        
        elif self.turret >= 0.6 and self.center_x < 260:
            self.turret_vel = -10

        elif 0.4 >= self.turret  and self.center_x > 400:
        	self.turret_vel = 10
        elif self.center_x < 400 and self.center_x > 260:
            if self.turret_vel > 0:
                self.turret_flag = self.turret_flag -4
                self.turret_vel = 2*self.turret_flag
            elif self.turret_vel < 0:
                self.turret_flag = self.turret_flag +4
                self.turret_vel = 2*self.turret_flag
            if self.turret_vel<-18:
                self.turret_vel = -20
            if self.turret_vel>18:
                self.turret_vel = 20
            if abs(self.turret_vel)<5:
                self.turret_vel = 0
                self.turret_flag = 0
        # v_goal = self.turret_vel
        # self.turret_vel = self.turret_filter.FilterCompute(v_goal,self.sensor_turret_vel,self.dt)

    def turret_back(self):
        if self.turret > 0.57:
            self.turret_vel = -10
        elif self.turret <0.53:
            self.turret_vel = 10
        else:
            self.turret_vel = 0
        v_goal = self.turret_vel
        self.turret_vel = self.turret_filter.FilterCompute(v_goal,self.sensor_turret_vel,self.dt)

        self.pubCmd_Turret.publish(Vector3(0, 0, self.turret_vel))

    def micmic(self):
        eye_dist = 10.0
        eye_Pixeldist = abs(self.closest_humanbody.joint_x[self.part_ls.index(14)] - self.closest_humanbody.joint_x[self.part_ls.index(15)])
        arm_l = 40.0

        # left arm
        if 2 in self.part_ls and 3 in self.part_ls:
            shoulder_ind = self.part_ls.index(2)
            elbow_ind = self.part_ls.index(3)
    
            shoulder = [self.closest_humanbody.joint_x[shoulder_ind], self.closest_humanbody.joint_y[shoulder_ind]]
            elbow = [self.closest_humanbody.joint_x[elbow_ind], self.closest_humanbody.joint_y[elbow_ind]]

            e_diff = math.sqrt( ((elbow[0]-self.elbow[0])**2)+((elbow[1]-self.elbow[1])**2) )
            
            elbow_d = self.closest_humanbody.joint_d[elbow_ind]            

            if (elbow[1] - shoulder[1]) < 0:
                joint2_left = 3.14 + math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0]))
        
            else:
                joint2_left = -(3.14 - math.atan2((elbow[1] - shoulder[1]),(elbow[0] - shoulder[0])))   

            joint1_left = 0

            ### Uncomment these two lines to enable 2d mimick
            # self.left_goal1 = joint1_left
            # self.left_goal2 = joint2_left

            ### Comment out to do 2d mimicking start
            if 3 in self.part_ls and (e_diff > 20):  # this block is for 3D mimicing
                 # arm moves
                Lwrist_ind = self.part_ls.index(3)
                Lwrist_d = self.closest_humanbody.joint_d[Lwrist_ind]

                Lwrist = [self.closest_humanbody.joint_x[Lwrist_ind], self.closest_humanbody.joint_y[Lwrist_ind]]   

                self.elbow = [self.closest_humanbody.joint_x[elbow_ind], self.closest_humanbody.joint_y[elbow_ind]]

                y_diffReal = abs(Lwrist[1]-shoulder[1])*(eye_dist/eye_Pixeldist)
                x_diffReal = abs(Lwrist[0]-shoulder[0])*(eye_dist/eye_Pixeldist)  

                arm_project = math.sqrt(y_diffReal**2+(arm_l-x_diffReal)**2)   

                temp1 = y_diffReal/(arm_l-x_diffReal)
                temp2 = arm_project/arm_l   

                if Lwrist[1] > shoulder[1]+15:
                    joint2_left = -temp2*1.2
                    joint1_left = -(1.57-math.atan(temp1))

                elif Lwrist[1] < shoulder[1]-15:
                    joint2_left = -temp2*1.2
                    joint1_left = -1.57-(1.57-math.atan(temp1))

                else:
                    joint1_left = -1.57

                    joint2_left = -temp2*1.2

                self.left_goal2 = joint2_left
                self.left_goal1 = joint1_left
            ### Comment out to do 2d mimicking end
            
            # limit the motor position for coast safety
            if self.left_goal2 > 1.25:
                self.left_goal2 = 1.25
            
            if self.left_goal2 < -1.25:
                self.left_goal2= -1.25

            # if self.left_goal1 > 1.25:
            #     self.left_goal1 = 1.25
            
            # if self.left_goal1 < -1.25:
            #     self.left_goal1= -1.25


        # right arm
        if 5 in self.part_ls and 6 in self.part_ls:    
            shoulder_ind_r = self.part_ls.index(5)
            elbow_ind_r = self.part_ls.index(6)
            
            shoulder_r = [self.closest_humanbody.joint_x[shoulder_ind_r], self.closest_humanbody.joint_y[shoulder_ind_r]]
            elbow_r = [self.closest_humanbody.joint_x[elbow_ind_r], self.closest_humanbody.joint_y[elbow_ind_r]]

            e_diff = math.sqrt( ((elbow_r[0]-self.elbow_r[0])**2)+((elbow_r[1]-self.elbow_r[1])**2) )
 
            joint2_right =  math.atan2((elbow_r[1] - shoulder_r[1]),(elbow_r[0] - shoulder_r[0]))
            
            joint1_right = 0

            ### Uncomment these two lines to enable 2d mimicking
            # self.right_goal2 = -joint2_right  
            # self.right_goal1 = joint1_right  

            ### comment out to enable 2d mimicking start
            if 6 in self.part_ls and (e_diff>20):  # this block is for 3D mimicing
                Rwrist_ind = self.part_ls.index(6)
                Rwrist_d = self.closest_humanbody.joint_d[Rwrist_ind]

                Rwrist = [self.closest_humanbody.joint_x[Rwrist_ind], self.closest_humanbody.joint_y[Rwrist_ind]]  
                self.elbow_r = [self.closest_humanbody.joint_x[elbow_ind], self.closest_humanbody.joint_y[elbow_ind]] 

                y_diffReal = abs(Rwrist[1]-shoulder_r[1])*(eye_dist/eye_Pixeldist)
                x_diffReal = abs(Rwrist[0]-shoulder_r[0])*(eye_dist/eye_Pixeldist)  

                arm_project = math.sqrt(y_diffReal**2+(arm_l-x_diffReal)**2)   

                temp1 = y_diffReal/(arm_l-x_diffReal)
                temp2 = arm_project/arm_l   

                if temp1 > 1:
                    temp1 = 1

                if Rwrist[1] > shoulder_r[1]+15:
                    joint2_right = -temp2*1.2
                    joint1_right = -(1.57-math.atan(temp1))

                elif Rwrist[1] < shoulder_r[1]-15:
                    joint2_right = -temp2*1.2
                    joint1_right = -1.57-(1.57-math.atan(temp1))

                else:
                    joint1_right = -1.57

                    joint2_right = -temp2*1.2

                self.right_goal2 = joint2_right
                self.right_goal1 = joint1_right
            ### comment out to enable 2d mimicking end

            # limit the motor position for coast safety
            if self.right_goal2 > 1.25:
                self.right_goal2 = 1.25
            
            if self.right_goal2 < -1.25:
                self.right_goal2 = -1.25

        self.pubCmd_RArm.publish(Vector3(-self.left_goal1,-self.left_goal2,0))
        self.pubCmd_LArm.publish(Vector3(self.right_goal1,self.right_goal2,0))

    def SayHi(self):
        bag_path = '~/catkin_ws/src/quori_ros/quori_bringup/quori_bag/recorded_motions/'

        # Roll dice to see whether special version of wave should be done
        special_roll = random.randrange(10)
        if special_roll >= self.min_special:
            bag_file = bag_path + 'wave_special_1.bag'
        else:
            bag_file = bag_path + 'wave_new_1.bag'

        play_cmd = 'rosbag play '
        sub_output = subprocess.call(play_cmd + bag_file, shell=True)

    def Dance(self):
        bag_path = '~/catkin_ws/src/quori_ros/quori_bringup/quori_bag/recorded_motions/'

        # Roll dice to see whether special version of dance should be done
        self.special_roll = random.randrange(10)
        if self.special_roll >= self.min_special:
            bag_file = bag_path + 'dance_new_3.bag'
        else:
            bag_file = bag_path + 'dance_short_1.bag'

        play_cmd = 'rosbag play '
        sub_output = subprocess.call(play_cmd + bag_file, shell=True)   

    def Surprise(self):
        bag_path = '~/catkin_ws/src/quori_ros/quori_bringup/quori_bag/recorded_motions/'
        bag_file = bag_path + 'surprise_new_1.bag'

        play_cmd = 'rosbag play '
        sub_output = subprocess.call(play_cmd + bag_file, shell=True)   


if __name__ == '__main__':
    #pdb.set_trace()
    rospy.init_node('mimic_node')
    rospy.loginfo('body listen node started')

    test_space = test()
    test_space.rate.sleep()
    rospy.spin()