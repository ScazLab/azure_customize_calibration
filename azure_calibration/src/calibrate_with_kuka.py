#!/usr/bin/env python

#import numpy as np

#import rospy

#from sensor_msgs.msg import JointState

#from control_wrapper.srv import SetJoints

#def robot_topic(topic):
    #robot = "kuka"
    #side = "left"
    
    #return "/{}/control_wrapper/{}/{}".format(robot, side, topic)

#class Robot(object):
    #def __init__(self):
        #self.angle_joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
    
    #def set_robot_angle(self, joint_angles): # angles is in degree
        #is_reached = False

        #try:
            #service_topic = robot_topic("set_joints")
            #rospy.wait_for_service(service_topic)
            #set_current_joints = rospy.ServiceProxy(service_topic, SetJoints)
                    
            #joints = JointState()
            #joints.name = self.angle_joint_names
            #joints.position = [np.deg2rad(angle) for angle in joint_angles]
            #is_reached = set_current_joints(joints).is_reached
           
        #except rospy.ServiceException as exc:
            #print "Service did not process request: " + str(exc)
        
        #return is_reached
        
import threading
import random

import numpy as np

import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Quaternion

from control_wrapper.srv import GetPose

#from meiying_crow_tool.srv import RobotArmController, RobotArmControllerRequest, RobotEndEffectorTransform
from meiying_crow_tool.msg import TargetPositions

from meiying_crow_tool import transformation_util
from meiying_crow_tool.robot_util import Robot

from meiying_crow_tool import constants

def get_Tworld_ee(Tworld_tool):
    return np.matmul(Tworld_tool, transformation_util.get_transformation_matrix_inverse(Tee_tool))

def neutral(theta=0.0):
    print "\tneutral"
    Tworld_ee = get_Tworld_ee(Tworld_tool_neutral)
    robot.set_robot_pose(Tworld_ee, ask_before_move=True)

def left(theta=20.0):
    print "\tleft"
    theta = np.deg2rad(theta)
    T = np.array([[np.cos(theta), -np.sin(theta), 0.0, 0.0],
                  [np.sin(theta),  np.cos(theta), 0.0, 0.0],
                  [0.0,            0.0,           1.0, 0.0],
                  [0.0,            0.0,           0.0, 1.0]])
    Tworld_tool_left = np.matmul(Tworld_tool_neutral, T)
    Tworld_ee = get_Tworld_ee(Tworld_tool_left)
    robot.set_robot_pose(Tworld_ee, ask_before_move=True)

def right(theta=-20.0):
    print "\tright"
    theta = np.deg2rad(theta)
    T = np.array([[np.cos(theta), -np.sin(theta), 0.0, 0.0],
                  [np.sin(theta),  np.cos(theta), 0.0, 0.0],
                  [0.0,            0.0,           1.0, 0.0],
                  [0.0,            0.0,           0.0, 1.0]])
    Tworld_tool_left = np.matmul(Tworld_tool_neutral, T)
    Tworld_ee = get_Tworld_ee(Tworld_tool_left)
    robot.set_robot_pose(Tworld_ee, ask_before_move=True)

def backward(theta=20.0):
    print "\tbackward"
    theta = np.deg2rad(theta)
    T = np.array([[1.0, 0.0,            0.0,           0.0],
                  [0.0, np.cos(theta), -np.sin(theta), 0.0],
                  [0.0, np.sin(theta),  np.cos(theta), 0.0],
                  [0.0, 0.0,            0.0,           1.0]])
    Tworld_tool_left = np.matmul(Tworld_tool_neutral, T)
    Tworld_ee = get_Tworld_ee(Tworld_tool_left)
    robot.set_robot_pose(Tworld_ee, ask_before_move=True)

def forward(theta=-20.0):
    print "\tforward"
    theta = np.deg2rad(theta)
    T = np.array([[1.0, 0.0,            0.0,           0.0],
                  [0.0, np.cos(theta), -np.sin(theta), 0.0],
                  [0.0, np.sin(theta),  np.cos(theta), 0.0],
                  [0.0, 0.0,            0.0,           1.0]])
    Tworld_tool_left = np.matmul(Tworld_tool_neutral, T)
    Tworld_ee = get_Tworld_ee(Tworld_tool_left)
    robot.set_robot_pose(Tworld_ee, ask_before_move=True)

if __name__ == '__main__':
    try:
        rospy.init_node('calibrate_azure', anonymous=True)
        
        Tworld_ee_neutral = transformation_util.get_homogeneous_transformation_matrix_from_quaternion(np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0247112845534, -0.0187448771399, 0.351088176064]))
        Tee_tool = transformation_util.get_homogeneous_transformation_matrix_from_quaternion(np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.3]))
        
        Tworld_tool_neutral = np.matmul(Tworld_ee_neutral, Tee_tool)
        
        robot = Robot()        
        
        group_5_num = 0
        group_9_num = 0
        
        group_5 = [neutral, left, right, backward, forward]
        group_5_thetas = [0.0, 20.0, -20.0, 20.0, -20.0]
        
        group_9 = [neutral, left, left, right, right, backward, backward, forward, forward]
        group_9_thetas = [0.0, 10.0, 20.0, -10.0, -20.0, 10.0, 20.0, -10.0, -20.0]        

        #group_5 = []
        #group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 76.1184507042]) # middle
        #group_5.append([257.8275, 116.439807692, -283.419, 174.63056338, 76.1184507042]) # forward
        #group_5.append([257.8275, 116.439807692, -283.419, 204.024788732, 76.1184507042]) # backward
        #group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 51.4805633803]) # left
        #group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 101.376197183]) # right
        
        #group_9 = []
        #group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 76.1184507042]) # middle
        #group_9.append([257.8275, 116.439807692, -283.419, 181.516478873, 76.1184507042]) # forward 1
        #group_9.append([257.8275, 116.439807692, -283.419, 174.63056338, 76.1184507042]) # forward 2
        #group_9.append([257.8275, 116.439807692, -283.419, 196.516478873, 76.1184507042]) # backward 1
        #group_9.append([257.8275, 116.439807692, -283.419, 204.016478873, 76.1184507042]) # backward 2
        #group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 63.6184507042]) # left 1
        #group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 51.1184507042]) # left 2
        #group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 88.6184507042]) # right 1
        #group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 101.1184507042]) # right 2

        robot = Robot()
        
        while not rospy.is_shutdown():
            group_choice = []
            group_theta_choice = []
            num = raw_input("5 or 9 samples per group? ")
            if num == "5":
                print "choose group 5"
                group_choice = group_5
                group_theta_choice = group_5_thetas
                group_5_num += 1
            elif num == "9":
                print "choose group 9"
                group_choice = group_9
                group_theta_choice = group_9_thetas
                group_9_num += 1
            
            for i in range(len(group_choice)):
                group_choice[i](group_theta_choice[i])
            #for angles in group_choice:
                #raw_input("press any key to move the robot...")
                #robot.set_robot_angle(angles)
            
            print "group 5 num: {}".format(group_5_num)
            print "group 9 num: {}".format(group_9_num)
            print "num samples: {}".format(group_5_num * 5 + group_9_num* 9)
            print "---------------------------------------------------"

    except rospy.ROSInterruptException:
        pass
