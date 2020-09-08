#!/usr/bin/env python

import numpy as np

import rospy

from sensor_msgs.msg import JointState

from control_wrapper.srv import SetJoints

def robot_topic(topic):
    robot = "kuka"
    side = "left"
    
    return "/{}/control_wrapper/{}/{}".format(robot, side, topic)

class Robot(object):
    def __init__(self):
        self.angle_joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
    
    def set_robot_angle(self, joint_angles): # angles is in degree
        is_reached = False

        try:
            service_topic = robot_topic("set_joints")
            rospy.wait_for_service(service_topic)
            set_current_joints = rospy.ServiceProxy(service_topic, SetJoints)
                    
            joints = JointState()
            joints.name = self.angle_joint_names
            joints.position = [np.deg2rad(angle) for angle in joint_angles]
            is_reached = set_current_joints(joints).is_reached
           
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
        
        return is_reached

if __name__ == '__main__':
    try:
        rospy.init_node('calibrate_azure', anonymous=True)
        group_5_num = 0
        group_9_num = 0

        group_5 = []
        group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 76.1184507042]) # middle
        group_5.append([257.8275, 116.439807692, -283.419, 174.63056338, 76.1184507042]) # forward
        group_5.append([257.8275, 116.439807692, -283.419, 204.024788732, 76.1184507042]) # backward
        group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 51.4805633803]) # left
        group_5.append([257.8275, 116.439807692, -283.419, 189.016478873, 101.376197183]) # right
        
        group_9 = []
        group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 76.1184507042]) # middle
        group_9.append([257.8275, 116.439807692, -283.419, 181.516478873, 76.1184507042]) # forward 1
        group_9.append([257.8275, 116.439807692, -283.419, 174.63056338, 76.1184507042]) # forward 2
        group_9.append([257.8275, 116.439807692, -283.419, 196.516478873, 76.1184507042]) # backward 1
        group_9.append([257.8275, 116.439807692, -283.419, 204.016478873, 76.1184507042]) # backward 2
        group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 63.6184507042]) # left 1
        group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 51.1184507042]) # left 2
        group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 88.6184507042]) # right 1
        group_9.append([257.8275, 116.439807692, -283.419, 189.016478873, 101.1184507042]) # right 2

        robot = Robot()
        
        while not rospy.is_shutdown():
            group_choice = []
            num = raw_input("5 or 9 samples per group? ")
            if num == "5":
                print "choose group 5"
                group_choice = group_5
                group_5_num += 1
            elif num == "9":
                print "choose group 9"
                group_choice = group_9
                group_9_num += 1
            
            for angles in group_choice:
                raw_input("press any key to move the robot...")
                robot.set_robot_angle(angles)
            
            print "group 5 num: {}".format(group_5_num)
            print "group 9 num: {}".format(group_9_num)
            print "num samples: {}".format(group_5_num * 5 + group_9_num* 9)
            print "---------------------------------------------------"

    except rospy.ROSInterruptException:
        pass
