#!/usr/bin/python3
# coding=utf8
#机械臂测试程序
import sys
import os
import time
import rospy
import numpy as np
from puppy_control.srv import SetRunActionName
from std_msgs.msg import Float32,Header

ROS_NODE_NAME = 'arm_test'

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse

def initMove():
    runActionGroup_srv('stand.d6a',True)
    time.sleep(1)
    setServoPulse(9,1200,500)
    time.sleep(1)
    setServoPulse(9,1500,500)
    time.sleep(1)
    runActionGroup_srv('grab.d6a',True)
    time.sleep(1)
    runActionGroup_srv('stand.d6a',True)
    time.sleep(1)
if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME,log_level=rospy.DEBUG)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    initMove()
