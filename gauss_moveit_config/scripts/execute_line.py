#!/usr/bin/env python

import yaml
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import String

from gauss_msgs.msg import RobotMoveAction
from gauss_msgs.msg import RobotMoveGoal
from gauss_msgs.msg import HardwareStatus

from gauss_msgs.srv import SetInt
from gauss_msgs.srv import GetDigitalIO
from gauss_msgs.srv import SetDigitalIO
from gauss_msgs.srv import GetPositionList

from gauss_commander.command_type import CommandType as MoveCommandType

from gauss_python_api.gauss_api import Gauss

import math
import time

def get_positions():
    f = open('goal.yaml')
    res = yaml.load(f)
    # print(res)
    goal = res['goal']
    trajectory =  goal['trajectory']
    points =  trajectory['points']
    # print points
    print len(points)
    l = []
    for i in range(len(points)):
        positions = points[i]['positions']
        # print positions
        l.append(positions)
    f.close()
    return l

def test_move_joint(gauss):
    # joints = [math.pi,0,0,0,0,0]
    # try:
    #     message = gauss.move_joints(joints)
    # except Exception as e:
    #     print e
    # else:
    #     print message
    pl = get_positions()
    size = len(pl)
    for i in range(size):
        print pl[i]
        print "doing... " + str(i)
        message = gauss.move_joints( pl[i])
        time.sleep(1)


if __name__ == '__main__':
  
    rospy.init_node('gauss_move_joint')
    g = Gauss()
    test_move_joint(g)
    rospy.spin()
