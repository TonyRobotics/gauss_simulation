#!/usr/bin/env python

import tf
import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gauss_gazebo.srv import *

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class GaussIKGazebo(object):
  """GaussIKGazebo"""
  def __init__(self):
    super(GaussIKGazebo, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    ## *** RViz ***
    #robot = moveit_commander.RobotCommander() 
    ## *** Gazebo + RViz ***
    robot = moveit_commander.RobotCommander(ns="gauss")

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Gauss manipulator, so we set the group's name to "gauss_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "gauss_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self, pose_target):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    # select Pilz Command Planner tyyype
    move_group.set_planner_id('PTP')

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the joint values from the group and adjust some of the values:
    print "+++++++++++++++++++++++++++++++"
    print "received pose target: "
    print pose_target

    for i in xrange(0, len(pose_target)):
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pose_target[i].positions[0]
        joint_goal[1] = pose_target[i].positions[1]
        joint_goal[2] = pose_target[i].positions[2]
        joint_goal[3] = pose_target[i].positions[3]
        joint_goal[4] = pose_target[i].positions[4]
        joint_goal[5] = pose_target[i].positions[5]

        print "+++++++++++++++++++++++++++++++"
        print "joint_goal %d: "%i
        print joint_goal

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        time.sleep(5)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  

  def go_to_pose_goals(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    rospy.wait_for_service('calculate_ik')

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    """
    Goal0: zero position
    j1 = 0.00   j1 = 0.0000
    j2 = 0.00   j2 = 0.0000
    j3 = 0.00   j3 = 0.0020
    j4 = 0.00   j4 = 0.0000
    j5 = 0.00   j5 = 0.0020
    j6 = 0.00   j6 = 0.0000
    """
    pose_goal0 = geometry_msgs.msg.Pose()
    (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose_goal0.orientation.w = qw
    pose_goal0.orientation.x = qx
    pose_goal0.orientation.y = qy
    pose_goal0.orientation.z = qz
    pose_goal0.position.x = 0.277 
    pose_goal0.position.y = 0.0
    pose_goal0.position.z = 0.435

    """
    Goal1:
    j1 = 0.00   j1 =  0.0000
    j2 = 0.00   j2 =  0.0108
    j3 = 0.00   j3 =  0.0069
    j4 = 0.00   j4 =  0.0000
    j5 = 1.57   j5 =  1.5530
    j6 = 0.00   j6 =  0.0000
    """
    pose_goal1 = geometry_msgs.msg.Pose()
    pose_goal1.orientation.w = 0.707144
    pose_goal1.orientation.x = 0.0
    pose_goal1.orientation.y = -0.707107
    pose_goal1.orientation.z = 0.0
    pose_goal1.position.x = 0.25 
    pose_goal1.position.y = 0.0
    pose_goal1.position.z = 0.461474

    """
    Goal2:
    j1 = 0.00    j1 = -0.0040
    j2 = 0.00    j2 =  0.0108
    j3 = 0.01    j3 =  0.0109
    j4 = 1.57    j4 =  1.5709
    j5 = 1.57    j5 =  1.5668
    j6 = 1.57    j6 =  1.5491
    """
    pose_goal2 = geometry_msgs.msg.Pose()
    pose_goal2.orientation.x = 0.707107
    pose_goal2.orientation.y = -0.707107
    pose_goal2.orientation.z = 0.0
    pose_goal2.orientation.w = 0.0
    pose_goal2.position.x = 0.25 
    pose_goal2.position.y = -0.026
    pose_goal2.position.z = 0.435474
    
    """
    Goal4:
    j1 =  1.12    j1 =  1.1234
    j2 = -0.03    j2 = -0.0023
    j3 =  1.13    j3 =  1.0866
    j4 =  1.32    j4 = -1.8308
    j5 = -1.46    j5 =  1.4525
    j6 = -1.59    j6 =  1.5405
    """
    pose_goal4 = geometry_msgs.msg.Pose()
    pose_goal4.orientation.x = -0.197907
    pose_goal4.orientation.y = -0.498488
    pose_goal4.orientation.z = 0.789525
    pose_goal4.orientation.w = 0.298316
    pose_goal4.position.x = 0.0325826 
    pose_goal4.position.y = 0.123356
    pose_goal4.position.z = 0.657869

    """
    Goal5:
    j1 = -0.13    j1 = -0.1286
    j2 = -0.93    j2 =  0.9237
    j3 =  0.40    j3 =  0.3860
    j4 = -1.61    j4 = -1.6115
    j5 =  1.08    j5 =  1.0773
    j6 =  2.72    j6 =  2.7202
    """
    pose_goal5 = geometry_msgs.msg.Pose()
    pose_goal5.orientation.x = 0.577592
    pose_goal5.orientation.y = 0.434889
    pose_goal5.orientation.z = 0.248598
    pose_goal5.orientation.w = 0.644561
    pose_goal5.position.x = 0.373485 
    pose_goal5.position.y = -0.0261094
    pose_goal5.position.z = 0.226124

    """
    Goal6:
    j1 = -0.70    j1 = -0.6952
    j2 = -0.23    j2 = -0.2256
    j3 = -0.01    j3 =  0.0103
    j4 =  2.80    j4 = -0.3732
    j5 = -0.28    j5 = 0 .2554
    j6 =  2.57    j6 = -0.5394
    """
    pose_goal6 = geometry_msgs.msg.Pose()
    pose_goal6.orientation.x = -0.419954
    pose_goal6.orientation.y = 0.119706
    pose_goal6.orientation.z = -0.271946
    pose_goal6.orientation.w = 0.857528
    pose_goal6.position.x = 0.240517 
    pose_goal6.position.y = -0.197631
    pose_goal6.position.z = 0.372479

    """
    Goal7:
    j1 =  0.17    j1 =  0.1721
    j2 = -0.06    j2 = -0.0482
    j3 =  0.09    j3 =  0.0848
    j4 =  2.05    j4 = -1.0952
    j5 = -1.50    j5 =  1.4975
    j6 = -1.13    j6 =  2.0132
    """
    pose_goal7 = geometry_msgs.msg.Pose()
    pose_goal7.orientation.x = 0.312885
    pose_goal7.orientation.y = 0.0036165
    pose_goal7.orientation.z = 0.740332
    pose_goal7.orientation.w = 0.594978
    pose_goal7.position.x = 0.254483 
    pose_goal7.position.y = 0.0667341
    pose_goal7.position.z = 0.455975

    """
    Goal8:
    j1 = -1.27    j1 = -1.2699
    j2 = -0.37    j2 = -0.3613
    j3 =  0.56    j3 =  0.5556
    j4 =  1.42    j4 = -1.7046
    j5 = -0.16    j5 =  0.1637
    j6 = -0.24    j6 =  2.8849
    """
    pose_goal8 = geometry_msgs.msg.Pose()
    pose_goal8.orientation.x = 0.423964
    pose_goal8.orientation.y = -0.345163
    pose_goal8.orientation.z = -0.399567
    pose_goal8.orientation.w = 0.735842
    pose_goal8.position.x = 0.103712 
    pose_goal8.position.y = -0.32063
    pose_goal8.position.z = 0.476441

    """
    Goal9:
    j1 =  0.88    j1 =  0.8807
    j2 = -1.24    j2 = -1.2266
    j3 =  0.97    j3 =  0.9554
    j4 = -2.17    j4 =  0.9841
    j5 =  0.37    j5 = -0.3700
    j6 =  2.08    j6 = -1.0766
    """
    pose_goal9 = geometry_msgs.msg.Pose()
    pose_goal9.orientation.x = -0.118023
    pose_goal9.orientation.y = 0.194808
    pose_goal9.orientation.z = 0.556168
    pose_goal9.orientation.w = 0.799248
    pose_goal9.position.x = 0.273069 
    pose_goal9.position.y = 0.342608
    pose_goal9.position.z = 0.234342

    """
    Goal10: 
    x+ max: 0.462
    j1 =  0.00    j1 =  0.0000 
    j2 = -1.57    j2 = -1.5707
    j3 =  1.57    j3 =  1.5707
    j4 =  0.00    j4 =  0.0000
    j5 =  0.00    j5 =  0.0000
    j6 =  0.00    j6 =  0.0000
    """
    pose_goal10 = geometry_msgs.msg.Pose()
    pose_goal10.orientation.x = 0.00
    pose_goal10.orientation.y = 0.00
    pose_goal10.orientation.z = 0.00
    pose_goal10.orientation.w = 1.00
    pose_goal10.position.x = 0.462
    pose_goal10.position.y = 0.00
    pose_goal10.position.z = 0.2505

    """
    Goal11: 
    y+ max: 0.462
    j1 =  1.57    j1 =  1.5707
    j2 = -1.57    j2 = -1.5707
    j3 =  1.57    j3 =  1.5707
    j4 =  0.00    j4 =  0.0000
    j5 =  0.00    j5 =  0.0000
    j6 =  0.00    j6 =  0.0000
    """
    pose_goal11 = geometry_msgs.msg.Pose()
    pose_goal11.orientation.x = 0.00
    pose_goal11.orientation.y = 0.00
    pose_goal11.orientation.z = 0.707107
    pose_goal11.orientation.w = 0.707107
    pose_goal11.position.x = 0.00 
    pose_goal11.position.y = 0.462
    pose_goal11.position.z = 0.2505

    """
    Goal12: 
    z+ max: 0.7125
    j1 =  0.00    j1 = 0.0000
    j2 =  0.00    j2 = 0.0000
    j3 =  1.57    j3 = 1.5707
    j4 =  0.00    j4 = 0.0000
    j5 =  0.00    j5 = 0.0000
    j6 =  0.00    j6 = 0.0000
    """
    pose_goal12 = geometry_msgs.msg.Pose()
    pose_goal12.orientation.x = 0.0
    pose_goal12.orientation.y = -0.707107
    pose_goal12.orientation.z = 0.00
    pose_goal12.orientation.w = 0.707107
    pose_goal12.position.x = 0.00 
    pose_goal12.position.y = 0.00
    pose_goal12.position.z = 0.7125

    """
    Goal13: 
    j1 =  0.00    j1 = 0.0000
    j2 =  0.00    j2 = 0.0000
    j3 =  0.00    j3 = 0.0000
    j4 =  0.00    j4 = 0.0000
    j5 =  0.00    j5 = 0.0000
    j6 =  0.00    j6 = 0.0000
    """
    pose_goal13 = geometry_msgs.msg.Pose()
    (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose_goal13.orientation.x = 0.0
    pose_goal13.orientation.y = 0.0
    pose_goal13.orientation.z = 0.0
    pose_goal13.orientation.w = 1.0
    pose_goal13.position.x = 0.2778
    pose_goal13.position.y = 0.00
    pose_goal13.position.z = 0.42969 - 0.1

    poses = []
    #poses.append(copy.deepcopy(pose_goal0))
    #poses.append(copy.deepcopy(pose_goal1))
    #poses.append(copy.deepcopy(pose_goal2))
    #poses.append(copy.deepcopy(pose_goal4))
    #poses.append(copy.deepcopy(pose_goal5))
    #poses.append(copy.deepcopy(pose_goal6))
    #poses.append(copy.deepcopy(pose_goal7))
    #poses.append(copy.deepcopy(pose_goal8))
    #poses.append(copy.deepcopy(pose_goal9))
    #poses.append(copy.deepcopy(pose_goal10))
    #poses.append(copy.deepcopy(pose_goal11))
    #poses.append(copy.deepcopy(pose_goal12))
    poses.append(copy.deepcopy(pose_goal13))

    calculate_pose_ik = rospy.ServiceProxy('calculate_ik', CalculateIK)

    return calculate_pose_ik(poses)
  

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt! MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = GaussIKGazebo()

    print "============ Press `Enter` to calculate target poses ..."
    raw_input()
    poses = tutorial.go_to_pose_goals()
    print "============ Press `Enter` to execute target poses ..."
    raw_input()
    tutorial.go_to_joint_state(poses.points)

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
