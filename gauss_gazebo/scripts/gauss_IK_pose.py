#!/usr/bin/env python

import copy
import rospy
import tf
import geometry_msgs.msg 
from gauss_gazebo.srv import *

def go_to_pose_goal():
    rospy.wait_for_service('calculate_ik')

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
    Goal3:
    j1 =  1.57    j1 =  1.5761
    j2 = -1.57    j2 = -1.5600
    j3 =  0.01    j3 =  0.0003
    j4 = -1.57    j4 = -1.5655
    j5 =  1.57    j5 =  1.5707
    j6 = -1.57    j6 = -1.5702
    """
    pose_goal3 = geometry_msgs.msg.Pose()
    pose_goal3.orientation.x = 0.0
    pose_goal3.orientation.y = 0.710818
    pose_goal3.orientation.z = -0.703376
    pose_goal3.orientation.w = 0.0
    pose_goal3.position.x = -0.026 
    pose_goal3.position.y = 0.18763
    pose_goal3.position.z = 0.00051

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
    j1 =  0.89    j1 = 0.0000
    j2 = -0.62    j2 = 0.0000
    j3 =  0.62    j3 = 1.5707
    j4 =  0.62    j4 = 0.0000
    j5 =  0.80    j5 = 0.0000
    j6 =  0.89    j6 = 0.0000
    """
    pose_goal13 = geometry_msgs.msg.Pose()
    pose_goal13.orientation.x = 0.736602
    pose_goal13.orientation.y = -0.0746132
    pose_goal13.orientation.z = 0.334717
    pose_goal13.orientation.w = 0.582936
    pose_goal13.position.x = 0.244703 
    pose_goal13.position.y = 0.28575
    pose_goal13.position.z = 0.414521

    poses = []
    """
    poses.append(copy.deepcopy(pose_goal0))
    poses.append(copy.deepcopy(pose_goal1))
    poses.append(copy.deepcopy(pose_goal2))
    poses.append(copy.deepcopy(pose_goal3))
    poses.append(copy.deepcopy(pose_goal4))
    poses.append(copy.deepcopy(pose_goal5))
    poses.append(copy.deepcopy(pose_goal6))
    poses.append(copy.deepcopy(pose_goal7))
    poses.append(copy.deepcopy(pose_goal8))
    poses.append(copy.deepcopy(pose_goal9))
    poses.append(copy.deepcopy(pose_goal10))
    poses.append(copy.deepcopy(pose_goal11))
    poses.append(copy.deepcopy(pose_goal12))
    """
    poses.append(copy.deepcopy(pose_goal13))

    calculate_pose_ik = rospy.ServiceProxy('calculate_ik', CalculateIK)
    resp = calculate_pose_ik(poses)
    if(len(resp.points) < 1):
        print "Calculate IK Service failed!"
        return False
    else:
        print "Find %d trajectory points!"%len(resp.points)
        print resp.points
        return True

if __name__ == '__main__':
    go_to_pose_goal()