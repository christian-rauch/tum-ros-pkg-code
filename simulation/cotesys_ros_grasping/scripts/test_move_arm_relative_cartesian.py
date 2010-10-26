#! /usr/bin/env python

import roslib; roslib.load_manifest('cotesys_ros_grasping')
import rospy

import actionlib
from cotesys_ros_grasping.msg import MoveArmRelativeCartesianPointAction, MoveArmRelativeCartesianPointGoal 

if __name__ == '__main__':

      rospy.init_node('test_move_arm_relative_cartesian_point')
      
      move_to_pos_client = actionlib.SimpleActionClient('move_arm_relative_cartesian_point', MoveArmRelativeCartesianPointAction)
      move_to_pos_client.wait_for_server()
      
      goal = MoveArmRelativeCartesianPointGoal()
      goal.arm_name = "right_arm"
      goal.rel_point.x = 0.0
      goal.rel_point.y = .1
      goal.rel_point.z = 0.0

      move_to_pos_client.send_goal(goal)
      print 'sent goal'
      move_to_pos_client.wait_for_result()
