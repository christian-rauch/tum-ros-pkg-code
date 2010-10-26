#! /usr/bin/env python

import roslib; roslib.load_manifest('cotesys_ros_grasping')
import rospy

import actionlib
from cotesys_ros_grasping.msg import MoveArmToPositionGoal, MoveArmToPositionAction

if __name__ == '__main__':

      rospy.init_node('test_move_arm_to_position')
      
      move_to_pos_client = actionlib.SimpleActionClient('move_arm_to_position', MoveArmToPositionAction)
      move_to_pos_client.wait_for_server()
      
      goal = MoveArmToPositionGoal()
      goal.arm_name = "right_arm"
      goal.point.x = .7
      goal.point.y = -.4
      goal.point.z = .8

      move_to_pos_client.send_goal(goal)
      print 'sent goal'
      move_to_pos_client.wait_for_result()
