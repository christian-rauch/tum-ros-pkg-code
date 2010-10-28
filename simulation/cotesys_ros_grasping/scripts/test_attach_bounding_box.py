#! /usr/bin/env python

import roslib; roslib.load_manifest('cotesys_ros_grasping')
import rospy

import actionlib
from cotesys_ros_grasping.msg import AttachBoundingBoxAction, AttachBoundingBoxGoal

if __name__ == '__main__':

      rospy.init_node('test_attach_bounding_box')
      
      attach_object_client = actionlib.SimpleActionClient('attach_bounding_box', AttachBoundingBoxAction)
      attach_object_client.wait_for_server()
      
      goal = AttachBoundingBoxGoal()
      goal.arm_name = "right_arm"
      goal.x_size = .03
      goal.y_size = .03
      goal.z_size = .2

      attach_object_client.send_goal(goal)
      print 'sent goal'
      attach_object_client.wait_for_result()

      rospy.sleep(3)
      
      goal.remove = True

      #attach_object_client.send_goal(goal)
      print 'sent goal'
      #attach_object_client.wait_for_result()
