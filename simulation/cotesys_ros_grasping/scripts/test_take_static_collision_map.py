#! /usr/bin/env python

import roslib; roslib.load_manifest('cotesys_ros_grasping')
import rospy

import actionlib
from cotesys_ros_grasping.msg import TakeStaticCollisionMapAction, TakeStaticCollisionMapGoal

if __name__ == '__main__':

      rospy.init_node('test_take_static_collision_map')
      
      take_static_client = actionlib.SimpleActionClient('/take_static_collision_map', TakeStaticCollisionMapAction)
      take_static_client.wait_for_server()
      
      goal = TakeStaticCollisionMapGoal()

      take_static_client.send_goal(goal)
      take_static_client.wait_for_result()
