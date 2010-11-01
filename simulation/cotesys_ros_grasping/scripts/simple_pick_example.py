#! /usr/bin/env python

import roslib; roslib.load_manifest('cotesys_ros_grasping')
import rospy

import actionlib
from cotesys_ros_grasping.msg import MoveArmRelativeCartesianPointAction, MoveArmRelativeCartesianPointGoal 
from cotesys_ros_grasping.msg import MoveArmToPositionGoal, MoveArmToPositionAction
from cotesys_ros_grasping.msg import TakeStaticCollisionMapAction, TakeStaticCollisionMapGoal
from cotesys_ros_grasping.msg import AttachBoundingBoxAction, AttachBoundingBoxGoal
from table_objects.srv import GetObjects, GetObjectsRequest
from sensor_msgs.msg import PointCloud2
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

def close_gripper(action):
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.0
    goal.command.max_effort = 1000
    action.send_goal(goal)
    action.wait_for_result(rospy.Duration(1.0))

def open_gripper(action):
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.08
    goal.command.max_effort = 1000
    action.send_goal(goal)
    action.wait_for_result(rospy.Duration(1.0))

latest_point_cloud = PointCloud2()
got_cloud = False

def point_cloud_cb(data):
    global latest_point_cloud
    
    latest_point_cloud = data

    got_cloud = True

if __name__ == '__main__':

      rospy.init_node('simple_pick_example')

      move_to_rel_pos_client = actionlib.SimpleActionClient('/move_arm_relative_cartesian_point', MoveArmRelativeCartesianPointAction)
      move_to_rel_pos_client.wait_for_server()

      take_static_client = actionlib.SimpleActionClient('/take_static_collision_map', TakeStaticCollisionMapAction)
      take_static_client.wait_for_server()

      move_to_arm_pos_client = actionlib.SimpleActionClient('/move_arm_to_position', MoveArmToPositionAction)
      move_to_arm_pos_client.wait_for_server()

      right_gripper_client_ = actionlib.SimpleActionClient('/r_gripper_controller/gripper_action', Pr2GripperCommandAction)
      right_gripper_client_.wait_for_server()

      left_gripper_client_ = actionlib.SimpleActionClient('/l_gripper_controller/gripper_action', Pr2GripperCommandAction)
      left_gripper_client_.wait_for_server()

      rospy.wait_for_service("/get_table_objects")
      get_table_objects_service_ = rospy.ServiceProxy("/get_table_objects", GetObjects)

      attach_object_client = actionlib.SimpleActionClient('attach_bounding_box', AttachBoundingBoxAction)
      attach_object_client.wait_for_server()

      rospy.Subscriber("/extract_objects_indices/output", PointCloud2, point_cloud_cb)

      print 'Connected to all clients'

      move_rarm_to_side_goal = MoveArmToPositionGoal()
      move_rarm_to_side_goal.arm_name = "right_arm"
      move_rarm_to_side_goal.grasp_name = "top_level"
      move_rarm_to_side_goal.point.x = .52
      move_rarm_to_side_goal.point.y = -.7
      move_rarm_to_side_goal.point.z = .9

      move_to_arm_pos_client.send_goal(move_rarm_to_side_goal)
      print 'sent right arm goal'
      move_to_arm_pos_client.wait_for_result()

      move_larm_to_side_goal = MoveArmToPositionGoal()
      move_larm_to_side_goal.arm_name = "left_arm"
      move_rarm_to_side_goal.grasp_name = "top_level"
      move_larm_to_side_goal.point.x = .52
      move_larm_to_side_goal.point.y = .7
      move_larm_to_side_goal.point.z = .9

      move_to_arm_pos_client.send_goal(move_larm_to_side_goal)
      print 'sent left arm goal'
      move_to_arm_pos_client.wait_for_result()

      open_gripper(right_gripper_client_)
      open_gripper(left_gripper_client_)

      stat_goal = TakeStaticCollisionMapGoal()
      take_static_client.send_goal(stat_goal)
      take_static_client.wait_for_result()

      if(not got_cloud):
          print 'No point cloud'
          exit

      get_table_objects = GetObjectsRequest()
      get_table_objects.data = latest_point_cloud
      table_objects = get_table_objects_service_.call(get_table_objects)

      if(len(table_objects.table_objects) == 0):
          print 'no objects'
          exit
      else:
          print 'got objects ', len(table_objects.table_objects)

      right_arm_pos_goal = MoveArmToPositionGoal()
      right_arm_pos_goal.arm_name = "right_arm"
      right_arm_pos_goal.grasp_name = "top"
      right_arm_pos_goal.point.x = table_objects.table_objects[0].position.x
      right_arm_pos_goal.point.y = table_objects.table_objects[0].position.y
      right_arm_pos_goal.point.z = table_objects.table_objects[0].position.z+.2

      move_to_arm_pos_client.send_goal(right_arm_pos_goal)
      move_to_arm_pos_client.wait_for_result()

      move_rarm_down_goal = MoveArmRelativeCartesianPointGoal()
      move_rarm_down_goal.arm_name = "right_arm"
      move_rarm_down_goal.rel_point.z = -.1
      move_to_rel_pos_client.send_goal(move_rarm_down_goal)
      move_to_rel_pos_client.wait_for_result()

      close_gripper(right_gripper_client_)

      attach_goal = AttachBoundingBoxGoal()
      attach_goal.arm_name = "right_arm"
      attach_goal.x_size = table_objects.table_objects[0].dimension[0]
      attach_goal.y_size = table_objects.table_objects[0].dimension[1]
      attach_goal.z_size = table_objects.table_objects[0].dimension[2]
      attach_object_client.send_goal(attach_goal)
      attach_object_client.wait_for_result()

      move_rarm_up_goal = MoveArmRelativeCartesianPointGoal()
      move_rarm_up_goal.arm_name = "right_arm"
      move_rarm_up_goal.rel_point.z = .2
      move_to_rel_pos_client.send_goal(move_rarm_up_goal)
      move_to_rel_pos_client.wait_for_result()

      move_to_arm_pos_client.send_goal(move_rarm_to_side_goal)
      move_to_arm_pos_client.wait_for_result()
   
      stat_goal = TakeStaticCollisionMapGoal()
      take_static_client.send_goal(stat_goal)
      take_static_client.wait_for_result()
      
      move_to_arm_pos_client.send_goal(right_arm_pos_goal)
      move_to_arm_pos_client.wait_for_result()

      move_rarm_down_goal = MoveArmRelativeCartesianPointGoal()
      move_rarm_down_goal.arm_name = "right_arm"
      move_rarm_down_goal.rel_point.z = -.1
      move_to_rel_pos_client.send_goal(move_rarm_down_goal)
      move_to_rel_pos_client.wait_for_result()

      open_gripper(right_gripper_client_)

      attach_goal.remove = True
      attach_object_client.send_goal(attach_goal)
      attach_object_client.wait_for_result()

      move_rarm_up_goal = MoveArmRelativeCartesianPointGoal()
      move_rarm_up_goal.arm_name = "right_arm"
      move_rarm_up_goal.rel_point.z = .2
      move_to_rel_pos_client.send_goal(move_rarm_up_goal)
      move_to_rel_pos_client.wait_for_result()
      
      move_to_arm_pos_client.send_goal(move_rarm_to_side_goal)
      move_to_arm_pos_client.wait_for_result()
