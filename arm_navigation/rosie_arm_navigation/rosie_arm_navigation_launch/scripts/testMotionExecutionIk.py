#! /usr/bin/env python

PKG = 'move_arm'

import roslib; roslib.load_manifest(PKG)
import rospy
import planning_environment_msgs.srv
import sys
import unittest
import actionlib
import actionlib_msgs
import math

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from motion_planning_msgs.msg import CollisionOperation, MotionPlanRequest, PositionConstraint, OrientationConstraint
from move_arm_msgs.msg import MoveArmGoal, MoveArmAction
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from tf import TransformListener
from motion_planning_msgs.msg import JointConstraint

padd_name = "ompl_planning/robot_padd"
extra_buffer = .1

class TestMotionExecutionBuffer(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_motion_execution_buffer')
        
        self.tf = TransformListener()        

        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
        
        self.move_arm_action_client = actionlib.SimpleActionClient("move_right_arm", MoveArmAction)
        self.move_arm_action_client.wait_for_server()

        obj1 = CollisionObject()
    
        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "obj1";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(1)]
        obj1.shapes[0].type = Shape.BOX
        obj1.shapes[0].dimensions = [float() for _ in range(3)]
        obj1.shapes[0].dimensions[0] = .1
        obj1.shapes[0].dimensions[1] = .1
        obj1.shapes[0].dimensions[2] = 2.0
        obj1.poses = [Pose() for _ in range(1)]
        obj1.poses[0].position.x = .9
        obj1.poses[0].position.y = -.5
        obj1.poses[0].position.z = 1.0
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
        
        #self.obj_pub.publish(obj1)

        #rospy.sleep(1.0)
        
    def tearDown(self):
        obj1 = CollisionObject()
        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "all";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE

        self.obj_pub.publish(obj1)
        
        #rospy.sleep(2.0)

    def testMotionExecutionBuffer(self):
        
        global padd_name
        global extra_buffer
        
        #too much trouble to read for now
        allow_padd = .05#rospy.get_param(padd_name)
        
        motion_plan_request = MotionPlanRequest()

        motion_plan_request.group_name = "right_arm"
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.allowed_planning_time = rospy.Duration(5.)
        motion_plan_request.planner_id = ""
        planner_service_name = "ompl_planning/plan_kinematic_path"

        motion_plan_request.goal_constraints.position_constraints = [PositionConstraint() for _ in range(1)]
        motion_plan_request.goal_constraints.position_constraints[0].header.stamp = rospy.Time.now()
        motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link"

        motion_plan_request.goal_constraints.position_constraints[0].link_name = "right_arm_hand_link"
        motion_plan_request.goal_constraints.position_constraints[0].position.x = .65
        motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.55
        motion_plan_request.goal_constraints.position_constraints[0].position.z = .9
    
        motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = Shape.BOX
        motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions = [float(.02) for _ in range(3)]
        
        motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0
        motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0
        
        motion_plan_request.goal_constraints.orientation_constraints = [OrientationConstraint() for _ in range(1)]
        motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = rospy.Time.now()
        motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link"    
        motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "right_arm_hand_link"

        vals = [float() for _ in range(4)]
        vals = [-.895249, -.187638, 0.092325, .393443]
        mag = math.sqrt(vals[0]*vals[0]+vals[1]*vals[1]+vals[2]*vals[2]+vals[3]*vals[3])
        vals[0] /= mag
        vals[1] /= mag
        vals[2] /= mag
        vals[3] /= mag

        motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = vals[0]
        motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = vals[1]
        motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = vals[2]
        motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = vals[3]
        
        motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04
        motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04
        motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04

        motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0

        goal = MoveArmGoal()
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        goal.motion_plan_request = motion_plan_request

        self.move_arm_action_client.send_goal(goal)

        while True:
             cur_state = self.move_arm_action_client.get_state()
             if(cur_state != actionlib_msgs.msg.GoalStatus.ACTIVE and
                cur_state != actionlib_msgs.msg.GoalStatus.PENDING):
                 break 

if __name__ == '__main__':

    import rostest
    rostest.unitrun('test_motion_execution_buffer', 'test_motion_execution_buffer', TestMotionExecutionBuffer)


    
