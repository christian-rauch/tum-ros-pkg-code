#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'approach_table_tools' # this package name
NAME = 'find_approach_pose'

import roslib; roslib.load_manifest(PKG) 
import rospy 
import tf
import numpy
import math

from geometry_msgs.msg import Quaternion, Pose
from icra_ros_tutorial.srv import GetApproachPose, GetApproachPoseResponse

def msg_to_quaternion(msg):
  return [msg.x, msg.y, msg.z, msg.w]

def quaternion_to_msg(q):
  msg = Quaternion()
  msg.x = q[0]
  msg.y = q[1]
  msg.z = q[2]
  msg.w = q[3]
  return msg

def msg_to_pose(msg):
  trans = tf.transformations.quaternion_matrix(msg_to_quaternion(msg.orientation))
  trans[0, 3] = msg.position.x
  trans[1, 3] = msg.position.y
  trans[2, 3] = msg.position.z
  trans[3, 3] = 1.0
  return trans

def pose_to_msg(pose):
  msg = Pose()
  msg.position.x = pose[0, 3]
  msg.position.y = pose[1, 3]
  msg.position.z = pose[2, 3]
  q = tf.transformations.quaternion_from_matrix(pose)
  msg.orientation.x = q[0]
  msg.orientation.y = q[1]
  msg.orientation.z = q[2]
  msg.orientation.w = q[3]
  return msg

class ApproachPoseFinder:
  def __init__(self):
    self.listener = tf.TransformListener()
    self.server = rospy.Service('get_approach_pose', GetApproachPose, self.get_approach_pose)

  def get_approach_pose(self, req):
    board_pose = req.board_pose
    #We need to create a goal that's actually reachable for the navigation stack... let's say 0.5 meters away from and centered on the board
    trans = msg_to_pose(board_pose.pose)
    origin = tf.transformations.translation_matrix([2 * 0.08, 0, -1.0])
    pose_mat =  numpy.dot(trans, origin)
    board_pose.pose = pose_to_msg(pose_mat)
  
    #Now that we have our pose, we want to transform it into a frame the navigation stack can move in
    #wait for the transform between the camera and odometric frame to be available
    self.listener.waitForTransform('odom_combined', board_pose.header.frame_id, board_pose.header.stamp, rospy.Duration(2.0))
  
    #transform the pose from the camera to odometric frame
    odom_pose = self.listener.transformPose('odom_combined', board_pose)
  
    #project the goal down to the ground
    odom_pose.pose.position.z = 0.0
  
    #we want our goal to point at the board, so we need to rotate it by 90 degrees
    rot_q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi / 2)
    new_q = tf.transformations.quaternion_multiply(rot_q, msg_to_quaternion(odom_pose.pose.orientation))
  
    #we want to remove any pitch and roll from the goal... so we'll just use the yaw element
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(new_q)
  
    #now, we're ready to construct the quaternion for what we'll send to navigation
    odom_pose.pose.orientation = quaternion_to_msg(tf.transformations.quaternion_from_euler(0.0, 0.0, yaw))
  
    return GetApproachPoseResponse(odom_pose)

def approach_pose_finder():
  rospy.init_node(NAME)
  af = ApproachPoseFinder()
  rospy.spin()

if __name__ == "__main__":
  approach_pose_finder()
