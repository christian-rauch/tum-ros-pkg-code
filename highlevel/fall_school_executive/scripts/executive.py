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
PKG = 'icra_ros_tutorial'
NAME = 'executive'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
from actionlib_msgs.msg import *
import tf
import numpy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from icra_ros_tutorial.srv import GetCheckerboardPose, GetApproachPose, GrabBox
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from pr2_common_action_msgs.msg import *
#from pr2_controllers_msgs.msg import *
#from pr2_arm_ik_action.tools import *
#from pr2_plugs_actions.posestampedmath import PoseStampedMath

def run(argv=None):
  rospy.init_node(NAME, anonymous=False)

  print "Waiting for services to come up"
  rospy.wait_for_service('wide_get_checkerboard_pose')
  rospy.wait_for_service('narrow_get_checkerboard_pose')
  rospy.wait_for_service('get_approach_pose')

  try:
    print "Building a tf history"
    pose_pub = rospy.Publisher("nav_pose", PoseStamped)

    #print "Tucking the arms"
    ##tuck the arms
    #tuck_arm_client = actionlib.SimpleActionClient('tuck_arms', TuckArmsAction)
    #tuck_arm_client.wait_for_server()
    #tuck_goal = TuckArmsGoal(True,True)
    #tuck_arm_client.send_goal_and_wait(tuck_goal)

    print "Attempting to get the pose of the board"	
    #Get the pose of the 3x4 checkerboard
    get_checkerboard_pose = rospy.ServiceProxy('wide_get_checkerboard_pose', GetCheckerboardPose)
    board_pose = get_checkerboard_pose(5, 4, .08, .08).board_pose

    print "Attempting to get approach pose"
    #given the pose of the checkerboard, get a good pose to approach it from
    get_approach_pose = rospy.ServiceProxy('get_approach_pose', GetApproachPose)
    nav_pose = get_approach_pose(board_pose).nav_pose

    #publish the final pose for debugging purposes
    pose_pub.publish(nav_pose)

    print "Waiting for move_base_local"
    #OK... our nav_pose is now ready to be sent to the navigation stack as a goal
    move_base_client = actionlib.SimpleActionClient('move_base_local', MoveBaseAction)
    move_base_client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose = nav_pose

    print "Attempting to approach table"
    #send the goal and wait for the base to get there
    move_base_client.send_goal_and_wait(goal)

    #Get the pose of the 5x4 checkerboard on the box
    #get_checkerboard_pose = rospy.ServiceProxy('wide_get_checkerboard_pose', GetCheckerboardPose)
    #board_pose = get_checkerboard_pose(5, 4, 0.022, 0.022).board_pose

    #Grab the box
    #grab_box = rospy.ServiceProxy('grab_box', GrabBox)
    #grab_box(board_pose)
    

  except rospy.ServiceException, e:
    rospy.logerr("The service call to get the checkerboard pose failed: %s" % e)
  except tf.Exception, ex:
    rospy.logerr("The transform failed: %s" % ex)


if __name__ == '__main__':
  run()
