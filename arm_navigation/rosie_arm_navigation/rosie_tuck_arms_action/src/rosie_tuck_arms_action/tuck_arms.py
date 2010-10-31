#!/usr/bin/env python
#
# Tuck-Arms Action Server for Rosie (using the armhand action server)
# Mimics the PR2 tuck interface
#
# Copyright (c) 2010 Alexis Maldonado <maldonado@tum.de>
# Technische Universitaet Muenchen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('rosie_tuck_arms_action')
import rospy

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
import actionlib
import cogman_msgs.msg

class TuckArmsActionServer:
    def __init__(self, node_name):
        self.node_name = node_name
        self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmsAction, self.executeCB)

        self.l_action_client = actionlib.SimpleActionClient('left_arm', cogman_msgs.msg.ArmHandAction)
        self.r_action_client = actionlib.SimpleActionClient('right_arm', cogman_msgs.msg.ArmHandAction)
    

    def executeCB(self, goal):
        if goal.tuck_right:
            self.tuckR()
        else:
            self.untuckR()
            
        if goal.tuck_left:
            self.tuckL()
        else:
            self.untuckL()

        self.l_action_client.wait_for_result(rospy.Duration(20.0))  #Wait up to 20s for the server
        self.r_action_client.wait_for_result(rospy.Duration(20.0))  #Wait up to 20s for the server
            
        result = TuckArmsResult()
            
        self.success = True  #FIXME: Check for success from the armhand server
        if self.success:
            result.tuck_right = goal.tuck_right
            result.tuck_left = goal.tuck_left
            self.action_server.set_succeeded(result)
        else:
            rospy.logerr("Tuck or untuck arms FAILED: %d %d"%(result.left, result.right))
            self.action_server.set_aborted(result)


    def tuckL(self):
        side = 'left'
        pose_name = 'parking'
        goal = cogman_msgs.msg.ArmHandGoal()
        goal.command = 'arm_joint_pose'
        goal.pose_name = side + "_" + pose_name
        self.l_action_client.send_goal(goal)
    
    def untuckL(self):
        side = 'left'
        pose_name = 'open'
        goal = cogman_msgs.msg.ArmHandGoal()
        goal.command = 'arm_joint_pose'
        goal.pose_name = side + "_" + pose_name
        self.l_action_client.send_goal(goal)

    def tuckR(self):
        side = 'right'
        pose_name = 'parking'
        goal = cogman_msgs.msg.ArmHandGoal()
        goal.command = 'arm_joint_pose'
        goal.pose_name = side + "_" + pose_name
        self.r_action_client.send_goal(goal)    
    
    def untuckR(self):
        side = 'right'
        pose_name = 'open'
        goal = cogman_msgs.msg.ArmHandGoal()
        goal.command = 'arm_joint_pose'
        goal.pose_name = side + "_" + pose_name
        self.r_action_client.send_goal(goal)
    
def main():
    action_name = 'rosie_tuck_arms_action'
    action_name = 'tuck_arms'
    rospy.loginfo("%s: Starting up!" %(action_name))
    
    rospy.init_node(action_name)
    tuck_arms_action_server = TuckArmsActionServer(action_name)
    rospy.spin()

if __name__ == '__main__':
    main()

