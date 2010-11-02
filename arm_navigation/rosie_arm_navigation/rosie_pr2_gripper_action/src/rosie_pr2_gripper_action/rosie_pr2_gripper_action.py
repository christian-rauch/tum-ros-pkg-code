#!/usr/bin/env python
#
# PR2-Gripper Action Server for Rosie (using the armhand action server)
# Mimics the PR2 gripper interface
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

import roslib; roslib.load_manifest('rosie_pr2_gripper_action')
import rospy

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
import actionlib
import cogman_msgs.msg



class RosiePR2GripperActionServer:
    def __init__(self):

        self.threshold = 0.05
        rospy.loginfo('open/close threshold = %5.3f' % (self.threshold))
        
        self.l_action_server = actionlib.simple_action_server.SimpleActionServer('/l_gripper_controller/gripper_action',Pr2GripperCommandAction, self.l_executeCB)
        self.r_action_server = actionlib.simple_action_server.SimpleActionServer('/r_gripper_controller/gripper_action',Pr2GripperCommandAction, self.r_executeCB)

        self.l_action_client = actionlib.SimpleActionClient('left_arm', cogman_msgs.msg.ArmHandAction)
        self.r_action_client = actionlib.SimpleActionClient('right_arm', cogman_msgs.msg.ArmHandAction)

        self.l_action_client.wait_for_server(rospy.Duration(20.0))
        self.r_action_client.wait_for_server(rospy.Duration(20.0))
        
        #Put the hands in a known state
        self.hand_primitive(self.l_action_client,'open_relax')
        self.hand_primitive(self.r_action_client,'open_relax')
        
        
    def r_executeCB(self, goal):
        self.executeCB(goal, 'right')
        
    def l_executeCB(self, goal):
        self.executeCB(goal, 'left')
        
    def executeCB(self, goal, side='right'):
        if side == 'left':
            client = self.l_action_client
            action_server = self.l_action_server
        elif side =='right':
            client = self.r_action_client
            action_server = self.r_action_server
        else:
            rospy.logerr('Wrong side specified, expected left/right')
            return(None)

        # position == 0.08 -> Full open  0.0 -> Closed
        if goal.command.position < self.threshold:
            self.hand_primitive(client, '3pinch')
        else:
            self.hand_primitive(client, 'open_relax')
        
        client.wait_for_result(rospy.Duration(20.0))  #Wait up to 20s for the server
            
        result = Pr2GripperCommandResult()
        # float64 position
        # float64 effort
        # bool stalled
        # bool reached_goal
        
        self.success = True  #FIXME: Check for success from the armhand server
        if self.success:
            result.position = goal.command.position  # Give the same position as desired
            result.effort = 0.5  # Some non-zero value
            result.stalled = False
            result.reached_goal = True
            action_server.set_succeeded(result)
        else:
            result.reached_goal = False
            rospy.logerr("PR2 gripper action FAILED")
            self.action_server.set_aborted(result)
    
    def hand_primitive(self, client, primitive_name):
        goal = cogman_msgs.msg.ArmHandGoal()
        goal.command = 'hand_primitive'
        goal.hand_primitive = primitive_name
        client.send_goal(goal)
        
        
def main():
    rospy.loginfo("rosie_pr2_gripper_action: Starting up!")
    rospy.init_node('rosie_pr2_gripper_action')
    tuck_arms_action_server = RosiePR2GripperActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()
