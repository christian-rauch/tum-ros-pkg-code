#!/usr/bin/env python
#
# Dummy Torso Action Server for Rosie
# Mimics the PR2 torso interface
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

import roslib; roslib.load_manifest('rosie_torso_action')
import rospy

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
import actionlib

class TorsoActionServer:
    def __init__(self, node_name):
        self.node_name = node_name
        self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name, SingleJointPositionAction, self.executeCB)
    
    def executeCB(self, goal):
        print("executing a torso action!")
        #Just answer success instantly
        result = SingleJointPositionResult()
        self.action_server.set_succeeded(result)
    
def main():
    action_name = 'rosie_torso_action'
    action_name = '/torso_controller/position_joint_action'
    rospy.loginfo("%s: Starting up!" %(action_name))
    
    rospy.init_node("torso_action")
    tuck_arms_action_server = TorsoActionServer(action_name)
    rospy.spin()

if __name__ == '__main__':
    main()
