#!/usr/bin/env python
#
# Test for the rosie_pr2_gripper_action
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


def gripper_command(client, position):
    goal = Pr2GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = 0.5
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(20.0))  #Wait up to 20s for the server
        
def main():
    rospy.loginfo("test_pr2_gripper_action: Starting up!")
    rospy.init_node('test_pr2_gripper_action')

    
    l_client = actionlib.SimpleActionClient('/l_gripper_controller/gripper_action', Pr2GripperCommandAction)
    r_client = actionlib.SimpleActionClient('/r_gripper_controller/gripper_action', Pr2GripperCommandAction)

    l_client.wait_for_server(rospy.Duration(20.0))
    r_client.wait_for_server(rospy.Duration(20.0))

    print("Left hand:")
    print("Closing the fingers.")
    gripper_command(l_client, 0.0)
    print("Opening the fingers.")
    gripper_command(l_client, 0.08)

    print("Right hand:")
    print("Closing the fingers.")
    gripper_command(r_client, 0.0)
    print("Opening the fingers.")
    gripper_command(r_client, 0.08)
    
    
if __name__ == '__main__':
    main()
