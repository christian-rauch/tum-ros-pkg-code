#!/usr/bin/env python

import roslib

roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'robot_msgs' )

import rospy
import robot_msgs.msg

import sys
import time

def main():
    if len( sys.argv ) > 1:
        topic = str( sys.argv[1] )
    else:
        print 'Usage:', sys.argv[0], '<topic>'
        sys.exit( 1 )

    rospy.init_node( 'set_joint_position', anonymous=True )
    pub = rospy.Publisher( topic, robot_msgs.msg.JointCmd )

    cmd = robot_msgs.msg.JointCmd()

    while( True ):
        print 'New position: ',
        cmd.positions = [ float( sys.stdin.readline() ) ]
        print 'New velocity: ',
        cmd.velocity = [ float( sys.stdin.readline() ) ]
        pub.publish( cmd )

if __name__ == '__main__':
    main()

