#!/usr/bin/env python

import roslib

roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'std_msgs' )

import rospy
import std_msgs.msg

import sys
import time

def main():
    if len( sys.argv ) > 1:
        topic = str( sys.argv[1] )
    else:
        print 'Usage:', sys.argv[0], '<topic>'
        sys.exit( 1 )

    rospy.init_node( 'set_joint_effort', anonymous=True )
    pub = rospy.Publisher( topic, std_msgs.msg.Float64 )

    while( True ):
        print 'New effort: ',
        cmd = float( sys.stdin.readline() )
        pub.publish( cmd )

if __name__ == '__main__':
    main()

