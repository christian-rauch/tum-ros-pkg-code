#!/usr/bin/env python
import roslib; roslib.load_manifest('oro_ros')

import sys

import rospy
from oro_ros.srv import *


def oro_ros_client(query, params):
    rospy.wait_for_service('oro/' + query)
    try:
        query_oro_server = rospy.ServiceProxy('oro/' + query, OroServerQuery)
        resp1 = query_oro_server(params)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s query [param1, param2, ...]"%sys.argv[0]

if __name__ == "__main__":
	
    if len(sys.argv) == 1:
        print usage()
        sys.exit(1)
        
    query = sys.argv[1]
    #params = sys.argv[2].split('@@')
    #params = sys.argv[2][1:-1].split(',')
    params = sys.argv[2:]
    
    oro_ros_client("add", ["[\"robot likes disco_music\", \"disco_music rdf:type Music\"]"])
    
    print "Requesting %s %s"%(query, params)
    print "%s %s => %s"%(query, params, oro_ros_client(query, params))
 

