#!/usr/bin/env python
import roslib; roslib.load_manifest('oro_ros')

import sys, json

from oro_ros.srv import *
from pyoro import *
import rospy
from rospy.exceptions import ROSException
from rospy.client import *

MAX_CONNECTION_TRIES = 10

oro = None

class ROSHandler:
	
	def __init__(self, oro):
		
		#add the the Oro class all the methods the server declares
		for m in oro.rpc_methods:
			self.add_methods(m)

	def add_methods(self, m):

		def innermethod(req):
			rospy.loginfo(">>>> Got request for a %s. Trying to parse the parameters %s as JSON content..."%(m[0], str(req)))
			params = [encode8bits(json.loads(p)) for p in req.params]
			rospy.loginfo(">>>> ok! querying oro-server for %s with parameters %s..."%(m[0], str(params)))
			try:
				handler = getattr(oro, m[0])
			except AttributeError as ae:
				raise ROSException("Ooops...the RPC method " + m[0] + " is not provided by ORO.")

			
			return OroServerQueryResponse(json.dumps(handler(*params)))
				
		innermethod.__doc__ = "Handler for ROS calls to the oro-server %s method." % m[0]
		innermethod.__name__ = "handle_query_" + m[0]
		setattr(self,innermethod.__name__,innermethod)
		


def oro_ros_server():
	rospy.init_node('oro_ros_server')
	
	rosHandler = ROSHandler(oro)
	
	for srv in oro.rpc_methods:
		try:
			s = rospy.Service("oro/" + srv[0], OroServerQuery, getattr(rosHandler, "handle_query_" + srv[0]))
			rospy.loginfo("Registered ROS service oro/" + srv[0])
		except rospy.service.ServiceException:
			continue

	rospy.loginfo("The ROS-ORO bridge services are running.")
	rospy.loginfo("I'm now waiting for some blingbling to eat!")
	rospy.spin()

def usage():
    return "Usage:\noro_ros_server.py ORO_HOSTNAME ORO_PORT"

def connect(tries):
	try:
		tries = tries - 1
		return Oro(HOST, PORT)
	except:
		if (tries == 0):
			raise OroServerError("Impossible to connect to the server!")
		sleep(2)
		rospy.loginfo("ROS-ORO bridge: Retrying to connect to ORO-server (" + str(tries) + " tries to go)...")
		return connect(tries)

def encode8bits(o):
	if isinstance(o, list):
		return [encode8bits(o2) for o2 in o]
	return o.encode('ascii')
	
if __name__ == "__main__":
	if len(sys.argv) >= 3:
		HOST = sys.argv[1]
		PORT = int(sys.argv[2])
	else:
		print usage()
		sys.exit(1)

	rospy.loginfo("Connecting to oro-server on " + HOST + ":" + str(PORT) + "...")

	try:
		oro = connect(MAX_CONNECTION_TRIES)

		#print "Available methods:\n"

		#for srv in oro.rpc_methods:
		#	print "\t- %s"%srv[0]

		oro_ros_server()

	except OroServerError as ose:
		rospy.logerr('Oups! An error occured on ORO server!')
		print(ose)
		raise ROSException("An error occured on ORO server! Check the oro_ros node log!")
	finally:
		if (oro):
			oro.close()

