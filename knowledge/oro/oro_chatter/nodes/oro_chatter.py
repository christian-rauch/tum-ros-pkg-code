#!/usr/bin/python
import roslib; roslib.load_manifest('oro_chatter')
import rospy
import json
from std_msgs.msg import String
from oro_ros.srv import *
from web_hri.srv import *

def callback(data):
	try:
		rospy.loginfo("Got input: " + data.data)
		oro_resp = process_nl(["\"" + data.data + "\""])
		
		try:
			if (oro_resp.res[1:-1] != ""):
				rospy.loginfo("ORO answered: " + oro_resp.res)
				tell_human(oro_resp.res[1:-1])
			else:
				rospy.loginfo("ORO didn't understand")
				
		except rospy.ServiceException, e:
			rospy.logerr("I couldn't send back ORO's answer to the human!")
		
	except rospy.ServiceException, e:
		rospy.logerr("ORO could not process request: %s"%str(e))


def listener():
    rospy.init_node('oro_listener', anonymous=True)
    rospy.Subscriber("/hri/listen_human", String, callback)
    rospy.spin()

if __name__ == '__main__':
	rospy.wait_for_service('/oro/processNL')
	process_nl = rospy.ServiceProxy('/oro/processNL', OroServerQuery)
	
	rospy.wait_for_service('/hri/tell_human')
	tell_human = rospy.ServiceProxy('/hri/tell_human', AskHuman)
	
	listener()
