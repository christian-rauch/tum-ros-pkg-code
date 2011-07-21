#!/usr/bin/env python

import roslib; roslib.load_manifest('tf_relay')

import rospy
import tf.msg as tf
import geometry_msgs.msg as geometry_msgs

class TFRelay(object):
    def __init__(self):
        self.tf_prefix = rospy.get_param('~tf_prefix')
        self.fixed_frame = rospy.get_param('~fixed_frame', '/map')
        if self.fixed_frame[0] != '/':
            self.fixed_frame = '/' + self.fixed_frame
        self.sub = rospy.Subscriber('tf_in', tf.tfMessage, self.callback)
        self.pub = rospy.Publisher('tf_out', tf.tfMessage)

    def is_fixed_frame(self, frame_id):
        if frame_id[0] == '/' and frame_id == self.fixed_frame:
            return True
        elif frame_id == self.fixed_frame[1:]:
            return True
        else:
            return False

    def prepend_prefix(self, frame_id):
        if frame_id[0] == '/':
            return self.tf_prefix + frame_id
        else:
            return self.tf_prefix + '/' + frame_id

    def callback(self, msg):
        for trans in msg.transforms:
            if not self.is_fixed_frame(trans.header.frame_id):
                trans.header.frame_id = self.prepend_prefix(trans.header.frame_id)
            trans.child_frame_id = self.prepend_prefix(trans.child_frame_id)

        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('tf_relay')
    relay = TFRelay()
    rospy.spin()

