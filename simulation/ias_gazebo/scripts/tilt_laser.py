#!/usr/bin/env python

import roslib

roslib.load_manifest('ias_gazebo')
import sensor_msgs.msg
import std_msgs.msg
import rospy
import threading
import time
from pr2_msgs.msg import LaserScannerSignal

max_pos=1
min_pos=0
vel=0.5


joint_state=None
joint_state_condition=threading.Condition()

def update_joint_state(s):
    global joint_state
    
    joint_state_condition.acquire()
    joint_state=s
    joint_state_condition.notify()
    joint_state_condition.release()

def get_joint_state(name):
    joint_state_condition.acquire()
    if not joint_state:
        rv = (None, None, None)
    else:
        index = joint_state.name.index(name)
        rv = (joint_state.position[index], joint_state.velocity[index], joint_state.effort[index])
    joint_state_condition.release()
    return rv
    
def main():
    rospy.init_node('tilt_laser')
    joint_state_reader = rospy.Subscriber( '/joint_states', sensor_msgs.msg.JointState, update_joint_state )
    control_publisher = rospy.Publisher( '/tilting_laser_controller/command', std_msgs.msg.Float64 )
    topic_name='/shoulder_scanner_signal'
    pub_laser_signal=rospy.Publisher(topic_name,LaserScannerSignal)
    lss_counter=0  #laser_scanner_signal counter


    state = 'MovingUp'
    while not rospy.is_shutdown():
        joint_state_condition.acquire()
        joint_state_condition.wait()
        joint_state_condition.release()
        tilting_joint_pos = get_joint_state('shoulder_tilting_joint')[0]
        if not tilting_joint_pos:
            pass
        elif state == 'MovingUp':
            if  tilting_joint_pos >= max_pos:
                state = 'MovingDown'
                control_publisher.publish( std_msgs.msg.Float64( -vel ) )
                lss=LaserScannerSignal()
                lss.header.seq=lss_counter
                lss_counter+=1
                lss.header.stamp=rospy.Time.now()
                lss.header.frame_id='shoulder_base'
                lss.signal=1
                pub_laser_signal.publish(lss)
            else:
                control_publisher.publish( std_msgs.msg.Float64( vel ) )

        elif state == 'MovingDown':
            if tilting_joint_pos <= min_pos:
                state = 'MovingUp'
                control_publisher.publish( std_msgs.msg.Float64( vel ) )
                lss=LaserScannerSignal()
                lss.header.seq=lss_counter
                lss_counter+=1
                lss.header.stamp=rospy.Time.now()
                lss.header.frame_id='shoulder_base'
                lss.signal=1
                pub_laser_signal.publish(lss)
            else:
                control_publisher.publish( std_msgs.msg.Float64( -vel ) )

if __name__ == '__main__':
    main()
