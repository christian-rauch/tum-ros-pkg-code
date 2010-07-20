#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_state_gui')
from sensor_msgs.msg import JointState
import rospy

from math import pi

import wx

class SliderPanel(wx.Panel):
  """ creates a panel with sliders """
  def __init__(self, parent, id, slider_names):
    wx.Panel.__init__(self, parent, id)

    self.SetBackgroundColour("white")
    self.sizer = wx.FlexGridSizer(2, 2, 0, 0)
    self.sizer.AddGrowableCol(1, 1)
    self.SetSizer(self.sizer)

    self.Bind(wx.EVT_SLIDER, self._slider_update)

    self.joint_names = slider_names
    self.joint_angles = [0.0]*len(self.joint_names)

    #Add labels and sliders
    for i in range(len(self.joint_names)):
      l = wx.StaticText(self, id=-1, label=self.joint_names[i])
      s = wx.Slider(self, id=i,
            style = wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
      s.SetRange(-180, 180)
      s.SetSizeHints(360, 40)

      self.sizer.Add(l, 1, wx.ALIGN_BOTTOM)
      self.sizer.Add(s, 1, wx.EXPAND)

  def set_slider_cb(self, callback):
    self.slider_callback = callback

  def _slider_update(self, event):
    self.joint_angles[event.Id] = event.Int
    self.slider_callback(self.joint_angles, self.joint_names)


class RosComm:
  def __init__(self):
    """ initialize ROS """

    self.joint_names = None

    rospy.init_node('joint_gui')
    self.name = rospy.names.get_name()

    if rospy.client.has_param('~axes'):
      self.joint_names = rospy.client.get_param('~axes')

    self._pub = rospy.Publisher('/joint_states', JointState)

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(100)

  def _on_shutdown_timer(self, event):
    """shut down the program when the node closes"""
    if rospy.is_shutdown():
      wx.Exit()

  def send_angles(self, joint_angles, joint_names):
    """ publish joint states """
    jss=JointState()
    for i in range(len(joint_names)):
      jss.name.append(joint_names[i])
      jss.position.append(joint_angles[i]*pi/180.0)
      jss.velocity.append(0.0)
      jss.header.stamp=rospy.Time.now()

    self._pub.publish(jss)


# main

if __name__ == "__main__":

  app = wx.PySimpleApp()

  ros = RosComm()

  if ros.joint_names == None:
    rospy.logerr("parameter ~axes not set")
    exit()

  frame = wx.Frame(None, -1, ros.name, size = (400, 310))
  panel = SliderPanel(frame,-1,ros.joint_names)
  panel.set_slider_cb(ros.send_angles)

  frame.ClientSize = panel.BestSize

  frame.Show(True)
  app.MainLoop()

