#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# @brief rviz上で定規の機能をするrqtモジュール
# @author Atsushi Sakai

import roslib
roslib.load_manifest('RvizRuler')
import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer

from std_msgs.msg import Int8
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from tf.transformations import euler_from_quaternion
import math

count=0

class MyPlugin(Plugin):
  def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        #final Inputの取得 TODO:ちゃんと表示させるようにする
        rospy.Subscriber('initialpose',PoseWithCovarianceStamped,self.InitialPoseCallback)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",dest="quiet",help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
           print 'arguments: ', args
           print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RvizRuler.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        
        #Topicデータ表示用タイマ
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.printdata)
        #このスロットが一定周期で呼ばれる
        self._timer_refresh_topics.start(500)#スレッドのループ周期 msec
        
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
          self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        def shutdown_plugin(self):
          # TODO unregister all publishers here
          pass

        def save_settings(self, plugin_settings, instance_settings):
          # TODO save intrinsic configuration, usually using:
          # instance_settings.set_value(k, v)
          pass

        def restore_settings(self, plugin_settings, instance_settings):
          # TODO restore intrinsic configuration, usually using:
          # v = instance_settings.value(k)
          pass
        #def trigger_configuration(self):
          # Comment in to signal that the plugin has a way to configure it
          # Usually used to open a configuration dialog
         
  def InitialPoseCallback(self,data):
    global InitMsg
    InitMsg=data

  def printdata(self):
    global InitMsg
    x="%3.2f" %(InitMsg.pose.pose.position.x)
    self._widget.x.setPlainText(x)
    y="%3.2f" %(InitMsg.pose.pose.position.y)
    self._widget.y.setPlainText(y) 
    (roll,pitch,yaw) = euler_from_quaternion([InitMsg.pose.pose.orientation.x,InitMsg.pose.pose.orientation.y,InitMsg.pose.pose.orientation.z,InitMsg.pose.pose.orientation.w]);
    yaw="%3.2f" %(yaw*180/math.pi)
    self._widget.yaw.setPlainText(yaw)
