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

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

count=0

class MyPlugin(Plugin):
  def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        #Rvizから位置を受け取るsubscriver
        rospy.Subscriber('initialpose',PoseWithCovarianceStamped,self.InitialPoseCallback)
        global pub
        pub = rospy.Publisher('line', Marker)
        #グローバル変数の初期化
        global InitMsg
        InitMsg=0
        global preInitMsg
        preInitMsg=0


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
    #rvizからのデータを取得するCallback関数
    global InitMsg
    global preInitMsg
    if InitMsg !=0:
      preInitMsg=InitMsg  #global変数に一つ前のデータを格納
    InitMsg=data        #global変数に現在のデータを格納

  def printdata(self):
    #GUIにデータを表示する関数
    global InitMsg
    if isinstance(InitMsg, int):#InitMsgが初期化されていない
      return 0
    fx=InitMsg.pose.pose.position.x
    fy=InitMsg.pose.pose.position.y
    x="%3.2f" %(fx)
    self._widget.x.setPlainText(x)
    y="%3.2f" %(fy)
    self._widget.y.setPlainText(y) 

    #オイラー角の取得
    ox=InitMsg.pose.pose.orientation.x
    oy=InitMsg.pose.pose.orientation.y
    oz=InitMsg.pose.pose.orientation.z
    ow=InitMsg.pose.pose.orientation.w
    (roll,pitch,yaw) = euler_from_quaternion([ox,oy,oz,ow]);
    yaw="%3.2f" %(yaw*180/math.pi)
    self._widget.yaw.setPlainText(yaw)

    #原点からの距離を計算
    dorigin="%3.2f" %(math.sqrt(fx*fx+fy*fy))
    self._widget.dorigin.setPlainText(dorigin)

    #原点からの角度を計算
    thetaorigin="%3.2f" %(math.atan2(fy,fx)*180/math.pi)
    self._widget.thetaorigin.setPlainText(thetaorigin)

    #前回の点からの距離を計算
    if isinstance(preInitMsg, int): #preInitMsgが初期化されていない
      return
    dx=fx-preInitMsg.pose.pose.position.x #xの変化量[m]
    dy=fy-preInitMsg.pose.pose.position.y #yの変化量[m]
    dprepos="%3.2f" %(math.sqrt(dx*dx+dy*dy)) #ノルム距離の計算
    self._widget.dprepos.setPlainText(dprepos)

    #前回の点からの角度を計算
    thetaprepos="%3.2f" %(math.atan2(dy,dx)*180/math.pi)  #角度[rad]の計算
    self._widget.thetaprepos.setPlainText(thetaprepos)

    #線描画用Markerのpublish
    marker = Marker()
    marker.header.frame_id = "nav";
    marker.ns = "Line";
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.frame_locked = True;
    marker.scale.x = 0.5; #線の太さ
    marker.color.a = 1.0; #透明度
    marker.color.g = 1.0; #色は緑
    marker.lifetime.secs = 1.0;#表示時間[s]

    p=Point();#始点
    p.x=InitMsg.pose.pose.position.x
    p.y=InitMsg.pose.pose.position.y
    marker.points.append(p);

    p=Point();#終点
    p.x=preInitMsg.pose.pose.position.x
    p.y=preInitMsg.pose.pose.position.y
    marker.points.append(p);

    pub.publish(marker);#publish




