#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from std_msgs.msg import Empty
from robotx_gazebo.msg import UsvDrive


class Trajectory(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        # Subscribers
        self.sub_time_stright = rospy.Subscriber('/time_trigger_stright', Empty, self.cb_stright, queue_size = 1)
        self.sub_time_right = rospy.Subscriber('/time_trigger_right', Empty, self.cb_right, queue_size = 1)
        self.sub_time_left = rospy.Subscriber('/time_trigger_left', Empty, self.cb_left, queue_size = 1)
        self.sub_time_s_shape = rospy.Subscriber('/time_trigger_s_shape', Empty, self.cb_s_shape, queue_size = 1)
        self.sub_time_r_shape = rospy.Subscriber('/time_trigger_r_shape', Empty, self.cb_r_shape, queue_size = 1)

        # Publishers
        self.pub_cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size = 1)
        self.count = 0

    def cb_stright(self, empty):
        self.rostime_control(1.0, 1.0, 5)

    def cb_right(self, empty):
        self.rostime_control(1.0, 0.8, 5)

    def cb_left(self, empty):
        self.rostime_control(0.8, 1.0, 5)

    def cb_s_shape(self, empty):
        self.rostime_control(1.0, 1.0, 2)
        self.rostime_control(0.6, 1.0, 2)
        self.rostime_control(1.0, 1.0, 2)
        self.rostime_control(1.0, 0.6, 2)
        self.rostime_control(1.0, 1.0, 2)

    def cb_r_shape(self, empty):
        self.rectangle()
        self.rectangle()
        self.rectangle()
        self.rectangle()
        self.rectangle()
        self.rectangle()


    def rectangle(self):
        self.rostime_control(1.0, 1.0, 1.6)
        self.rostime_control(0.4, 1.0, 1)
        self.rostime_control(1.0, 1.0, 1.6)
        self.rostime_control(0.4, 1.0, 1)
        self.rostime_control(1.0, 1.0, 1.6)
        self.rostime_control(0.4, 1.0, 1)
        self.rostime_control(1.0, 1.0, 1.6)
        self.rostime_control(0.4, 1.0, 1)

    def rostime_control(self, left, right, time):
        usv_drive = UsvDrive()
        usv_drive.left = left
        usv_drive.right = right
        rate = rospy.Rate(10)
        for i in range(int(time*10.0)):
            print self.count
            self.count += 1
            self.pub_cmd_drive.publish(usv_drive)
            rate.sleep()
        for i in range(3):
            usv_drive.left = 0.0
            usv_drive.right = 0.0
            print self.count
            self.count += 1
            self.pub_cmd_drive.publish(usv_drive)
            rate.sleep()


    def realtime_control(self, left, right):
        usv_drive = UsvDrive()
        usv_drive.left = left
        usv_drive.right = right
        while(self.count < 20):
            self.pub_cmd_drive.publish(usv_drive)
            self.count += 1
            print self.count

    def onShutdown(self):
        rospy.loginfo("[ModelStatePathNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('trajectory',anonymous=False)
    trajectory = Trajectory()
    rospy.on_shutdown(trajectory.onShutdown)
    rospy.spin()
