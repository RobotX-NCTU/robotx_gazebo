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

        self.max_speed = 0.65
        self.time = 1
        self.empty = Empty()

        # Subscribers
        self.sub_time_stright = rospy.Subscriber('/time_trigger_stright', Empty, self.cb_stright, queue_size = 1)
        self.sub_time_right = rospy.Subscriber('/time_trigger_right', Empty, self.cb_right, queue_size = 1)
        self.sub_time_left = rospy.Subscriber('/time_trigger_left', Empty, self.cb_left, queue_size = 1)
        self.sub_time_rotate_right = rospy.Subscriber('/time_trigger_rotate_right', Empty, self.cb_rotate_right, queue_size = 1)
        self.sub_time_rotate_left = rospy.Subscriber('/time_trigger_rotate_left', Empty, self.cb_rotate_left, queue_size = 1)
        self.sub_time_s_shape = rospy.Subscriber('/time_trigger_s_shape', Empty, self.cb_s_shape, queue_size = 1)
        self.sub_time_r_shape = rospy.Subscriber('/time_trigger_r_shape', Empty, self.cb_r_shape, queue_size = 1)

        # Publishers
        self.pub_cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size = 1)
        self.count = 0

    def cb_stright(self, empty):
        for j in range(4):
            for i in range(3):
                self.rostime_control_pluse(self.max_speed,self.max_speed)
        self.stop()
        #for i in range(2):
        #    self.rostime_control()

    def cb_right(self, empty):
        for j in range(4):
            for i in range(2):
                self.rostime_control_pluse(self.max_speed,self.max_speed)
            for i in range(1):
                self.rostime_control_pluse(self.max_speed,0)
        self.stop()
        #self.rostime_control(right=self.max_speed/4.0*3)
        #self.rostime_control(right=self.max_speed/4.0*3)

    def cb_left(self, empty):
        for j in range(4):
            for i in range(2):
                self.rostime_control_pluse(self.max_speed,self.max_speed)
            for i in range(1):
                self.rostime_control_pluse(0,self.max_speed)
        self.stop()
        #self.rostime_control(left=self.max_speed/4.0*3)
        #self.rostime_control(left=self.max_speed/4.0*3)

    def cb_rotate_right(self, empty):
        for j in range(4):
            for i in range(1):
                self.rostime_control_pluse(self.max_speed,self.max_speed)
            for i in range(2):
                self.rostime_control_pluse(self.max_speed,0)
        self.stop()
        #self.rostime_control(right=self.max_speed/4.0*2)

    def cb_rotate_left(self, empty):
        for j in range(4):
            for i in range(1):
                self.rostime_control_pluse(self.max_speed,self.max_speed)
            for i in range(2):
                self.rostime_control_pluse(0,self.max_speed)
        self.stop()
        #self.rostime_control(left=self.max_speed/4.0*2)

    def cb_s_shape(self, empty):
        self.cb_stright(self.empty)
        self.cb_left(self.empty)
        self.cb_stright(self.empty)
        self.cb_right(self.empty)
        self.cb_stright(self.empty)
        self.cb_stright(self.empty)

    def cb_r_shape(self, empty):
        self.rectangle()
        self.rectangle()
        self.rectangle()

    def rectangle(self):
        for i in range(1):
            self.cb_stright(self.empty)
            self.cb_rotate_left(self.empty)

    def rostime_control_pluse(self, left, right):
        usv_drive = UsvDrive()
        usv_drive.left = left
        usv_drive.right = right
        rate = rospy.Rate(3)
        print self.count
        self.count += 1
        self.pub_cmd_drive.publish(usv_drive)
        rate.sleep()

    def rostime_control(self, left=None, right=None, time=None):
        if left is None:
            left = self.max_speed
        if right is None:
            right = self.max_speed
        if time is None:
            time = self.time
        usv_drive = UsvDrive()
        usv_drive.left = left
        usv_drive.right = right
        rate = rospy.Rate(5)
        for i in range(int(time*10.0)):
            print self.count
            self.count += 1
            self.pub_cmd_drive.publish(usv_drive)
            rate.sleep()

    def stop(self):
        usv_drive = UsvDrive()
        usv_drive.left = 0.0
        usv_drive.right = 0.0
        for i in range(3):
            rate = rospy.Rate(5)
            self.pub_cmd_drive.publish(usv_drive)
            rate.sleep()
        print 'stop!'


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
