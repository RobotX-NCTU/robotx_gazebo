#!/usr/bin/python
import time
import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from robotx_gazebo.msg import UsvDrive
from std_msgs.msg import Float32
from std_srvs.srv import *
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
start_flag_cw = 0
start_flag_ccw = 0
x_pos = 0.0000001   # to prevent divid by zero
y_pos = 0
x_vel = 0
y_vel = 0
yaw = 0

class PID:

    def __init__(self, P=0.2, I=0.0, D=0.0, set_point=0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = set_point
        self.clear()

    def clear(self):
        #self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):

        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

def cb_nav(msg):
    global x_pos, x_vel, y_pos, y_vel, yaw
    # get pos, vel
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    x_vel = msg.twist.twist.linear.x
    y_vel = msg.twist.twist.linear.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

def start_circling_cw_handler(req):
    global start_flag_cw
    print "start circling clockwise"
    start_flag_cw = 1
    res = TriggerResponse()
    res.success = True
    res.message = "start circling clockwise"
    return res
def start_circling_ccw_handler(req):
    global start_flag_ccw
    print "start circling counter-clockwise"
    start_flag_ccw = 1
    res = TriggerResponse()
    res.success = True
    res.message = "start circling counter-clockwise"
    return res


if __name__ == "__main__":
    rospy.init_node("wamv_pid_node")
    rospy.Subscriber("/odometry/filtered", Odometry, cb_nav)
    pub_vel = rospy.Publisher("/cmd_drive", UsvDrive, queue_size = 20)
    pub_d = rospy.Publisher("/d", Float32, queue_size = 20)
    pub_phi = rospy.Publisher("/phi", Float32, queue_size = 20)
    startcw_srv = rospy.Service("/start_circling_cw", Trigger, start_circling_cw_handler)
    startccw_srv = rospy.Service("/start_circling_ccw", Trigger, start_circling_ccw_handler)
    targetpoint_x = 0
    targetpoint_y = 0
    d_pid = PID(P=0.1, I=0, D=0.05, set_point=5)
    phi_pid = PID(P=0.2, I=0, D=0, set_point=0)
    
    while not rospy.is_shutdown():
        
        # calculate current d and phi
        if start_flag_cw == 1:
            d = np.sqrt((x_pos-targetpoint_x)*(x_pos-targetpoint_x)+(y_pos-targetpoint_y)*(y_pos-targetpoint_y))
            theta = math.atan2((y_pos-targetpoint_y), (x_pos-targetpoint_x))
            if theta < 0:
                theta = theta + 2*np.pi
            if yaw < 0:
                yaw = yaw + 2*np.pi
            phi = -(theta - yaw - np.pi/2)
            if phi < -np.pi:
                phi = phi + 2*np.pi
            if phi > np.pi:
                phi = phi - 2*np.pi
            # tangent line direction vector: -(y_pos-targetpoint_y)/d, (x_pos-targetpoint_x)/d
        
            d_pid.update(d)
            phi_pid.update(phi)

        
            #print d, phi
            #print phi, phi_pid.output
            #print d, d_pid.output
            print d, phi, d_pid.output, phi_pid.output
            pub_d.publish(d)
            pub_phi.publish(phi)
        
            msg = UsvDrive()
            msg.left = 0.3 - d_pid.output - phi_pid.output
            msg.right = 0.3 + d_pid.output + phi_pid.output
            pub_vel.publish(msg)


        if start_flag_ccw == 1:

            d = np.sqrt((x_pos-targetpoint_x)*(x_pos-targetpoint_x)+(y_pos-targetpoint_y)*(y_pos-targetpoint_y))
            theta = math.atan2((y_pos-targetpoint_y), (x_pos-targetpoint_x))
            if theta < 0:
                theta = theta + 2*np.pi

            if yaw < 0:
                yaw = yaw + 2*np.pi

            phi = -(theta - yaw + np.pi/2)
            if phi < -np.pi:
                phi = phi + 2*np.pi
            if phi > np.pi:
                phi = phi - 2*np.pi
            # tangent line direction vector: -(y_pos-targetpoint_y)/d, (x_pos-targetpoint_x)/d
        
            d_pid.update(d)
            phi_pid.update(phi)
            #print theta, yaw, phi
            #print d, phi
            #print phi, phi_pid.output
            #print d, d_pid.output
            print d, phi, d_pid.output, phi_pid.output
            pub_d.publish(d)
            pub_phi.publish(phi)

            msg = UsvDrive()
            msg.left = 0.3 + d_pid.output - phi_pid.output
            msg.right = 0.3 - d_pid.output + phi_pid.output
            pub_vel.publish(msg)