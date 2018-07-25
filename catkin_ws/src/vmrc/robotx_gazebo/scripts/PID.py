#!/usr/bin/python
import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from robotx_gazebo.msg import UsvDrive
import tf
x_pos = 0.0000001
y_pos = 0
yaw = 0
class PID:

    def __init__(self, P=0.1, I=0.005, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.pos_SetPointx = 0.0
        self.pos_SetPointy = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, x_pos, y_pos, yaw):
        
        error = np.arctan2((self.pos_SetPointy - y_pos),(self.pos_SetPointx - x_pos)) - yaw
        pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
        if np.abs(pos_error) < 1:
            error = 0

        #print "error", np.arctan((self.pos_SetPointy - y_pos)/(self.pos_SetPointx - x_pos)) - yaw
        if np.abs(error)> np.pi:
            if error > 0:
                error = error-2*np.pi
            else:
                error = error+2*np.pi

        #print "error", error
        #print "yaw", yaw
        #print "target", np.arctan2((self.pos_SetPointy - y_pos), (self.pos_SetPointx - x_pos))
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
def cb_vel(msg):
    global x_pos, y_pos, yaw 
    # get pos, vel
    x_pos = -msg.pose.pose.position.y
    y_pos = msg.pose.pose.position.x
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
    #print "x", x_pos
    #print "y", y_pos

if __name__ == "__main__":
    rospy.init_node("pid_ang_vel_node")
    rospy.Subscriber("/odometry/filtered", Odometry, cb_vel)
    pub_vel = rospy.Publisher("/cmd_drive_ang", UsvDrive, queue_size = 20)
    pid = PID()
    pid.pos_SetPointx = 20
    pid.pos_SetPointy = 0
    while not rospy.is_shutdown():
        pid.update(x_pos, y_pos, yaw)
        print pid.output
        msg = UsvDrive()
        if abs(pid.output) < 0.05:
            pid.output = 0
        msg.left = -pid.output
        msg.right = pid.output
        pub_vel.publish(msg)
        rospy.sleep(0.05)
