#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates


class ModelStatePath(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        # previous position of vehicle
        self.pre_vehicle_loalization = Point()

        # apriltag localization path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("gazebo/model_states", ModelStates, self.cbModelStates2Pose, queue_size=1)

        # Publishers
        self.pub_path = rospy.Publisher('~wamv_model_path', Path, queue_size = 20)

    def cb_path(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose = pose
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.seq = pose_stamped.header.seq
        pose_stamped.header.stamp = self.path_msg.header.stamp
        self.path_msg.poses.append(pose_stamped)
        self.pub_path.publish(self.path_msg)
        print 'poses------------'
        print self.path_msg.poses
        print '------------'


    def cbModelStates2Pose(self, model_states):
        #print model_states
        wamv_id = 0
        for name in model_states.name:
            if(name != 'wamv'):
                wamv_id = wamv_id + 1
            else:
                if(self.verbose): print model_states.name[wamv_id], model_states.pose[wamv_id]
                pose = Pose()
                pose = model_states.pose[wamv_id]
                pose.position.x = pose.position.x + 5
                pose.position.y = pose.position.y - 5
                self.cb_path(pose)

    def onShutdown(self):
        rospy.loginfo("[ModelStatePathNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('model_state_path_node',anonymous=False)
    model_state_path_node = ModelStatePath()
    rospy.on_shutdown(model_state_path_node.onShutdown)
    rospy.spin()
