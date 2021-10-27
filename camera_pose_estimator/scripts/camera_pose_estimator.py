#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf.transformations
import tf

class CameraPoseEstimator():

    def __init__(self):
        rospy.init_node("apriltags_tf")       
        self.get_params()
        rospy.loginfo('Publishing static transformations...')
        self.static_tf()
        rospy.loginfo('Initializing tf broadcasters...')
        self.init_tf()
        rospy.loginfo('Pose estimation running')
        self.tf_listener = tf.TransformListener()
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.listener_callback)
    
    def get_params(self):
        self.base_frame = rospy.get_param('base_frame')
        if not self.base_frame:
            raise RuntimeError()
        self.camera_frame = rospy.get_param('camera_frame')
        if not self.camera_frame:
            raise RuntimeError()
        self.tag_frame_prefix = rospy.get_param('tag_frame_prefix')
        if not self.tag_frame_prefix:
            raise RuntimeError()
        self.tags_ids = rospy.get_param('tags_ids')
        if not self.tags_ids:
            raise RuntimeError()
        self.tags_x = rospy.get_param('tags_x')
        if not self.tags_x:
            raise RuntimeError()
        self.tags_y = rospy.get_param('tags_y')
        if not self.tags_y:
            raise RuntimeError()
        if not(len(self.tags_ids) == len(self.tags_x) == len(self.tags_y)):
            raise RuntimeError()
        self.tags_x_dict = {}
        self.tags_y_dict = {}
        for i in range(len(self.tags_ids)):
            self.tags_x_dict[self.tags_ids[i]] = self.tags_x[i]
            self.tags_y_dict[self.tags_ids[i]] = self.tags_y[i]
        print(self.base_frame)


    def static_tf(self):
        self.tf_static_publishers = []
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        for i in range(len(self.tags_ids)):
            tag_id = self.tags_ids[i]
            tag_x = self.tags_x[i]
            tag_y = self.tags_y[i]
            static_transformStamped.header.frame_id = self.base_frame
            static_transformStamped.child_frame_id = f'{self.tag_frame_prefix}{tag_id}'
            static_transformStamped.transform.translation.x = tag_x
            static_transformStamped.transform.translation.y = tag_y
            static_transformStamped.transform.translation.z = 0.0
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 1.0
            self.tf_static_publishers.append(tf2_ros.StaticTransformBroadcaster())
            self.tf_static_publishers[-1].sendTransform(static_transformStamped)

            static_transformStamped.header.frame_id = f'tag_{tag_id}'
            static_transformStamped.child_frame_id = f'tag_{tag_id}_'
            static_transformStamped.transform.translation.x = 0.0
            static_transformStamped.transform.translation.y = 0.0
            static_transformStamped.transform.translation.z = 0.0
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = -0.7071
            static_transformStamped.transform.rotation.w = 0.7071
            self.tf_static_publishers.append(tf2_ros.StaticTransformBroadcaster())
            self.tf_static_publishers[-1].sendTransform(static_transformStamped)


    def init_tf(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


    def listener_callback(self, msg):
        # detections_count = 0
        # cum_trans = np.array([0.0, 0.0, 0.0])
        # cum_rot_euler = np.array([0.0, 0.0, 0.0])
        for detection in msg.detections:
            if detection.id[0] in self.tags_ids:
                tag_id = detection.id[0]
                try:
                    (trans,rot) = self.tf_listener.lookupTransform(
                        f'tag_{tag_id}_',
                        'camera_link', 
                        rospy.Time(0) )
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                trans[0] += self.tags_x_dict[tag_id]
                trans[1] += self.tags_y_dict[tag_id]
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.base_frame
                t.child_frame_id = self.camera_frame
                t.transform.translation.x = trans[0]
                t.transform.translation.y = trans[1]
                t.transform.translation.z = trans[2]
                t.transform.rotation.x = rot[0]
                t.transform.rotation.y = rot[1]
                t.transform.rotation.z = rot[2]
                t.transform.rotation.w = rot[3]
                self.tf_broadcaster.sendTransform(t)

                break

if __name__ == '__main__':
    simple_publisher = CameraPoseEstimator()
    while not rospy.is_shutdown():
        rospy.spin()