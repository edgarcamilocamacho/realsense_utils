#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf.transformations

class CameraPoseEstimator():

	def __init__(self):
		rospy.init_node("apriltags_tf")       
		self.get_params()
		if self.publish_tags_tf:
			rospy.loginfo('Publishing static transformations...')
			self.static_tf()
		rospy.loginfo('Initializing tf broadcasters...')
		self.init_tf()
		rospy.loginfo('Pose estimation running')
		self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.listener_callback)


	
	def get_params(self):
		self.base_frame = rospy.get_param('base_frame')
		if not self.base_frame:
			raise RuntimeError()
		self.camera_frame = rospy.get_param('camera_frame')
		if not self.camera_frame:
			raise RuntimeError()
		self.publish_tags_tf = rospy.get_param('publish_tags_tf')
		if self.publish_tags_tf:
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
		static_transformStamped.header.frame_id = self.base_frame
		static_transformStamped.transform.translation.z = 0.0
		static_transformStamped.transform.rotation.x = 0.0
		static_transformStamped.transform.rotation.y = 0.0
		static_transformStamped.transform.rotation.z = 0.0
		static_transformStamped.transform.rotation.w = 1.0
		for i in range(len(self.tags_ids)):
			self.tf_static_publishers.append(tf2_ros.StaticTransformBroadcaster())
			tag_id = self.tags_ids[i]
			tag_x = self.tags_x[i]
			tag_y = self.tags_y[i]
			static_transformStamped.child_frame_id = f'{self.tag_frame_prefix}{tag_id}'
			static_transformStamped.transform.translation.x = tag_x
			static_transformStamped.transform.translation.y = tag_y
			self.tf_static_publishers[i].sendTransform(static_transformStamped)

	def init_tf(self):
		# self.tf_broadcasters = {}
		# for tag_id in self.tags_ids:
		# 	self.tf_broadcasters[tag_id] = tf2_ros.TransformBroadcaster()
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

	def listener_callback(self, msg):
		detections_count = 0
		cum_trans = np.array([0.0, 0.0, 0.0])
		cum_rot_euler = np.array([0.0, 0.0, 0.0])
		for detection in msg.detections:
			if detection.id[0] == 1:
				rospy.loginfo('detection 1')
				tag_id = detection.id[0]
				trans = tf.transformations.translation_matrix([
						detection.pose.pose.pose.position.x,
						detection.pose.pose.pose.position.y,
						detection.pose.pose.pose.position.z 
					])
				rot = tf.transformations.quaternion_matrix([
						detection.pose.pose.pose.orientation.x,
						detection.pose.pose.pose.orientation.y,
						detection.pose.pose.pose.orientation.z,
						detection.pose.pose.pose.orientation.w,
					])
				Ry = tf.transformations.rotation_matrix(np.pi/2, (0,0,1))
				transform = tf.transformations.concatenate_matrices(trans, rot, Ry)

				# trans = tf_transformations.translation_from_matrix(transform)
				# rot = tf_transformations.quaternion_from_matrix(transform)
				inversed_transform = tf.transformations.inverse_matrix(transform)

				Rx = tf.transformations.rotation_matrix(np.pi/2, (1,0,0))
				Rz = tf.transformations.rotation_matrix(np.pi/2, (0,0,1))
				inversed_transform = tf.transformations.concatenate_matrices(inversed_transform, Rx, Rz)
				
				trans_inv = tf.transformations.translation_from_matrix(inversed_transform)
				rot_inv = tf.transformations.quaternion_from_matrix(inversed_transform)
				rot_inv_euler = tf.transformations.euler_from_quaternion(rot_inv)

				# cum_trans[0] += trans_inv[0] + self.tags_x_dict[tag_id]
				# cum_trans[1] += trans_inv[1] + self.tags_y_dict[tag_id]
				# cum_trans[2] += trans_inv[2]
				# cum_rot_euler[0] += rot_inv_euler[0]
				# cum_rot_euler[1] += rot_inv_euler[1]
				# cum_rot_euler[2] += rot_inv_euler[2]

		# if detections_count>0:
		# 	avg_trans = cum_trans / detections_count
		# 	avg_rot_euler = cum_rot_euler / detections_count
		# 	avg_rot = tf.transformations.quaternion_from_euler(
		# 			avg_rot_euler[0], avg_rot_euler[1], avg_rot_euler[2])
				rot_inv_quat = tf.transformations.quaternion_from_euler(
					rot_inv_euler[0], rot_inv_euler[1], rot_inv_euler[2])
				t = TransformStamped()
				t.header.stamp = rospy.Time.now()
				t.header.frame_id = self.base_frame
				t.child_frame_id = self.camera_frame
				t.transform.translation.x = trans_inv[0]# + self.tags_x_dict[tag_id]
				t.transform.translation.y = trans_inv[1]# + self.tags_y_dict[tag_id]
				t.transform.translation.z = trans_inv[2]
				t.transform.rotation.x = rot_inv_quat[0]
				t.transform.rotation.y = rot_inv_quat[1]
				t.transform.rotation.z = rot_inv_quat[2]
				t.transform.rotation.w = rot_inv_quat[3]
				self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
	simple_publisher = CameraPoseEstimator()
	while not rospy.is_shutdown():
		rospy.spin()