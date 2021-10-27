import rclpy
import numpy as np
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
import tf_transformations

class CameraPoseEstimator():

    def __init__(self):
        rclpy.init(args=None)
        self.node = Node('camera_pose_estimator')
        self.log = self.node.get_logger()
        self.log.info('Reading parameters...')
        self.get_params()
        if self.publish_tags_tf:
            self.log.info('Publishing static transformations...')
            self.static_tf()
        self.log.info('Initializing tf broadcasters...')
        self.init_tf()
        self.log.info('Pose estimation running.')
        self.sub = self.node.create_subscription(AprilTagDetectionArray, 
                '/apriltag_detections', self.listener_callback, 10 )

    def get_params(self):
        # base_frame
        self.node.declare_parameter('base_frame', '')
        self.base_frame = \
            self.node.get_parameter('base_frame').get_parameter_value().string_value
        if not self.base_frame:
            self.log.error(f'\'base_frame\' parameter not defined.')
            raise RuntimeError()
        # camera_frame
        self.node.declare_parameter('camera_frame', '')
        self.camera_frame = \
            self.node.get_parameter('camera_frame').get_parameter_value().string_value
        if not self.camera_frame:
            self.log.error(f'\'camera_frame\' parameter not defined.')
            raise RuntimeError()
        # publish_tags_tf
        self.node.declare_parameter('publish_tags_tf', False)
        self.publish_tags_tf = \
            self.node.get_parameter('publish_tags_tf').get_parameter_value().bool_value
        # tag_frame_prefix
        if self.publish_tags_tf:
            self.node.declare_parameter('tag_frame_prefix', '')
            self.tag_frame_prefix = \
                self.node.get_parameter('tag_frame_prefix').get_parameter_value().string_value
            if not self.tag_frame_prefix:
                self.log.error(f'\'tag_frame_prefix\' parameter not defined.')
                raise RuntimeError()
        # tags_ids
        self.node.declare_parameter('tags_ids', [])
        self.tags_ids = \
            self.node.get_parameter('tags_ids').get_parameter_value().integer_array_value
        if not self.tags_ids:
            self.log.error(f'\'tags_ids\' parameter not defined.')
            raise RuntimeError()
        # tags_x
        self.node.declare_parameter('tags_x', [])
        self.tags_x = \
            self.node.get_parameter('tags_x').get_parameter_value().double_array_value
        if not self.tags_x:
            self.log.error(f'\'tags_x\' parameter not defined.')
            raise RuntimeError()
        # tags_y
        self.node.declare_parameter('tags_y', [])
        self.tags_y = \
            self.node.get_parameter('tags_y').get_parameter_value().double_array_value
        if not self.tags_y:
            self.log.error(f'\'tags_y\' parameter not defined.')
            raise RuntimeError()
        
        if not(len(self.tags_ids) == len(self.tags_x) == len(self.tags_y)):
            self.log.error(f'\'tags_ids\', \'tags_x\' and \'tags_y\' parameter must be same lenght.')
            raise RuntimeError()
        
        self.tags_x_dict = {}
        self.tags_y_dict = {}
        for i in range(len(self.tags_ids)):
            self.tags_x_dict[self.tags_ids[i]] = self.tags_x[i]
            self.tags_y_dict[self.tags_ids[i]] = self.tags_y[i]


    def static_tf(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.node.get_clock().now().to_msg()
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
            tf_static_publisher = StaticTransformBroadcaster(self.node)
            tf_static_publisher.sendTransform(static_transformStamped)

            static_transformStamped.header.frame_id = f'tag36h11:{tag_id}'
            static_transformStamped.child_frame_id = f'tag36h11:{tag_id}_'
            static_transformStamped.transform.translation.x = 0.0
            static_transformStamped.transform.translation.y = 0.0
            static_transformStamped.transform.translation.z = 0.0
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.7071
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 0.7071
            tf_static_publisher2 = StaticTransformBroadcaster(self.node)
            tf_static_publisher2.sendTransform(static_transformStamped)



    def init_tf(self):
        self.tf_broadcasters = {}
        for tag_id in self.tags_ids:
            self.tf_broadcasters[tag_id] = TransformBroadcaster(self.node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)


    def run(self) -> None:
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()


    def broadcast_tf(self, parent_frame, child_frame, transform):
        trans = tf_transformations.translation_from_matrix(transform)
        rot = tf_transformations.quaternion_from_matrix(transform)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        self.tf_broadcasters[1].sendTransform(t)

    def listener_callback(self, msg: AprilTagDetectionArray):
        detections_count = 0
        cum_trans = np.array([0.0, 0.0, 0.0])
        cum_rot_euler = np.array([0.0, 0.0, 0.0])
        for detection in msg.detections:
            if detection.id == 11:
                tag_id = detection.id
                try:
                    frame_trans_msg = self.tf_buffer.lookup_transform(
                        f'tag36h11:{tag_id}_',
                        'camera_link',
                        rclpy.time.Time() )
                    frame_trans = tf_transformations.translation_matrix([
                                        frame_trans_msg.transform.translation.x,
                                        frame_trans_msg.transform.translation.y,
                                        frame_trans_msg.transform.translation.z 
                                    ])
                    frame_rot = tf_transformations.quaternion_matrix([
                                        frame_trans_msg.transform.rotation.x,
                                        frame_trans_msg.transform.rotation.y,
                                        frame_trans_msg.transform.rotation.z,
                                        frame_trans_msg.transform.rotation.w 
                                    ])
                    frame_transform = tf_transformations.concatenate_matrices(frame_trans, frame_rot)
                except TransformException:
                    continue
                frame_transform_trans = tf_transformations.translation_from_matrix(frame_transform)
                frame_transform_rot = tf_transformations.quaternion_from_matrix(frame_transform)
                frame_transform_trans[0] += self.tags_x_dict[tag_id]
                frame_transform_trans[1] += self.tags_y_dict[tag_id]
                frame_trans = tf_transformations.translation_matrix(frame_transform_trans)
                frame_rot = tf_transformations.quaternion_matrix(frame_transform_rot)
                frame_transform_2 = tf_transformations.concatenate_matrices(frame_trans, frame_rot)
                self.broadcast_tf('world', 'camera_link', frame_transform_2)

        #     if detection.id in self.tf_broadcasters:
        #         detections_count += 1
        #         
        #         trans = tf_transformations.translation_matrix([
        #                 detection.pose.pose.pose.position.x,
        #                 detection.pose.pose.pose.position.y,
        #                 detection.pose.pose.pose.position.z 
        #             ])
        #         rot = tf_transformations.quaternion_matrix([
        #                 detection.pose.pose.pose.orientation.x,
        #                 detection.pose.pose.pose.orientation.y,
        #                 detection.pose.pose.pose.orientation.z,
        #                 detection.pose.pose.pose.orientation.w,
        #             ])
        #         Ry = tf_transformations.rotation_matrix(np.pi/2, (0,1,0))
        #         transform = tf_transformations.concatenate_matrices(trans, rot, Ry)

        #         # trans = tf_transformations.translation_from_matrix(transform)
        #         # rot = tf_transformations.quaternion_from_matrix(transform)
        #         inversed_transform = tf_transformations.inverse_matrix(transform)

        #         Rx = tf_transformations.rotation_matrix(np.pi/2, (1,0,0))
        #         Rz = tf_transformations.rotation_matrix(np.pi/2, (0,0,1))
        #         inversed_transform = tf_transformations.concatenate_matrices(inversed_transform, Rx, Rz)
                
        #         trans_inv = tf_transformations.translation_from_matrix(inversed_transform)
        #         rot_inv = tf_transformations.quaternion_from_matrix(inversed_transform)
        #         rot_inv_euler = tf_transformations.euler_from_quaternion(rot_inv)

        #         cum_trans[0] += trans_inv[0] + self.tags_x_dict[tag_id]
        #         cum_trans[1] += trans_inv[1] + self.tags_y_dict[tag_id]
        #         cum_trans[2] += trans_inv[2]
        #         cum_rot_euler[0] += rot_inv_euler[0]
        #         cum_rot_euler[1] += rot_inv_euler[1]
        #         cum_rot_euler[2] += rot_inv_euler[2]

        # if detections_count>0:
        #     avg_trans = cum_trans / detections_count
        #     avg_rot_euler = cum_rot_euler / detections_count
        #     avg_rot = tf_transformations.quaternion_from_euler(
        #             avg_rot_euler[0], avg_rot_euler[1], avg_rot_euler[2])
        #     t = TransformStamped()
        #     t.header.stamp = self.node.get_clock().now().to_msg()
        #     t.header.frame_id = self.base_frame
        #     t.child_frame_id = f'{self.camera_frame}'
        #     t.transform.translation.x = avg_trans[0]
        #     t.transform.translation.y = avg_trans[1]
        #     t.transform.translation.z = avg_trans[2]
        #     t.transform.rotation.x = avg_rot[0]
        #     t.transform.rotation.y = avg_rot[1]
        #     t.transform.rotation.z = avg_rot[2]
        #     t.transform.rotation.w = avg_rot[3]
        #     self.tf_broadcasters[tag_id].sendTransform(t)




def main():
    simple_publisher = CameraPoseEstimator()
    simple_publisher.run()


if __name__ == '__main__':
    main()
