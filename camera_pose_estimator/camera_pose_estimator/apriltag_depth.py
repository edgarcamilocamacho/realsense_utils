import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from apriltag import apriltag
import pyrealsense2 as rs2

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker

class MarkerPublisher():

    def __init__(self, node:Node, name:str, frame:str,
                 namespace:str='', color:tuple=(0.0,1.0,0.0)):
        self.node = node
        self.pub = self.node.create_publisher(Marker, f'marker_{name}', 10)
        self.msg = Marker()
        self.msg.header.frame_id = frame
        self.msg.ns = namespace
        self.msg.type = Marker.SPHERE
        self.msg.action = Marker.ADD
        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0
        self.msg.pose.orientation.z = 0.0
        self.msg.pose.orientation.w = 1.0
        self.msg.scale.x = 0.02
        self.msg.scale.y = 0.02
        self.msg.scale.z = 0.02
        self.msg.color.a = 1.0
        self.msg.color.r = color[0]
        self.msg.color.g = color[1]
        self.msg.color.b = color[2]
    
    def publish(self, id, point_3d):
        self.msg.header.stamp = self.node.get_clock().now().to_msg()
        self.msg.id = id
        self.msg.pose.position.x = point_3d[0]/1000
        self.msg.pose.position.y = point_3d[1]/1000
        self.msg.pose.position.z = point_3d[2]/1000
        self.pub.publish(self.msg)


class ApriltagDepth():

    def __init__(self):
        rclpy.init(args=None)
        self.node = Node('camera_pose_estimator')
        self.log = self.node.get_logger()
        self.bridge = CvBridge()
        self.apriltag_detector = apriltag("tag36h11")
        self.intrinsics = None
        self.image_msg_color = None
        self.image_msg_depth = None
        self.markerPubRed = MarkerPublisher(self.node, 'hola1', 'camera_color_optical_frame', color=(1.0,0.0,0.0))
        self.markerPubGreen = MarkerPublisher(self.node, 'hola2', 'camera_color_optical_frame', color=(0.0,1.0,0.0))
        self.sub_color = self.node.create_subscription(Image,   
                '/camera/color/image_raw', self.callback_color_image, 10 )
        self.sub_depth = self.node.create_subscription(Image, 
                '/camera/aligned_depth_to_color/image_raw', self.callback_depth_image, 10 )
        self.sub_info = self.node.create_subscription(CameraInfo, 
                '/camera/aligned_depth_to_color/camera_info', self.callback_camera_info, 10 )


    def run(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()


    def callback_color_image(self, msg: Image):
        self.image_msg_color = msg
        self.check_same_frame()


    def callback_depth_image(self, msg: Image):
        self.image_msg_depth = msg
        self.check_same_frame()


    def check_same_frame(self):
        if (self.image_msg_color is not None) and (self.image_msg_depth is not None):
            if self.image_msg_color.header.stamp.sec == self.image_msg_depth.header.stamp.sec \
                    and self.image_msg_color.header.stamp.nanosec == self.image_msg_depth.header.stamp.nanosec:
                self.process_image()


    def process_image(self):
        if self.intrinsics:
            rgb_image = self.bridge.imgmsg_to_cv2(self.image_msg_color, self.image_msg_color.encoding)
            depth_image = self.bridge.imgmsg_to_cv2(self.image_msg_depth, self.image_msg_depth.encoding)
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            detections = self.apriltag_detector.detect(gray_image)
            for detection in detections:
                if detection['id']==11:
                    center_x = int(detection['center'][0])
                    center_y = int(detection['center'][1])
                    rgb_image = cv2.circle(rgb_image, (center_x,center_y), radius=10, color=(0, 0, 255), thickness=-1)
                    for i, point in enumerate(detection['lb-rb-rt-lt']):
                        corner_x = int(point[0])
                        corner_y = int(point[1])
                        rgb_image = cv2.circle(rgb_image, (corner_x,corner_y), radius=10, color=(255, 0, 0), thickness=-1)
                        corner_d = depth_image[corner_y, corner_x]
                        corner_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [corner_x, corner_y], corner_d)
                        self.markerPubGreen.publish(i, corner_3d)
                    center_d = depth_image[center_y, center_x]
                    center_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center_x, center_y], center_d)
                    self.markerPubRed.publish(10, center_3d)
            cv2.imshow('cv_image', rgb_image)
            cv2.waitKey(1)

    def callback_camera_info(self, cameraInfo: CameraInfo):
        if self.intrinsics:
                return
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.k[2]
        self.intrinsics.ppy = cameraInfo.k[5]
        self.intrinsics.fx = cameraInfo.k[0]
        self.intrinsics.fy = cameraInfo.k[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.d]
        

def main():
    simple_publisher = ApriltagDepth()
    simple_publisher.run()


if __name__ == '__main__':
    main()
