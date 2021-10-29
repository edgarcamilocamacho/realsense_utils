import rclpy
import numpy as np
import math
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from apriltag import apriltag
import pyrealsense2 as rs2

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

# Clonas https://github.com/matthew-brett/transforms3d
# entras, "pip3 install ."
from transforms3d._gohlketransforms import superimposition_matrix

RADIUS = 4
COLOR_CENTER = (0,0,255)
COLOR_CORNERS = (0,255,0)
COLOR_LINES = (255,0,0)

MARKERS_FRAME = 'camera_color_optical_frame'

COLOR_RED =   (1.0, 0.0, 0.0)
COLOR_GREEN = (0.0, 1.0, 0.0)
COLOR_BLUE =  (0.0, 0.0, 1.0)
COLOR_WHITE = (1.0, 1.0, 1.0)


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
        self.pub_markers = self.node.create_publisher(MarkerArray, f'april_markers', 10)
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
            markers = MarkerArray()
            markers_i = 0
            # Get color and depth frames
            rgb_image = self.bridge.imgmsg_to_cv2(self.image_msg_color, self.image_msg_color.encoding)
            depth_image = self.bridge.imgmsg_to_cv2(self.image_msg_depth, self.image_msg_depth.encoding)
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            # AprilTag detection
            detections = self.apriltag_detector.detect(gray_image)            
            for detection in detections:
                if detection['id'] in [13, 19]:
                    # Punto del Centro
                    center = detection['center']
                    center_int = ( int(center[0]), int(center[1]) )
                    rgb_image = cv2.circle(rgb_image, center_int, radius=RADIUS, color=COLOR_CENTER, thickness=-1)
                    try:
                        center_d = depth_image[center_int[1], center_int[0]]
                    except IndexError:
                        continue
                    center_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, center_int, center_d)
                    markers.markers.append(self.createMarker(markers_i, center_3d, COLOR_RED))
                    markers_i += 1
                    # Esquinas
                    corners_3d = []
                    for i, corner in enumerate(detection['lb-rb-rt-lt']):
                        corner_int = ( int(corner[0]), int(corner[1]) )
                        rgb_image = cv2.circle(rgb_image, corner_int, radius=RADIUS, color=COLOR_CORNERS, thickness=-1)
                        try:
                            corner_d = depth_image[corner_int[1], corner_int[0]]
                        except IndexError:
                            continue
                        corner_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, corner, corner_d)
                        markers.markers.append(self.createMarker(markers_i, corner_3d, COLOR_GREEN))
                        markers_i += 1
                        corners_3d.append(np.array(corner_3d))
                    if len(corners_3d)==4:
                        if self.corners_avg_dist(corners_3d)>40:
                            continue
                        try:
                            m = self.get_transform_matrix(corners_3d)
                        except ZeroDivisionError:
                            continue

                        cube_corners_3d_points = np.array([
                                [40.0, -5.0, 0.0],
                                [40.0, 40.0, 0.0],
                                [-5.0, 40.0, 0.0],
                                [-5.0, -5.0, 0.0],
                                [40.0, -5.0, -45],
                                [40.0, 40.0, -45],
                                [-5.0, 40.0, -45],
                                [-5.0, -5.0, -45],
                            ])
                        cube_corners_pixel = []

                        for cube_point in cube_corners_3d_points:
                            cube_point_t = self.transform_point(cube_point, m)
                            cube_corner_pixel = rs2.rs2_project_point_to_pixel(self.intrinsics, cube_point_t)
                            cube_corner_pixel_int = ( int(cube_corner_pixel[0]), int(cube_corner_pixel[1]) )
                            cube_corners_pixel.append(cube_corner_pixel_int)

                            # rgb_image = cv2.circle(rgb_image, cube_corner_pixel_int, radius=RADIUS, color=COLOR_CORNERS, thickness=-1)
                            markers.markers.append(self.createMarker(markers_i, cube_point_t[:3], COLOR_BLUE, size=0.01))
                            markers_i += 1
                        
                        cv2.line(rgb_image, cube_corners_pixel[0], cube_corners_pixel[1], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[1], cube_corners_pixel[2], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[2], cube_corners_pixel[3], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[3], cube_corners_pixel[0], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[4], cube_corners_pixel[5], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[5], cube_corners_pixel[6], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[6], cube_corners_pixel[7], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[7], cube_corners_pixel[4], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[0], cube_corners_pixel[4], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[1], cube_corners_pixel[5], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[2], cube_corners_pixel[6], COLOR_LINES, thickness=2)
                        cv2.line(rgb_image, cube_corners_pixel[3], cube_corners_pixel[7], COLOR_LINES, thickness=2)
                    
            self.pub_markers.publish(markers)
            cv2.imshow('cv_image', rgb_image)
            cv2.waitKey(1)


    def corners_avg_dist(self, corners_3d):
        avg_dist = ( np.linalg.norm(corners_3d[0]-corners_3d[1]) + 
                     np.linalg.norm(corners_3d[1]-corners_3d[2]) + 
                     np.linalg.norm(corners_3d[2]-corners_3d[3]) + 
                    np.linalg.norm(corners_3d[3]-corners_3d[0]) ) /4.0
        return avg_dist



    def get_transform_matrix(self, corners_3d):
        ref_points_A, ref_points_B = self.compute_base_points(corners_3d)
        m = superimposition_matrix(np.swapaxes(ref_points_A, 0, 1), np.swapaxes(ref_points_B, 0, 1))
        return m


    def transform_point(self, point, m):
        point = np.array([point[0], point[1], point[2], 1.0])
        return np.dot(m, point)[:3]


    def compute_base_points(self, corners_3d):
        ref_points_A = np.empty((3,4))
        ref_points_B = np.empty((3,4))
        x_axis_vector = corners_3d[3]-corners_3d[0]
        x_norm = np.linalg.norm(x_axis_vector)
        x_axis_unit = x_axis_vector / x_norm
        y_axis_vector = corners_3d[3]-corners_3d[2]
        y_norm = np.linalg.norm(y_axis_vector)
        if np.linalg.norm(y_norm)==0.0:
            raise ZeroDivisionError()
        y_axis_unit = y_axis_vector / y_norm
        y_angle = np.arccos(np.clip(np.dot(x_axis_unit, y_axis_unit), -1.0, 1.0))

        # Marco del tag
        ref_points_A[0] = [0.0, 0.0, 0.0, 1.0]
        ref_points_A[1] = [x_norm, 0.0, 0.0, 1.0]
        ref_points_A[2] = [y_norm*np.cos(y_angle), y_norm*np.sin(y_angle), 0.0, 1.0]

        # Marco de la camara
        ref_points_B[0] = [corners_3d[3][0], corners_3d[3][1], corners_3d[3][2], 1.0]
        ref_points_B[1] = [corners_3d[0][0], corners_3d[0][1], corners_3d[0][2], 1.0]
        ref_points_B[2] = [corners_3d[2][0], corners_3d[2][1], corners_3d[2][2], 1.0]

        return np.array(ref_points_A), np.array(ref_points_B)


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
        
    
    def createMarker(self, id, point_3d, color=COLOR_WHITE, size=0.01, frame=MARKERS_FRAME):
        marker = Marker()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = frame
        marker.ns = ''
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_3d[0]/1000
        marker.pose.position.y = point_3d[1]/1000
        marker.pose.position.z = point_3d[2]/1000
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

def main():
    simple_publisher = ApriltagDepth()
    simple_publisher.run()


if __name__ == '__main__':
    main()
