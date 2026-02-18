#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
# from image_transport_py import ImageTransport
import cv2
from cv_bridge import CvBridge
import threading
from collections import deque
import struct

class PointCloudProjector(Node):
    def __init__(self):
        super().__init__('pointcloud_projector')

        # Set expected topics
        COLOR_TOPIC = '/camera/camera/color/image_raw'
        COLOR_INFO  = '/camera/camera/color/camera_info'
        DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
        DEPTH_INFO  = '/camera/camera/aligned_depth_to_color/camera_info'
        LIDAR_TOPIC = '/livox/points'
        PROJ_TOPIC  = '/pc2/proj'
        DIAG_TOPIC  = '/pc2/diag'

        # Set compressed image timer period
        # TIMER_PERIOD = 0.5

        # (use adaptive for now) # Set expected depth range
        # DEPTH_MIN = 
        # DEPTH_MAX = 
        
        # Initialize CV Bridge and buffers
        self.bridge = CvBridge()
        self.color_K = None  # intrinsic camera matrix
        self.color_d = None  # distortion parameters (k1, k2, t1, t2, k3)
        self.color_height = None
        self.color_width  = None
        self.diag_image = None
        self.depth_K = None
        self.depth_d = None
        self.depth_height = None
        self.depth_width  = None
        self.depth_max    = 0
        self.tf2_buffer   = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        # Create a QoS profile that matches the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try this first
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Declare subscribers using message filters for synchronization
        self.color_image_subscriber = Subscriber(self, Image, COLOR_TOPIC)
        self.color_info_subscriber  = Subscriber(self, CameraInfo, COLOR_INFO)
        self.depth_image_subscriber = Subscriber(self, Image, DEPTH_TOPIC)
        self.depth_info_subscriber  = Subscriber(self, CameraInfo, DEPTH_INFO)
        self.lidar_subscriber = Subscriber(self, PointCloud2, LIDAR_TOPIC,
                                           qos_profile=qos_profile)
        
        # Synchronize messages with approximate time policy
        self.sync = ApproximateTimeSynchronizer(
            [self.color_image_subscriber, self.color_info_subscriber, 
             self.depth_image_subscriber, self.depth_info_subscriber, 
             self.lidar_subscriber],
            queue_size=50,
            slop=0.2  # 100ms tolerance
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Set up publisher for projection
        # self.image_transport = ImageTransport(
        #     'imagetransport_pub', image_transport='compressed'
        # )
        # self.timer = self.create_timer(TIMER_PERIOD, self.publish_callback)
        # self.advertiser = self.image_transport.advertise(PROJ_TOPIC, 10)
        # self.diag_advertiser = self.image_transport.advertise(DIAG_TOPIC, 10)

        self.publisher = self.create_publisher(Image, PROJ_TOPIC, 10)
        self.diag_publisher = self.create_publisher(Image, DIAG_TOPIC, 10)
        
        self.get_logger().info('Initialized PointCloudProjector node')

    def sync_callback(self, color_image_msg, color_info_msg, depth_image_msg, 
                      depth_info_msg, pointcloud_msg):
        """Callback for receiving (synchronized) messages"""

        # Set expected frames
        COLOR_FRAME_ID = 'camera_color_optical_frame'
        DEPTH_FRAME_ID = 'camera_depth_optical_frame'
        LIDAR_FRAME_ID = 'livox_frame'

        # Set expected encodings
        COLOR_ENCODING = 'rgb8'
        DEPTH_ENCODING = '16UC1'

        # Update intrinsic camera parameters
        self.update_color_params(color_info_msg)
        self.update_depth_params(depth_info_msg)
        
        # Convert ROS image to OpenCV
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_image_msg,
                                                    COLOR_ENCODING)
        except Exception as err:
            self.get_logger().error(f'Failed to convert image: {err}')
            return
        
        # TBD: Depth
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg,
                                                    DEPTH_ENCODING)
        except Exception as err:
            self.get_logger().error(f'Failed to convert image: {err}')
            return
        
        # Get transform from LiDAR to camera frame
        try:
            transform = self.tf2_buffer.lookup_transform(
                COLOR_FRAME_ID,  # target frame (camera)
                LIDAR_FRAME_ID,  # source frame (lidar)
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as err:
            self.get_logger().warn(f'Failed to get transform: {err}')
            return
        
        # Extract points from PointCloud2
        points = np.array(list(
            self.extract_points_from_pointcloud2(pointcloud_msg)
        ))
        
        # Transform points to camera frame
        points = self.transform_points(points, transform)

        # Note max depth
        depths = points[:, 2]  # treat 3rd axis after transform as depth
        maxval = np.max(depths)
        if maxval > self.depth_max:
            self.depth_max = maxval
        
        # Project points onto image
        projected_image, just_points_image = self.project_points_onto_image(
            points, color_image
        )

        # Add colorbar for depth (red=close, blue=far)
        if not isinstance(projected_image, np.ndarray):
            print('image is not numpy before func')
        # projected_image = self.add_colorbar(projected_image)
        if not isinstance(projected_image, np.ndarray):
            print('image is not numpy after func')
        
        # Publish projected image
        try:
            projected_msg = self.bridge.cv2_to_imgmsg(projected_image,
                                                      COLOR_ENCODING)
            projected_msg.header = color_image_msg.header
            self.publisher.publish(projected_msg)
        except Exception as err:
            self.get_logger().error(f'Failed to publish image: {err}')

        # Publish image of 2D points by themselves
        try:
            points_msg = self.bridge.cv2_to_imgmsg(just_points_image,
                                                   COLOR_ENCODING)
            points_msg.header = color_image_msg.header
            self.diag_publisher.publish(points_msg)
        except Exception as err:
            self.get_logger().error(f'Failed to publish image: {err}')

    def update_color_params(self, camera_info):
        """Extract camera parameters from CameraInfo message"""
        self.color_K = np.array(camera_info.k).reshape(3, 3)
        self.color_d = np.array(camera_info.d)
        self.color_height = camera_info.height
        self.color_width  = camera_info.width
    
    def update_depth_params(self, camera_info):
        self.depth_K = np.array(camera_info.k).reshape(3, 3)
        self.depth_d = np.array(camera_info.d)
        self.depth_height = camera_info.height
        self.depth_width  = camera_info.width

    def extract_points_from_pointcloud2(self, pc2: PointCloud2):
        """Extract XYZ points from PointCloud2 message"""
        points = []

        # Set datatype dictionary
        DATATYPES = {
            PointField.INT8    : ('b', 1),
            PointField.UINT8   : ('B', 1),
            PointField.INT16   : ('h', 2),
            PointField.UINT16  : ('H', 2),
            PointField.INT32   : ('i', 4),
            PointField.UINT32  : ('I', 4),
            PointField.FLOAT32 : ('f', 4),  # should be this
            PointField.FLOAT64 : ('d', 8)
        }

        # Format
        fmt = '>' if pc2.is_bigendian else '<'
        off = 0
        xyz_fields = pc2.fields[0:3]  # x, y, z
        for field in (f for f in sorted(xyz_fields, key=lambda f: f.offset)):
            if off < field.offset:
                fmt += 'x'*(field.offset - off)
                off = field.offset

            fmt += field.count*DATATYPES[field.datatype][0]
            off += field.count*DATATYPES[field.datatype][1]
        
        # Parse PointCloud2 message
        for h in range(pc2.height):
            offset = h*pc2.row_step
            
            for w in range(pc2.width):
                points = struct.Struct(fmt).unpack_from(pc2.data, offset)

                if not np.any(np.isnan(points)):
                    yield points  # generator: keep state until next itr

                offset += pc2.point_step

        # OLD: Parse PointCloud2 message
        # fmt = 'fff'  # Assuming float32 (datatype = 7) for x, y, z
        # point_step = pointcloud_msg.point_step
        
        # for i in range(0, len(pointcloud_msg.data), point_step):
        #     # Extract x, y, z (assuming they are the first 3 float32 values)
        #     x, y, z = struct.unpack_from(fmt, pointcloud_msg.data, i)
            
        #     # Filter out invalid points
        #     if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
        #         points.append([x, y, z])
        
        # return np.array(points)

    def transform_points(self, points, transform):
        """LiDAR to camera"""

        # Parse rotation from quaternion
        q = transform.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        
        # Compose homogeneous transformation
        d = transform.transform.translation
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3]  = [d.x, d.y, d.z]
        
        # Apply transformation to points
        points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
        points_transformed = (T @ points_homogeneous.T).T
        
        return points_transformed[:, :3]

    def project_points_onto_image(self, points, image):
        """Project 3D points onto 2D image plane"""
        K = self.color_K
        d = self.color_d
        self.init_diag_image(image.shape)

        if K is None:
            self.get_logger().warn('Still awaiting intrinsics')
            return image, self.diag_image
        
        # Get visible points
        MIN_DEPTH  = 0.1  # to avoid near-plane issues
        is_visible = (points[:,2] > MIN_DEPTH)  # 3rd axis after transform

        if not np.any(is_visible):
            self.get_logger().warn('No visible points')
            return image, self.diag_image
        
        # Perform planar projection
        uv, _ = cv2.projectPoints(  # image coordinates
            points[is_visible, :],
            np.zeros(3),  # addtl rotation
            np.zeros(3),  # addtl translation
            K,
            d
        )
        uv = uv.reshape(-1, 2) # .astype(int)

        # Filter out invalid non-finite values
        is_valid = np.isfinite(uv).all(axis=1)
        if not np.any(is_valid):
            self.get_logger().warn('No valid projections')
            return image, self.diag_image

        # Get depths (just use 3rd axis for now)
        depths = points[is_visible, 2]
        mindepth = 0  # np.min(depths)
        maxdepth = self.depth_max  # np.max(depths)

        # Apply valid mask to both uv coordinates and depths
        uv = uv[is_valid].astype(int)
        depths = depths[is_valid]

        invalid_uv = ~np.isfinite(uv).all(axis=1)
        if np.any(invalid_uv):
            self.get_logger().warn(f'Found {np.sum(invalid_uv)} invalids')

        # Create overlay visualization
        projected_image = image.copy()
        points_image = np.zeros(image.shape, dtype=np.uint8)
        for ii, (u, v) in enumerate(uv):
            in_image = (0 <= u < image.shape[1]) and (0 <= v < image.shape[0])
            if not in_image:
                continue

            # Colorize normalized depth
            range = (maxdepth - mindepth + 1e-6)  # eps to avoid div 0
            if maxdepth > mindepth:
                depth = (depths[ii] - mindepth) / range
            else:
                depth = 0.5  # default to middle value if no depth range
            color = self.depth2color(depth)
                
            # Draw points
            cv2.circle(projected_image, (u, v), 2, color, -1)
            cv2.circle(points_image, (u, v), 2, color, -1)
        
        return projected_image, points_image
    
    def init_diag_image(self, shape):
        if self.diag_image is not None:
            return
        
        TEXT      = 'Data unavailable'
        COLOR     = (0, 0, 255)  # red
        TYPE      = cv2.FONT_HERSHEY_SIMPLEX
        SCALE     = 1.5
        THICKNESS = 2

        self.diag_image = np.zeros(shape, dtype=np.uint8)
        text_size = cv2.getTextSize(TEXT, TYPE, SCALE, THICKNESS)[0]
        u = (shape[1] - text_size[0]) // 2
        v = (shape[0] + text_size[1]) // 2

        cv2.putText(self.diag_image, TEXT, (u, v), TYPE, SCALE, COLOR, 
                    THICKNESS, cv2.LINE_AA)
        
        return

    def depth2color(self, depth):  # (red=close, blue=far)
        # HSV colormap: Red (0) -> Yellow -> Green -> Cyan -> Blue (240)
        hue = int((1.0 - depth) * 240)
        hsv = np.array([[[hue, 255, 255]]], dtype=np.uint8)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]

        return tuple(int(x) for x in bgr)

    def add_colorbar(self, image):
        if not isinstance(image, np.ndarray):
            print('image is not numpy at start')

        bar_height = 200
        bar_width  = 20
        bar_x = image.shape[1] - 40
        bar_y = 20

        if not isinstance(image, np.ndarray):
            print('image is not numpy close to start')

        for ii in range(bar_height):
            bar_value = ii / bar_height
            bar_color = self.depth2color(bar_value)
            cv2.line(image, (bar_x, bar_y + ii),  # does this work?
                     (bar_x + bar_width, bar_y + ii), bar_color, 1)
        
        if not isinstance(image, np.ndarray):
            print('image is not numpy after line')

        cv2.putText(image, 
                    f'{self.depth_max:.1f}m', 
                    (bar_x - 50, bar_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (255, 255, 255), 
                    1)
        if not isinstance(image, np.ndarray):
            print('image is not numpy after putText 1')

        cv2.putText(image, 
                    f'{self.depth_max:.1f}m', 
                    (bar_x - 50, bar_y + bar_height), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (255, 255, 255), 
                    1)
        
        if not isinstance(image, np.ndarray):
            print('image is not numpy after putText 2')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProjector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()