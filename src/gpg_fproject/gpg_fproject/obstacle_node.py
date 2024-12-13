import cv2
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from gpg_fproject.utils import get_shape
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PointStamped, Vector3Stamped


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.create_subscription(Image, '/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.po