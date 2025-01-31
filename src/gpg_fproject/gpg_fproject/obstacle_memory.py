import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_geometry_msgs import do_transform_point


class ObstacleMemoryNode(Node):
    def __init__(self):
        super().__init__('obstacle_memory_node')
        self.create_subscription(PointStamped, '/obstacle_position', self.obstacle_callback, 10)
        self.publisher_ = self.create_publisher(PointStamped, '/obstacle_memory', 10)
        self.obstacle_memory = None
        self.obstacle_memory_list = []

    def obstacle_callback(self, msg):
        """
        Callback function for the obstacle topic
        """
        self.obstacle_memory = msg
        self.publish_obstacle_memory()

    # Save into a list memory all the coordinates of the obstacles sended by the obstacle_node over the topic /obstacle_position
    # /obstacle_position is a PointStamped message that contains the x, y, z coordinates of the obstacle
    def save_obstacle_memory(self):
        """
        Save the obstacle memory
        """
        if self.obstacle_memory is not None:
            obstacle_dict = {
                'X': self.obstacle_memory.point.x, 
                'Y': self.obstacle_memory.point.y, 
                'Z': self.obstacle_memory.point.z
                }
            self.obstacle_memory_list.append(obstacle_dict)
        return self.obstacle_memory_list
    
    def publish_obstacle_memory(self):
        """
        Publish the obstacle memory
        """
        self.obstacle_memory_list = self.save_obstacle_memory()
        for obstacle in self.obstacle_memory_list:
            obstacle_msg = PointStamped()
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            obstacle_msg.point.x = obstacle['X']
            obstacle_msg.point.y = obstacle['Y']
            obstacle_msg.point.z = obstacle['Z']
            self.publisher_.publish(obstacle_msg)

def main(args=None):
    rclpy.init(args=args)

    obstacle_memory_node = ObstacleMemoryNode()

    rclpy.spin(obstacle_memory_node)

    obstacle_memory_node.destroy_node()
    rclpy.shutdown()


    

