import cv2
import rclpy
import rclpy.time
import tf2_ros
import numpy as np
import image_geometry

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_geometry_msgs import do_transform_point


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.create_subscription(Image, '/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.publisher_ = self.create_publisher(PointStamped, '/obstacle_position', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info = None

    def camera_info_callback(self, msg):
        """
        Callback function for the camera info topic
        """
        self.camera_info = msg
        self.camera_model.from_camera_info(msg)

    def image_callback(self, msg):
        """
        Callback function for the image topic
        """
        if self.camera_info is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Image received in the obstacle node')
            self.process_image(cv_image)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

    def process_image(self, cv_image):
        """
        Process the image to detect obstacles
        """
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        if np.all(mask == 0):
            self.get_logger().info('No red color detected from the obstacle node')
            return
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('No contours detected from the obstacle node')
            return

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                bottom_point = tuple(contour[contour[:, :, 1].argmax()][0])
                bottom_point = (cX, bottom_point[1])
                self.publish_obstacle_position(cv_image, bottom_point, cX, cY)

    def publish_obstacle_position(self, cv_image, bottom_point, x, y):
        """
        Publish the position of the obstacle
        """
        cv2.circle(cv_image, bottom_point, 5, (255, 0, 0), -1)
        cv2.putText(cv_image, "obstacle bottom Point", (bottom_point[0] - 25, bottom_point[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(cv_image, "centroid", (x - 25, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.circle(cv_image, (x, y), 5, (255, 255, 255), -1)
        cv2.imshow("Original Image", cv_image)
        cv2.waitKey(1)
        
        ray = self.camera_model.projectPixelTo3dRay(bottom_point)
        self.get_logger().info(f'Obstacle ray direction: {ray}')

        origin = PointStamped()
        origin.header.stamp = self.get_clock().now().to_msg()
        origin.header.frame_id = 'camera_link'
        origin.point.x = 0.0
        origin.point.y = 0.0
        origin.point.z = 0.0

        direction = Vector3Stamped()
        direction.header.stamp = self.get_clock().now().to_msg()
        direction.header.frame_id = 'camera_link'
        direction.vector.x = ray[0]
        direction.vector.y = ray[1]
        direction.vector.z = ray[2]

        target_frame = "odom"
        try:
                transform_origin = self.tf_buffer.transform(origin, target_frame, timeout=rclpy.duration.Duration(seconds=1.0))
                transform_direction = self.tf_buffer.transform(direction, target_frame, timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transformation failed: {e}")
            return

        if transform_direction.vector.z == 0:
            self.get_logger().warn("Ray is parallel to the XY plane.")
            return

        lamda = - transform_origin.point.z / transform_direction.vector.z
        x = transform_origin.point.x + transform_direction.vector.x * lamda
        y = transform_origin.point.y + transform_direction.vector.y * lamda
        z = transform_origin.point.z + transform_direction.vector.z * lamda

        intersection_point = PointStamped()
        intersection_point.header.frame_id = target_frame
        intersection_point.header.stamp = self.get_clock().now().to_msg()
        intersection_point.point.x = x
        intersection_point.point.y = y
        intersection_point.point.z = z

        self.get_logger().info(f"Intersection Point of the obstacle (odom): x={x}, y={y}, z={z}")
        self.publisher_.publish(intersection_point)
        #verify if intersection point is published
        self.get_logger().info(f'Intersection obstacle node published.')


def main(args=None):
    """
    Main function to run the obstacle node
    """
    rclpy.init(args=args)
    obstacle_node = ObstacleNode()
    rclpy.spin(obstacle_node)
    obstacle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()