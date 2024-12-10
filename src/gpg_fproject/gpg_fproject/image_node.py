import cv2
import imutils
import rclpy
import numpy as np
import image_geometry

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, String
from gpg_fproject.gpg_fproject.utils import get_shape
from geometry_msgs.msg import PointStamped, Vector3Stamped
from gpg_fproject.gpg_fproject.utils import get_hsv_value_based_on_click

class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.create_subscription(Image, '/image', self.image_callback, 10)
        self.create_publisher(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.servo_publisher = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        self.point_publisher = self.create_publisher(PointStamped, '/object_position', 10)
        self.color_subscription = self.create_subscription(String, '/object_color', self.color_callback, 10)
        self.form_subscription = self.create_subscription(String, '/object_form', self.form_callback, 10)

        self.tf_buffer = Buffer()
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_servo_position = 0.0
        self.camera_info = None
        self.user_color = None
        self.user_form = None

    def camera_info_callback(self, msg):
        """
        Callback function for the camera info topic"""
        self.camera_info = msg
        self.camera_model.from_camera_info(msg)

    def color_callback(self, msg):
        """
        Callback function for the color topic"""
        self.user_color = msg.data
        self.get_logger().info(f'I heard the color asked by the user: {self.user_color}')

    def form_callback(self, msg):
        """
        Callback function for the form topic"""
        self.user_form = msg.data
        self.get_logger().info(f'I heard the form asked by the user: {self.user_form}')

    def image_callback(self, msg):
        """
        Callback function for the image topic"""
        if self.camera_info is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Image received and processing and will be processed')
            self.process_image(cv_image, msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'Error while converting image: {e}')

    def get_hsv_limit(self, user_color):
        """
        Function to get the HSV limits for the color asked by the user"""
        if user_color=='blue':
            lower_hsv = np.array([120, 209, 220])
            upper_hsv = np.array([120, 209, 220])
        elif user_color=='red':
            lower_hsv = np.array([0, 145, 200])
            upper_hsv = np.array([0, 145, 200])
        elif user_color=='green':
            lower_hsv = np.array([60, 255, 120])
            upper_hsv = np.array([60, 255, 120])
        elif user_color=='yellow':
            lower_hsv = np.array([22, 255, 178])
            upper_hsv = np.array([22, 255, 178])
        return lower_hsv, upper_hsv

    def process_image(self, cv_image):
        """
        Function to process the image and detect the object
        
        Args:

            cv_image: The image to be processed as a NumPy array
        """
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        user_color = self.user_color
        user_form = self.user_form

        if user_color is None or user_form is None:
            self.get_logger().info('User color or form not provided yet or not already received')
            return

        lower_hsv, upper_hsv = self.get_hsv_limit(user_color)
        mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            approx_object_form, c, text, x, y = get_shape(cv_image, contours, 0.04)
            self.get_logger().info(f'Approximated object form: {approx_object_form}')
            cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 3)
            cv2.putText(cv_image, text, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("HSV Image", hsv_img)
            cv2.imshow("Mask", mask)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            if user_form == 'triangle' and approx_object_form == 'triangle':
                self.get_logger().info('Triangle detected')
            elif user_form == 'square' and approx_object_form == 'square':
                self.get_logger().info('Square detected')
            elif user_form == 'circle' and approx_object_form == 'circle':
                self.get_logger().info('Circle detected')
            else:
                pass

            M = cv2.moments(c)
            if M["m00"] !=0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(cv_image, "centroid", (cX - 25, cY - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            img_center_x = cv_image.shape[1] / 2
            error = img_center_x - cX
            kp = 0.005
            self.current_servo_position = kp * error
            self.current_servo_position = np.clip(self.current_servo_position, -1.5, 1.5)
            
            servo_msg = Float64MultiArray()
            servo_msg.data = [self.current_servo_position]
            self.servo_publisher.publish(servo_msg)

            bottom_point = tuple(c[c[:, :, 1].argmax()][0])
            bottom_point = [cX, bottom_point[1]]
            self.get_logger().info(f'Bottom point of the object: {bottom_point}')

            cv2.circle(cv_image, bottom_point, 5, (255, 0, 0), -1)
            cv2.putText(cv_image, "Bottom Point", (bottom_point[0] - 25, bottom_point[1] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.imshow("Original Image", cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            ray = self.camera_model.projectPixelTo3dRay(bottom_point)
            self.get_logger().info(f'Ray: {ray}')

            origin = PointStamped()
            origin.header.frame_id = "camera_link"
            origin.header.stamp = rclpy.time.Time().to_msg()
            
            direction = Vector3Stamped()
            direction.header.frame_id = "camera_link"
            direction.header.stamp = rclpy.time.Time().to_msg()
            direction.vector.x = ray[0]
            direction.vector.y = ray[1]
            direction.vector.z = ray[2]

            target_frame = "odom"

            try:
                transform_origin = self.tf_buffer.transform(origin,
                                                            target_frame,
                                                            timeout=rclpy.time.Duration(seconds=1.0))
                transform_direction = self.tf_buffer.transform(direction,
                                                            target_frame,
                                                            timeout=rclpy.time.Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f'Error while transforming the point: {e}')
                return
            
            lambda_ = - transform_origin.point.z / transform_direction.vector.z
            x = transform_origin.point.x + transform_direction.vector.x * lambda_
            y = transform_origin.point.y + transform_direction.vector.y * lambda_
            z = transform_origin.point.z + transform_direction.vector.z * lambda_

            intersection_point = PointStamped()
            intersection_point.header.frame_id = target_frame
            intersection_point.header.stamp = rclpy.time.Time().to_msg()
            intersection_point.point.x = x
            intersection_point.point.y = y
            intersection_point.point.z = z
            self.point_publisher.publish(intersection_point)

def main(args=None):
    """
    Main function to run the image node"""
    rclpy.init(args=args)
    image_node = ImageNode()
    rclpy.spin(image_node)
    image_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()