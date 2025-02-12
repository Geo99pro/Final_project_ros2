import cv2
import rclpy
import numpy as np
import image_geometry

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros  
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, String
from gpg_fproject.utils import get_shape
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_geometry_msgs import do_transform_point

class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.create_subscription(Image, '/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.servo_publisher = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        self.point_publisher = self.create_publisher(PointStamped, '/object_position', 10)
        self.color_subscription = self.create_subscription(String, '/object_color', self.color_callback, 10)
        self.form_subscription = self.create_subscription(String, '/object_form', self.form_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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
            self.process_image(cv_image)
            
        except CvBridgeError as e:
            self.get_logger().error(f'Error while converting image: {e}')

    def get_hsv_limit(self, user_color):
        """
        Function to get the HSV limits for the color asked by the user"""
        if user_color=='blue':
            lower_hsv = np.array([100, 150, 50])
            upper_hsv = np.array([140, 255, 255])
        elif user_color=='red':
            lower_hsv = np.array([0, 145, 200])
            upper_hsv = np.array([0, 145, 200])
        elif user_color=='green':
            lower_hsv = np.array([35, 100, 100])
            upper_hsv = np.array([85, 255, 255])
        elif user_color=='yellow':
            lower_hsv = np.array([20, 100, 100])
            upper_hsv = np.array([30, 255, 255])
        elif user_color=='black':
            lower_hsv = np.array([0, 0, 0])
            upper_hsv = np.array([180, 255, 30])

        return lower_hsv, upper_hsv

    def check_condition(self, user_form, approx_object_form):
        if user_form == 'triangle' and approx_object_form in ['triangle', 'triangle approximation distorted']:
            self.get_logger().info('Triangle detected')
        elif user_form == 'box' and approx_object_form in ['box', 'box approximation distorted']:
            self.get_logger().info('box detected')
        #elif user_form == 'circle' and approx_object_form in ['circle', 'circle approximation distorted']:
        #    self.get_logger().info('Circle detected')
        #else:
        else:
            self.get_logger().info('Object not detected')
            return

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
        if np.all(mask == 0):
            self.get_logger().info('No object detected')
            return
        
        contours, approx_object_form, max_contours, text, coords = get_shape(cv_image, mask, 0.04, user_form)
        self.get_logger().info(f'Approximated object form: {approx_object_form}')
        
        self.check_condition(user_form, approx_object_form)
        if contours:
            M = cv2.moments(max_contours)
            if M["m00"] !=0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

            img_center_x = cv_image.shape[1] / 2
            error = img_center_x - cX
            kp = 0.005
            self.current_servo_position = kp * error
            self.current_servo_position = np.clip(self.current_servo_position, -1.5, 1.5)
            
            servo_msg = Float64MultiArray()
            servo_msg.data = [self.current_servo_position]
            self.servo_publisher.publish(servo_msg)

            bottom_point = tuple(max_contours[max_contours[:, :, 1].argmax()][0])
            bottom_point = [cX, bottom_point[1]]
            #self.get_logger().info(f'Bottom point of the object: {bottom_point}')

            cv2.circle(cv_image, bottom_point, 5, (255, 0, 0), -1)
            cv2.putText(cv_image, "Bottom Point", (bottom_point[0] - 25, bottom_point[1] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            cv2.drawContours(cv_image, [max_contours], -1, (0, 255, 0), 3)
            cv2.putText(cv_image, text, (coords[0], coords[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(cv_image, "centroid", (cX - 25, cY - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
            
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("HSV Image", hsv_img)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)


            ray = self.camera_model.projectPixelTo3dRay(bottom_point)
            self.get_logger().info(f"Ray direction: {ray}")

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
                transform_origin = self.tf_buffer.transform(
                    origin, target_frame, timeout=rclpy.duration.Duration(seconds=2.0))
                transform_direction = self.tf_buffer.transform(
                    direction, target_frame, timeout=rclpy.duration.Duration(seconds=2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Transformation failed: {e}")
                return

            if transform_direction.vector.z == 0:
                self.get_logger().warn("Ray is parallel to the XY plane.")
                return

            lamda = - transform_origin.point.z / transform_direction.vector.z
            x = transform_origin.point.x + transform_direction.vector.x * lamda
            y = transform_origin.point.y + transform_direction.vector.y * lamda
            z = 0.0 

            intersection_point = PointStamped()
            intersection_point.header.frame_id = target_frame
            intersection_point.header.stamp = rclpy.time.Time().to_msg()
            intersection_point.point.x = x
            intersection_point.point.y = y
            intersection_point.point.z = z

            self.get_logger().info(f"Intersection Point (odom): x={x}, y={y}, z={z}")
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