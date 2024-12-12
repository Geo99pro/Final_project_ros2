import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped


class MoveRobot(Node):
    """
    This class is responsible for moving the robot to a goal position
    """
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        self.goal_pose = None
        self.current_pose = None

        self.declare_parameter('linear_gain', 0.1)
        self.declare_parameter('angular_gain', 0.5)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg):
        """
        Callback function for the goal topic
        """

        self.goal_pose = msg

    def pose_callback(self, msg):
        """
        Callback function for the odometry topic
        """

        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_robot()

    def stop_robot(self, euclidean_distance):
        """
        Stop the robot when it reaches the goal
        """
        if euclidean_distance < 0.1:
            self.get_logger().info('Goal reached')
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = 0.0
            velocity_msg.twist.angular.z = 0.0
            self.publisher_.publish(velocity_msg)
            self.goal_pose = None   
            return

    def move_robot(self):
        """
        Move the robot to the goal position
        """

        if self.goal_pose is None or self.current_pose is None:
            return 

        goal_point = PointStamped()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = self.goal_pose.x
        goal_point.point.y = self.goal_pose.y
        goal_point.point.z = 0.0

        
        try:
            #https://stackoverflow.com/questions/74976911/create-an-odometry-publisher-node-in-python-ros2
            #https://github.com/pal-robotics/object_recognition_clusters/blob/master/src/object_recognition_clusters/convert_functions.py
            transformed_goal = self.tf_buffer.transform(goal_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1))
            euclidean_distance = sqrt(pow(transformed_goal.point.x, 2) + pow(transformed_goal.point.y, 2))
            self.stop_robot(euclidean_distance)

            angle_to_goal = atan2(transformed_goal.point.y, transformed_goal.point.x)
            linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
            angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.1)
            velocity_msg.twist.angular.z = angular_gain * angle_to_goal

            self.publisher_.publish(velocity_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transformation failed: {e}")

def main(args=None):
    """
    Main function to run the move robot node
    """
    rclpy.init(args=args)
    move_robot = MoveRobot()
    rclpy.spin(move_robot)
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

