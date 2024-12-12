import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


class MovePhysicalRobot(Node):
    def __init__(self):
        super().__init__('move_physical_robot')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        self.goal_pose = None
        self.current_pose = None

        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 0.5)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_physical_robot()

    def move_physical_robot(self):
        if self.goal_pose is None or self.current_pose is None:
            return 

        goal_point = PointStamped()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = self.goal_pose.x
        goal_point.point.y = self.goal_pose.y
        goal_point.point.z = 0.0

        try:
            # Lookup transform between 'odom' and 'base_link'
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                goal_point.header.frame_id,  # Source frame
                rclpy.time.Time(),  # Latest transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Transform the goal point to the 'base_link' frame
            transformed_goal = do_transform_point(goal_point, transform)

            # Calculate the distance and angle to the goal
            euclidean_distance = sqrt(pow(transformed_goal.point.x, 2) + 
                                    pow(transformed_goal.point.y, 2))

            if euclidean_distance < 0.1:  # If within goal threshold, stop the robot
                velocity_msg = TwistStamped()
                velocity_msg.header.stamp = self.get_clock().now().to_msg()
                velocity_msg.twist.linear.x = 0.0
                velocity_msg.twist.angular.z = 0.0
                self.publisher_.publish(velocity_msg)
                self.get_logger().info('Goal reached. Stopping the robot.')
                return

            angle_to_goal = atan2(transformed_goal.point.y, transformed_goal.point.x)

            # Get parameters for linear and angular gain
            linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
            angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

            # Calculate velocity commands
            linear_velocity = min(linear_gain * euclidean_distance, 0.2)  # Limit max linear velocity
            angular_velocity = max(min(angular_gain * angle_to_goal, 1.0), -1.0)  # Clamp angular velocity

            # Publish velocity commands
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = linear_velocity
            velocity_msg.twist.angular.z = angular_velocity
            self.publisher_.publish(velocity_msg)

            self.get_logger().info(f'Moving towards goal: distance={euclidean_distance:.2f}, angle={angle_to_goal:.2f}')

        except TransformException as e:
            self.get_logger().warn(f"Transformation failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

