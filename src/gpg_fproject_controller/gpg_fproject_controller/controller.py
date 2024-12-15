import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
import time

class MovePhysicalRobot(Node):
    MOVING_TOWARDS_GOAL = 0
    AVOIDING_OBSTACLE = 1

    def __init__(self):
        super().__init__('move_physical_robot')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        self.create_subscription(PointStamped, '/obstacle_position', self.obstacle_callback, 10)

        self.goal_pose = None
        self.current_pose = None
        self.obstacle_pose = None

        self.declare_parameter('linear_gain', 0.1)
        self.declare_parameter('angular_gain', 0.7)
        self.declare_parameter('obstacle_threshold', 0.6)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State machine variables
        self.state = self.MOVING_TOWARDS_GOAL
        self.avoid_direction = 1
        self.avoid_start_time = None
        self.avoid_max_duration = 5.0  # seconds to attempt avoidance before giving up or trying a different strategy

    def goal_callback(self, msg):
        self.goal_pose = msg

    def obstacle_callback(self, msg):
        self.obstacle_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_physical_robot()

    def stop_robot(self, euclidean_distance):
        if euclidean_distance < 0.1:
            self.get_logger().info('Goal reached')
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = 0.0
            velocity_msg.twist.angular.z = 0.0
            self.publisher_.publish(velocity_msg)
            self.goal_pose = None
            return True
        return False

    def move_physical_robot(self):
        if self.goal_pose is None or self.current_pose is None:
            return

        # If we have a goal, transform it into the base_link frame
        goal_point = PointStamped()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = self.goal_pose.x
        goal_point.point.y = self.goal_pose.y
        goal_point.point.z = 0.0

        try:
            transformed_goal = self.tf_buffer.transform(goal_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1))
            euclidean_distance = sqrt(pow(transformed_goal.point.x, 2) + pow(transformed_goal.point.y, 2))

            # Check if goal reached
            if self.stop_robot(euclidean_distance):
                return

            angle_to_goal = atan2(transformed_goal.point.y, transformed_goal.point.x)

            # Attempt to transform obstacle as well, if present
            obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
            obstacle_detected = False
            obstacle_distance = float('inf')

            if self.obstacle_pose is not None:
                try:
                    transformed_obstacle = self.tf_buffer.transform(self.obstacle_pose, 'base_link', timeout=rclpy.duration.Duration(seconds=1))
                    obstacle_distance = sqrt(pow(transformed_obstacle.point.x, 2) + pow(transformed_obstacle.point.y, 2))
                    obstacle_detected = obstacle_distance < obstacle_threshold
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f"Obstacle transform failed: {e}")

            linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
            angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

            # Finite State Machine for Obstacle Avoidance
            if self.state == self.MOVING_TOWARDS_GOAL:
                if obstacle_detected:
                    self.get_logger().info(f'Obstacle detected at {obstacle_distance:.2f}m. Starting avoidance maneuver.')
                    self.state = self.AVOIDING_OBSTACLE
                    # Decide which direction to avoid
                    # If obstacle is to the left, we turn right, else turn left
                    self.avoid_direction = -1 if transformed_obstacle.point.y > 0 else 1
                    self.avoid_start_time = time.time()
                else:
                    # No obstacle, move straight towards the goal
                    velocity_msg = TwistStamped()
                    velocity_msg.header.stamp = self.get_clock().now().to_msg()
                    velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.15)
                    velocity_msg.twist.angular.z = angular_gain * angle_to_goal
                    self.publisher_.publish(velocity_msg)

            elif self.state == self.AVOIDING_OBSTACLE:
                # In this state, we try to steer around the obstacle.
                # A simple strategy: turn in place and move slightly forward to steer around the obstacle.
                # If we spend too long here, we revert to moving towards the goal and hope for a new approach.
                elapsed = time.time() - self.avoid_start_time

                if not obstacle_detected or elapsed > self.avoid_max_duration:
                    # If obstacle is gone or we've tried long enough, return to goal-seeking state
                    self.get_logger().info('Obstacle cleared or max avoidance time reached, resuming goal pursuit.')
                    self.state = self.MOVING_TOWARDS_GOAL
                else:
                    # Keep avoiding: turn and move forward slightly
                    velocity_msg = TwistStamped()
                    velocity_msg.header.stamp = self.get_clock().now().to_msg()
                    # Move slowly forward and turn away from obstacle
                    velocity_msg.twist.linear.x = 0.1
                    velocity_msg.twist.angular.z = angular_gain * self.avoid_direction
                    self.publisher_.publish(velocity_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transformation failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
