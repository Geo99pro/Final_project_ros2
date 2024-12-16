import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


class MovePhysicalRobot(Node):
    
    """
    Class to move the physical robot to a goal position
    """
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
        self.declare_parameter('angular_gain', 0.5)
        self.declare_parameter('obstacle_threshold', 0.5)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg):
        """
        Callback function for the goal topic
        """
        self.goal_pose = msg

    def obstacle_callback(self, msg):
        """
        Callback function for the obstacle topic
        """
        self.obstacle_pose = msg

    def pose_callback(self, msg):
        """
        Callback function for the odometry topic
        """
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_physical_robot()
            
    def stop_robot(self, euclidean_distance):
        """
        Stop the robot if the goal is reached
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

    def move_physical_robot(self):
        if self.goal_pose is None or self.current_pose is None:
            return 
        """
        Move the physical robot to the goal position
        """
        goal_point = PointStamped()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = self.goal_pose.x
        goal_point.point.y = self.goal_pose.y
        goal_point.point.z = 0.0

        #obstacle_point = PointStamped()
        #obstacle_point.header.frame_id = 'odom'
        #obstacle_point.point.x = self.obstacle_pose.point.x
        #obstacle_point.point.y = self.obstacle_pose.point.y
        #obstacle_point.point.z = 0.0
        
        try:
            transformed_goal = self.tf_buffer.transform(goal_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1))
            euclidean_distance = sqrt(pow(transformed_goal.point.x, 2) + pow(transformed_goal.point.y, 2))
            self.stop_robot(euclidean_distance)
            
            angle_to_goal = atan2(transformed_goal.point.y, transformed_goal.point.x)

            base_angvel = 0
            if self.obstacle_pose and (self.get_clock().now().to_msg() - self.obstacle_pose.header.stamp) < 0.5:
                transformed_obstacle = self.tf_buffer.transform(self.obstacle_pose, 'base_link', timeout=rclpy.duration.Duration(seconds=1))
                obstacle_distance = sqrt(pow(transformed_obstacle.point.x, 2) + pow(transformed_obstacle.point.y, 2))
                obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

                if obstacle_distance < obstacle_threshold:
                    self.get_logger().info('Obstacle detected')
                    avoid_direction = -1 if self.obstacle_pose.point.y > 0 else 1
                    angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
                    base_angvel = angular_gain * avoid_direction
                
            linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
            angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.1)
            velocity_msg.twist.angular.z = angular_gain * angle_to_goal + base_angvel

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

