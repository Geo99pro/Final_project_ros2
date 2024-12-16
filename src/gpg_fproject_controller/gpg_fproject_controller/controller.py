import rclpy
import tf2_ros
import random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, cos, sin, pi
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped

class MovePhysicalRobot(Node):
    def __init__(self):
        super().__init__('move_physical_robot')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        self.create_subscription(PointStamped, '/obstacle_position', self.obstacle_callback, 10)

        self.goal_pose = None    
        self.current_pose = None
        self.obstacle_pose = None
        self.avoid_goal = None
        self.obstacle_cleared = False

        # Settings for the controller  
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 0.5)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('random_distance_min', 1.0)
        self.declare_parameter('random_distance_max', 2.0)

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.avoid_goal = None  # reinstialize the avoid goal
        self.obstacle_cleared = False

    def obstacle_callback(self, msg):
        self.obstacle_pose = msg
        if msg and not self.avoid_goal:  # If there is no avoid goal, define one
            self.define_avoid_goal()

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_robot()

    def define_avoid_goal(self):
        """
        Create a random sub-goal around the obstacle by adding a random distance.
        """
        random_distance = random.uniform(
            self.get_parameter('random_distance_min').get_parameter_value().double_value,
            self.get_parameter('random_distance_max').get_parameter_value().double_value
        )
        random_angle = random.uniform(0, 2 * pi)  # Random angle in radians

        self.avoid_goal = Pose2D(
            x=self.obstacle_pose.point.x + random_distance * cos(random_angle),
            y=self.obstacle_pose.point.y + random_distance * sin(random_angle),
            theta=0.0
        )
        self.get_logger().info(f'Sub-goal created at ({self.avoid_goal.x}, {self.avoid_goal.y}) to avoid the obstacle')

    def move_robot(self):
        """
        Move the robot towards the goal position
        """
        if self.avoid_goal:
            target = self.avoid_goal
        else:
            target = self.goal_pose

        euclidean_distance = sqrt(pow(target.x - self.current_pose.pose.pose.position.x, 2) +
                                pow(target.y - self.current_pose.pose.pose.position.y, 2))

        # Verify if the robot is close to the goal
        if self.avoid_goal and euclidean_distance < 0.2:
            self.get_logger().info("Sub-goal reached, moving to the main goal")
            self.avoid_goal = None 
            self.obstacle_cleared = True 

        # checks if obstacle is far away after bypassing
        if self.obstacle_cleared and self.obstacle_pose:
            obstacle_distance = sqrt(pow(self.obstacle_pose.point.x - self.current_pose.pose.pose.position.x, 2) +
                                    pow(self.obstacle_pose.point.y - self.current_pose.pose.pose.position.y, 2))
            if obstacle_distance > 1.5:  #  If the obstacle is far away, consider it as "out of danger"
                self.obstacle_pose = None  # Consider the obstacle as cleared
                self.get_logger().info("Obstacle avoided, moving to the goal")

        angle_to_goal = atan2(target.y - self.current_pose.pose.pose.position.y,
                            target.x - self.current_pose.pose.pose.position.x)


        linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.1)
        velocity_msg.twist.angular.z = angular_gain * angle_to_goal
        self.publisher_.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
