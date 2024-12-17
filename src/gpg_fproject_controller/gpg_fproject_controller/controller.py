import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, cos, sin, pi
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
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
        self.declare_parameter('linear_gain', 0.1)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('avoid_distance', 5.0) 
        
        self.tf_buffer = Buffer()   
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.avoid_goal = None  # Reset sub-goal
        self.obstacle_cleared = False
        
        
    def obstacle_callback(self, msg):
        self.obstacle_pose = msg
        self.define_avoid_goal()


    #def obstacle_callback(self, msg):
    #    self.obstacle_pose = msg
    #    if msg and not self.avoid_goal: 
    #        self.define_avoid_goal()

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_robot()

    def define_avoid_goal(self):
        """
        Dynamically create a sub-goal to the left or right of the obstacle, depending on free space.
        """
        avoid_distance = self.get_parameter('avoid_distance').get_parameter_value().double_value
        
        if self.current_pose is None or self.goal_pose is None or self.obstacle_pose is None:
            self.get_logger().warn("Insufficient data to define avoid goal.")
            return

        odom_obstacle = self.obstacle_pose
        odom_obstacle.header.stamp = self.get_clock().now().to_msg()
        try:
            base_obstacle = self.tf_buffer.transform(odom_obstacle, 'base_link')
        except Exception as e:
            self.get_logger().warn(e)
            return
        
        if sqrt(base_obstacle.point.x**2 + base_obstacle.point.y**2) > avoid_distance:
            return

        odom_goal = self.goal_pose
        odom_goal.header.stamp = self.get_clock().now().to_msg()
        try:
            base_goal = self.tf_buffer.transform(odom_goal, 'base_link')
        except Exception as e:
            self.get_logger().warn(e)
            return
        
        base_avoid = PointStamped()
        base_avoid.header.stamp = self.get_clock().now().to_msg()
        base_avoid.header.frame_id = 'base_link'
        base_avoid.point.x = avoid_distance if base_goal.point.y > 0 else -avoid_distance
            
        try:
            self.avoid_goal = self.tf_buffer.transform(base_avoid, 'odom')   
        except Exception as e:
            self.get_logger().warn(e)
            return
            
        self.get_logger().info(f"Sub-goal created at ({self.avoid_goal.x:.2f}, {self.avoid_goal.y:.2f}) to avoid the obstacle.")

    def move_robot(self):
        """
        Move the robot towards the goal position or the sub-goal if defined.
        """
        if self.avoid_goal:
            target = self.avoid_goal
        else:
            target = self.goal_pose

        # Calculate Euclidean distance to the target
        euclidean_distance = sqrt(pow(target.x - self.current_pose.pose.pose.position.x, 2) +
                                  pow(target.y - self.current_pose.pose.pose.position.y, 2))

        if self.avoid_goal and euclidean_distance < 0.2:
            self.get_logger().info("Sub-goal reached, resuming to main goal.")
            self.avoid_goal = None
            self.obstacle_cleared = True

        # Check if obstacle is far enough to be considered cleared
        if self.obstacle_cleared and self.obstacle_pose:
            obstacle_distance = sqrt(pow(self.obstacle_pose.point.x - self.current_pose.pose.pose.position.x, 2) +
                                     pow(self.obstacle_pose.point.y - self.current_pose.pose.pose.position.y, 2))
            if obstacle_distance > 1.5:
                self.obstacle_pose = None  # Mark the obstacle as cleared
                self.get_logger().info("Obstacle avoided, moving to the main goal.")

        # Compute angle to target
        angle_to_goal = atan2(target.y - self.current_pose.pose.pose.position.y,
                              target.x - self.current_pose.pose.pose.position.x)

        # Gain parameters
        linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

        # Create and publish velocity commands
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.2)
        velocity_msg.twist.angular.z = max(min(angular_gain * angle_to_goal, 1.5), -1.5)
        self.publisher_.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
