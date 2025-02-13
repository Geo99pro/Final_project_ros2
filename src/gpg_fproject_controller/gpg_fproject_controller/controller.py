import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
from geometry_msgs.msg import TwistStamped, Pose2D, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from std_msgs.msg import Int32
from custom_msg_fproject.msg import UnboundedFloat
from tf_transformations import euler_from_quaternion

class MovePhysicalRobot(Node):
    """
    Class to move the physical robot to a goal position
    """
    def __init__(self):
        super().__init__('move_physical_robot')
        self.publishers_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        self.create_subscription(PointStamped, '/obstacle_position', self.obstacle_callback, 10)
        self.create_subscription(UnboundedFloat, '/obstacle_memory', self.obstacle_memory_callback, 10)
        #self.create_subscription(Int32, '/obstacle_memory_size', self.obstacle_memory_size_callback, 10) 

        self.goal_pose = None
        self.current_pose = None
        self.obstacle_pose = None

        self.declare_parameter('linear_gain', 0.1)
        self.declare_parameter('angular_gain', 0.5)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('repulsive_gain', 0.5)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg):
        """
        Callback function for the goal topic
        """
        self.goal_pose = msg # Pose2D msg
        self.get_logger().info(f'New goal received: {self.goal_pose.x}, {self.goal_pose.y}')

    def obstacle_callback(self, msg):
        """
        Callback function for the obstacle topic
        """
        self.obstacle_pose = msg #
        self.get_logger().info(f'New live obstacle received: {self.obstacle_pose.point.x}, {self.obstacle_pose.point.y}')

    def obstacle_memory_callback(self, msg):
        """
        Callback function for the obstacle memory topic
        """

        data = msg.float32_values # UnboundedFloat msg
        self.get_logger().info(f'Obstacle memory received: {data}')
        obstacles = []

        if len(data) % 3 != 0:
            self.get_logger().info('Invalid obstacle memory, from a point of view of the obstacle memory size')
        
        else:
            for i in range(0, len(data), 3):
                obstacles.append((data[i], data[i+1], data[i+2]))
            self.obstacle_memory = obstacles # list of tuples
            self.get_logger().info(f'Obstacle memory received: {self.obstacle_memory}')
            
    def pose_callback(self, msg):
        """
        Callback function for the odometry topic
        """
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_physical_robot()
            
    def stop_robot(self, euclidean_distance_to_goal):
        """
        Stop the robot if the goal is reached
        """
        if euclidean_distance_to_goal < 0.05:
            self.get_logger().info('Goal reached')
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.twist.linear.x = 0.0
            velocity_msg.twist.angular.z = 0.0
            self.publishers_.publish(velocity_msg)
            self.goal_pose = None

    def move_physical_robot(self):
        if self.goal_pose is None or self.current_pose is None:
            return 

        """
        Move the physical robot to the goal position
        """
        current_x_position = self.current_pose.pose.pose.position.x
        current_y_position = self.current_pose.pose.pose.position.y

        #convert quaternion to yaw
        #https://robotics.stackexchange.com/questions/96357/ros2-python-quaternion-to-euler
        orientation_q = self.current_pose.pose.pose.orientation
        quanternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quanternion)


        goal_x_position = self.goal_pose.x
        goal_y_position = self.goal_pose.y

        f_goal_x = goal_x_position - current_x_position
        f_goal_y = goal_y_position - current_y_position

        f_rep_x = 0.0
        f_rep_y = 0.0

        repulsive_gain = self.get_parameter('repulsive_gain').get_parameter_value().double_value
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

        if self.obstacle_pose is not None:
            obs_x = self.obstacle_pose.point.x
            obs_y = self.obstacle_pose.point.y
            distance = sqrt((obs_x - current_x_position)**2 + (obs_y - current_y_position)**2)
            if distance < obstacle_threshold and distance > 0.0:
                force = repulsive_gain / distance**2
                f_rep_x += force * (current_x_position - obs_x) / distance
                f_rep_y += force * (current_y_position - obs_y) / distance

        for obs in self.obstacle_memory:
            obs_x, obs_y, obs_z = obs
            distance = sqrt((obs_x - current_x_position)**2 + (obs_y - current_y_position)**2)
            if distance < obstacle_threshold and distance > 0.0:
                force = repulsive_gain / distance**2
                f_rep_x += force * (current_x_position - obs_x) / distance
                f_rep_y += force * (current_y_position - obs_y) / distance

        f_total_x = f_goal_x + f_rep_x
        f_total_y = f_goal_y + f_rep_y
        desired_angle = atan2(f_total_y, f_total_x)
        angle_diff = desired_angle - yaw

        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        distance_to_goal = sqrt((goal_x_position - current_x_position)**2 + (goal_y_position - current_y_position)**2)
        if distance_to_goal < 0.05:
            self.stop_robot(distance_to_goal)
            return
        
        linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        
        linear_speed = linear_gain* sqrt(f_total_x**2 + f_total_y**2)
        angular_speed = angular_gain * angle_diff

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.twist.linear.x = linear_speed
        velocity_msg.twist.angular.z = angular_speed
        self.publishers_.publish(velocity_msg)
        self.get_logger().info(f'Linear speed: {linear_speed:.2f}, Angular speed: {angular_speed:.2f}, goal_distance: {distance_to_goal:.2f}')

def main(args=None):
    """
    Main function to run the move_physical_robot node
    """
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()