# goal_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose2D

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')

        self.create_subscription(PointStamped, '/object_position', self.object_position_callback, 10)
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)
        self.get_logger().info('Goal node initialized')

    def object_position_callback(self, msg):
        goal = Pose2D()
        goal.x = msg.point.x
        goal.y = msg.point.y
        goal.theta = 0.0 

        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Published goal: x={goal.x:.2f}, y={goal.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    goal_node = GoalNode()
    rclpy.spin(goal_node)
    goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()