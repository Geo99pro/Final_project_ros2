import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from custom_msg_fproject.msg import UnboundedFloat

class ObstacleMemoryNode(Node):
    def __init__(self):
        super().__init__('obstacle_memory_node')
        self.create_subscription(PointStamped, '/obstacle_position', self.obstacle_callback, 10)
        self.memory_publisher = self.create_publisher(UnboundedFloat, '/obstacle_memory', 10)
        self.publishers = {}

        self.memory_size = 5
        self.x_coords, self.y_coords, self.z_coords = [], [], []

    def obstacle_callback(self, msg):
        """
        Callback function for the obstacle topic
        """
        self.x_coords.append(float(msg.point.x))
        self.y_coords.append(float(msg.point.y))
        self.z_coords.append(float(msg.point.z))

        if len(self.x_coords) > self.memory_size:
            self.x_coords.pop(0)
            self.y_coords.pop(0)
            self.z_coords.pop(0)

        self.get_logger().info('Obstacle position received in the obstacle memory node')
        self.publish_obstacle_memory()

    def publish_obstacle_memory(self):
        """
        Publish the obstacle memory
        """
        obstacle_msg = UnboundedFloat()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.float_storage = np.array([self.x_coords, self.y_coords, self.z_coords]).T.tolist() # convert matrix to list of lists with each list containing x, y, z coordinates
        self.memory_publisher.publish(obstacle_msg)
        
        for i, obs_coords in enumerate(obstacle_msg.float_storage): # obs_coords is a 1D array of x, y, z coordinates
            topic_name = f'/obstacle_position_{i}'
            if topic_name not in self.publishers:
                self.publishers[topic_name] = self.create_publisher(PointStamped, topic_name, 10)
            
            PS = PointStamped()
            PS.header.stamp = self.get_clock().now().to_msg()
            PS.point.x, PS.point.y, PS.point.z = obs_coords

            try:
                self.publishers[topic_name].publish(PS)
            except Exception as e:
                self.get_logger().error(f'Error publishing obstacle to topic {topic_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_memory_node = ObstacleMemoryNode()
    rclpy.spin(obstacle_memory_node)
    obstacle_memory_node.destroy_node()
    rclpy.shutdown()



















            

    # def save_obstacle_memory(self):
    #     """
    #     Save the obstacle memory
    #     """
    #     if self.obstacle_memory is not None:
    #         obstacle_dict = {
    #             'X': self.obstacle_memory.point.x, 
    #             'Y': self.obstacle_memory.point.y, 
    #             'Z': self.obstacle_memory.point.z
    #             }
    #         self.obstacle_memory_list.append(obstacle_dict)
    #     return self.obstacle_memory_list
    
    # def publish_obstacle_memory(self):
    #     """
    #     Publish the obstacle memory
    #     """
    #     self.obstacle_memory_list = self.save_obstacle_memory()
    #     for obstacle in self.obstacle_memory_list:
    #         obstacle_msg = PointStamped()
    #         obstacle_msg.header.stamp = self.get_clock().now().to_msg()
    #         obstacle_msg.point.x = obstacle['X']
    #         obstacle_msg.point.y = obstacle['Y']
    #         obstacle_msg.point.z = obstacle['Z']
    #         self.publisher_.publish(obstacle_msg)




    

