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
        self.obstacle_publishers = {} 

        self.memory_size = 5
        self.duplicate_threshold = 0.001
        self.x_coords, self.y_coords, self.z_coords = [], [], []

    def obstacle_callback(self, msg):
        """
        Callback function for the obstacle topic
        """
        x_get_time_step, y_get_time_step, z_get_time_step = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        if self.verify_duplicated_obstacle(x_get_time_step, y_get_time_step, z_get_time_step):
            self.get_logger().info('Obstacle is duplicated in the obstacle memory node')
            return
        
        self.x_coords.append(x_get_time_step)
        self.y_coords.append(y_get_time_step)
        self.z_coords.append(z_get_time_step)

        if len(self.x_coords) > self.memory_size:
            self.x_coords.pop(0)
            self.y_coords.pop(0)
            self.z_coords.pop(0)

        self.get_logger().info('Obstacle position received in the obstacle memory node')
        self.publish_obstacle_memory()

    def verify_duplicated_obstacle(self, x_get_time_step, y_get_time_step, z_get_time_step):
        """
        Verify if the obstacle is already in the memory
        """
        if not self.x_coords:
            return False
        
        distance_euclideen = np.sqrt(
            (x_get_time_step - np.array(self.x_coords))**2 + 
            (y_get_time_step - np.array(self.y_coords))**2 + 
            (z_get_time_step - np.array(self.z_coords))**2
        )
        return np.any(distance_euclideen < self.duplicate_threshold)

    def publish_obstacle_memory(self):
        """
        Publish the obstacle memory
        """
        obstacle_msg = UnboundedFloat()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.float_storage  = [val for sublist in zip(self.x_coords, self.y_coords, self.z_coords) for val in sublist]
        self.memory_publisher.publish(obstacle_msg)
        
        float_array = np.array(obstacle_msg.float_storage)
        if float_array.size % 3 == 0:
            array_format = float_array.reshape(-1, 3)
            for i, obs_coords in enumerate(array_format):
                self.get_logger().info(f'Obstacle {i} position: {obs_coords}')
                topic_name = f'/obstacle_position_{i}'
                if topic_name not in self.obstacle_publishers:
                    self.obstacle_publishers[topic_name] = self.create_publisher(PointStamped, topic_name, 10)
                PS = PointStamped()
                PS.header.stamp = self.get_clock().now().to_msg()
                PS.header.frame_id = ''
                PS.point.x, PS.point.y, PS.point.z = obs_coords
                try:
                    self.obstacle_publishers[topic_name].publish(PS)
                    self.get_logger().info(f'Obstacle {i} published. Well done buddy!')
                except Exception as e:
                    self.get_logger().error(f'Error publishing obstacle to topic {topic_name}: {e} ! Check it out!')
        else:
            self.get_logger().error('Obstacle memory storage has incorrect shape!')
            return        

def main(args=None):
    rclpy.init(args=args)
    obstacle_memory_node = ObstacleMemoryNode()
    rclpy.spin(obstacle_memory_node)
    obstacle_memory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()