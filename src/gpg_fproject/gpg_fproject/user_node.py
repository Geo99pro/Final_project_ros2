#The purpose is to take a input of a user and return the user's information as a topic to the image node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserNode(Node):
    def __init__(self):
        super().__init__('user_node')
        self.user_publisher = self.create_publisher(String, '/object_color', 10)
        self.another_user_publisher = self.create_publisher(String, '/object_form', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.user_callback)

    def user_callback(self):
        color_input = input('Enter a color. Color should be between [blue, red, green, yellow]: ')
        form_input = input('Enter a form. Form should be between [box, triangle, circle]: ')
        color_info = self.get_user_color(color_input)
        object_info = self.get_user_form(form_input)
        self.user_publisher.publish(color_info)
        self.another_user_publisher.publish(object_info)
        self.get_logger().info(f'Dear user the color published is : {color_info.data}\nand the object form is : {object_info.data}')

    def get_user_color(self, color_input):
        color_info = String()
        if color_input not in ['blue', 'red', 'green', 'yellow']:
            raise ValueError('Invalid input color. Color must be one of the following: blue, red, green, yellow')
        color_info.data = color_input
        return color_info

    def get_user_form(self, form_input):
        form_info = String()
        if form_input not in ['box', 'triangle', 'circle']:
            raise ValueError('Invalid input form. Form must be one of the following: box, triangle, circle')
        form_info.data = form_input
        return form_info

def main(args=None):
    rclpy.init(args=args)
    user_node = UserNode()
    rclpy.spin(user_node)
    user_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()