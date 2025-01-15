from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts,  #type
                                        'add_two_ints',  # name
                                         self.add_two_ints_callback)  # callback

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()  # 初始化ROS2

    minimal_service = MinimalService()  # 创建节点

    rclpy.spin(minimal_service)  

    rclpy.shutdown()


if __name__ == '__main__':
    main()