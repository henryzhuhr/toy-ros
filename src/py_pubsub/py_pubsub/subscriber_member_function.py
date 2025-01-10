import rclpy
from rclpy.node import Node  # 导入Node类

from std_msgs.msg import String  # 导入String消息类型


class MinimalSubscriber(Node):     # 创建一个继承自Node的类

    def __init__(self):          # 构造函数，在类实例化时会自动调用
        super().__init__('minimal_subscriber')         # 调用父类的构造 
        self.subscription = self.create_subscription(      # 创建一个订阅者
            String,
            'topic',
            self.listener_callback,      # 回调函数
            10)             # 订阅队列长度为10
        self.subscription  # prevent unused variable warning 预防未使用参数警告

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)          # 打印日志


def main(args=None):
    rclpy.init(args=args)               # 初始化ROS2    

    minimal_subscriber = MinimalSubscriber()            # 创建节点实例                    

    rclpy.spin(minimal_subscriber)                  # 进入spin循环

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()