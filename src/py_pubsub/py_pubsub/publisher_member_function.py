import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):  # 创建一个继承自Node的类
    def __init__(self):  # 构造函数，在类实例化时会自动调用
        super().__init__("minimal_publisher")  # 调用父类的构造
        self.publisher_ = self.create_publisher(
            String,  # msg_type
            "topic",  # topic: str
            10,  # qos_profile
        )
        timer_period = 0.5  # 单位seconds
        self.timer = self.create_timer(  # 创建一个定时器，每隔0.5秒调用一次timer_callback
            timer_period,
            self.timer_callback,
        )
        self.i = 0

    def timer_callback(self):  # 定时器回调函数
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)  # 发布消息
        self.get_logger().debug('Publishing: "%s"' % msg.data)  # 打印日志
        self.i += 1


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2

    minimal_publisher = MinimalPublisher()  # 创建节点实例
    rclpy.spin(minimal_publisher)  # 进入spin循环

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2


if __name__ == "__main__":  # 用于判断当前脚本是否作为主程序执行。如果是，则执行main()函数
    main()
