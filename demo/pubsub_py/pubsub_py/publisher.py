import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(
            String,  # msg_type
            "demo_topic_py",  # topic:str
            10,  # qos_profile: QoSProfile | int,
        )

        # 创建一个计时器，它将每 0.5 秒调用一次 timer_callback() 方法。
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(
            timer_period,  # timer_period_sec
            self.timer_callback,  # callback 回调函数
        )
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.i += 1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
