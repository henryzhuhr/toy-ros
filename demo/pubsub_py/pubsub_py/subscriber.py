import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String,  # msg_type
            "demo_topic_py",  # topic
            self.listener_callback,  # callback
            10,  # qos_profile
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: String):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
