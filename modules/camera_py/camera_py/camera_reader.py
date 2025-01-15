import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CameraReader(Node):

    def __init__(self):
        super().__init__("camera_reader")
        self.publisher_ = self.create_publisher(
            String,
            "topic",
            10,
        )
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = "Hello World: %d" % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    # opencv2 
    # def publish_frame


def main(args=None):
    rclpy.init(args=args)

    camera_reader = CameraReader()

    rclpy.spin(camera_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
