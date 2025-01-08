import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pubsub_py",
                executable="subscriber",
                name="demo_subscriber",
            ),
            launch_ros.actions.Node(
                package="pubsub_py",
                executable="publisher",
                name="demo_publisher",
            ),
        ]
    )
