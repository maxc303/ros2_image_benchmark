import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='image_latency_benchmark',
            executable='test_image_pub',
            name='image_pub_node'),
  ])