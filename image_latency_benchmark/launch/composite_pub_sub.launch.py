import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='test_pub_sub',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_latency_benchmark',
                    plugin='image_latency_benchmark::ImagePubTestNode',
                    name='pub_test',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='image_latency_benchmark',
                    plugin='image_latency_benchmark::ImageSubTestNode',
                    name='sub_test',
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])