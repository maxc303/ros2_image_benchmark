import launch
import yaml
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""

    config_path = os.path.join(
        get_package_share_directory('image_latency_benchmark'),
        'config',
        'pub_sub_test_params.yaml'
        )

    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)['pub_sub_test']['ros__parameters']

    #Ref: https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html
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
                    parameters=[config_params],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='image_latency_benchmark',
                    plugin='image_latency_benchmark::ImageSubTestNode',
                    name='sub_test',
                    parameters=[config_params],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])