import yaml
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
    get_package_share_directory('image_latency_benchmark'),
    'config',
    'pub_sub_test_params.yaml'
    )
    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)['pub_sub_test']['ros__parameters']

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='image_latency_benchmark',
            executable='test_image_sub',
            name='image_pub_node',
            parameters=[config_params],),
  ])