import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_path('edu_robot')
    parameter_file = os.path.join(
      './',
      'universal_bot.yaml'
    )

    ethernet_gateway_flex_bot = Node(
      package='edu_robot',
      executable='ethernet-gateway-universal-bot',
      name='ethernet_gateway_universal_bot',
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),      
      # prefix=['gdbserver localhost:3000'],
      parameters=[parameter_file],
      output='screen'
    )

    return LaunchDescription([
      ethernet_gateway_flex_bot
    ])
    