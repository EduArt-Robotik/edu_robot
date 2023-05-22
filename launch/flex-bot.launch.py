import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_path('edu_robot')
    parameter_file = os.path.join(
      package_path,
      'parameter',
      'flex_bot.yaml'
    )

    ethernet_gateway_flex_bot = Node(
      package='edu_robot',
      executable='ethernet-gateway-flex-bot',
      name='ethernet_gateway_flex_bot',
      # prefix=['gdbserver localhost:3000'],
      parameters=[parameter_file],
      output='screen'
    )

    return LaunchDescription([
      ethernet_gateway_flex_bot
    ])
    