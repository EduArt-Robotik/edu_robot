import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_path = FindPackageShare('edu_robot')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'ohmni-bot.yaml'
    ])

    ohmni_bot = Node(
      package='edu_robot',
      executable='ethernet-gateway-ohmni-bot',
      name='ohmni_bot',
      parameters=[parameter_file],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      ohmni_bot
    ])
    