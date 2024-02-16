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
      'pi-bot.yaml'
    ])

    pi_bot = Node(
      package='edu_robot',
      executable='can-gateway-universal-bot',
      name='pi_bot',
      parameters=[parameter_file],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      pi_bot
    ])
    