import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_path = get_package_share_path('edu_robot')
    parameter_file = os.path.join(
      package_path,
      'parameter',
      'eduard-ipc127e.yaml'
    )

    eduard_ipc = Node(
      package='edu_robot',
      executable='ethernet-gateway',
      name='eduard',
      parameters=[parameter_file],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      eduard_ipc
    ])
    