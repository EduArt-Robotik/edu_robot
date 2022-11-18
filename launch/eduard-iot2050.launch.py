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
      'eduard.yaml'
    )

    eduard_iot2050 = Node(
      package='edu_robot',
      executable='iotbot-shield',
      name='eduard_iot2050',
      parameters= [parameter_file]
    )

    return LaunchDescription([
      eduard_iot2050
    ])
    