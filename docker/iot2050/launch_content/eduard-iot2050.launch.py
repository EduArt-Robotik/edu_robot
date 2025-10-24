import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    motor_model = os.environ.get('EDU_MOTOR_MODEL', 'faulhaber')
    package_path = FindPackageShare('edu_robot')

    # pick corresponding parameter file
    if motor_model == 'leison':
      # Leison motor
      parameter_file = PathJoinSubstitution([
        './',
        'eduard-iot2050-leison.yaml'
      ])      
    else:
      # Faulhaber motor
      parameter_file = PathJoinSubstitution([
        './',
        'eduard-iot2050-faulhaber.yaml'
      ])

    eduard_iot2050 = Node(
      package='edu_robot',
      executable='eduard-iot-bot',
      name='eduard',
      parameters=[parameter_file],
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      # prefix=['gdbserver localhost:3000'],
      output='screen'      
    )

    aggregator = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          package_path,
          'launch',
          'eduard-diagnostic.launch.py'
        ]),
      ])
    )

    return LaunchDescription([
      eduard_iot2050,
      aggregator
    ])
    