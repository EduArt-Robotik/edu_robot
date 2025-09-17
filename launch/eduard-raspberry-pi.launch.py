import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # motor model
    motor_model = os.environ.get('EDU_MOTOR_MODEL', 'faulhaber')    
    # robot namespace
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )    

    # pick corresponding parameter file
    if motor_model == 'leison':
      # Leison motor
      motor_parameter_file = PathJoinSubstitution([
         package_path,
        'parameter',
        'eduard_raspberry_motor_leison.yaml'
      ])      
    else:
      # Faulhaber motor
      motor_parameter_file = PathJoinSubstitution([
        package_path,
        'parameter',
        'eduard_raspberry_motor_faulhaber.yaml'
      ])

    # Eduard Raspberry Pi Bot Node
    package_path = FindPackageShare('edu_robot')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'eduard-raspberry-pi.yaml'
    ])

    eduard_ipc = Node(
      package='edu_robot',
      executable='eduard-360-pi-bot',
      name='eduard',
      parameters=[
        parameter_file,
        motor_parameter_file
      ],
      namespace=edu_robot_namespace,
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
      edu_robot_namespace_arg,
      eduard_ipc,
      aggregator
    ])
