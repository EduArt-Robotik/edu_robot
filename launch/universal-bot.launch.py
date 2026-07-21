import os

from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # get general parameters
    package_path = get_package_share_path('edu_robot')
    parameter_file = os.path.join(
      package_path,
      'parameter',
      'universal_bot.yaml'
    )

    # robot namespace
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )  

    # get motor model
    motor_model_arg = DeclareLaunchArgument(
      'motor_model', default_value=os.getenv('EDU_MOTOR_MODEL', default='faulhaber'),
      description='Motor model to use (faulhaber or leison)'
    )

    # Universal Ethernet Gateway Bot Node
    motor_parameter_file_path = PathJoinSubstitution([
      package_path,
      'parameter',
      PythonExpression(["'motor_faulhaber.yaml' if '", LaunchConfiguration('motor_model'), "' == 'faulhaber' else 'motor_leison.yaml'"])
    ])
    
    ethernet_gateway_flex_bot = Node(
      package='edu_robot',
      executable='universal-ethernet-gateway-bot',
      name='universal_ethernet_gateway_bot',
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),      
      # prefix=['gdbserver localhost:3000'],
      parameters=[parameter_file],
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
      motor_model_arg,
      LogInfo(msg=["Using motor parameter file: ", motor_parameter_file_path]),
      ethernet_gateway_flex_bot,
      aggregator
    ])
