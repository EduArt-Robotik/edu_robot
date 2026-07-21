import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # get general parameters
    package_path = FindPackageShare('edu_robot')
    parameter_file = PathJoinSubstitution([
      './',
      'eduard-iot2050.yaml'
    ])

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

    # Eduard IoT2050 Node
    motor_parameter_file_path = PathJoinSubstitution([
      './',
      PythonExpression(["'motor_faulhaber.yaml' if '", LaunchConfiguration('motor_model'), "' == 'faulhaber' else 'motor_leison.yaml'"])
    ])

    eduard_iot2050 = Node(
      package='edu_robot',
      executable='eduard-iot-bot',
      name='eduard',
      parameters=[
        parameter_file,
        motor_parameter_file_path
      ],
      namespace=edu_robot_namespace,
      # prefix=['gdbserver localhost:3000'],
      output='screen',
      # enable for debug prints
      # arguments=[
      #   "--ros-args",
      #   "--log-level",
      #   "edu_robot:=debug"
      # ]    
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
      eduard_iot2050,
      aggregator
    ])
    