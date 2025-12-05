import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # get general parameters
    package_path = FindPackageShare('edu_robot')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'eduard-ipc127e.yaml'
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

    # Eduard IPC127E Node
    motor_parameter_file_path = PathJoinSubstitution([
      package_path,
      'parameter',
      PythonExpression(["'motor_faulhaber.yaml' if '", LaunchConfiguration('motor_model'), "' == 'faulhaber' else 'motor_leison.yaml'"])
    ])

    eduard_ipc = Node(
      package='edu_robot',
      executable='eduard-ethernet-gateway-bot',
      name='eduard',
      parameters=[
        parameter_file,
        motor_parameter_file_path
      ],
      namespace=edu_robot_namespace,
      # prefix=['gdbserver localhost:3000'],
      output='screen',
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
      eduard_ipc,
      aggregator
    ])
