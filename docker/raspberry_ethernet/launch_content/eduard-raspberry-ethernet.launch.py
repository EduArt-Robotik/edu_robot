import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile 
from launch_ros.substitutions import FindPackageShare

def get_motor_parameter_file(context: LaunchContext, motor_model_arg: LaunchConfiguration):
    motor_model = motor_model_arg.perform(context)
    print("Using motor model: ", motor_model)
    if motor_model == 'leison':
        # Leison motor
        return PathJoinSubstitution([
            FindPackageShare('edu_robot'),
            './',
            'motor_leison.yaml'
        ]).perform(context)
    else:
        # Faulhaber motor
        return PathJoinSubstitution([
            FindPackageShare('edu_robot'),
            './',
            'motor_faulhaber.yaml'
        ]).perform(context)

def generate_launch_description():
    package_path = FindPackageShare('edu_robot')
    parameter_file = PathJoinSubstitution([
      './',
      'eduard-raspberry.yaml'
    ])

    # robot namespace
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
      'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )  

    # get motor model
    motor_model_arg = DeclareLaunchArgument(
      'motor_model', default_value=os.getenv('EDU_MOTOR_MODEL', default='faulhaber'), description='Motor model to use (faulhaber or leison)'
    )

    # pick corresponding parameter file
    if os.getenv('EDU_MOTOR_MODEL', default='faulhaber') == 'leison':
      # Leison motor
      motor_parameter_file = PathJoinSubstitution([
        './',
        'motor_leison.yaml'
      ])      
    else:
      # Faulhaber motor
      motor_parameter_file = PathJoinSubstitution([
        './',
        'motor_faulhaber.yaml'
      ])

    print("Using motor parameter file: ", motor_parameter_file)

    eduard_raspberry = Node(
      package='edu_robot',
      executable='eduard-ethernet-gateway-bot',
      name='eduard',
      parameters=[
        parameter_file,
        motor_parameter_file
        # ParameterFile(OpaqueFunction(function=get_motor_parameter_file, args=[LaunchConfiguration('motor_model')]))
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
      motor_model_arg,
      eduard_raspberry,
      aggregator
    ])
