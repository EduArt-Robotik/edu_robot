import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, TextSubstitution, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_robot_model(context: LaunchContext, robot_name_arg: LaunchConfiguration, wheel_type_arg: LaunchConfiguration,
                         hardware_type_arg: LaunchConfiguration, urdf_eduard_model_path_arg: PathJoinSubstitution) -> str:
    robot_name = robot_name_arg.perform(context)
    wheel_type = wheel_type_arg.perform(context)
    hardware_type = hardware_type_arg.perform(context)
    urdf_eduard_model_path = urdf_eduard_model_path_arg.perform(context)

    print('urdf file = ', urdf_eduard_model_path)
    print("use robot name = ", robot_name)
    print("use wheel type = ", wheel_type)
    robot_description = xacro.process_file(
        urdf_eduard_model_path,
        mappings={
            'robot_name': robot_name,
            'wheel_type': wheel_type,
            'hardware'  : hardware_type
        }
    ).toprettyxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
        ],
        namespace=robot_name
    )

    return [ robot_state_publisher_node ]

def generate_launch_description():
    # launch file arguments
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )

    wheel_type = LaunchConfiguration('wheel_type')
    wheel_type_arg = DeclareLaunchArgument(
        'wheel_type', default_value=os.getenv('EDU_ROBOT_WHEEL_TYPE', default='mecanum')
    )

    hardware_type = LaunchConfiguration('hardware_type')
    hardware_type_arg = DeclareLaunchArgument('hardware_type', default_value='unknown')

    # create robot model
    urdf_eduard_model_path = PathJoinSubstitution([
        './',
        'eduard.urdf'
    ])

    # publishing robot model via node
    robot_description_publisher = OpaqueFunction(
        function=generate_robot_model,
        args=[
            edu_robot_namespace,
            wheel_type,
            hardware_type,
            urdf_eduard_model_path
        ]
    )

    # Optional GPIO Hardware
    robot_controller = PathJoinSubstitution([
      './',
      'eduard_ros2_control_raspberry.yaml'
    ])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controller],
        namespace=edu_robot_namespace,
        output="both",
        condition=LaunchConfigurationEquals('hardware_type', 'raspberry')
    )
    gpio_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["gpio_controller", "--param-file", robot_controller],
      namespace=edu_robot_namespace,
      condition=LaunchConfigurationEquals('hardware_type', 'raspberry'),
      output='both'
    )

    return LaunchDescription([
        edu_robot_namespace_arg,
        wheel_type_arg,
        hardware_type_arg,
        robot_description_publisher,
        control_node,
        gpio_controller
    ])
