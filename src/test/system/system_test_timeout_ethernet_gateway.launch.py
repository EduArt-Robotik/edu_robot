import os
import sys
import rclpy
import pytest
import unittest
from time import time

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions

from edu_robot.srv import SetMode
from edu_robot.msg import Mode, RobotStatusReport
from geometry_msgs.msg import Twist

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
  path_to_test = os.path.dirname(__file__)
  package_path = FindPackageShare('edu_robot')
  parameter_file = PathJoinSubstitution([
    package_path,
    'parameter',
    'eduard-ipc127e.yaml'
  ])    

  eduard_ipc = Node(
    package='edu_robot',
    executable='ethernet-gateway',
    name='eduard',
    parameters=[parameter_file],
    namespace='test',
    output='screen'
  )

  return LaunchDescription([
    eduard_ipc,
    launch_testing.actions.ReadyToTest()
  ])

class SystemTestTimeout(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    # Initialize the ROS context for the test node
    rclpy.init()

  @classmethod
  def tearDownClass(cls):
    # Shutdown the ROS context
    rclpy.shutdown()

  def setUp(self):
    # Create a ROS node for tests
    self.node = rclpy.create_node('system_test_timeout')
    self.pub_twist = self.node.create_publisher(Twist, "/test/cmd_vel", 1)
    self.sub_state = self.node.create_subscription(
      RobotStatusReport, "/test/status_report", self.callback_status_report, 100
    )
    self.current_robot_mode = Mode.UNKNOWN

  def tearDown(self):
    self.node.destroy_node()

  def callback_status_report(self, report):
    self.current_robot_mode = report.robot_state.mode

  def enable_robot(self):
    print('enable robot')
    twist_msg = Twist()
    stamp_start = time()

    while rclpy.ok() and self.current_robot_mode is not Mode.REMOTE_CONTROLLED:
      self.pub_twist.publish(twist_msg)
      rclpy.spin_once(self.node, timeout_sec=0.01)

      assert time() - stamp_start < 10.0

  def test_no_timeout_occurred(self, launch_service, proc_output):
    self.enable_robot()

    stamp_last = time()
    wait_time = 1.0 / 10.0 # 10 Hz
    twist_msg = Twist()

    while rclpy.ok():
      # publishing twist msg with 10 Hz
      if time() - stamp_last > wait_time:
        self.pub_twist.publish(twist_msg)
        stamp_last = time()

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

      assert self.current_robot_mode is Mode.REMOTE_CONTROLLED

