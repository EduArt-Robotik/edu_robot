import os
import sys
import rclpy
import pytest
import unittest
from time import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions

from edu_robot.srv import SetMode
from edu_robot.msg import Mode, RobotStatusReport
from geometry_msgs.msg import Twist

@pytest.mark.launch_test
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
    namespace='test'
    # prefix=['gdbserver localhost:3000']
    # output='screen'
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
    self.pub_twist = self.node.create_publisher(Twist, '/test/cmd_vel', 2)
    self.sub_state = self.node.create_subscription(
      RobotStatusReport, '/test/status_report', self.callback_status_report, QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100
      )
    )
    self.srv_set_mode = self.node.create_client(SetMode, '/test/set_mode')
    self.current_robot_mode = Mode.UNKNOWN

  def tearDown(self):
    self.node.destroy_node()

  def callback_status_report(self, report):
    self.current_robot_mode = report.robot_state.mode.mode
    print("received mode: " + str(report.robot_state.mode))

  def enable_robot(self):
    print('enable robot')
    self.stamp_last_twist = time()
    stamp_last_set_mode = self.stamp_last_twist
    stamp_start = self.stamp_last_twist
    wait_time = 1.0 / 10.0 # 10 Hz
    twist_msg = Twist()

    while rclpy.ok() and (self.current_robot_mode is not Mode.REMOTE_CONTROLLED):
      # publishing twist msg with 10 Hz
      if time() - self.stamp_last_twist > wait_time:
        self.pub_twist.publish(twist_msg)
        self.stamp_last_twist = time()

      # set mode to REMOTE_CONTROLLED
      if self.current_robot_mode != Mode.REMOTE_CONTROLLED and time() - stamp_last_set_mode > 1.0:
        set_mode_request = SetMode.Request()
        set_mode_request.mode.mode = Mode.REMOTE_CONTROLLED
        stamp_last_set_mode = time()

        if (self.srv_set_mode.service_is_ready()):
          print("calling set mode service: try to enable robot")
          future = self.srv_set_mode.call_async(set_mode_request)
          rclpy.spin_until_future_complete(self.node, future)

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

      assert time() - stamp_start < 1000.0

    print('robot is enabled')

  def test_no_timeout_occurred(self, launch_service, proc_output):
    self.enable_robot()
    print("test_no_timeout_occurred")
    stamp_start = time()
    wait_time = 0.4 # 400 ms --> no timeout
    twist_msg = Twist()

    # execute test for 10 seconds
    while rclpy.ok() and time() - stamp_start < 10.0:
      # publishing twist msg with 1/0.4 Hz
      if time() - self.stamp_last_twist > wait_time:
        self.pub_twist.publish(twist_msg)
        self.stamp_last_twist = time()

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

      assert self.current_robot_mode is Mode.REMOTE_CONTROLLED

  def test_timeout_occurred(self, launch_service, proc_output):
    self.enable_robot()
    print("test_timeout_occurred")

    stamp_start = self.stamp_last_twist
    wait_time = 0.4 # 400 ms --> timeout
    wait_time_timeout = 0.6 # 600ms --> timeout
    twist_msg = Twist()

    # execute test for 10 seconds
    while rclpy.ok() and time() - stamp_start < 10.0:
      # publishing twist msg until 3s
      if time() - self.stamp_last_twist > wait_time and time() - stamp_start < 3.0:
        self.pub_twist.publish(twist_msg)
        self.stamp_last_twist = time()
        assert self.current_robot_mode is Mode.REMOTE_CONTROLLED

      # interrupt sending twist message until 3.6s
      if time() - self.stamp_last_twist > wait_time_timeout and time() - stamp_start > 3.0:
        self.pub_twist.publish(twist_msg)
        self.stamp_last_twist = time()

      if time() - stamp_start > 4.5: # 1 second interval for feedback --> 3s + 1s + tolerance
        assert self.current_robot_mode is Mode.INACTIVE


      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

      