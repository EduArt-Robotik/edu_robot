import os
import sys
import rclpy
import pytest
import unittest
from time import time, sleep
from select import select
import termios
import tty

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions

from edu_robot.srv import SetMode
from edu_robot.msg import Mode, RobotStatusReport
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

@pytest.mark.launch_test
def generate_test_description():
  dummy_node = Node(
    package='demo_nodes_cpp',
    executable='listener',
    output='log'
  )

  # dummy_node = Command(
  #   command='tail -F anything'
  # )

  return LaunchDescription([
    dummy_node, # need node to make test running
    launch_testing.actions.ReadyToTest()
  ])

class SystemTestIotBotOdometry(unittest.TestCase):

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
    namespace = 'eduard/blue'
    self.node = rclpy.create_node('system_test_timeout')
    self.pub_twist = self.node.create_publisher(Twist, namespace + '/cmd_vel', 2)
    self.sub_odom = self.node.create_subscription(
      Odometry, namespace + '/odometry', self.callbackOdometry, QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100
      )
    )
    self.srv_set_mode = self.node.create_client(SetMode, namespace + '/set_mode')
    self.srv_reset_odometry = self.node.create_client(Trigger, namespace + '/reset_odometry')
    self.odom_msg = Odometry()
    self.odom_msg.pose.pose.position.x = 0.0
    self.terminal_settings = termios.tcgetattr(sys.stdin)

  def callbackOdometry(self, msg):
    self.odom_msg = msg

  def getKey(self, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)

    return key

  def enableRobot(self):
    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = Mode.REMOTE_CONTROLLED
    set_mode_request.mode.drive_kinematic = Mode.MECANUM_DRIVE
    set_mode_request.mode.feature_mode = Mode.COLLISION_AVOIDANCE_OVERRIDE

    print('Enabling IotBot')
    assert self.srv_set_mode.service_is_ready() is True
    future = self.srv_set_mode.call_async(set_mode_request)
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().state.mode.mode is Mode.REMOTE_CONTROLLED

  def disableRobot(self):
    print('Disabling IotBot')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    self.pub_twist.publish(twist_msg)

    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = Mode.INACTIVE
    set_mode_request.mode.drive_kinematic = Mode.MECANUM_DRIVE
    set_mode_request.disable_feature = Mode.COLLISION_AVOIDANCE_OVERRIDE

    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_set_mode.call_async(set_mode_request)
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().state.mode.mode is Mode.INACTIVE

  def test_drive_one_meter_in_x_with_mecanum(self, launch_service, proc_output):
    # Wait for User to be ready.
    print('###########################################################################################')
    print('# Test drive one meter in x direction with Mecanum                                        #')
    print('###########################################################################################')
    print('Please place the IoTBot on the ground with 1m space in front of the robot.')
    print('Please make sure the Mecanum wheels are mounted.')
    print('Mark the starting position so you can measure the driven distance after the test finished.')
    print('Press any key "s" to start the test...')
    while self.getKey(0.1) != 's': pass
    print('Test is running...')

    # Drive 1 meter straight in x direction.
    ## Reset Odometry
    print('Reset Odometry')
    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_reset_odometry.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().success is True

    ## Enable Robot
    self.enableRobot()

    ## Drive in x direction until distance of one meter is reached.
    stamp_last_sent = time()
    stamp_start = stamp_last_sent
    wait_time_twist = 1.0 / 10.0 # 10 Hz
    goal_distance = 1.0
    slow_down_distance = 0.1

    print('Driving One Meter in X Direction')
    while rclpy.ok() and self.odom_msg.pose.pose.position.x < goal_distance:
      # Calculate
      position_diff = self.odom_msg.pose.pose.position.x - goal_distance
      velocity_x = 0.3 if abs(position_diff) > slow_down_distance else 0.1

      # Sending Twist with 10 Hz
      if time() - stamp_last_sent > wait_time_twist:
        twist_msg = Twist()
        twist_msg.linear.x = velocity_x
        self.pub_twist.publish(twist_msg)
        stamp_last_sent = time()

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

    ## Disabling Robot
    print('Test finished after ' + str(time() - stamp_start) + ' s.')
    self.disableRobot()


  def test_drive_one_meter_in_y_with_mecanum(self, launch_service, proc_output):
    # Wait for User to be ready.
    print('###########################################################################################')
    print('# Test drive one meter in y direction with Mecanum                                        #')
    print('###########################################################################################')
    print('Please place the IoTBot on the ground with 1m space on left side of the robot.')
    print('Please make sure the Mecanum wheels are mounted.')
    print('Mark the starting position so you can measure the driven distance after the test finished.')
    print('Press any key "s" to start the test...')
    while self.getKey(0.1) != 's': pass
    print('Test is running...')

    # Drive 1 meter straight in x direction.
    ## Reset Odometry
    print('Reset Odometry')
    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_reset_odometry.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().success is True

    ## Enable Robot
    self.enableRobot()

    ## Drive in x direction until distance of one meter is reached.
    stamp_last_sent = time()
    stamp_start = stamp_last_sent
    wait_time_twist = 1.0 / 10.0 # 10 Hz
    goal_distance = 1.0
    slow_down_distance = 0.1

    print('Driving One Meter in X Direction')
    while rclpy.ok() and self.odom_msg.pose.pose.position.y < goal_distance:
      # Calculate
      position_diff = self.odom_msg.pose.pose.position.y - goal_distance
      velocity_y = 0.3 if abs(position_diff) > slow_down_distance else 0.1

      # Sending Twist with 10 Hz
      if time() - stamp_last_sent > wait_time_twist:
        twist_msg = Twist()
        twist_msg.linear.y = velocity_y
        self.pub_twist.publish(twist_msg)
        stamp_last_sent = time()

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

    ## Disabling Robot
    print('Test finished after ' + str(time() - stamp_start) + ' s.')
    self.disableRobot()