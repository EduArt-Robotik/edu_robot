import os
import sys
import rclpy
import pytest
import unittest
from time import time, sleep
from select import select
import termios
import tty
import math
from enum import Enum

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

class OrientationQuadrant(Enum):
  I=1   #   0° ..   90°
  II=2  #  90° ..  180°
  III=3 # -90° .. -180°
  IV=4  #   0° ..  -90°

def euler_to_radian(euler):
  return (euler / 180.0) * math.pi

def quaternion_to_yaw(q):
  """Convert a quaternion (geometry_msgs/Quaternion) to yaw (radians). [-pi:pi]"""
  x, y, z, w = q.x, q.y, q.z, q.w
  return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def yaw_to_quadrant(yaw):
  print('yaw = ', yaw)
  if yaw >= euler_to_radian(0.0) and yaw < euler_to_radian(90.0):
    return OrientationQuadrant.I
  elif yaw >= euler_to_radian(90.0) and yaw <= euler_to_radian(180.0):
    return OrientationQuadrant.II
  elif yaw < euler_to_radian(0.0) and yaw >= euler_to_radian(-90.0):
    return OrientationQuadrant.IV
  elif yaw < euler_to_radian(-90.0) and yaw > euler_to_radian(-180.0):
    return OrientationQuadrant.III
  else:
    raise "yaw angle out of range"

def print_quadrant(quadrant):
  if quadrant == OrientationQuadrant.I:
    print('quadrant I')
  elif quadrant == OrientationQuadrant.II:
    print('quadrant II')
  elif quadrant == OrientationQuadrant.III:
    print('quadrant III')
  elif quadrant == OrientationQuadrant.IV:
    print('quadrant IV')     

@pytest.mark.launch_test
def generate_test_description():
  dummy_node = Node(
    package='demo_nodes_cpp',
    executable='listener',
    output='log'
  )

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
    # Test will publish velocity commands for a boogie woogie robot dance. It brings a high load on the motors.
    namespace = 'eduard/pi_bot'
    self.node = rclpy.create_node('boogie_woogie_motor_test')
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

  def resetOdometry(self):
    print('Resetting Odometry')
    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_reset_odometry.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().success is True

  def test_drive_one_meter_in_x_with_mecanum(self, launch_service, proc_output):
    # Wait for User to be ready.
    print('###########################################################################################')
    print('# Boogie Woogie Motor Test                                                                #')
    print('###########################################################################################')
    print('The robot will rotate in both direction alternately.')
    print('Please make sure the robot can do this movement without colliding with something.')
    print('Press any key "s" to start the test...')
    while self.getKey(0.1) != 's': pass
    print('Test is running...')

    # Reset Odometry
    self.resetOdometry()

    ## Enable Robot
    twist_msg = Twist()
    self.pub_twist.publish(twist_msg)
    self.enableRobot()

    ## Drive in x direction until distance of one meter is reached.
    stamp_last_sent = time()
    stamp_start = stamp_last_sent
    wait_time_twist = 1.0 / 10.0 # 10 Hz
    last_quadrant = OrientationQuadrant.I
    velocity_abs = math.pi / 2.0
    velocity = velocity_abs
    through_zero = True

    print('Driving One Meter in X Direction')
    while rclpy.ok():
      # Rotate in 
      print('')
      quadrant = yaw_to_quadrant(quaternion_to_yaw(self.odom_msg.pose.pose.orientation))

      print('current quadrant: ')
      print_quadrant(quadrant)
      print('last quadrant: ')
      print_quadrant(last_quadrant)

      # Detecting zero crossing
      if quadrant == OrientationQuadrant.I and last_quadrant == OrientationQuadrant.IV:
        print('from IV to I')
        through_zero = True
      elif quadrant == OrientationQuadrant.IV and last_quadrant == OrientationQuadrant.I:
        print('from I to IV')
        through_zero = True

      # Detecting 180° crossing
      if quadrant == OrientationQuadrant.II and last_quadrant == OrientationQuadrant.III and through_zero:
        print('from III to II')
        through_zero = False
        velocity = velocity_abs
      elif quadrant == OrientationQuadrant.III and last_quadrant == OrientationQuadrant.II and through_zero:
        print('from II to III')
        through_zero = False
        velocity = -velocity_abs

      last_quadrant = quadrant

      # Sending Twist with 10 Hz
      if time() - stamp_last_sent > wait_time_twist:
        twist_msg.angular.z = velocity
        self.pub_twist.publish(twist_msg)
        stamp_last_sent = time()

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)

    ## Disabling Robot
    print('Test finished after ' + str(time() - stamp_start) + ' s.')
    self.disableRobot()
