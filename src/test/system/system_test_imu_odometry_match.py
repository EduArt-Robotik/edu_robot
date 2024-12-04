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
from sensor_msgs.msg import Imu

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
    self.collect_data = False
    self.imu_data = []
    self.odom_data = []

    self.node = rclpy.create_node('system_test_timeout')
    self.pub_twist = self.node.create_publisher(Twist, namespace + '/cmd_vel', 2)
    self.sub_odom = self.node.create_subscription(
      Odometry, namespace + '/odometry', self.callbackOdometry, QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100
      )
    )
    self.sub_imu = self.node.create_subscription(
      Imu, namespace + '/imu', self.callbackImu, QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100
      )
    )
    self.srv_set_mode = self.node.create_client(SetMode, namespace + '/set_mode')
    self.srv_reset_odometry = self.node.create_client(Trigger, namespace + '/reset_odometry')
    self.terminal_settings = termios.tcgetattr(sys.stdin)

  def callbackOdometry(self, msg : Odometry):
    if self.collect_data is False:
      return
    # collecting data is in progress

    # pick only yaw rate
    self.odom_data.append(msg.twist.twist.angular.z)

  def callbackImu(self, msg : Imu):
    if self.collect_data is False:
      return
    # collecting data is in progress

    # pick only yaw rate   
    self.imu_data.append(msg.angular_velocity.z)

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
    twist_msg.angular.z = 0.0
    self.pub_twist.publish(twist_msg)

    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = Mode.INACTIVE
    set_mode_request.mode.drive_kinematic = Mode.MECANUM_DRIVE
    set_mode_request.disable_feature = Mode.COLLISION_AVOIDANCE_OVERRIDE

    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_set_mode.call_async(set_mode_request)
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().state.mode.mode is Mode.INACTIVE

  def test_compare_imu_and_odometry(self, launch_service, proc_output):
    # Wait for User to be ready.
    print('###########################################################################################')
    print('# Compare IMU and Odometry while rotating around z axis                                   #')
    print('###########################################################################################')
    print('Please place the IoTBot on the ground.')
    print('Please make sure the IoTBot can rotate.')
    print('Press any key "s" to start the test...')
    while self.getKey(0.1) != 's': pass
    print('Test is running...')

    # Rotate around the z axis.
    ## Reset Odometry
    print('Reset Odometry')
    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_reset_odometry.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().success is True

    ## Enable Robot
    velocity = 1.0
    test_running = True
    twist_msg = Twist()
    twist_msg.angular.z = velocity

    self.pub_twist.publish(twist_msg)
    self.enableRobot()

    ## Drive in x direction until distance of one meter is reached.
    stamp_last_sent = time()
    stamp_start = stamp_last_sent
    wait_time_twist = 1.0 / 10.0 # 10 Hz

    print('Rotating robot...')
    while rclpy.ok() and test_running:
      # Sending Twist with 10 Hz
      stamp = time()

      if stamp - stamp_last_sent > wait_time_twist:
        self.pub_twist.publish(twist_msg)
        stamp_last_sent = time()

      if stamp - stamp_start > 2.0:
        self.collect_data = True

      if stamp - stamp_start > 5.0:
        self.collect_data = False

        # Check if there were any data received.
        if len(self.imu_data) == 0 or len(self.odom_data) == 0:
          self.fail()
          return
        
        # calculate test result
        mean_imu = sum(self.imu_data) / len(self.imu_data)
        mean_odom = sum(self.odom_data) / len(self.odom_data)

        print('Received IMU yaw rate measurements:')
        print('count = ', len(self.imu_data))
        [print(data) for data in self.imu_data]
        print('Recieved Odometry yaw rate measurements:')
        print('count = ', len(self.odom_data))
        [print(data) for data in self.odom_data]

        print('IMU mean = ', mean_imu)
        print('Odometry mean = ', mean_odom)

        # check test result
        ## mean values should be close to given yaw rate
        assert abs(mean_imu - velocity) < 0.1

        ## mean values should be close together
        assert abs(mean_imu - mean_odom) < 0.1

        ## terminate test
        test_running = False

      # spinning with 100 Hz
      rclpy.spin_once(self.node, timeout_sec=0.01)


    ## Disabling Robot
    print('Test finished after ' + str(time() - stamp_start) + ' s.')
    self.disableRobot()
