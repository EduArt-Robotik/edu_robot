# Imports
import rclpy
import pytest
from time import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

# Globals
_DEBUG = False
_TOLERANCE = 0.5

# Helper Functions
def print_green(text):
    print(f"\033[92m{text}\033[0m") 
def print_blue(text):
    print(f"\033[94m{text}\033[0m") 
def print_yellow(text):
    print(f"\033[93m{text}\033[0m") 
def print_red(text):
    print(f"\033[91m{text}\033[0m") 

def print_debug(text):
    print_yellow(f'Debug - {text}')
def print_error(text):
    print_red(f'Error - {text}')

def checkTolerance(value, targetValue, tolerance):
    offset = value - targetValue
    
    if (abs(offset) < tolerance):
        print_green(f'Deviation from target value:{offset:.4f}')
        return True
    else:
        print_error(f'Offset:{offset:.4f}')
        pytest.fail("Outside Tolerance!")
        return False


# Fixtures
## Can be called by/as a function parameter. Return the object/variable thats besides the "yield" keyword
@pytest.fixture(scope='module', autouse=True)
def ros_setup():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def imu_test_node():
    node = rclpy.create_node('system_test_imu')
    yield node
    node.destroy_node()

@pytest.fixture(scope='function')
def imu_subscriber(imu_test_node):

    # Dictionary to hold IMU data
    imu_data = {'linear_accel_x': 0, 'linear_accel_y': 0, 'linear_accel_z': 0, 'count': 0, 'running': False}

    def callback(msg):
        if imu_data['running']:
            imu_data['linear_accel_x'] += msg.linear_acceleration.x
            imu_data['linear_accel_y'] += msg.linear_acceleration.y
            imu_data['linear_accel_z'] += msg.linear_acceleration.z
            imu_data['count'] += 1
            if _DEBUG:
                print_debug(f'Linear Acceleration: x: {msg.linear_acceleration.x:.4f}, y: {msg.linear_acceleration.y:.4f}, z: {msg.linear_acceleration.z:.4f}')
                print_debug(f'Sum Acceleration: x: {imu_data["linear_accel_x"]:.4f}, y: {imu_data["linear_accel_y"]:.4f}, z: {imu_data["linear_accel_z"]:.4f}, count: {imu_data["count"]}')

    imu_test_node.create_subscription(
        Imu, 'eduard/orange/imu', callback, QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
    )
    
    yield imu_data 

# Tests
def test_z_acceleration(imu_test_node, imu_subscriber):
    implementedTest('Z', 'All wheels on the ground, base plate is parallel to the floor', imu_test_node, imu_subscriber)

def test_y_acceleration(imu_test_node, imu_subscriber):
    implementedTest('Y', 'On the right two wheels, base plate is vertical to the floor', imu_test_node, imu_subscriber)

def test_x_acceleration(imu_test_node, imu_subscriber):
    implementedTest('X', 'On the back, base plate is vertical to the floor', imu_test_node, imu_subscriber)


# Test implementation
def implementedTest(axis, description, node, imu_data):
    # Wait for User to be ready.
    print('\n')
    print_blue(f'Linear {axis} Acceleration of IMU')
    print('Please place the Robot in the following way:')
    print(f'{description}')
    print('')
    
    # Wait for user input to start the test
    user_input = input('Press "s" to start the test or any other key to skip this test: ')
    
    if user_input.strip().lower() != 's':
        pytest.skip()
    
    print('Getting linear accelerations...')
    imu_data['running'] = True  # Start the test
    start_time = time()

    # Run the test for a specific duration
    while (time() - start_time <= 1.0):
        rclpy.spin_once(node)  # Process any incoming messages

    imu_data['running'] = False  # Stop the test

    if imu_data['count'] <= 0:
        pytest.fail("No data received during the 1-second window")

    print('Calculating average...')
    avg_x = imu_data['linear_accel_x'] / imu_data['count']
    avg_y = imu_data['linear_accel_y'] / imu_data['count']
    avg_z = imu_data['linear_accel_z'] / imu_data['count']
    if _DEBUG:
        print_debug(f'Average Linear Acceleration: x: {avg_x:.4f}, y: {avg_y:.4f}, z: {avg_z:.4f}')

    if avg_x == 0 or avg_y == 0 or avg_z == 0: 
        pytest.fail('No data received during the 1-second window')

    
    if (axis.lower() == 'x'):
        checkTolerance(avg_x, 9.81, _TOLERANCE)
    elif (axis.lower() == 'y'):
        checkTolerance(avg_y, 9.81, _TOLERANCE)
    elif (axis.lower() == 'z'):
        checkTolerance(avg_z, 9.81, _TOLERANCE)
    else:
        pytest.fail('No vaild axis')
   
    print_green("Passed")

