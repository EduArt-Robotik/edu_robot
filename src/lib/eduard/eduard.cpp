#include <edu_robot/eduard/eduard.hpp>
#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/robot.hpp>
#include <edu_robot/range_sensor.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace eduard {

Eduard::Eduard(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
  : robot::Robot(robot_name, std::move(hardware_interface))
{ }

void Eduard::initialize(std::map<std::string, std::shared_ptr<HardwareComponentInterface<Color, Lighting::Mode>>> lightings_hardware,
                        std::map<std::string, std::shared_ptr<HardwareComponentInterface<Rpm>>> motor_controller_hardware,
                        std::map<std::string, std::shared_ptr<HardwareSensorInterface<Rpm>>> motor_sensor_hardware,
                        std::map<std::string, std::shared_ptr<HardwareSensorInterface<float>>> range_sensor_hardware,
                        std::map<std::string, std::shared_ptr<HardwareSensorInterface<Eigen::Quaterniond>>> imu_sensor_hardware)
{
  // Lightings
  registerLighting(std::make_shared<robot::Lighting>(
    "head",
    COLOR::DEFAULT::HEAD,
    1.0f,
    lightings_hardware.at("head")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "right_side",
    COLOR::DEFAULT::HEAD,
    1.0f,
    lightings_hardware.at("right_side")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "left_side",
    COLOR::DEFAULT::BACK,
    1.0f,
    lightings_hardware.at("left_side")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "back",
    COLOR::DEFAULT::BACK,
    1.0f,
    lightings_hardware.at("back")
  ));

  // Use all representation to set a initial light.
  auto lighting_all = std::make_shared<robot::Lighting>(
    "all",
    COLOR::DEFAULT::HEAD,
    1.0f,
    lightings_hardware.at("all")
  );
  lighting_all->setColor(COLOR::DEFAULT::BACK, Lighting::Mode::RUNNING);
  registerLighting(lighting_all);


  // Motor Controllers
  registerMotorController(std::make_shared<robot::MotorController>(
    "motor_a",
    0u,
    robot::MotorController::Parameter{ },
    "base_to_wheel_rear_right",
    *this,
    motor_controller_hardware.at("motor_a"),
    motor_sensor_hardware.at("motor_a")
  ));
  registerMotorController(std::make_shared<robot::MotorController>(
    "motor_b",
    1u,
    robot::MotorController::Parameter{ },
    "base_to_wheel_front_right",
    *this,
    motor_controller_hardware.at("motor_b"),
    motor_sensor_hardware.at("motor_b")
  ));
  registerMotorController(std::make_shared<robot::MotorController>(
    "motor_c",
    2u,
    robot::MotorController::Parameter{ },
    "base_to_wheel_rear_left",
    *this,
    motor_controller_hardware.at("motor_c"),
    motor_sensor_hardware.at("motor_c")
  ));
  registerMotorController(std::make_shared<robot::MotorController>(
    "motor_d",
    3u,
    robot::MotorController::Parameter{ },
    "base_to_wheel_front_left",
    *this,
    motor_controller_hardware.at("motor_d"),
    motor_sensor_hardware.at("motor_d")
  ));


  // Range Sensors
  constexpr eduart::robot::RangeSensor::Parameter range_sensor_parameter{ 10.0 * M_PI / 180.0, 0.01, 5.0 };

  auto range_sensor = std::make_shared<robot::RangeSensor>(
    "range/front/left",
    "range/front/left",
    "base_link",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.17, 0.063, 0.045)),
    range_sensor_parameter,
    *this,
    range_sensor_hardware.at("range/front/left")
  );
  registerSensor(range_sensor);
  range_sensor->registerComponentInput(_collision_avoidance_component);

  range_sensor = std::make_shared<robot::RangeSensor>(
    "range/front/right",
    "range/front/right",
    "base_link",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.17, -0.063, 0.045)),
    range_sensor_parameter,
    *this,
    range_sensor_hardware.at("range/front/right")
  );
  registerSensor(range_sensor);
  range_sensor->registerComponentInput(_collision_avoidance_component);

  range_sensor = std::make_shared<robot::RangeSensor>(
    "range/rear/left",
    "range/rear/left",
    "base_link",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, 0.063, 0.05)),
    range_sensor_parameter,
    *this,
    range_sensor_hardware.at("range/rear/left")
  );
  registerSensor(range_sensor);
  range_sensor->registerComponentInput(_collision_avoidance_component);

  range_sensor = std::make_shared<robot::RangeSensor>(
    "range/rear/right",
    "range/rear/right",
    "base_link",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, -0.063, 0.05)),
    range_sensor_parameter,
    *this,
    range_sensor_hardware.at("range/rear/right")
  );
  registerSensor(range_sensor);
  range_sensor->registerComponentInput(_collision_avoidance_component);

  // IMU Sensor
  auto imu_sensor = std::make_shared<robot::ImuSensor>(
    "imu",
    "imu/base",
    "base_footprint",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.1)),
    ImuSensor::Parameter{ false, "base_link" },
    getTfBroadcaster(),
    *this,
    imu_sensor_hardware.at("imu")
  );
  registerSensor(imu_sensor);

  // Set Up Drive Kinematic
  // \todo make it configurable
  // \todo handle Mecanum kinematic, too
  constexpr float l_y = 0.32f;
  constexpr float l_x = 0.25f;
  constexpr float l_squared = l_x * l_x + l_y * l_y;
  constexpr float wheel_diameter = 0.17f;

  _kinematic_matrix.resize(4, 3);
  _kinematic_matrix << -1.0f, 0.0f, -l_squared / (2.0f * l_y),
                       -1.0f, 0.0f, -l_squared / (2.0f * l_y),
                        1.0f, 0.0f, -l_squared / (2.0f * l_y),
                        1.0f, 0.0f, -l_squared / (2.0f * l_y);
  _kinematic_matrix *= 1.0f / wheel_diameter;

  // Configure Processing Components
  
}

Eduard::~Eduard()
{

}

} // end namespace eduard
} // end namespace robot
} // end namespace eduart
