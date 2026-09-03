#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace quadcopter_sim
{
class MotorAllocator : public rclcpp::Node
{
public:
  MotorAllocator() : Node("motor_allocator")
  {
    thrust_coefficient_ = declare_parameter("thrust_coefficient", 8.5e-6);
    torque_coefficient_ = declare_parameter("torque_coefficient", 1.6e-7);
    arm_x_ = declare_parameter("arm_x", 0.23);
    arm_y_ = declare_parameter("arm_y", 0.23);
    min_speed_ = declare_parameter("min_rotor_speed", 0.0);
    max_speed_ = declare_parameter("max_rotor_speed", 1100.0);

    rotor_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "command/rotor_speed", 10);
    wrench_sub_ = create_subscription<geometry_msgs::msg::Wrench>(
      "command/wrench", 10,
      std::bind(&MotorAllocator::allocate, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Motor allocator started (order: FL, FR, RL, RR; command in rad/s)");
  }

private:
  void allocate(const geometry_msgs::msg::Wrench::ConstSharedPtr command)
  {
    const double yaw_arm = torque_coefficient_ / thrust_coefficient_;
    const double thrust = std::max(0.0, command->force.z);
    const double roll = command->torque.x / arm_y_;
    const double pitch = command->torque.y / arm_x_;
    const double yaw = command->torque.z / yaw_arm;

    // Rotor order: front-left, front-right, rear-left, rear-right.
    // FL and RR rotate CCW; FR and RL rotate CW.
    const std::array<double, 4> rotor_thrust{
      0.25 * (thrust + roll - pitch + yaw),
      0.25 * (thrust - roll - pitch - yaw),
      0.25 * (thrust + roll + pitch - yaw),
      0.25 * (thrust - roll + pitch + yaw)};

    std_msgs::msg::Float64MultiArray speeds;
    speeds.data.resize(4);
    for (size_t i = 0; i < rotor_thrust.size(); ++i) {
      const double omega = std::sqrt(std::max(0.0, rotor_thrust[i]) / thrust_coefficient_);
      speeds.data[i] = std::clamp(omega, min_speed_, max_speed_);
    }
    rotor_pub_->publish(speeds);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rotor_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
  double thrust_coefficient_, torque_coefficient_, arm_x_, arm_y_;
  double min_speed_, max_speed_;
};
}  // namespace quadcopter_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadcopter_sim::MotorAllocator>());
  rclcpp::shutdown();
  return 0;
}
