#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace quadcopter_sim
{
class GazeboActuator : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    link_ = model_->GetLink(sdf->Get<std::string>("body_link", "base_link").first);
    if (!link_) {gzerr << "Quadcopter: body link not found\n"; return;}
    thrust_coefficient_ = sdf->Get<double>("thrust_coefficient", 8.5e-6).first;
    torque_coefficient_ = sdf->Get<double>("torque_coefficient", 1.6e-7).first;
    arm_x_ = sdf->Get<double>("arm_x", 0.23).first;
    arm_y_ = sdf->Get<double>("arm_y", 0.23).first;
    motor_time_constant_ = sdf->Get<double>("motor_time_constant", 0.06).first;
    max_rotor_speed_ = sdf->Get<double>("max_rotor_speed", 1100.0).first;
    node_ = gazebo_ros::Node::Get(sdf);
    command_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "command/rotor_speed", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) return;
        std::lock_guard<std::mutex> lock(command_mutex_);
        for (size_t i = 0; i < 4; ++i) {
          commanded_speed_[i] = std::clamp(msg->data[i], 0.0, max_rotor_speed_);
        }
      });
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    speed_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("rotor_speed", 10);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboActuator::on_update, this));
    last_update_time_ = model_->GetWorld()->SimTime();
    RCLCPP_INFO(node_->get_logger(), "Four-rotor motor model loaded");
  }

private:
  void on_update()
  {
    const auto now = model_->GetWorld()->SimTime();
    const double dt = (now - last_update_time_).Double();
    last_update_time_ = now;

    std::array<double, 4> target;
    {std::lock_guard<std::mutex> lock(command_mutex_); target = commanded_speed_;}
    if (dt > 0.0 && dt < 0.1) {
      const double response = 1.0 - std::exp(-dt / motor_time_constant_);
      for (size_t i = 0; i < 4; ++i) {
        rotor_speed_[i] += response * (target[i] - rotor_speed_[i]);
      }
    }

    std::array<double, 4> rotor_thrust{};
    for (size_t i = 0; i < 4; ++i) {
      rotor_thrust[i] = thrust_coefficient_ * rotor_speed_[i] * rotor_speed_[i];
    }
    const double thrust = rotor_thrust[0] + rotor_thrust[1] +
      rotor_thrust[2] + rotor_thrust[3];
    const double roll_torque = arm_y_ * (
      rotor_thrust[0] - rotor_thrust[1] + rotor_thrust[2] - rotor_thrust[3]);
    const double pitch_torque = arm_x_ * (
      -rotor_thrust[0] - rotor_thrust[1] + rotor_thrust[2] + rotor_thrust[3]);
    const double yaw_torque = torque_coefficient_ * (
      rotor_speed_[0] * rotor_speed_[0] - rotor_speed_[1] * rotor_speed_[1] -
      rotor_speed_[2] * rotor_speed_[2] + rotor_speed_[3] * rotor_speed_[3]);

    link_->AddRelativeForce({0.0, 0.0, thrust});
    link_->AddRelativeTorque({roll_torque, pitch_torque, yaw_torque});
    publish_rotor_speed();
    publish_odom();
  }
  void publish_rotor_speed()
  {
    // Reduce ROS traffic: Gazebo physics runs at 500 Hz, telemetry at 50 Hz is enough.
    if (++speed_publish_divider_ < 10) return;
    speed_publish_divider_ = 0;
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(rotor_speed_.begin(), rotor_speed_.end());
    speed_pub_->publish(msg);
  }
  void publish_odom()
  {
    const auto time = model_->GetWorld()->SimTime();
    const auto pose = model_->WorldPose();
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = time.sec;
    msg.header.stamp.nanosec = time.nsec;
    msg.header.frame_id = "world";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();
    msg.pose.pose.position.z = pose.Pos().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    // nav_msgs/Odometry defines twist in child_frame_id, hence in base_link here.
    const auto linear = link_->RelativeLinearVel(), angular = link_->RelativeAngularVel();
    msg.twist.twist.linear.x = linear.X();
    msg.twist.twist.linear.y = linear.Y();
    msg.twist.twist.linear.z = linear.Z();
    msg.twist.twist.angular.x = angular.X();
    msg.twist.twist.angular.y = angular.Y();
    msg.twist.twist.angular.z = angular.Z();
    odom_pub_->publish(msg);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr speed_pub_;
  gazebo::common::Time last_update_time_;
  std::array<double, 4> commanded_speed_{};
  std::array<double, 4> rotor_speed_{};
  std::mutex command_mutex_;
  double thrust_coefficient_, torque_coefficient_, arm_x_, arm_y_;
  double motor_time_constant_, max_rotor_speed_;
  int speed_publish_divider_{0};
};
GZ_REGISTER_MODEL_PLUGIN(GazeboActuator)
}  // namespace quadcopter_sim
