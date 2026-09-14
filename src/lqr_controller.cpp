#include <algorithm>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace quadcopter_sim
{
class LqrController : public rclcpp::Node
{
public:
  LqrController() : Node("controller")
  {
    mass_ = declare_parameter("mass", 1.5);
    gravity_ = declare_parameter("gravity", 9.81);
    
    //Dodanie maksymalnych ograniczeń dla ciągu i momentu obrotowego
    max_thrust_ = declare_parameter("max_thrust", 32.0);
    max_torque_xy_ = declare_parameter("max_torque_xy", 5.0);
    max_torque_z_ = declare_parameter("max_torque_z", 2.0);
    
    //Dodanie parametrów regulatora LQR dla osi x, y, z i yaw
    k_xy_position_ = declare_parameter("k_xy.position", 0.63245553);
    k_xy_velocity_ = declare_parameter("k_xy.velocity", 0.98346093);
    k_xy_angle_ = declare_parameter("k_xy.angle", 4.39887596);
    k_xy_rate_ = declare_parameter("k_xy.rate", 0.81482057);
    k_z_position_ = declare_parameter("k_z.position", 3.16227766);
    k_z_velocity_ = declare_parameter("k_z.velocity", 3.67244237);
    k_yaw_angle_ = declare_parameter("k_yaw.angle", 1.58113883);
    k_yaw_rate_ = declare_parameter("k_yaw.rate", 0.82092952);

    //Publikowanie informacji na temat jakie siły powinien gazebo zastosować na quadcopterze, aby osiągnąć pozycję docelową. Subskrybowanie informacji o pozycji docelowej i odometrii quadcoptera.
    command_pub_ = create_publisher<geometry_msgs::msg::Wrench>("command/wrench", 10);
    
    //Sukskrypcja informacji o celu
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "command/pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        target_ = *msg;
        have_target_ = true;
      });
    
    //Sukskrypcja informacji o odometrii  
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&LqrController::on_odometry, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "LQR flight controller started");
  }

private:
  //Sprowadzenie kąta do -180 do 180 stopni
  static double wrap(double value)
  {
    return std::atan2(std::sin(value), std::cos(value));
  }

  //sILNIK NIE JEST W STANIE wytwarzac dowolnie duzego momentu, zatem potrzebna jest saturacja
  static double limited(double value, double limit)
  {
    return std::clamp(value, -limit, limit);
  }

  //Konwersja kwaternionu do kątów Euleroa
  static void to_rpy(
    const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
  {
    roll = std::atan2(
      2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    pitch = std::asin(std::clamp(
      2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  //Wyznaczanie prędkości w osi pionowej 
  static double body_vertical_velocity_in_world(
    const geometry_msgs::msg::Quaternion & q, const geometry_msgs::msg::Vector3 & velocity)
  {
    return 2.0 * (q.x * q.z - q.w * q.y) * velocity.x +
      2.0 * (q.y * q.z + q.w * q.x) * velocity.y +
      (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * velocity.z;
  }

  //Obliczenia regulatora LQR w oparciu o aktualną pozycję quadcoptera i pozycję docelową
  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (!have_target_) {
      command_pub_->publish(geometry_msgs::msg::Wrench());
      return;
    }

    double roll, pitch, yaw, unused_roll, unused_pitch, yaw_ref;
    to_rpy(odom->pose.pose.orientation, roll, pitch, yaw);
    to_rpy(target_.pose.orientation, unused_roll, unused_pitch, yaw_ref);

    const double ex_world = target_.pose.position.x - odom->pose.pose.position.x;
    const double ey_world = target_.pose.position.y - odom->pose.pose.position.y;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);

    // Rotate world-frame position and velocity into the horizontal body axes.
    const double ex = cosine * ex_world + sine * ey_world;
    const double ey = -sine * ex_world + cosine * ey_world;
    // Odometry twist is expressed in base_link, as required by nav_msgs/Odometry.
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double vz = body_vertical_velocity_in_world(
      odom->pose.pose.orientation, odom->twist.twist.linear);
    const double ez = target_.pose.position.z - odom->pose.pose.position.z;

    geometry_msgs::msg::Wrench command;
    // u = -K(x-x_ref) Wyznaczenie wartosci sterowan
    const double vertical_correction =
      k_z_position_ * ez - k_z_velocity_ * vz;
    const double tilt_compensation = std::max(0.5, std::cos(roll) * std::cos(pitch));
    
    command.force.z = std::clamp(
      (mass_ * gravity_ + vertical_correction) / tilt_compensation,
      0.0, max_thrust_);
    
      command.torque.y = limited(
      k_xy_position_ * ex - k_xy_velocity_ * vx -
      k_xy_angle_ * pitch - k_xy_rate_ * odom->twist.twist.angular.y,
      max_torque_xy_);
    
      command.torque.x = limited(
      -k_xy_position_ * ey + k_xy_velocity_ * vy -
      k_xy_angle_ * roll - k_xy_rate_ * odom->twist.twist.angular.x,
      max_torque_xy_);
    
      command.torque.z = limited(
      k_yaw_angle_ * wrap(yaw_ref - yaw) -
      k_yaw_rate_ * odom->twist.twist.angular.z,
      max_torque_z_);
    
      command_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::PoseStamped target_;
  bool have_target_{false};
  double mass_, gravity_, max_thrust_, max_torque_xy_, max_torque_z_;
  double k_xy_position_, k_xy_velocity_, k_xy_angle_, k_xy_rate_;
  double k_z_position_, k_z_velocity_, k_yaw_angle_, k_yaw_rate_;
};
} 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadcopter_sim::LqrController>());
  rclcpp::shutdown();
  return 0;
}
