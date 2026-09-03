#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace quadcopter_sim
{
template<size_t N>
using Matrix = std::array<std::array<double, N>, N>;

template<size_t N>
using Vector = std::array<double, N>;

// First control gain of a finite-horizon discrete optimal-control problem.
template<size_t N>
Vector<N> finite_horizon_gain(
  const Matrix<N> & a, const Vector<N> & b, const Vector<N> & q,
  double r, int horizon, double terminal_scale)
{
  Matrix<N> p{};
  for (size_t i = 0; i < N; ++i) p[i][i] = terminal_scale * q[i];
  Vector<N> gain{};
  for (int step = horizon - 1; step >= 0; --step) {
    Vector<N> pb{};
    for (size_t i = 0; i < N; ++i)
      for (size_t j = 0; j < N; ++j) pb[i] += p[i][j] * b[j];
    double denominator = r;
    for (size_t i = 0; i < N; ++i) denominator += b[i] * pb[i];
    for (size_t j = 0; j < N; ++j) {
      gain[j] = 0.0;
      for (size_t i = 0; i < N; ++i) gain[j] += pb[i] * a[i][j];
      gain[j] /= denominator;
    }

    Matrix<N> next{};
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = 0; j < N; ++j) {
        double value = i == j ? q[i] : 0.0;
        for (size_t row = 0; row < N; ++row) {
          for (size_t col = 0; col < N; ++col) {
            value += a[row][i] * p[row][col] * a[col][j];
          }
          value -= a[row][i] * pb[row] * gain[j];
        }
        next[i][j] = value;
      }
    }
    p = next;
  }
  return gain;
}

template<size_t N>
double feedback(const Vector<N> & gain, const Vector<N> & state)
{
  double command = 0.0;
  for (size_t i = 0; i < N; ++i) command -= gain[i] * state[i];
  return command;
}

class MpcController : public rclcpp::Node
{
public:
  MpcController() : Node("controller")
  {
    mass_ = declare_parameter("mass", 1.5);
    gravity_ = declare_parameter("gravity", 9.81);
    ixx_ = declare_parameter("ixx", 0.030);
    iyy_ = declare_parameter("iyy", 0.030);
    izz_ = declare_parameter("izz", 0.055);
    dt_ = declare_parameter("prediction_dt", 0.04);
    horizon_ = declare_parameter("horizon", 35);
    terminal_scale_ = declare_parameter("terminal_scale", 8.0);
    max_thrust_ = declare_parameter("max_thrust", 32.0);
    max_torque_xy_ = declare_parameter("max_torque_xy", 5.0);
    max_torque_z_ = declare_parameter("max_torque_z", 2.0);

    const Vector<4> q_xy{
      declare_parameter("q_xy.position", 2.0),
      declare_parameter("q_xy.velocity", 2.0),
      declare_parameter("q_xy.angle", 20.0),
      declare_parameter("q_xy.rate", 2.0)};
    const Vector<2> q_z{
      declare_parameter("q_z.position", 10.0),
      declare_parameter("q_z.velocity", 4.0)};
    const Vector<2> q_yaw{
      declare_parameter("q_yaw.angle", 5.0),
      declare_parameter("q_yaw.rate", 1.0)};
    const double r_xy = declare_parameter("r_xy", 5.0);
    const double r_z = declare_parameter("r_z", 1.0);
    const double r_yaw = declare_parameter("r_yaw", 2.0);

    Matrix<4> ax{};
    for (size_t i = 0; i < 4; ++i) ax[i][i] = 1.0;
    ax[0][1] = dt_;
    ax[1][2] = gravity_ * dt_;
    ax[2][3] = dt_;
    Vector<4> bx{0.0, 0.0, 0.0, dt_ / iyy_};
    k_x_ = finite_horizon_gain(ax, bx, q_xy, r_xy, horizon_, terminal_scale_);

    Matrix<4> ay = ax;
    ay[1][2] = -gravity_ * dt_;
    Vector<4> by{0.0, 0.0, 0.0, dt_ / ixx_};
    k_y_ = finite_horizon_gain(ay, by, q_xy, r_xy, horizon_, terminal_scale_);

    Matrix<2> az{{{{1.0, dt_}}, {{0.0, 1.0}}}};
    k_z_ = finite_horizon_gain(
      az, Vector<2>{0.0, dt_ / mass_}, q_z, r_z, horizon_, terminal_scale_);
    k_yaw_ = finite_horizon_gain(
      az, Vector<2>{0.0, dt_ / izz_}, q_yaw, r_yaw, horizon_, terminal_scale_);

    command_pub_ = create_publisher<geometry_msgs::msg::Wrench>("command/wrench", 10);
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "command/pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        target_ = *msg;
        have_target_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&MpcController::on_odometry, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "MPC started: horizon=%d, prediction_dt=%.3f s", horizon_, dt_);
  }

private:
  static double wrap(double value)
  {
    return std::atan2(std::sin(value), std::cos(value));
  }
  static double limited(double value, double limit)
  {
    return std::clamp(value, -limit, limit);
  }
  static void to_rpy(
    const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
  {
    roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  static double world_vertical_velocity(
    const geometry_msgs::msg::Quaternion & q, const geometry_msgs::msg::Vector3 & velocity)
  {
    return 2.0 * (q.x * q.z - q.w * q.y) * velocity.x +
      2.0 * (q.y * q.z + q.w * q.x) * velocity.y +
      (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * velocity.z;
  }

  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (!have_target_) {
      command_pub_->publish(geometry_msgs::msg::Wrench());
      return;
    }
    const rclcpp::Time stamp(odom->header.stamp, get_clock()->get_clock_type());
    if (last_control_.nanoseconds() != 0 && (stamp - last_control_).seconds() < 0.8 * dt_) return;
    last_control_ = stamp;

    double roll, pitch, yaw, unused_roll, unused_pitch, yaw_ref;
    to_rpy(odom->pose.pose.orientation, roll, pitch, yaw);
    to_rpy(target_.pose.orientation, unused_roll, unused_pitch, yaw_ref);
    const double cosine = std::cos(yaw), sine = std::sin(yaw);
    const double dx_world = odom->pose.pose.position.x - target_.pose.position.x;
    const double dy_world = odom->pose.pose.position.y - target_.pose.position.y;
    const double dx = cosine * dx_world + sine * dy_world;
    const double dy = -sine * dx_world + cosine * dy_world;
    const double dz = odom->pose.pose.position.z - target_.pose.position.z;
    const double vz = world_vertical_velocity(
      odom->pose.pose.orientation, odom->twist.twist.linear);

    const Vector<4> x_state{dx, odom->twist.twist.linear.x,
      pitch, odom->twist.twist.angular.y};
    const Vector<4> y_state{dy, odom->twist.twist.linear.y,
      roll, odom->twist.twist.angular.x};
    const Vector<2> z_state{dz, vz};
    const Vector<2> yaw_state{wrap(yaw - yaw_ref), odom->twist.twist.angular.z};

    geometry_msgs::msg::Wrench command;
    const double tilt_compensation = std::max(0.5, std::cos(roll) * std::cos(pitch));
    command.force.z = std::clamp(
      (mass_ * gravity_ + feedback(k_z_, z_state)) / tilt_compensation,
      0.0, max_thrust_);
    command.torque.y = limited(feedback(k_x_, x_state), max_torque_xy_);
    command.torque.x = limited(feedback(k_y_, y_state), max_torque_xy_);
    command.torque.z = limited(feedback(k_yaw_, yaw_state), max_torque_z_);
    command_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::PoseStamped target_;
  rclcpp::Time last_control_{0, 0, RCL_ROS_TIME};
  Vector<4> k_x_{}, k_y_{};
  Vector<2> k_z_{}, k_yaw_{};
  double mass_, gravity_, ixx_, iyy_, izz_, dt_, terminal_scale_;
  double max_thrust_, max_torque_xy_, max_torque_z_;
  int horizon_;
  bool have_target_{false};
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadcopter_sim::MpcController>());
  rclcpp::shutdown();
  return 0;
}
