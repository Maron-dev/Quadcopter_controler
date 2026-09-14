#include <algorithm>//Dzieki niej mozemy uzyc funkcji std::clamp, ktora ogranicza wartosc do okreslonego zakresu
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp> //Zadana pozycja i orientacja
#include <geometry_msgs/msg/wrench.hpp> //
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

//Przestrzeń nazw zapobiega konfliktom z klasami o takich samych nazwach w innych częsciach programu. W tym przypadku, wszystkie klasy i struktury związane z symulatorem quadcoptera są umieszczone w przestrzeni nazw "quadcopter_sim".
namespace quadcopter_sim
{
struct Gains
{
  double kp;
  double ki;
  double kd;
};

class QuadcopterController : public rclcpp::Node
{
public:
  QuadcopterController() : Node("controller")
  {
    //Definujemy parametry kontrolera, takie jak masa, grawitacja, maksymalny kąt przechylenia, maksymalny ciąg i maksymalny moment obrotowy. Parametry te są deklarowane jako parametry węzła ROS 2, co pozwala na ich łatwe dostosowanie w czasie działania programu.
    mass_ = declare_parameter("mass", 1.5);
    gravity_ = declare_parameter("gravity", 9.81);
    max_tilt_ = declare_parameter("max_tilt", 0.52);
    max_thrust_ = declare_parameter("max_thrust", 32.0);
    max_torque_xy_ = declare_parameter("max_torque_xy", 5.0);
    max_torque_z_ = declare_parameter("max_torque_z", 2.0);

    //Nastawy regulatorów PIDÓw
    xy_ = gains("xy", {1.15, 0.02, 1.35});//Regulacja położenia w osiach x i y
    z_ = gains("z", {5.0, 0.65, 3.1});  //Regulacja położenia w osi z
    attitude_ = gains("attitude", {7.0, 0.03, 2.4});  //regulacja kątów roll i pitch
    yaw_ = gains("yaw", {3.0, 0.02, 0.9});  //regulacja kąta yaw

    
    //Tworzymy publisher do wysyłania komend siły i momentu obrotowego (wrench) oraz subskrybujemy się na temat docelowej pozycji (pose) i odometrii (odom). W callbacku dla docelowej pozycji zapisujemy otrzymaną wartość, a w callbacku dla odometrii wywołujemy funkcję on_odometry, która oblicza i publikuje komendy sterujące.
    wrench_pub_ = create_publisher<geometry_msgs::msg::Wrench>("command/wrench", 10);
    
    //Odebranie pozycji zadanej
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "command/pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        target_ = *msg;
        have_target_ = true;
      });
    
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&QuadcopterController::on_odometry, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Cascaded ROS 2 flight controller started");
  }

private:
  Gains gains(const std::string & prefix, const Gains & value)
  {
    return {
      declare_parameter(prefix + ".kp", value.kp),
      declare_parameter(prefix + ".ki", value.ki),
      declare_parameter(prefix + ".kd", value.kd)};
  }

  static double clamp(double value, double limit)
  {
    return std::clamp(value, -limit, limit);
  }

  //sprowadza kąt do przedziału od -pi do pi
  static double wrap(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  //Konwersja kawterionu na katy Eulera
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

  static void body_velocity_to_world(
    const geometry_msgs::msg::Quaternion & q,
    const geometry_msgs::msg::Vector3 & body,
    double & vx, double & vy, double & vz)
  {
    vx = (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * body.x +
      2.0 * (q.x * q.y - q.w * q.z) * body.y +
      2.0 * (q.x * q.z + q.w * q.y) * body.z;
    vy = 2.0 * (q.x * q.y + q.w * q.z) * body.x +
      (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * body.y +
      2.0 * (q.y * q.z - q.w * q.x) * body.z;
    vz = 2.0 * (q.x * q.z - q.w * q.y) * body.x +
      2.0 * (q.y * q.z + q.w * q.x) * body.y +
      (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * body.z;
  }

  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (!have_target_) {
      publish_zero();
      return;
    }

    const rclcpp::Time stamp(odom->header.stamp, get_clock()->get_clock_type());
    if (last_stamp_.nanoseconds() == 0) {
      last_stamp_ = stamp;
      return;
    }
    const double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (dt <= 0.0 || dt > 0.05) {
      reset_integrators();
      return;
    }

    const double ex = target_.pose.position.x - odom->pose.pose.position.x;
    const double ey = target_.pose.position.y - odom->pose.pose.position.y;
    const double ez = target_.pose.position.z - odom->pose.pose.position.z;
    ix_ = clamp(ix_ + ex * dt, 2.0);
    iy_ = clamp(iy_ + ey * dt, 2.0);
    iz_ = clamp(iz_ + ez * dt, 3.0);

    double vx, vy, vz;
    body_velocity_to_world(
      odom->pose.pose.orientation, odom->twist.twist.linear, vx, vy, vz);
    const double ax = xy_.kp * ex + xy_.ki * ix_ - xy_.kd * vx;
    const double ay = xy_.kp * ey + xy_.ki * iy_ - xy_.kd * vy;
    const double az = z_.kp * ez + z_.ki * iz_ - z_.kd * vz;

    double roll, pitch, yaw;
    double target_roll_unused, target_pitch_unused, yaw_ref;
    to_rpy(odom->pose.pose.orientation, roll, pitch, yaw);
    to_rpy(target_.pose.orientation, target_roll_unused, target_pitch_unused, yaw_ref);

    // Convert desired world acceleration to desired body attitude.
    const double roll_ref = clamp(
      (ax * std::sin(yaw) - ay * std::cos(yaw)) / gravity_, max_tilt_);
    const double pitch_ref = clamp(
      (ax * std::cos(yaw) + ay * std::sin(yaw)) / gravity_, max_tilt_);
    const double eroll = wrap(roll_ref - roll);
    const double epitch = wrap(pitch_ref - pitch);
    const double eyaw = wrap(yaw_ref - yaw);
    iroll_ = clamp(iroll_ + eroll * dt, 0.5);
    ipitch_ = clamp(ipitch_ + epitch * dt, 0.5);
    iyaw_ = clamp(iyaw_ + eyaw * dt, 0.5);

    geometry_msgs::msg::Wrench command;
    const double attitude_compensation = std::max(0.5, std::cos(roll) * std::cos(pitch));
    command.force.z = std::clamp(
      mass_ * (gravity_ + az) / attitude_compensation, 0.0, max_thrust_);
    command.torque.x = clamp(
      attitude_.kp * eroll + attitude_.ki * iroll_ -
      attitude_.kd * odom->twist.twist.angular.x, max_torque_xy_);
    command.torque.y = clamp(
      attitude_.kp * epitch + attitude_.ki * ipitch_ -
      attitude_.kd * odom->twist.twist.angular.y, max_torque_xy_);
    command.torque.z = clamp(
      yaw_.kp * eyaw + yaw_.ki * iyaw_ - yaw_.kd * odom->twist.twist.angular.z,
      max_torque_z_);
    wrench_pub_->publish(command);
  }

  void publish_zero()
  {
    wrench_pub_->publish(geometry_msgs::msg::Wrench());
  }

  void reset_integrators()
  {
    ix_ = iy_ = iz_ = iroll_ = ipitch_ = iyaw_ = 0.0;
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::PoseStamped target_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  Gains xy_, z_, attitude_, yaw_;
  double mass_, gravity_, max_tilt_, max_thrust_, max_torque_xy_, max_torque_z_;
  double ix_{0.0}, iy_{0.0}, iz_{0.0};
  double iroll_{0.0}, ipitch_{0.0}, iyaw_{0.0};
  bool have_target_{false};
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadcopter_sim::QuadcopterController>());
  rclcpp::shutdown();
  return 0;
}
