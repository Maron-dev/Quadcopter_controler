#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace quadcopter_sim
{

using State = std::array<double, 17>;
using Control = std::array<double, 4>; 

struct NmpcParameters
{
  double mass=1.5, gravity=9.81, ixx=0.030, iyy=0.030, izz=0.055;
  double kf=8.5e-6, km=1.6e-7, arm_x=0.23, arm_y=0.23;
  double motor_tau=0.06, max_speed=1100.0, dt=0.04, terminal_scale=8.0;
  int horizon=35, iterations=12;
  double gradient_tolerance=1e-3;
  std::array<double, 3> position{2,2,10}, velocity{2,2,4};
  std::array<double, 3> attitude{20,20,5}, rate{2,2,1};
  std::array<double, 4> effort{1,5,5,2};

  void validate() const
  {
    for (double v : {mass, gravity, ixx, iyy, izz, kf, km, arm_x, arm_y,
        motor_tau, max_speed, dt, terminal_scale, gradient_tolerance}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid NMPC parameter");
    }
    if (horizon < 1 || horizon > 100 || iterations < 1 || iterations > 100 || dt > 0.1)
      throw std::invalid_argument("Invalid NMPC horizon, iterations or prediction_dt");
    for (const auto & weights : {position, velocity, attitude, rate})
      for (double v : weights)
        if (!std::isfinite(v) || v < 0) throw std::invalid_argument("Invalid NMPC weight");
    for (double v : effort)
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid NMPC effort weight");
  }
};

inline bool normalize(State & s)
{
  for (double v : s) if (!std::isfinite(v)) return false;
  double n=0;
  for (int i=6; i<10; ++i) n += s[i]*s[i];
  if (n < 1e-12) return false;
  n=std::sqrt(n);
  for (int i=6; i<10; ++i) s[i] /= n;
  return true;
}

inline std::array<double, 3> rotate(const State & s, const std::array<double, 3> & v)
{
  const double w=s[6], x=s[7], y=s[8], z=s[9];
  return {(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2],
    2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2],
    2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2]};
}

class NonlinearMpc
{
public:
  explicit NonlinearMpc(NmpcParameters parameters) : p(parameters)
  {
    p.validate();
    reset();
  }
  NmpcParameters p;
  double initial_cost=0, final_cost=0;
  int accepted_iterations=0;

  void reset()
  {
    const double hover=std::clamp(std::sqrt(p.mass*p.gravity/(4*p.kf))/p.max_speed, 0.0, 1.0);
    sequence.assign(4*p.horizon, hover);
  }

  // [total thrust, roll torque, pitch torque, yaw torque], matching Gazebo.
  std::array<double, 4> wrench(const Control & speed) const
  {
    Control f{};
    for (int i=0; i<4; ++i) f[i]=p.kf*speed[i]*speed[i];
    return {f[0]+f[1]+f[2]+f[3], p.arm_y*(f[0]-f[1]+f[2]-f[3]),
      p.arm_x*(-f[0]-f[1]+f[2]+f[3]), p.km/p.kf*(f[0]-f[1]-f[2]+f[3])};
  }

  State derivative(const State & s, const Control & u) const
  {
    State d{};
    const auto force=wrench({s[13],s[14],s[15],s[16]});
    const auto thrust=rotate(s, {0,0,force[0]});
    for (int i=0; i<3; ++i) {d[i]=s[3+i]; d[3+i]=thrust[i]/p.mass;}
    d[5]-=p.gravity;
    const double w=s[6], x=s[7], y=s[8], z=s[9], a=s[10], b=s[11], c=s[12];
    d[6]=-0.5*(x*a+y*b+z*c);
    d[7]=0.5*(w*a+y*c-z*b);
    d[8]=0.5*(w*b+z*a-x*c);
    d[9]=0.5*(w*c+x*b-y*a);
    d[10]=(force[1]+(p.iyy-p.izz)*b*c)/p.ixx;
    d[11]=(force[2]+(p.izz-p.ixx)*a*c)/p.iyy;
    d[12]=(force[3]+(p.ixx-p.iyy)*a*b)/p.izz;
    for (int i=0; i<4; ++i) d[13+i]=(u[i]*p.max_speed-s[13+i])/p.motor_tau;
    return d;
  }

  State step(State s, const Control & u) const
  {
    // RK4, subdivided to resolve the motor lag as well as rigid-body motion.
    const int count=std::max(2, static_cast<int>(std::ceil(p.dt/std::min(0.02,p.motor_tau/2))));
    const double h=p.dt/count;
    for (int n=0; n<count; ++n) {
      const auto a=derivative(s,u);
      State tmp{};
      for (int j=0; j<17; ++j) tmp[j]=s[j]+h*0.5*a[j];
      const auto b=derivative(tmp,u);
      for (int j=0; j<17; ++j) tmp[j]=s[j]+h*0.5*b[j];
      const auto c=derivative(tmp,u);
      for (int j=0; j<17; ++j) tmp[j]=s[j]+h*c[j];
      const auto d=derivative(tmp,u);
      for (int j=0; j<17; ++j) s[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;
      if (!normalize(s)) {s.fill(std::numeric_limits<double>::quiet_NaN()); return s;}
    }
    return s;
  }

  Control solve(const State & state, const State & target)
  {
    std::rotate(sequence.begin(),sequence.begin()+4,sequence.end());
    if (p.horizon>1)
      std::copy(sequence.end()-8,sequence.end()-4,sequence.end()-4);
    initial_cost=cost(state,target,sequence);
    final_cost=initial_cost;
    accepted_iterations=0;
    auto g=gradient(state,target,sequence);
    std::vector<std::vector<double>> sh, yh;
    std::vector<double> rho;
    for (int iteration=0; iteration<p.iterations; ++iteration) {
      auto direction=g;
      std::vector<double> alpha(sh.size());
      for (int i=static_cast<int>(sh.size())-1; i>=0; --i) {
        alpha[i]=rho[i]*dot(sh[i],direction);
        for (size_t j=0; j<g.size(); ++j) direction[j]-=alpha[i]*yh[i][j];
      }
      const double scale=sh.empty() ? 1e-3 : dot(sh.back(),yh.back())/dot(yh.back(),yh.back());
      for (double & v : direction) v*=scale;
      for (size_t i=0; i<sh.size(); ++i) {
        const double beta=rho[i]*dot(yh[i],direction);
        for (size_t j=0; j<g.size(); ++j) direction[j]+=sh[i][j]*(alpha[i]-beta);
      }
      double projected_norm=0;
      for (size_t j=0; j<g.size(); ++j) {
        direction[j]=-direction[j];
        const double pg=sequence[j]-std::clamp(sequence[j]-g[j],0.0,1.0);
        projected_norm=std::max(projected_norm,std::abs(pg));
        if ((sequence[j]<=0 && direction[j]<0) || (sequence[j]>=1 && direction[j]>0))
          direction[j]=0;
      }
      if (projected_norm<p.gradient_tolerance) break;
      if (dot(g,direction)>=0) {
        for (size_t j=0; j<g.size(); ++j) direction[j]=-1e-3*g[j];
      }
      auto trial=sequence;
      bool accepted=false;
      double value=final_cost;
      for (double length=1; length>=1.0/4096; length*=0.5) {
        double slope=0;
        for (size_t j=0; j<g.size(); ++j) {
          trial[j]=std::clamp(sequence[j]+length*direction[j],0.0,1.0);
          slope+=g[j]*(trial[j]-sequence[j]);
        }
        value=cost(state,target,trial);
        if (std::isfinite(value) && slope<0 && value<=final_cost+1e-4*slope) {
          accepted=true; break;
        }
      }
      if (!accepted) break;
      auto next_g=gradient(state,target,trial);
      std::vector<double> ds(g.size()), dy(g.size());
      for (size_t j=0; j<g.size(); ++j) {ds[j]=trial[j]-sequence[j]; dy[j]=next_g[j]-g[j];}
      const double curvature=dot(ds,dy);
      if (curvature>1e-10) {
        if (sh.size()==6) {sh.erase(sh.begin()); yh.erase(yh.begin()); rho.erase(rho.begin());}
        sh.push_back(ds); yh.push_back(dy); rho.push_back(1/curvature);
      }
      sequence=trial; g=next_g; final_cost=value; ++accepted_iterations;
    }
    return {sequence[0],sequence[1],sequence[2],sequence[3]};
  }

private:
  std::vector<double> sequence;
  static double dot(const std::vector<double> & a, const std::vector<double> & b)
  {
    double v=0; for (size_t i=0; i<a.size(); ++i) v+=a[i]*b[i]; return v;
  }
  double state_cost(const State & s, const State & ref) const
  {
    double value=0;
    // Vector part of conjugate(q_ref)*q; squared cost is invariant to q -> -q.
    const std::array<double,3> error{
      2*(ref[6]*s[7]-ref[7]*s[6]-ref[8]*s[9]+ref[9]*s[8]),
      2*(ref[6]*s[8]+ref[7]*s[9]-ref[8]*s[6]-ref[9]*s[7]),
      2*(ref[6]*s[9]-ref[7]*s[8]+ref[8]*s[7]-ref[9]*s[6])};
    for (int i=0; i<3; ++i) {
      value+=p.position[i]*std::pow(s[i]-ref[i],2)+p.velocity[i]*s[3+i]*s[3+i]+
        p.attitude[i]*error[i]*error[i]+p.rate[i]*s[10+i]*s[10+i];
    }
    return value;
  }
  double cost(State s, const State & ref, const std::vector<double> & controls) const
  {
    double value=0;
    for (int k=0; k<p.horizon; ++k) {
      Control u{}, speed{};
      for (int i=0; i<4; ++i) {u[i]=controls[4*k+i]; speed[i]=u[i]*p.max_speed;}
      auto force=wrench(speed); force[0]-=p.mass*p.gravity;
      value+=state_cost(s,ref);
      for (int i=0; i<4; ++i) value+=p.effort[i]*force[i]*force[i];
      s=step(s,u);
      if (!std::isfinite(s[0])) return std::numeric_limits<double>::infinity();
    }
    return value+p.terminal_scale*state_cost(s,ref);
  }
  std::vector<double> gradient(const State & initial, const State & ref,
    const std::vector<double> & controls) const
  {
    std::vector<State> states(p.horizon+1);
    states[0]=initial;
    for (int k=0; k<p.horizon; ++k) {
      Control u{};
      for (int i=0; i<4; ++i) u[i]=controls[4*k+i];
      states[k+1]=step(states[k],u);
    }
    auto state_gradient=[&](State s) {
      State result{};
      for (int j=0; j<17; ++j) {
        const double saved=s[j], epsilon=1e-5*std::max(1.0,std::abs(saved));
        s[j]=saved+epsilon; const double plus=state_cost(s,ref);
        s[j]=saved-epsilon; const double minus=state_cost(s,ref);
        result[j]=(plus-minus)/(2*epsilon); s[j]=saved;
      }
      return result;
    };
    auto effort_cost=[&](Control u) {
      for (double & v : u) v*=p.max_speed;
      auto f=wrench(u); f[0]-=p.mass*p.gravity;
      double value=0;
      for (int j=0; j<4; ++j) value+=p.effort[j]*f[j]*f[j];
      return value;
    };
    State lambda=state_gradient(states.back());
    for (double & v : lambda) v*=p.terminal_scale;
    std::vector<double> g(controls.size());
    for (int k=p.horizon-1; k>=0; --k) {
      Control u{};
      for (int i=0; i<4; ++i) u[i]=controls[4*k+i];
      constexpr double epsilon=1e-5;
      for (int j=0; j<4; ++j) {
        auto plus_u=u, minus_u=u; plus_u[j]+=epsilon; minus_u[j]-=epsilon;
        const auto plus=step(states[k],plus_u), minus=step(states[k],minus_u);
        double value=effort_cost(plus_u)-effort_cost(minus_u);
        for (int i=0; i<17; ++i) value+=lambda[i]*(plus[i]-minus[i]);
        g[4*k+j]=value/(2*epsilon);
      }
      auto previous=state_gradient(states[k]);
      for (int j=0; j<17; ++j) {
        auto plus_s=states[k], minus_s=states[k];
        const double delta=epsilon*std::max(1.0,std::abs(states[k][j]));
        plus_s[j]+=delta; minus_s[j]-=delta;
        const auto plus=step(plus_s,u), minus=step(minus_s,u);
        for (int i=0; i<17; ++i) previous[j]+=lambda[i]*(plus[i]-minus[i])/(2*delta);
      }
      lambda=previous;
    }
    return g;
  }
};
} 
#ifndef QUADCOPTER_MPC_MODEL_ONLY
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace quadcopter_sim
{
class MpcController : public rclcpp::Node
{
public:
  MpcController() : Node("controller")
  {
    NmpcParameters p;
    p.mass=declare_parameter("mass",p.mass);
    p.gravity=declare_parameter("gravity",p.gravity);
    p.ixx=declare_parameter("ixx",p.ixx);
    p.iyy=declare_parameter("iyy",p.iyy);
    p.izz=declare_parameter("izz",p.izz);
    p.kf=declare_parameter("thrust_coefficient",p.kf);
    p.km=declare_parameter("torque_coefficient",p.km);
    p.arm_x=declare_parameter("arm_x",p.arm_x);
    p.arm_y=declare_parameter("arm_y",p.arm_y);
    p.motor_tau=declare_parameter("motor_time_constant",p.motor_tau);
    p.max_speed=declare_parameter("max_rotor_speed",p.max_speed);
    p.dt=declare_parameter("prediction_dt",p.dt);
    p.horizon=declare_parameter("horizon",p.horizon);
    p.iterations=declare_parameter("solver_iterations",p.iterations);
    p.gradient_tolerance=declare_parameter("gradient_tolerance",p.gradient_tolerance);
    p.terminal_scale=declare_parameter("terminal_scale",p.terminal_scale);
    p.position[0]=p.position[1]=declare_parameter("q_xy.position",2.0);
    p.velocity[0]=p.velocity[1]=declare_parameter("q_xy.velocity",2.0);
    p.attitude[0]=p.attitude[1]=declare_parameter("q_xy.angle",20.0);
    p.rate[0]=p.rate[1]=declare_parameter("q_xy.rate",2.0);
    p.position[2]=declare_parameter("q_z.position",10.0);
    p.velocity[2]=declare_parameter("q_z.velocity",4.0);
    p.attitude[2]=declare_parameter("q_yaw.angle",5.0);
    p.rate[2]=declare_parameter("q_yaw.rate",1.0);
    p.effort[0]=declare_parameter("r_z",1.0);
    p.effort[1]=p.effort[2]=declare_parameter("r_xy",5.0);
    p.effort[3]=declare_parameter("r_yaw",2.0);
    solver_=std::make_unique<NonlinearMpc>(p);
    command_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("command/rotor_speed",1);
    target_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("command/pose",1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        State target{};
        target[0]=msg->pose.position.x; target[1]=msg->pose.position.y; target[2]=msg->pose.position.z;
        target[6]=msg->pose.orientation.w; target[7]=msg->pose.orientation.x;
        target[8]=msg->pose.orientation.y; target[9]=msg->pose.orientation.z;
        if (normalize(target)) {target_=target; have_target_=true;}
      });
    rotor_sub_=create_subscription<std_msgs::msg::Float64MultiArray>("rotor_speed",1,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size()!=4) return;
        for (double speed : msg->data)
          if (!std::isfinite(speed) || speed<0 || speed>solver_->p.max_speed*1.001) return;
        std::copy(msg->data.begin(),msg->data.end(),rotors_.begin());
        have_rotors_=true;
      });
    auto qos=rclcpp::SensorDataQoS(); qos.keep_last(1);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odom",qos,
      std::bind(&MpcController::on_odometry,this,std::placeholders::_1));
    RCLCPP_INFO(get_logger(),"Nonlinear MPC: 17 states, horizon=%d, dt=%.3f",p.horizon,p.dt);
  }
private:
  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!have_target_ || !have_rotors_) {publish({0,0,0,0}); return;}
    const double stamp=rclcpp::Time(msg->header.stamp).seconds();
    const double elapsed=stamp-last_control_;
    if (last_control_>=0 && elapsed>=0 && elapsed<solver_->p.dt-1e-6) return;
    if (elapsed<0 || elapsed>2*solver_->p.dt) solver_->reset();
    last_control_=stamp;
    State s{};
    s[0]=msg->pose.pose.position.x; s[1]=msg->pose.pose.position.y; s[2]=msg->pose.pose.position.z;
    s[6]=msg->pose.pose.orientation.w; s[7]=msg->pose.pose.orientation.x;
    s[8]=msg->pose.pose.orientation.y; s[9]=msg->pose.pose.orientation.z;
    s[10]=msg->twist.twist.angular.x; s[11]=msg->twist.twist.angular.y; s[12]=msg->twist.twist.angular.z;
    std::copy(rotors_.begin(),rotors_.end(),s.begin()+13);
    if (!normalize(s)) {publish({0,0,0,0}); solver_->reset(); return;}
    const auto v=rotate(s,{msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z});
    std::copy(v.begin(),v.end(),s.begin()+3);
    if (!normalize(s)) {publish({0,0,0,0}); solver_->reset(); return;}
    const auto start=std::chrono::steady_clock::now();
    const auto command=solver_->solve(s,target_);
    if (!std::isfinite(solver_->final_cost)) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"NMPC prediction failed; stopping motors");
      publish({0,0,0,0}); solver_->reset(); return;
    }
    publish(command);
    const double duration=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    if (duration>solver_->p.dt)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"NMPC solve %.1f ms exceeds %.1f ms period",
        duration*1000,solver_->p.dt*1000);
  }
  void publish(const Control & u)
  {
    std_msgs::msg::Float64MultiArray msg;
    for (double v : u) msg.data.push_back(std::clamp(v,0.0,1.0)*solver_->p.max_speed);
    command_pub_->publish(msg);
  }
  std::unique_ptr<NonlinearMpc> solver_;
  State target_{};
  Control rotors_{};
  bool have_target_=false, have_rotors_=false;
  double last_control_=-1;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotor_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<quadcopter_sim::MpcController>());
  rclcpp::shutdown();
  return 0;
}
#endif  // QUADCOPTER_MPC_MODEL_ONLY
