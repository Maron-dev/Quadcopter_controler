#define QUADCOPTER_MPC_MODEL_ONLY
#include "mpc_controller.cpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
using namespace quadcopter_sim;
void check(bool ok, const char * message) {if (!ok) throw std::runtime_error(message);}
int main()
{
  NmpcParameters p;
  NonlinearMpc model(p);
  State s{}; s[6]=1;
  const double hover=std::sqrt(p.mass*p.gravity/(4*p.kf));
  for (int i=13; i<17; ++i) s[i]=hover;
  Control u{}; u.fill(hover/p.max_speed);
  auto next=model.step(s,u);
  check(std::abs(next[5])<1e-12,"Hover equilibrium");
  State freefall{}; freefall[6]=1;
  next=model.step(freefall,{0,0,0,0});
  check(std::abs(next[5]+p.gravity*p.dt)<1e-10,"Gravity acceleration");
  check(std::abs(next[2]+0.5*p.gravity*p.dt*p.dt)<1e-10,"Freefall position");
  auto tilted=s; tilted[6]=std::cos(0.4); tilted[8]=std::sin(0.4);
  const auto derivative=model.derivative(tilted,u);
  check(std::abs(derivative[3]-p.gravity*std::sin(0.8))<1e-10,"Nonlinear thrust rotation");
  tilted=s; tilted[10]=1; tilted[11]=2; tilted[12]=3;
  check(std::abs(model.derivative(tilted,u)[10]-(p.iyy-p.izz)*6/p.ixx)<1e-10,"Gyroscopic coupling");
  next=model.step(freefall,u);
  check(std::abs(next[13]-hover*(1-std::exp(-p.dt/p.motor_tau)))<0.05,"Motor lag");
  for (int i=0; i<100; ++i) tilted=model.step(tilted,u);
  double norm=0; for (int i=6; i<10; ++i) norm+=tilted[i]*tilted[i];
  check(std::abs(norm-1)<1e-10,"Quaternion normalization");
  bool rejected=false;
  try {auto bad=p; bad.mass=0; NonlinearMpc invalid(bad);} catch(const std::invalid_argument &) {rejected=true;}
  check(rejected,"Parameter validation");
  State target{}; target[0]=1; target[1]=-0.5; target[2]=1; target[6]=std::cos(0.25); target[9]=std::sin(0.25);
  double total_ms=0,max_ms=0;
  for (int k=0; k<200; ++k) {
    auto start=std::chrono::steady_clock::now();
    const auto command=model.solve(s,target);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    total_ms+=ms; max_ms=std::max(max_ms,ms);
    check(std::isfinite(model.final_cost) && model.final_cost<=model.initial_cost+1e-8,"Optimization cost reduction");
    for(double c : command) check(c>=0 && c<=1 && std::isfinite(c),"Rotor bounds");
    s=model.step(s,command);
    if (k%50==0) std::cout<<k<<" position "<<s[0]<<" "<<s[1]<<" "<<s[2]<<" cost "<<model.final_cost<<" ms "<<ms<<std::endl;
  }
  const double error=std::sqrt(std::pow(s[0]-target[0],2)+std::pow(s[1]-target[1],2)+std::pow(s[2]-target[2],2));
  std::cout<<"Final position error: "<<error<<" m; mean/max solve: "<<total_ms/200<<"/"<<max_ms<<" ms\n";
  check(error<0.2,"Closed-loop position convergence");
  check(std::abs(2*std::atan2(s[9],s[6])-0.5)<0.1,"Yaw convergence");
  // A separate flight starts tilted, with a half-period actuator-command delay.
  model.reset();
  s.fill(0); s[2]=2; s[6]=std::cos(0.3); s[7]=std::sin(0.3);
  for (int i=13; i<17; ++i) s[i]=hover;
  target.fill(0); target[2]=2; target[6]=1;
  auto previous=u;
  NonlinearMpc plant(p); plant.p.dt=p.dt/2;
  for (int k=0; k<200; ++k) {
    const auto command=model.solve(s,target);
    s=plant.step(s,previous);
    s=plant.step(s,command);
    previous=command;
    check(std::isfinite(s[0]) && s[2]>0.5, "Tilt recovery with command delay");
  }
  check(std::hypot(s[0],s[1])<0.2 && std::abs(s[2]-2)<0.1,
    "Delayed closed-loop convergence");
  std::cout<<"All nonlinear MPC checks passed\n";
}
