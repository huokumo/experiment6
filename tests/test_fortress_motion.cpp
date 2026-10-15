// Headless integration against the real Fortress physics/transport runtime.
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

class Feedback {
 public:
  std::mutex mutex;
  double x{0},y{0},yaw{0},vx{0};int count{0};
  void Receive(const ignition::msgs::Odometry &m) {
    std::lock_guard<std::mutex> lock(mutex);
    x=m.pose().position().x();y=m.pose().position().y();
    const auto &q=m.pose().orientation();
    yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
    vx=m.twist().linear().x();++count;
  }
};
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
int main() {
  ignition::gazebo::ServerConfig config;
  const std::string world=std::string(R"(<sdf version="1.8"><world name="smoke">
    <physics name="p" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>0</real_time_factor></physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <model name="r1"><static>true</static><link name="base_link"><collision name="c"><geometry><box><size>.2 .2 .2</size></box></geometry></collision></link>
    <plugin filename=")")+PLUGIN_PATH+R"(" name="experiment6::PlanarMotion">
      <footprint_radius>.45</footprint_radius>
      <obstacle><x>2.5</x><y>0</y><yaw>0</yaw><half_x>.05</half_x><half_y>2</half_y></obstacle>
    </plugin></model></world></sdf>)";
  config.SetSdfString(world);
  ignition::gazebo::Server server(config);
  Feedback feedback;
  ignition::transport::Node node;
  node.Subscribe("/r1/sim/odometry",&Feedback::Receive,&feedback);
  auto publisher=node.Advertise<ignition::msgs::Twist>("/r1/sim/cmd_vel");
  server.Run(true,10,false);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto tick=[&](double vx,double vy,double w,int batches) {
    for(int i=0;i<batches;++i) {
      ignition::msgs::Twist command;
      command.mutable_linear()->set_x(vx);command.mutable_linear()->set_y(vy);
      command.mutable_angular()->set_z(w);
      publisher.Publish(command);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      server.Run(true,10,false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  };
  tick(.2,0,0,10);
  {
    std::lock_guard<std::mutex> lock(feedback.mutex);
    require(feedback.count>0,"No measured odometry was published");
    require(feedback.x>.12&&feedback.x<.25,"Static standing model did not move as commanded");
  }
  // Stop producing commands. The plugin watchdog must stop movement.
  std::this_thread::sleep_for(std::chrono::milliseconds(450));
  server.Run(true,20,false);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  {
    std::lock_guard<std::mutex> lock(feedback.mutex);
    require(std::abs(feedback.vx)<1e-6,"Command timeout did not stop measured movement");
  }
  tick(.35,0,0,100);
  {
    std::lock_guard<std::mutex> lock(feedback.mutex);
    require(feedback.x>1.95&&feedback.x<=2.00001,"Wall collision guard did not stop the footprint");
  }
  tick(0,0,.5,10);
  {
    std::lock_guard<std::mutex> lock(feedback.mutex);
    require(feedback.yaw>.4&&feedback.yaw<.6,"Positive yaw command did not turn left");
  }
  tick(-.2,0,0,10);
  {
    std::lock_guard<std::mutex> lock(feedback.mutex);
    require(feedback.x<1.9,"Robot could not reverse away from the wall");
  }
  std::cout<<"Fortress: actual pose feedback, translation, yaw, collision blocking, reverse and watchdog passed\n";
}
