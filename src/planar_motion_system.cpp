// Fixed-standing planar teaching simulator. Not a biped dynamics controller.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/Utility.hh>
#include "experiment6_navigation/planar_geometry.hpp"

namespace experiment6 {
class PlanarMotion : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate,
                     public ignition::gazebo::ISystemPostUpdate {
  using Clock=std::chrono::steady_clock;
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  ignition::transport::Node node;
  ignition::transport::Node::Publisher odomPub, blockedPub;
  std::mutex mutex;
  double vx{0},vy{0},omega{0},radius{0.45},timeout{0.35},frequency{30};
  double maxLinear{0.35},maxAngular{0.8}, minX{-5.5},maxX{5.5},minY{-3.5},maxY{3.5};
  Clock::time_point lastCommand{};
  std::vector<Box2> obstacles;
  ignition::math::Pose3d previous;
  double previousTime{-1},publishedTime{-1};
  bool collisionBlocked{false}, configured{false};

  void OnCommand(const ignition::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    vx=msg.linear().x(); vy=msg.linear().y(); omega=msg.angular().z();
    if(!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(omega)) vx=vy=omega=0;
    const double norm=std::hypot(vx,vy);
    if(norm>maxLinear) { vx*=maxLinear/norm; vy*=maxLinear/norm; }
    omega=std::clamp(omega,-maxAngular,maxAngular);
    lastCommand=Clock::now();
  }
 public:
  ~PlanarMotion() override {
    node.Unsubscribe("/r1/sim/cmd_vel");
  }
  void Configure(const ignition::gazebo::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &ecm,
      ignition::gazebo::EventManager &) override {
    model=ignition::gazebo::Model(entity);
    if(!model.Valid(ecm)) throw std::runtime_error("PlanarMotion requires a model");
    auto get=[&](const std::string &key,double fallback) {
      return sdf->HasElement(key)?sdf->Get<double>(key):fallback;
    };
    radius=get("footprint_radius",radius); timeout=get("command_timeout",timeout);
    frequency=get("publish_frequency",frequency); maxLinear=get("max_linear",maxLinear);
    maxAngular=get("max_angular",maxAngular);
    minX=get("min_x",minX);maxX=get("max_x",maxX);minY=get("min_y",minY);maxY=get("max_y",maxY);
    if(!(radius>0&&timeout>0&&frequency>0&&maxLinear>0&&maxAngular>0&&minX<maxX&&minY<maxY))
      throw std::runtime_error("Invalid planar motion parameters");
    auto mutableSdf=sdf->Clone();
    if(mutableSdf->HasElement("obstacle")) {
      auto item=mutableSdf->GetElement("obstacle");
      while(item) {
        obstacles.push_back({item->Get<double>("x"),item->Get<double>("y"),
          item->Get<double>("yaw"),item->Get<double>("half_x"),item->Get<double>("half_y")});
        item=item->GetNextElement("obstacle");
      }
    }
    odomPub=node.Advertise<ignition::msgs::Odometry>("/r1/sim/odometry");
    blockedPub=node.Advertise<ignition::msgs::Boolean>("/r1/sim/collision_blocked");
    if(!node.Subscribe("/r1/sim/cmd_vel",&PlanarMotion::OnCommand,this))
      throw std::runtime_error("Cannot subscribe to simulator velocity");
    configured=true;
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if(!configured) return;
    const double dt=std::chrono::duration<double>(info.dt).count();
    if(dt<0) {
      std::lock_guard<std::mutex> lock(mutex);
      vx=vy=omega=0;lastCommand={};previousTime=publishedTime=-1;return;
    }
    if(info.paused||dt<=0) return;
    double x,y,w;
    {
      std::lock_guard<std::mutex> lock(mutex);
      const bool fresh=lastCommand!=Clock::time_point{}&&
        std::chrono::duration<double>(Clock::now()-lastCommand).count()<timeout;
      x=fresh?vx:0;y=fresh?vy:0;w=fresh?omega:0;
    }
    // Do not jump forward after a stalled simulator step.
    if(dt>0.1) x=y=w=0;
    auto actual=ignition::gazebo::worldPose(model.Entity(),ecm);
    Pose2 start{actual.Pos().X(),actual.Pos().Y(),actual.Rot().Yaw()}, p=start;
    collisionBlocked=false;
    const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(x,y)*dt/0.005)));
    for(int i=0;i<steps;++i) {
      const auto next=integrate(p,x,y,w,dt/steps);
      if(next.x-radius<minX||next.x+radius>maxX||next.y-radius<minY||next.y+radius>maxY||
         blocked(p,next,radius,obstacles)) {
        collisionBlocked=true;
        // Rotation can continue inside the conservative circular footprint.
        p.yaw=wrap(p.yaw+w*dt/steps);
      } else p=next;
    }
    if(std::abs(p.x-start.x)+std::abs(p.y-start.y)+std::abs(w*dt)>1e-12)
      model.SetWorldPoseCmd(ecm,{p.x,p.y,actual.Pos().Z(),0,0,p.yaw});
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
      const ignition::gazebo::EntityComponentManager &ecm) override {
    if(!configured||info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const auto actual=ignition::gazebo::worldPose(model.Entity(),ecm);
    const double dt=now-previousTime;
    double bx=0,by=0,w=0;
    if(previousTime>=0&&dt>0&&dt<=0.1) {
      const double dx=(actual.Pos().X()-previous.Pos().X())/dt;
      const double dy=(actual.Pos().Y()-previous.Pos().Y())/dt;
      const double yaw=actual.Rot().Yaw();
      bx=std::cos(yaw)*dx+std::sin(yaw)*dy;
      by=-std::sin(yaw)*dx+std::cos(yaw)*dy;
      w=wrap(yaw-previous.Rot().Yaw())/dt;
    }
    previous=actual;previousTime=now;
    if(publishedTime>=0&&now>=publishedTime&&now-publishedTime<1.0/frequency) return;
    publishedTime=now;
    ignition::msgs::Odometry message;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto frame=message.mutable_header()->add_data();frame->set_key("frame_id");frame->add_value("odom");
    auto child=message.mutable_header()->add_data();child->set_key("child_frame_id");child->add_value("base_link");
    ignition::msgs::Set(message.mutable_pose(),actual);
    message.mutable_twist()->mutable_linear()->set_x(bx);
    message.mutable_twist()->mutable_linear()->set_y(by);
    message.mutable_twist()->mutable_angular()->set_z(w);
    odomPub.Publish(message);
    ignition::msgs::Boolean status;status.set_data(collisionBlocked);blockedPub.Publish(status);
  }
};
}  // namespace experiment6
IGNITION_ADD_PLUGIN(experiment6::PlanarMotion, ignition::gazebo::System,
  experiment6::PlanarMotion::ISystemConfigure, experiment6::PlanarMotion::ISystemPreUpdate,
  experiment6::PlanarMotion::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(experiment6::PlanarMotion,"experiment6::PlanarMotion")
