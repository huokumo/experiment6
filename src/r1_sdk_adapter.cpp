// Optional real-hardware backend. The SDK calls are NOT a Gazebo gait controller.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/r1/loco/r1_loco_client.hpp>

class R1SdkAdapter : public rclcpp::Node {
  using Clock=std::chrono::steady_clock;
  std::unique_ptr<unitree::robot::r1::LocoClient> client;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
  rclcpp::TimerBase::SharedPtr timer;
  Clock::time_point received{};
  double vx{0},vy{0},omega{0};
  double maxLinear,maxAngular,timeout;
 public:
  R1SdkAdapter() : Node("r1_sdk_adapter") {
    const bool enabled=declare_parameter("enable_hardware",false);
    const auto interface=declare_parameter<std::string>("network_interface","");
    if(!enabled||interface.empty())
      throw std::runtime_error("Hardware backend requires enable_hardware:=true and an explicit network_interface");
    maxLinear=declare_parameter("max_linear",0.2);
    maxAngular=declare_parameter("max_angular",0.5);
    timeout=declare_parameter("command_timeout",0.3);
    if(!std::isfinite(maxLinear)||!std::isfinite(maxAngular)||!std::isfinite(timeout)||
       maxLinear<=0||maxAngular<=0||timeout<=0)
      throw std::runtime_error("Invalid hardware adapter limits");
    unitree::robot::ChannelFactory::Instance()->Init(0,interface);
    client=std::make_unique<unitree::robot::r1::LocoClient>();
    client->SetTimeout(0.15f);
    client->Init();
    int state=-1;
    const auto result=client->GetFsmId(state);
    if(result!=0) throw std::runtime_error("R1 motion service handshake failed: "+std::to_string(result));
    RCLCPP_INFO(get_logger(),"Connected to R1, FSM=%d. No automatic StandUp/Start is sent.",state);
    subscription=create_subscription<geometry_msgs::msg::Twist>("/r1/cmd_vel",1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const bool valid=std::isfinite(msg->linear.x)&&std::isfinite(msg->linear.y)&&
          std::isfinite(msg->angular.z)&&msg->linear.z==0&&msg->angular.x==0&&msg->angular.y==0;
        vx=valid?msg->linear.x:0;vy=valid?msg->linear.y:0;omega=valid?msg->angular.z:0;
        const auto speed=std::hypot(vx,vy);
        if(speed>maxLinear) { vx*=maxLinear/speed;vy*=maxLinear/speed; }
        omega=std::clamp(omega,-maxAngular,maxAngular);
        received=Clock::now();
      });
    timer=create_wall_timer(std::chrono::milliseconds(50),[this] {
      const bool fresh=received!=Clock::time_point{}&&
        std::chrono::duration<double>(Clock::now()-received).count()<timeout;
      // A finite duration also stops execution if this process disappears.
      const auto result=client->SetVelocity(fresh?vx:0,fresh?vy:0,fresh?omega:0,0.2f);
      if(result!=0) {
        vx=vy=omega=0;received={};
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"R1 SetVelocity failed: %d",result);
      }
    });
  }
  void Stop() { timer->cancel();client->StopMove(); }
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<R1SdkAdapter>();
    rclcpp::spin(node);
    node->Stop();
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<std::endl;
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
