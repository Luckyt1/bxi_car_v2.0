#pragma once

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace chassis
{

class JoystickLinkMonitor;

// 由同一个执行器线程访问：遥控器提出启动请求，主程序完成硬件启动后置 running。
struct RemoteControlSession
{
  enum class Stage {waiting, requested, starting, running};
  bool start_chassis{true};
  Stage stage{Stage::waiting};
};

// 不传 session 时仅发布遥控指令；传入 session 时默认等待启动键并协调硬件启动。
// enable_arm_controls=false 时不创建机械臂位姿订阅/参数客户端，也不处理其手柄操作。
// 配置文件只提供遥控速度/加减速度默认值；显式 ROS 参数仍优先。
std::shared_ptr<rclcpp::Node> make_remote_control(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::shared_ptr<RemoteControlSession> session = nullptr,
  bool enable_arm_controls = true,
  const std::string & default_steering_config_file = {},
  std::shared_ptr<JoystickLinkMonitor> link_monitor = nullptr);

}  // namespace chassis
