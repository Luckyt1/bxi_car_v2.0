#pragma once

#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace chassis
{

// 由同一个执行器线程访问：遥控器提出启动请求，主程序完成硬件启动后置 running。
struct RemoteControlSession
{
  enum class Stage {waiting, requested, starting, running};
  bool start_chassis{true};
  Stage stage{Stage::waiting};
};

// 不传 session 时仅发布遥控指令；传入 session 时默认等待启动键并协调硬件启动。
std::shared_ptr<rclcpp::Node> make_remote_control(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::shared_ptr<RemoteControlSession> session = nullptr);

}  // namespace chassis
