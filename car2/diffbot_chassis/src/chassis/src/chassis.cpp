#include "chassis/chassis_controller.hpp"
#include "chassis/motor/bxi_can3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS 2 底盘节点：负责参数、速度消息和定时调度，硬件控制由 ChassisController 完成。
// public 继承 Node 的接口；final 表示不允许再派生子类。
class ChassisNode final : public rclcpp::Node
{
public:
  ChassisNode()
  : Node("chassis")  // 初始化基类，并将 ROS 节点命名为 chassis。
  {
    // 声明配置路径参数，可通过 --ros-args -p steering_config_file:=... 传入。
    // 默认空路径不允许启动，避免使用不明确的机械参数控制电机。
    const auto path = declare_parameter<std::string>("steering_config_file", "");
    if (path.empty()) {
      throw std::invalid_argument("set steering_config_file to a chassis YAML configuration file");
    }
    // 校准后的自动前进速度，单位为行进轮输出轴 RPM；0 表示停车等待外部指令。
    const double forward_rpm = declare_parameter<double>("post_calibration_rpm", 0.0);
    // 读取机械参数、CAN 映射和控制策略；启动调参时可能向此 YAML 回填电机参数。
    auto config = chassis::load_steering_config(path);
    // 拒绝 NaN、无穷值、负值及超过配置上限的启动速度。
    if (!std::isfinite(forward_rpm) || forward_rpm < 0.0 ||
      forward_rpm > config.drive_max_output_rpm)
    {
      throw std::invalid_argument("post_calibration_rpm must be in [0, drive_max_output_rpm]");
    }
    rcl_interfaces::msg::ParameterDescriptor limit_descriptor;
    limit_descriptor.read_only = true;
    limit_descriptor.description = "Absolute CAN3 speed that cuts shared motor power (RPM)";
    declare_parameter<double>("can3_speed_limit_rpm", 60.0, limit_descriptor, true);
    for (unsigned id = 1; id <= 3; ++id) {
      const auto name = "can3_motor_" + std::to_string(id) + "_angle_deg";
      if (declare_parameter<double>(name, 0.0) != 0.0) {
        throw std::invalid_argument("CAN3 startup angle must be zero: " + name);
      }
    }
    // 节点独占控制器；节点销毁时自动析构控制器，执行停机、失能和断电清理。
    controller_ = std::make_unique<chassis::ChassisController>(config);
    // 初始化和校准中的耗时循环通过此回调检查 ROS 是否仍在运行，支持取消启动。
    const auto keep_running = [this] {
        if (!rclcpp::ok()) {return false;}
        // 校准会同步阻塞，借每次取消检查刷新 CAN3 的零点保持指令。
        if (bxi_can3_) {
          bxi_can3_->hold_zero();
          log_can3_feedback();
        }
        return true;
      };
    // 打开 CAN0/1/2、建立电机对象并上电；校准完成前禁止行进轮运动。
    controller_->initialize(keep_running);
    // CAN3 独立控制三个 BXI8515-19，ID 1/2/3；共享电源已由底盘开启。
    bxi_can3_ = std::make_unique<chassis::BxiCan3>();
    bxi_can3_->initialize();
    // 每次启动各保存一次当前位置；全部发送成功后才使能并保持零位。
    bxi_can3_->save_zero_positions();
    if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
    bxi_can3_->hold_zero();
    RCLCPP_INFO(
      get_logger(),
      "CAN3: save-zero commands sent; enable and zero-hold commands sent to IDs 1, 2, 3 "
      "(position=0, velocity=0, Kp=200, Kd=4, torque=0; no device acknowledgement)");
    RCLCPP_INFO(
      get_logger(), "CAN3 protection: abs(speed) >= 60 RPM cuts shared motor power; "
      "fault stays latched until restart; missing feedback >= 1 s also stops control");
    log_can3_feedback();
    RCLCPP_INFO(
      get_logger(),
      "CAN0: calibrating all four steering axes concurrently; travel axes inhibited");
    // 四个转向轴并行搜索机械限位并独立回中；确认全部到位后才使能行进轮。
    // 此调用同步执行，完成前尚未创建速度订阅和控制定时器。
    controller_->calibrate(keep_running);
    if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
    // 正值进入持续前进状态，后续速度消息可替换该指令；默认 0 不自动前进。
    if (forward_rpm > 0.0) {controller_->forward(forward_rpm);}
    if (forward_rpm > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; CAN1/2 ISWV travel forward %.3f output RPM; waiting for /cmd_vel_car",
        forward_rpm);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; travel wheels stopped; waiting for /cmd_vel_car");
    }
    // 输出运动模式和转向未对齐时的行进策略，便于核对当前配置。
    RCLCPP_INFO(
      get_logger(),
      "steering control: swerve; alignment policy=%s",
      config.drive_alignment_policy.c_str());
    // 解析逐轮参数；未配置 modules 时由全局机械参数生成。
    // 轮序 FL/RL/RR/FR 对应前左、后左、后右、前右。
    const auto modules = chassis::resolved_modules(config);
    RCLCPP_INFO(
      get_logger(), "steering target rate limits [FL,RL,RR,FR]: [%.3f, %.3f, %.3f, %.3f] rad/s",
      modules[0].steering_rate_limit_rad_s, modules[1].steering_rate_limit_rad_s,
      modules[2].steering_rate_limit_rad_s, modules[3].steering_rate_limit_rad_s);
    // 遥控器和可选第二终端通过参数服务修改目标，不另开 PCI/CAN 设备。
    angle_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          for (unsigned id = 1; id <= 3; ++id) {
            const auto name = "can3_motor_" + std::to_string(id) + "_angle_deg";
            if (parameter.get_name() != name) {continue;}
            try {
              if (parameters.size() != 1) {
                throw std::invalid_argument("change exactly one CAN3 motor per request");
              }
              const double angle = parameter.as_double();
              if (!std::isfinite(angle) ||
              std::abs(angle - bxi_can3_->target_degrees(id)) > 1.000001)
              {
                throw std::invalid_argument(
                  "CAN3 target changes are limited to 1 degree per request");
              }
              bxi_can3_->set_target_degrees(id, angle);
              RCLCPP_INFO(get_logger(), "CAN3_TARGET motor_id=%u target_deg=%.3f", id, angle);
            } catch (const std::exception & error) {
              result.successful = false;
              result.reason = error.what();
            }
          }
        }
        return result;
      });
    RCLCPP_INFO(
      get_logger(), "CAN3 angle control ready: button3 selects previous motor, button1 selects next; "
      "axis7 changes target by 1 degree per deflection (return to center between steps)");
    // 接收车体速度指令，消息历史深度为 10；[this] 让回调访问当前节点成员。
    // linear.x/y 为前后/横向速度（m/s），angular.z 为偏航角速度（rad/s）。
    // 控制器保持最后一条指令；停止发布不会自动停车，零速度指令用于停车。
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_car", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        try {
          controller_->set_velocity(message->linear.x, message->linear.y, message->angular.z);
        } catch (const std::exception & error) {on_fault(error);}
      });
    // 每 20 ms 调度一次（目标 50 Hz），执行运动解算、指令输出和电机反馈检查。
    // 实际周期受执行器调度和回调耗时影响，不是硬实时保证。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this] {
        try {
          bxi_can3_->hold_zero();
          controller_->update();
          log_can3_feedback();
          // 消费最近生成的一秒零速反馈报告；无新报告时为空，不额外读取 CAN。
          const auto report = controller_->take_stop_diagnostic();
          if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
        } catch (const std::exception & error) {on_fault(error);}
      });
  }

  bool failed() const noexcept {return failed_;}

private:
  void log_can3_feedback()
  {
    const auto report = bxi_can3_->take_feedback_diagnostic();
    if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
  }

  void on_fault(const std::exception & error)
  {
    // 回调异常统一在这里记录并结束 ROS 运行；硬件清理由控制器故障路径和析构负责。
    RCLCPP_ERROR(get_logger(), "chassis stopped: %s", error.what());
    failed_ = true;
    if (bxi_can3_ && !bxi_can3_->fault_reason().empty()) {
      RCLCPP_ERROR(get_logger(), "%s", bxi_can3_->fault_reason().c_str());
    }
    if (timer_) {timer_->cancel();}
    try {
      if (bxi_can3_) {bxi_can3_->stop();}
    } catch (const std::exception & stop_error) {
      RCLCPP_ERROR(get_logger(), "CAN3 stop failed: %s", stop_error.what());
    }
    rclcpp::shutdown();
  }

  // 保存对象所有权，使控制器、订阅和定时器在节点运行期间持续有效。
  std::unique_ptr<chassis::ChassisController> controller_;
  // 声明在 controller_ 后，正常退出时先让 BXI 电机退出模式，再由底盘断电。
  std::unique_ptr<chassis::BxiCan3> bxi_can3_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr angle_callback_;
  bool failed_{false};
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 上下文并处理命令行中的 ROS 参数。
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // 先构造节点并完成初始化、校准，再由默认单线程执行器处理消息和定时器。
    // 两类回调串行调用控制器，避免并发访问；spin 阻塞到 ROS 关闭或异常退出。
    auto node = std::make_shared<ChassisNode>();
    rclcpp::spin(node);
    if (node->failed()) {result = 1;}
  } catch (const std::exception & error) {
    // 构造或 spin 中未被内部捕获的异常返回失败码；on_fault 已捕获的异常不经过此处。
    RCLCPP_FATAL(rclcpp::get_logger("chassis"), "%s", error.what());
    result = 1;
  }
  // 释放 ROS 上下文；若回调中已关闭，这里再次调用不会重新启动任何操作。
  rclcpp::shutdown();
  return result;
}
