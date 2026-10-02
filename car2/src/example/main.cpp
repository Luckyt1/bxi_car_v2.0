#include "chassis/control.h"
#include "arm/control.h"
#include "example/arm_parameters.h"
#include "example/arm_current_remote.h"
#include "example/velocity_command_watchdog.hpp"
#include "bxi/board.h"
#include "example/remote_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <set>
#include <vector>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

// ROS 2 底盘节点：负责参数、速度消息和定时调度，硬件控制由 ChassisController 完成。
namespace
{
std::filesystem::path default_config_directory()
{
  // 开发时直接读取源码配置；独立部署时使用可执行程序所在安装目录的配置。
  const std::filesystem::path source(CHASSIS_SOURCE_CONFIG_DIR);
  if (std::filesystem::is_directory(source)) {return source;}
  const auto executable = std::filesystem::canonical("/proc/self/exe");
  return executable.parent_path().parent_path().parent_path() / "share/chassis/config";
}

class ChassisNode final : public rclcpp::Node
{
public:
  explicit ChassisNode(
    bool enable_arm, const rclcpp::NodeOptions & options,
    std::function<void()> poll_remote = {})
  : Node("chassis", options)
  {
    const auto startup = read_startup_config(enable_arm);
    initialize_hardware(startup, enable_arm, poll_remote);
    setup_ros_interfaces();
  }

  bool failed() const noexcept {return failed_;}

private:
  // 只在启动阶段传递，先完成全部参数校验，再创建硬件对象。
  struct StartupConfig
  {
    chassis::SteeringConfigFile chassis_config;
    double forward_rpm;
    chassis::ethercat::Options ethercat_options;
    chassis::ArmMotionConfig arm_motion;
    std::array<std::uint16_t, 3> ethercat_positions;
  };

  StartupConfig read_startup_config(bool enable_arm)
  {
    rcl_interfaces::msg::ParameterDescriptor timeout_descriptor;
    timeout_descriptor.read_only = true;
    timeout_descriptor.description = "Stop chassis after this many seconds without cmd_vel_car";
    command_timeout_ = declare_parameter<double>("command_timeout_sec", 0.3, timeout_descriptor);
    command_watchdog_ = std::make_unique<chassis::VelocityCommandWatchdog>(
      std::chrono::duration<double>(command_timeout_));
    // 默认配置不依赖工作目录，仍可通过 ROS 参数指定其他文件。
    const auto path = declare_parameter<std::string>(
      "steering_config_file", (default_config_directory() / "steering.yaml").string());
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
    chassis::ethercat::Options ethercat_options;
    chassis::ArmMotionConfig arm_motion;
    std::array<std::uint16_t, 3> ethercat_positions{};
    if (enable_arm) {
      rcl_interfaces::msg::ParameterDescriptor topology_descriptor;
      topology_descriptor.read_only = true;
      ethercat_options.interface = declare_parameter<std::string>(
        "yiyou_ethercat_interface", "", topology_descriptor);
      const auto positions = declare_parameter<std::vector<std::int64_t>>(
        "yiyou_ethercat_slaves", {1, 2, 3, 4}, topology_descriptor);
      if ((positions.size() != 3 && positions.size() != 4) ||
        std::set<std::int64_t>(positions.begin(), positions.end()).size() != positions.size())
      {
        throw std::invalid_argument(
                "set three or four unique yiyou_ethercat_slaves; fourth entry uses CST");
      }
      for (std::size_t i = 0; i < positions.size(); ++i) {
        if (positions[i] < 1 || positions[i] > 199) {
          throw std::invalid_argument("yiyou_ethercat_slaves must be in 1..199");
        }
        ethercat_options.slaves.push_back(static_cast<std::uint16_t>(positions[i]));
        if (i < 3) {ethercat_positions[i] = static_cast<std::uint16_t>(positions[i]);}
      }
      if (positions.size() == 4) {ethercat_options.cst_slave = ethercat_options.slaves[3];}
      chassis::ethercat::validate_selection(ethercat_options);
      chassis::declare_arm_current_parameter(*this, ethercat_options.cst_slave.has_value());
      arm_motion = chassis::declare_arm_motion_parameters(*this);
      chassis::declare_arm_pose_parameter(*this);
      rcl_interfaces::msg::ParameterDescriptor limit_descriptor;
      limit_descriptor.read_only = true;
      limit_descriptor.description =
        "Absolute EtherCAT arm speed that cuts shared motor power (RPM)";
      declare_parameter<double>("can3_speed_limit_rpm", 60.0, limit_descriptor, true);
      rcl_interfaces::msg::ParameterDescriptor hold_descriptor;
      hold_descriptor.read_only = true;
      hold_descriptor.description = "Legacy joint jog disabled; use yiyou_arm_target [x,z,pitch]";
      for (unsigned id = 1; id <= 3; ++id) {
        const auto name = "can3_motor_" + std::to_string(id) + "_angle_deg";
        if (declare_parameter<double>(name, 0.0, hold_descriptor) != 0.0) {
          throw std::invalid_argument("CAN3 startup angle must be zero: " + name);
        }
      }
    }
    return {
      std::move(config), forward_rpm, std::move(ethercat_options),
      std::move(arm_motion), ethercat_positions};
  }

  // 启动顺序：底盘上电 -> 机械臂初始化/回零 -> 底盘校准。
  void initialize_hardware(
    const StartupConfig & startup, bool enable_arm,
    const std::function<void()> & poll_remote)
  {
    // 节点独占控制器；节点销毁时自动析构控制器，执行停机、失能和断电清理。
    controller_ = std::make_unique<chassis::ChassisController>(startup.chassis_config);
    // 初始化和校准中的耗时循环通过此回调检查 ROS 是否仍在运行，支持取消启动。
    const auto keep_running = [this, poll_remote] {
        // 统一入口在启动等待和校准期间继续处理遥控器，包括急停按钮。
        if (poll_remote) {poll_remote();}
        if (!rclcpp::ok()) {return false;}
        // 回零与校准会同步阻塞，取消检查同时推进机械臂反馈和指令握手。
        if (arm_) {
          arm_->update();
          log_arm_feedback();
        }
        return true;
      };
    // 打开 CAN0/1/2、建立电机对象并上电；校准完成前禁止行进轮运动。
    controller_->initialize(keep_running);
    if (poll_remote) {poll_remote();}
    if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
    if (enable_arm) {
      // 前三轴 PP，列表第四项 CST；共享电源已由底盘开启。
      auto master = std::make_shared<chassis::ethercat::Master>(startup.ethercat_options);
      // BXI remains the shared power switch; no Yiyou CAN frames are sent.
      auto power = std::make_shared<chassis::BxiPciTransport>(3);
      arm_ = std::make_unique<chassis::ArmController>(
        master, startup.ethercat_positions, [power] {
          const auto result = power->set_motor_power(false);
          if (!result) {throw std::runtime_error(result.error().message);}
        }, startup.ethercat_options.cst_slave, startup.arm_motion);
      arm_->initialize();
      if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
      arm_->update();
      RCLCPP_INFO(
        get_logger(),
        "EtherCAT: Yiyou axes 1-3 initially hold startup positions in PP mode; "
        "no zero saving or EEPROM writes");
      if (arm_->has_current_axis()) {
        RCLCPP_INFO(
          get_logger(), "EtherCAT: Yiyou axis 4 uses CST at slave %u; "
          "yiyou_motor_4_current_permille=0 at startup, runtime range [-1000,1000]",
          static_cast<unsigned>(*startup.ethercat_options.cst_slave));
      }
      RCLCPP_INFO(
        get_logger(), "EtherCAT protection: abs(speed) >= 60 RPM cuts shared motor power; "
        "fault stays latched until restart; EtherCAT feedback failures also stop control");
      log_arm_feedback();
      if (startup.arm_motion.home_on_start) {
        RCLCPP_INFO(
          get_logger(), "ARM_HOME_START target_counts=[%d,%d,%d] speed_rpm=%.3f; "
          "waiting for saved joint zero before chassis calibration (timeout 300 s)",
          startup.arm_motion.zero_counts[0], startup.arm_motion.zero_counts[1],
          startup.arm_motion.zero_counts[2],
          startup.arm_motion.speed_rpm);
        arm_->return_to_zero(keep_running);
        RCLCPP_INFO(get_logger(), "ARM_HOME_DONE: all three axes reached saved joint zero");
      } else {
        RCLCPP_INFO(get_logger(), "ARM_HOME_DISABLED: holding startup positions");
      }
    } else {
      RCLCPP_INFO(get_logger(), "Chassis only: CAN0/1/2 enabled; CAN3 arm control disabled");
    }
    RCLCPP_INFO(
      get_logger(),
      "CAN0: calibrating all four steering axes concurrently; travel axes inhibited");
    // 四个转向轴并行搜索机械限位并独立回中；确认全部到位后才使能行进轮。
    // 此调用同步执行，完成前尚未创建速度订阅和控制定时器。
    controller_->calibrate(keep_running);
    if (!rclcpp::ok()) {throw std::runtime_error("startup cancelled");}
    // 正值进入持续前进状态，后续速度消息可替换该指令；默认 0 不自动前进。
    if (startup.forward_rpm > 0.0) {controller_->forward(startup.forward_rpm);}
    if (startup.forward_rpm > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; CAN1/2 ISWV travel forward %.3f output RPM; waiting for /cmd_vel_car",
        startup.forward_rpm);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; travel wheels stopped; waiting for /cmd_vel_car");
    }
    // 输出运动模式和转向未对齐时的行进策略，便于核对当前配置。
    RCLCPP_INFO(
      get_logger(),
      "steering control: swerve; alignment policy=%s",
      startup.chassis_config.drive_alignment_policy.c_str());
    // 解析逐轮参数；未配置 modules 时由全局机械参数生成。
    // 轮序 FL/RL/RR/FR 对应前左、后左、后右、前右。
    const auto modules = chassis::resolved_modules(startup.chassis_config);
    RCLCPP_INFO(
      get_logger(), "steering target rate limits [FL,RL,RR,FR]: [%.3f, %.3f, %.3f, %.3f] rad/s",
      modules[0].steering_rate_limit_rad_s, modules[1].steering_rate_limit_rad_s,
      modules[2].steering_rate_limit_rad_s, modules[3].steering_rate_limit_rad_s);
  }

  // 校准完成后才接收控制命令，并启动 50 Hz 调度。
  void setup_ros_interfaces()
  {
    // 末端目标在校准完成后开放；旧单轴调角参数仍为只读。
    if (arm_) {
      arm_current_remote_ = std::make_unique<chassis::ArmCurrentRemote>(
        *this, arm_->has_current_axis());
      arm_pose_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
          return chassis::validate_arm_pose_parameters(*arm_, parameters);
        });
      arm_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/arm_pose", 10);
      RCLCPP_INFO(
        get_logger(),
        "EtherCAT arm IK ready: set yiyou_arm_target=[x_m,z_m,pitch_rad]; feedback on /arm_pose");
    }
    RCLCPP_INFO(get_logger(), "Chassis command watchdog: %.3f s", command_timeout_);
    // 接收车体速度指令，只保留最新一条，避免断连后旧速度排队重放。
    // linear.x/y 为前后/横向速度（m/s），angular.z 为偏航角速度（rad/s）。
    // 控制器保持最后一条指令；停止发布超过 command_timeout_sec 后强制停车。
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_car", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        try {
          controller_->set_velocity(message->linear.x, message->linear.y, message->angular.z);
          command_watchdog_->record_command(chassis::VelocityCommandWatchdog::Clock::now());
        } catch (const std::exception & error) {on_fault(error);}
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {update_control();});
  }

  // 每周期先检查速度指令是否断流，再继续机械臂、底盘和反馈处理。
  void update_control()
  {
    try {
      if (command_watchdog_->consume_timeout(chassis::VelocityCommandWatchdog::Clock::now())) {
        RCLCPP_WARN(
          get_logger(),
          "/cmd_vel_car silent for %.3f s; forcing zero chassis velocity",
          command_timeout_);
        controller_->stop();
      }
      if (arm_) {
        arm_->update();
        arm_current_remote_->update();
        chassis::apply_arm_current_parameter(*this, *arm_);
        try {
          chassis::apply_arm_pose_parameter(*this, *arm_, last_arm_pose_request_);
        } catch (const std::invalid_argument & error) {
          RCLCPP_WARN(get_logger(), "Arm target rejected before motion: %s", error.what());
        }
        const auto pose = arm_->actual_pose();
        geometry_msgs::msg::PoseStamped feedback;
        feedback.header.stamp = get_clock()->now();
        feedback.header.frame_id = "arm_shoulder";
        feedback.pose.position.x = pose.x;
        feedback.pose.position.z = pose.z;
        feedback.pose.orientation.y = -std::sin(pose.pitch / 2.0);
        feedback.pose.orientation.w = std::cos(pose.pitch / 2.0);
        arm_pose_publisher_->publish(feedback);
      }
      controller_->update();
      log_arm_feedback();
      // 消费最近生成的一秒零速反馈报告；无新报告时为空，不额外读取 CAN。
      const auto report = controller_->take_stop_diagnostic();
      if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
    } catch (const std::exception & error) {on_fault(error);}
  }

  void log_arm_feedback()
  {
    if (!arm_) {return;}
    const auto report = arm_->take_feedback_diagnostic();
    if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
  }

  void on_fault(const std::exception & error)
  {
    // 回调异常统一在这里记录并结束 ROS 运行；硬件清理由控制器故障路径和析构负责。
    RCLCPP_ERROR(get_logger(), "chassis stopped: %s", error.what());
    failed_ = true;
    if (arm_ && !arm_->fault_reason().empty()) {
      RCLCPP_ERROR(get_logger(), "%s", arm_->fault_reason().c_str());
    }
    if (timer_) {timer_->cancel();}
    try {
      if (arm_) {arm_->stop();}
    } catch (const std::exception & stop_error) {
      RCLCPP_ERROR(get_logger(), "EtherCAT stop failed: %s", stop_error.what());
    }
    rclcpp::shutdown();
  }

  // 保存对象所有权，使控制器、订阅和定时器在节点运行期间持续有效。
  std::unique_ptr<chassis::ChassisController> controller_;
  // 声明在 controller_ 后，正常退出时先停止意优电机，再由底盘断电。
  std::unique_ptr<chassis::ArmController> arm_;
  std::unique_ptr<chassis::ArmCurrentRemote> arm_current_remote_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr arm_pose_callback_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_publisher_;
  std::vector<double> last_arm_pose_request_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<chassis::VelocityCommandWatchdog> command_watchdog_;
  double command_timeout_{0.3};
  bool failed_{false};
};

void print_help(const char * program_name, const std::string & mode, bool enable_arm)
{
  std::cout << "Usage: " << program_name << " --ros-args -p name:=value ...\n"
            << "Mode: " << mode << '\n'
            << "key_control/robot_control wait for button14 before chassis startup.\n"
            << "Defaults: source src/config, or installed share/chassis/config.\n"
            << "steering.yaml loads automatically; override with steering_config_file:=/path.\n"
            << "Use unqualified steering_config_file for the same chassis and remote file.\n"
            <<
    "Remote speed/acceleration defaults use its remote_ctrl section; ROS parameters override.\n"
            << "Remote parameters: start_button:=14, emergency_stop_button:=11.\n"
            << "Set start_chassis:=false for remote commands only (external chassis node).\n"
            << "chassis/chassis_only start hardware directly without a remote button.\n"
            << "chassis_only_remote waits for button14 before chassis startup.\n";
  if (enable_arm) {
    std::cout <<
      "yiyou_ethercat.yaml loads automatically; --params-file and -p override its values.\n"
      "Set yiyou_ethercat_interface:=enpXsY; yiyou_ethercat_slaves defaults to [1,2,3,4].\n"
      "First three entries hold startup positions in PP; fourth entry uses CST.\n"
      "Three entries retain PP-only operation; entries are physical slave positions.\n"
      "yiyou_motor_4_current_permille starts at 0; runtime signed integer [-1000,1000].\n"
      "Runtime arm target: yiyou_arm_target=[x_m,z_m,pitch_rad]; feedback: /arm_pose.\n"
      "Remote buttons 1/3 select x/z/pitch; axis7 steps by 1 cm or 1 degree.\n"
      "Holding axis7 repeats every 0.1 s; neutral stops repeats; busy steps are discarded.\n"
      "Hold button0/4 for fourth-axis +/-5% rated current; release or both buttons clears it.\n"
      "Release both current buttons before use; remote current expires after 300 ms.\n"
      "Joint zero matches simulation [+150,-150,0] geometric offsets.\n"
      "yiyou_arm_home_on_start defaults true: return to saved zero before chassis calibration.\n"
      "Set yiyou_arm_home_on_start:=false to hold startup positions instead.\n"
      "PP speed defaults 0.6 output RPM; acceleration/deceleration 1.0 RPM/s.\n"
      "Legacy joint jog is disabled; CST zero current does not hold position.\n";
  } else {
    std::cout << "Chassis only: CAN0/1/2; no CAN3 initialization or arm motor commands.\n";
  }
  std::cout << "Help does not open CAN devices or command motors.\n";
}

// 加载默认 YAML，同时保留用户本地参数优先于全局默认值的行为。
rclcpp::NodeOptions initialize_ros(int argc, char ** argv, bool enable_arm)
{
  const std::vector<std::string> user_arguments(argv, argv + argc);
  std::vector<std::string> arguments{argv[0]};
  if (enable_arm) {
    const auto config = default_config_directory() / "yiyou_ethercat.yaml";
    arguments.insert(arguments.end(), {"--ros-args", "--params-file", config.string(), "--"});
  }
  arguments.insert(arguments.end(), user_arguments.begin() + 1, user_arguments.end());
  std::vector<const char *> argument_pointers;
  for (const auto & argument : arguments) {
    argument_pointers.push_back(argument.c_str());
  }
  rclcpp::init(static_cast<int>(argument_pointers.size()), argument_pointers.data());
  // 用户参数作为节点本地参数再次应用，优先于全局默认配置。
  // 这样默认 YAML 的节点分组顺序不会改变用户 -p / --params-file 的覆盖行为。
  return rclcpp::NodeOptions().arguments(user_arguments);
}

// 统一管理遥控等待、节点启动和执行器，返回前释放节点及硬件。
int run_control(bool enable_arm, bool enable_remote, const rclcpp::NodeOptions & options)
{
  std::shared_ptr<ChassisNode> chassis_node;
  std::shared_ptr<rclcpp::Node> remote_node;
  auto session = std::make_shared<chassis::RemoteControlSession>();
  rclcpp::executors::SingleThreadedExecutor executor;
  if (enable_remote) {
    remote_node = chassis::make_remote_control(
      options, session, enable_arm, (default_config_directory() / "steering.yaml").string());
    executor.add_node(remote_node);
  }
  if (!enable_remote || session->start_chassis) {
    if (enable_remote) {
      while (rclcpp::ok() && session->stage == chassis::RemoteControlSession::Stage::waiting) {
        executor.spin_once(std::chrono::milliseconds(100));
      }
    }
    if (rclcpp::ok()) {
      // 启动请求由遥控回调提出；在回调外初始化，校准时才能继续 spin 遥控器。
      session->stage = chassis::RemoteControlSession::Stage::starting;
      chassis_node = std::make_shared<ChassisNode>(
        enable_arm, options, [&executor] {executor.spin_some();});
      executor.add_node(chassis_node);
      session->stage = chassis::RemoteControlSession::Stage::running;
    }
  }
  // 串行调度硬件控制与遥控器；参数请求使用异步回调，不阻塞执行器。
  if (rclcpp::ok()) {executor.spin();}
  return chassis_node && chassis_node->failed() ? 1 : 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string mode = CHASSIS_CONTROL_MODE;
  const bool enable_arm = mode != "chassis_only" && mode != "chassis_only_remote";
  const bool enable_remote = mode != "chassis" && mode != "chassis_only";
  if (argc == 2 && std::string(argv[1]) == "--help") {
    print_help(argv[0], mode, enable_arm);
    return 0;
  }

  int result = 0;
  try {
    const auto options = initialize_ros(argc, argv, enable_arm);
    result = run_control(enable_arm, enable_remote, options);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("control"), "%s", error.what());
    result = 1;
  }
  // run_control 返回或抛出时已析构节点：先停机械臂，再由底盘关闭共享电源。
  rclcpp::shutdown();
  return result;
}
