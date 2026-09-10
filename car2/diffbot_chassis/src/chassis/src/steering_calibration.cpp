#include "chassis/bxi_pci_transport.hpp"
#include "chassis/steering_calibration.hpp"
#include "chassis/steering_direction.hpp"
#include "chassis/steering_config.hpp"
#include "iswv/iswv.hpp"
#include "rclcpp/rclcpp.hpp"


#include <chrono>
#include <cerrno>
#include <iostream>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{

using namespace std::chrono_literals;

void stop_steering(iswv::SteeringLayout & steering) noexcept
{
  for (std::size_t index = 0; index < 4; ++index) {
    const auto axis = steering.at(static_cast<iswv::WheelPosition>(index));
    (void)axis->command_motor_velocity_rpm(0.0);
    (void)axis->drive().quick_stop();
    (void)axis->drive().disable();
  }
}

void require_hold_health(iswv::SteeringLayout & steering)
{
  for (std::size_t index = 0; index < 4; ++index) {
    const auto axis = steering.at(static_cast<iswv::WheelPosition>(index));
    const auto status = axis->drive().read_status();
    const auto error1 = axis->node()->read(iswv::objects::error_status);
    const auto error2 = axis->node()->read(iswv::objects::error_status_2);
    if (!status || !error1 || !error2 ||
      iswv::cia402::decode_state(status.value()) !=
      iswv::cia402::DriveState::operation_enabled ||
      error1.value() != 0 || error2.value() != 0)
    {
      throw std::runtime_error("steering hold health check failed");
    }
  }
}

}  // namespace

class SteeringCalibration final : public rclcpp::Node
{
public:
  SteeringCalibration()
  : Node("steering_calibration"),
    config_path_(declare_parameter<std::string>("steering_config_file", ""))
  {
    if (config_path_.empty()) {
      throw std::invalid_argument("set steering_config_file to a writable calibration YAML file");
    }
    hold_after_calibration_ = declare_parameter<bool>("hold_after_calibration", true);
    confirm_directions_ = declare_parameter<bool>("confirm_directions", false);
    config_ = chassis::load_steering_config(config_path_);
  }

  ~SteeringCalibration() override
  {
    if (steering_) {
      stop_steering(*steering_);
    }
    if (transport_) {
      (void)transport_->set_motor_power(false);
    }
  }

  int run()
  {
    config_.steering_calibrated = false;
    chassis::save_steering_config(config_path_, config_);

    transport_ = std::make_shared<chassis::BxiPciTransport>(
      static_cast<unsigned int>(config_.can_bus));
    master_ = std::make_unique<iswv::canopen::CanopenMaster>(transport_);
    const auto power = transport_->set_motor_power(true);
    if (!power) {
      throw std::runtime_error(power.error().message);
    }
    const auto warmup_deadline = std::chrono::steady_clock::now() + 4s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < warmup_deadline) {
      std::this_thread::sleep_for(100ms);
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("steering calibration cancelled during motor power warmup");
    }
    steering_ = std::make_unique<iswv::SteeringLayout>(*master_, config_.motor);

    const auto limits = chassis::calibrate_steering_with_limits(
      *steering_, config_, [] {return rclcpp::ok();});
    config_.zero_offset_inc = limits.midpoints;
    config_.steering_calibrated = true;
    chassis::save_steering_config(config_path_, config_);

    RCLCPP_INFO(
      get_logger(), "steering zero offsets saved to %s", config_path_.c_str());
    if (confirm_directions_) {
      confirm_directions(limits);
      stop_steering(*steering_);
      steering_.reset();
      const auto off = transport_->set_motor_power(false);
      if (!off) {throw std::runtime_error("power-off failed: " + off.error().message);}
      std::cout << "四轮方向记录完成，电源已关闭；inverted 配置未修改。\n";
      return 0;
    }
    if (!hold_after_calibration_) {
      stop_steering(*steering_);
      return 0;
    }
    RCLCPP_INFO(get_logger(), "steering motors are holding center; Ctrl-C exits safely");
    while (rclcpp::ok()) {
      require_hold_health(*steering_);
      std::this_thread::sleep_for(200ms);
    }
    stop_steering(*steering_);
    return 0;
  }

private:
  std::string answer(const std::string & prompt)
  {
    std::cout << prompt << std::flush;
    const auto deadline = std::chrono::steady_clock::now() + 120s;
    auto next_health = std::chrono::steady_clock::now();
    std::string line;
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {throw std::runtime_error("input timeout; stopping direction test");}
      if (now >= next_health) {
        require_hold_health(*steering_);
        next_health = now + 200ms;
      }
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int ready = poll(&input, 1, 100);
      if (ready < 0) {
        if (errno == EINTR) {continue;}
        throw std::runtime_error("stdin poll failed");
      }
      if (!ready) {continue;}
      char value;
      if (read(STDIN_FILENO, &value, 1) != 1) {
        throw std::runtime_error("stdin closed; stopping direction test");
      }
      if (value == '\n') {return line;}
      if (value != '\r') {line += value;}
      if (line.size() > 80) {throw std::runtime_error("input too long");}
    }
    throw std::runtime_error("direction test cancelled");
  }

  void confirm_directions(const chassis::SteeringCalibrationResult & limits)
  {
    constexpr const char * names[] = {"前左", "后左", "后右", "前右"};
    std::array<std::string, 4> observations;
    std::cout << "校准回中完成：保持同一次上电，四轮依次正向点动 +50 度。\n"
              << "从车顶向下观察：1=顺时针，2=逆时针，0=未动，3=动错轮；输入后按回车。\n"
              << "等待输入时电机保持位置；每次最多等待 120 秒。Ctrl+C 退出断电。\n";
    for (std::size_t i = 0; i < 4; ++i) {
      const auto move = answer(std::string("准备观察 ") + names[i] + "，输入 1 回车开始（其他输入退出）：");
      if (move != "1" && move != "MOVE") {
        throw std::runtime_error("direction sequence cancelled by user");
      }
      chassis::test_positive_steering_direction(
        *steering_, config_, limits, i, [] {return rclcpp::ok();});
      while (true) {
        observations[i] = answer("刚才俯视方向？1=顺时针，2=逆时针，0=未动，3=动错轮，q=退出：");
        if (observations[i] == "1") {observations[i] = "CW";}
        else if (observations[i] == "2") {observations[i] = "CCW";}
        else if (observations[i] == "0") {observations[i] = "NONE";}
        else if (observations[i] == "3") {observations[i] = "WRONG";}
        else if (observations[i] == "q") {observations[i] = "Q";}
        if (observations[i] == "Q") {throw std::runtime_error("direction sequence cancelled");}
        if (observations[i] == "CW" || observations[i] == "CCW" ||
          observations[i] == "NONE" || observations[i] == "WRONG") {break;}
        std::cout << "请输入 1、2、0、3 或 q，然后按回车。\n";
      }
      std::cout << "STEERING_OBSERVATION wheel=" << i + 1 << " name=" << names[i]
                << " command=positive top_view=" << observations[i]
                << " inverted=" << config_.motor.inverted[i] << std::endl;
      if (observations[i] == "NONE" || observations[i] == "WRONG") {
        throw std::runtime_error("unexpected physical motion; stop sequence and check mapping");
      }
    }
    std::cout << "=== 请复制下面的四轮方向汇总 ===\n";
    for (std::size_t i = 0; i < 4; ++i) {
      std::cout << names[i] << " 正向+50deg: " << observations[i]
                << " inverted=" << config_.motor.inverted[i] << '\n';
    }
  }

  std::string config_path_;
  chassis::SteeringConfigFile config_;
  bool hold_after_calibration_{true};
  bool confirm_directions_{false};
  std::shared_ptr<chassis::BxiPciTransport> transport_;
  std::unique_ptr<iswv::canopen::CanopenMaster> master_;
  std::unique_ptr<iswv::SteeringLayout> steering_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SteeringCalibration>();
    const auto result = node->run();
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("steering_calibration"),
      "calibration failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
