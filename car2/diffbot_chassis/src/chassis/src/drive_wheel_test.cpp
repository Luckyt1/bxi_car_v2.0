#include "chassis/bxi_pci_transport.hpp"
#include "chassis/iswv_drive.hpp"
#include "chassis/steering_config.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

long long integer(const char * text, long long min, long long max)
{
  std::size_t used = 0;
  const std::string value(text);
  const auto parsed = std::stoll(value, &used);
  if (used != value.size() || parsed < min || parsed > max) {
    throw std::invalid_argument("invalid integer argument: " + value);
  }
  return parsed;
}

double decimal(const char * text)
{
  std::size_t used = 0;
  const std::string value(text);
  const double parsed = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument("invalid RPM argument: " + value);
  }
  return parsed;
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  constexpr const char * wheel_names[] = {"前左", "后左", "后右", "前右"};

  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout <<
      "Usage: drive_wheel_test CONFIG WHEEL RPM\n"
      "WHEEL: 1=front-left, 2=rear-left, 3=rear-right, 4=front-right.\n"
      "RPM: positive chassis-forward output-shaft speed, limited by drive_max_output_rpm.\n"
      "Only the selected ISWV travel motor is enabled; Ctrl+C stops, disables and powers off.\n";
    return 0;
  }
  if (argc != 4) {
    std::cerr << "Usage: drive_wheel_test CONFIG WHEEL RPM\n";
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);

  std::shared_ptr<chassis::BxiPciTransport> transport;
  std::unique_ptr<iswv::canopen::CanopenMaster> master;
  std::unique_ptr<chassis::IswvDrive> wheel;
  bool power_enabled = false;
  std::string failure;

  try {
    const auto config = chassis::load_steering_config(argv[1]);
    const auto wheel_index = static_cast<std::size_t>(integer(argv[2], 1, 4) - 1);
    const double forward_rpm = decimal(argv[3]);
    if (forward_rpm <= 0.0 || forward_rpm > config.drive_max_output_rpm) {
      throw std::invalid_argument(
              "RPM must be positive and no greater than drive_max_output_rpm=" +
              std::to_string(config.drive_max_output_rpm));
    }

    const unsigned int bus = config.drive_can_buses[wheel_index];
    const std::uint8_t node_id = config.drive_node_ids[wheel_index];
    const bool inverted = config.drive_inverted[wheel_index];
    transport = std::make_shared<chassis::BxiPciTransport>(bus);
    master = std::make_unique<iswv::canopen::CanopenMaster>(transport);

    auto axis_config = iswv::AxisConfiguration::manual_travel(node_id);
    axis_config.encoder_resolution = config.drive_encoder_resolution;
    axis_config.gear_ratio = config.drive_gear_ratio;
    axis_config.wheel_diameter_m = config.motor.wheel_diameter_m;
    wheel = std::make_unique<chassis::IswvDrive>(*master, axis_config);

    const auto power_on = transport->set_motor_power(true);
    if (!power_on) {
      throw std::runtime_error("motor power on failed: " + power_on.error().message);
    }
    power_enabled = true;
    std::cout << "共享电机电源已开启，等待 4 秒。\n";
    for (int count = 0; count < 40 && !stop_requested; ++count) {
      std::this_thread::sleep_for(100ms);
    }

    if (!stop_requested) {
      wheel->stop();
      wheel->enable_rpm(
        config.drive_acceleration_rpm_s, config.drive_deceleration_rpm_s);
      wheel->set_velocity_rpm(inverted ? -forward_rpm : forward_rpm);
      std::cout << wheel_names[wheel_index] << "行进轮：CAN" << bus
                << " Node-ID " << static_cast<unsigned>(node_id)
                << "，底盘前进方向 " << forward_rpm
                << " 输出轴 RPM。按 Ctrl+C 停止。\n";

      while (!stop_requested) {
        const double raw_rpm = wheel->actual_velocity_rpm();
        if (!std::isfinite(raw_rpm) ||
          std::abs(raw_rpm) > config.drive_max_output_rpm * 1.25 + 0.1)
        {
          throw std::runtime_error("actual RPM exceeded the safety limit");
        }
        const double chassis_rpm = inverted ? -raw_rpm : raw_rpm;
        std::cout << std::fixed << std::setprecision(3)
                  << wheel_names[wheel_index] << "反馈：" << chassis_rpm << " RPM\n";
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (const std::exception & error) {
    failure = error.what();
  }

  if (wheel) {
    try {
      wheel->stop();
    } catch (const std::exception & error) {
      if (failure.empty()) {
        failure = std::string("stop failed: ") + error.what();
      } else {
        std::cerr << "Stop failed: " << error.what() << '\n';
      }
    }
    wheel.reset();
  }
  if (power_enabled && transport) {
    const auto power_off = transport->set_motor_power(false);
    if (!power_off && failure.empty()) {
      failure = "motor power off failed: " + power_off.error().message;
    }
  }

  if (!failure.empty()) {
    std::cerr << "Drive wheel test: " << failure << '\n';
    return 1;
  }
  std::cout << "所选行进轮已停止、失能并断电。\n";
  return stop_requested ? 130 : 0;
}
