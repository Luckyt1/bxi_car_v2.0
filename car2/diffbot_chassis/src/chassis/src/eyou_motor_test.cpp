#include "chassis/bxi_pci_transport.hpp"
#include "chassis/eyou_motor.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) {interrupted = 1;}

long long number(const char * text, long long min, long long max)
{
  std::size_t used = 0;
  const std::string value(text);
  const auto parsed = std::stoll(value, &used);
  if (used != value.size() || parsed < min || parsed > max) {
    throw std::invalid_argument("invalid argument: " + value);
  }
  return parsed;
}

double decimal(const char * text, double min, double max)
{
  std::size_t used = 0;
  const std::string value(text);
  const double parsed = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(parsed) || parsed < min || parsed > max) {
    throw std::invalid_argument("invalid RPM argument: " + value);
  }
  return parsed;
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  if (argc < 3 || argc > 6) {
    std::cout <<
      "Usage: eyou_motor_test NODE_ID RPM [SECONDS=5|--continuous] [ACCEL_RPM_S=10] [DECEL_RPM_S=10]\n"
      "BXI can2 (bus=2), classic CAN, motor default 1 Mbps.\n"
      "Speed: reducer OUTPUT shaft RPM, signed decimal, smoke-test range +/-30 RPM.\n"
      "Acceleration/deceleration: output RPM/s. Duration: 1..60 s.\n"
      "--continuous: run until Ctrl+C (SIGINT/SIGTERM), then stop and disable.\n"
      "Scale read from encoder 0x2025 and reduction ratio 0x26A2/0x26A3.\n"
      "Exclusive use: stop chassis/calibration first. This tool switches shared motor power.\n"
      "Example: eyou_motor_test 1 1.5 5\n";
    return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 1;
  }
  try {
    const auto node = static_cast<std::uint8_t>(number(argv[1], 1, 127));
    const double rpm = decimal(argv[2], -30, 30);
    const bool continuous = argc > 3 && std::string(argv[3]) == "--continuous";
    const auto seconds = continuous ? 0 : (argc > 3 ? number(argv[3], 1, 60) : 5);
    const double accel = argc > 4 ? decimal(argv[4], 0.001, 1000) : 10;
    const double decel = argc > 5 ? decimal(argv[5], 0.001, 1000) : 10;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    auto transport = std::make_shared<chassis::BxiPciTransport>(2);
    iswv::canopen::CanopenMaster master(transport);
    chassis::EyouMotor motor(master, node);
    const auto power = transport->set_motor_power(true);
    if (!power) {throw std::runtime_error(power.error().message);}
    for (int i = 0; i < 40 && !interrupted; ++i) {
      std::this_thread::sleep_for(100ms);
    }
    if (interrupted) {return 130;}
    try {
      const double pulses_per_rev = motor.configure_rpm_units();
      (void)motor.velocity_from_rpm(rpm);  // Validate before enabling the motor.
      if (interrupted) {return 130;}
      motor.enable_rpm(accel, decel);
      if (!interrupted) {motor.set_velocity_rpm(rpm);}
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
      auto next_electrical = std::chrono::steady_clock::now();
      std::string electrical;
      while (!interrupted && (continuous || std::chrono::steady_clock::now() < end)) {
        const auto actual_rpm = motor.actual_velocity_rpm();
        const double position_rad = motor.actual_position() * (2.0 * std::acos(-1.0)) /
          pulses_per_rev;
        if (std::chrono::steady_clock::now() >= next_electrical) {
          electrical = motor.electrical_report();
          next_electrical = std::chrono::steady_clock::now() + 1s;
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "转速: " << actual_rpm << " RPM  位置: " << std::setprecision(6)
                  << position_rad << " rad  " << electrical << '\n';
        std::this_thread::sleep_for(100ms);
      }
      motor.stop();
    } catch (const std::exception & original) {
      std::cerr << "Original operation failed: " << original.what() << std::endl;
      try {
        motor.stop();
      } catch (const std::exception & error) {
        std::cerr << "Stop failed: " << error.what() << '\n';
      }
      throw;
    }
    return interrupted ? 130 : 0;
  } catch (const std::exception & error) {
    std::cerr << "Eyou motor: " << error.what() << '\n';
    return 1;
  }
}
