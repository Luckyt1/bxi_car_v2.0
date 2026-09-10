#include "chassis/bxi_pci_transport.hpp"
#include "chassis/steering_config.hpp"
#include "iswv/iswv.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t cancelled = 0;
void cancel(int) {cancelled = 1;}
void running()
{
  if (cancelled) {throw std::runtime_error("test cancelled");}
}
template<typename T>
T checked(iswv::Result<T> result)
{
  if (!result) {throw std::runtime_error(result.error().message);}
  return result.take_value();
}
void checked(iswv::Result<void> result)
{
  if (!result) {throw std::runtime_error(result.error().message);}
}
double number(const char * value)
{
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != std::string(value).size() || !std::isfinite(result)) {
    throw std::invalid_argument("invalid numeric argument");
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  constexpr double pi = 3.14159265358979323846;
  constexpr const char * names[] = {"front_left", "rear_left", "rear_right", "front_right"};
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: steering_direction_test CONFIG WHEEL DELTA_DEG\n"
              << "WHEEL 1..4: front_left, rear_left, rear_right, front_right.\n"
              << "Signed DELTA_DEG: nonzero, at most 3 degrees; configured steering direction.\n"
              << "Requires calibrated config and position within 10 degrees of saved midpoint.\n"
              <<
      "One slow relative jog, no homing or automatic return. Ctrl+C stops and powers off.\n";
    return 0;
  }
  if (argc != 4) {
    std::cerr << "Usage: steering_direction_test CONFIG WHEEL DELTA_DEG\n";
    return 1;
  }
  std::signal(SIGINT, cancel);
  std::signal(SIGTERM, cancel);
  std::signal(SIGHUP, cancel);
  std::shared_ptr<chassis::BxiPciTransport> transport;
  std::unique_ptr<iswv::canopen::CanopenMaster> master;
  std::unique_ptr<iswv::Axis> axis;
  bool power_attempted = false;
  std::string failure;
  try {
    const double wheel_number = number(argv[2]);
    const double delta_deg = number(argv[3]);
    if (wheel_number < 1 || wheel_number > 4 || std::floor(wheel_number) != wheel_number ||
      delta_deg == 0 || std::abs(delta_deg) > 3)
    {
      throw std::invalid_argument("wheel must be 1..4; signed angle must be nonzero and <= 3 deg");
    }
    const auto index = static_cast<std::size_t>(wheel_number - 1);
    const auto config = chassis::load_steering_config(argv[1]);
    if (!config.steering_calibrated) {
      throw std::runtime_error("saved calibration required; this test does not seek hard stops");
    }
    const double counts_per_deg = config.motor.encoder_resolution * config.motor.gear_ratio / 360.0;
    transport = std::make_shared<chassis::BxiPciTransport>(config.can_bus);
    master = std::make_unique<iswv::canopen::CanopenMaster>(transport);
    power_attempted = true;
    checked(transport->set_motor_power(true));
    std::cout << "共享电源已开启，等待 4 秒；仅使能所选转向轴。\n";
    for (int i = 0; i < 40; ++i) {
      running(); std::this_thread::sleep_for(100ms);
    }
    auto settings = iswv::AxisConfiguration::manual_steering(config.motor.node_ids[index]);
    settings.encoder_resolution = config.motor.encoder_resolution;
    settings.gear_ratio = config.motor.gear_ratio;
    settings.inverted = config.motor.inverted[index];
    axis = std::make_unique<iswv::Axis>(master->node(settings.node_id), settings);
    checked(axis->node()->send_nmt(iswv::canopen::NmtCommand::enter_pre_operational));
    checked(axis->drive().transition_to(iswv::cia402::DriveState::switch_on_disabled));
    checked(axis->command_motor_velocity_rpm(0));
    checked(axis->command_torque_percent(0));
    // Output speed 0.2 RPM = 1.2 deg/s; same inversion convention as chassis.
    checked(axis->configure_position_mode(0.2 * config.motor.gear_ratio, 0.1, 0.1));
    const auto mode = checked(axis->node()->read(iswv::objects::mode));
    if (mode != static_cast<std::int8_t>(iswv::cia402::OperationMode::profile_position)) {
      throw std::runtime_error("position mode readback mismatch");
    }
    const auto initial = checked(axis->node()->read(iswv::objects::actual_position));
    const double from_center =
      (static_cast<std::int64_t>(initial) - config.zero_offset_inc[index]) /
      counts_per_deg;
    if (std::abs(from_center) > 10 ||
      std::abs(from_center + delta_deg) > std::min(13.0, config.max_steering_angle_rad * 180 / pi))
    {
      throw std::runtime_error("not near saved midpoint; refuse jog (no automatic centering)");
    }
    const auto delta = std::llround(delta_deg * counts_per_deg);
    const auto target = static_cast<std::int64_t>(initial) + delta;
    if (!delta || target < std::numeric_limits<std::int32_t>::min() ||
      target > std::numeric_limits<std::int32_t>::max())
    {
      throw std::runtime_error("angle cannot be represented safely");
    }
    // Clear any previous position target before enabling; never restore an old goal.
    checked(axis->node()->write(iswv::objects::target_position, initial));
    iswv::SafetyPolicy safety;
    safety.enabled = true;
    safety.configure_heartbeat_producer = true;
    safety.heartbeat_producer_time = 100ms;
    safety.local_heartbeat_timeout = 350ms;
    safety.timeout_action = iswv::TimeoutAction::quick_stop;
    safety.configure_device_interruption_fault = true;
    checked(axis->apply_safety_policy(safety));
    checked(axis->node()->send_nmt(iswv::canopen::NmtCommand::start));
    running();
    checked(axis->drive().enable());
    running();
    std::cout << "DIRECTION_TEST wheel=" << names[index] << " CAN=" << config.can_bus
              << " node=" << static_cast<unsigned>(settings.node_id)
              << " inverted=" << settings.inverted << " command_deg=" << delta_deg
              << " start_inc=" << initial << " target_inc=" << target << std::endl;
    checked(axis->move_absolute_inc(static_cast<std::int32_t>(target)));
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    auto next_print = std::chrono::steady_clock::now();
    while (true) {
      running();
      const auto status = checked(axis->drive().read_status());
      const auto error1 = checked(axis->node()->read(iswv::objects::error_status));
      const auto error2 = checked(axis->node()->read(iswv::objects::error_status_2));
      const auto position = checked(axis->node()->read(iswv::objects::actual_position));
      const auto current = checked(axis->node()->read(iswv::objects::actual_current));
      const double moved = (static_cast<std::int64_t>(position) - initial) / counts_per_deg;
      const double torque = std::abs(static_cast<double>(current)) *
        (config.motor_peak_current_a / 1.414) / 2048.0 *
        config.motor_torque_constant_nm_per_a * config.motor.gear_ratio;
      if (iswv::cia402::decode_state(status) != iswv::cia402::DriveState::operation_enabled ||
        error1 || error2 || torque >= std::min(1.0, config.calibration_torque_limit_nm) ||
        std::abs(moved) > std::abs(delta_deg) + 1.0 || moved * delta_deg < -0.5)
      {
        throw std::runtime_error(
                "jog protection: fault/torque/unexpected encoder motion; moved_deg=" +
                std::to_string(moved) + " torque_Nm=" + std::to_string(torque));
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_print) {
        std::cout << "encoder_delta_deg=" << moved << " current_raw=" << current << std::endl;
        next_print = now + 200ms;
      }
      if (std::abs(static_cast<double>(position) - target) <=
        std::max(1.0, counts_per_deg * 0.15))
      {
        std::cout << "DIRECTION_RESULT wheel=" << names[index] << " command_deg=" << delta_deg
                  << " encoder_delta_deg=" << moved << " inverted=" << settings.inverted <<
          std::endl;
        break;
      }
      if (now >= deadline) {throw std::runtime_error("jog timed out");}
      std::this_thread::sleep_for(20ms);
    }
  } catch (const std::exception & error) {
    failure = error.what();
  }
  if (axis) {
    for (const auto & result : {axis->drive().quick_stop(), axis->drive().disable()}) {
      if (!result) {
        std::cerr << "Stop/disable failed: " << result.error().message << '\n';
        if (failure.empty()) {failure = "stop/disable failed";}
      }
    }
    axis.reset();
  }
  if (power_attempted) {
    const auto off = transport->set_motor_power(false);
    if (!off) {failure += " power-off failed: " + off.error().message;}
  }
  if (!failure.empty()) {std::cerr << failure << '\n'; return 1;}
  std::cout << "测试结束，已停机失能并断电；不会自动回中或改配置。\n";
  return 0;
}
