#include "chassis/chassis_controller.hpp"
#include "chassis/steering_calibration.hpp"
#include "fake_iswv.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using chassis::test::FakeCan0;
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
template<typename Function>
void rejects(Function function)
{
  try {
    function();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error("operation should have been rejected");
}
struct Rig
{
  std::array<FakeCan0, 3> buses;
  std::vector<bool> power;
  chassis::SteeringConfigFile config()
  {
    auto value = buses[0].config();
    value.drive_can_buses = {1, 1, 2, 2};
    value.drive_node_ids = {1, 2, 3, 4};
    value.drive_encoder_resolution = 65536;
    value.drive_gear_ratio = 10.0;
    value.drive_inverted = {false, false, true, true};
    value.command_timeout_s = 0.02;
    value.drive_max_output_rpm = 1.0;
    return value;
  }
  chassis::ChassisController::Hardware hardware()
  {
    return {
      [this](unsigned int bus) {return buses.at(bus).transport;},
      [this](bool on) {power.push_back(on);}, 0ms};
  }
  void check_stopped()
  {
    for (unsigned i = 0; i < 4; ++i) {
      auto & axis = buses[i < 2 ? 1 : 2].axes[i];
      require(axis.velocity_raw == 0, "all travel targets must be zero");
    }
  }
  void feedback()
  {
    for (auto & bus : buses) {bus.publish_feedback();}
    std::this_thread::sleep_for(2ms);
  }
};
}  // namespace

int main()
{
  try {
    using State = chassis::ChassisController::State;
    Rig rig;
    {
      chassis::ChassisController controller(rig.config(), rig.hardware());
      rejects([&] {controller.forward(0.5);});
      controller.initialize();
      rejects([&] {controller.initialize();});
      require(
        controller.state() == State::uninitialized,
        "initialization cannot bypass calibration");
      rig.check_stopped();
      for (unsigned i = 0; i < 4; ++i) {
        require(
          rig.buses[i < 2 ? 1 : 2].axes[i].status != 0x0037,
          "travel must stay disabled before midpoint");
      }
      controller.calibrate(
        [&] {
          rig.feedback();
          rig.check_stopped();
          // During search and return, every travel motor must remain disabled.
          if (rig.buses[0].axes[3].position_pdo_frames == 0) {
            for (unsigned i = 0; i < 4; ++i) {
              require(
                rig.buses[i < 2 ? 1 : 2].axes[i].status != 0x0037,
                "travel enabled before steering return");
            }
          }
          return true;
        });
      require(controller.state() == State::ready, "calibration should reach ready");
      require(controller.config().steering_calibrated, "calibration should publish valid midpoint");
      for (const auto & axis : rig.buses[0].axes) {
        require(
          axis.position_pdo_frames > 0 && axis.position == axis.target_position,
          "all four steering axes must currently be centered");
      }
      controller.forward(0.5);
      std::this_thread::sleep_for(30ms);
      controller.update();
      require(
        controller.state() == State::forward_wait,
        "forward wait must survive command timeout");
      require(
        rig.buses[1].axes[0].velocity_raw > 0 && rig.buses[1].axes[1].velocity_raw > 0,
        "CAN1 forward wheel mapping");
      require(
        rig.buses[2].axes[2].velocity_raw < 0 && rig.buses[2].axes[3].velocity_raw < 0,
        "CAN2 mirror mounting forward mapping");
      const auto forward_raw = rig.buses[1].axes[0].velocity_raw;
      require(std::abs(forward_raw - 89478) <= 1, "output RPM must use ISWV motor RPM encoding");
      controller.set_velocity(-0.001, 0, 0);
      controller.update();
      // Reversals must decelerate through zero instead of jumping signs.
      for (unsigned step = 0; step < 3; ++step) {
        std::this_thread::sleep_for(20ms);
        controller.set_velocity(-0.001, 0, 0);
        controller.update();
      }
      require(
        rig.buses[1].axes[0].velocity_raw < 0 && rig.buses[2].axes[2].velocity_raw > 0,
        "next reverse command must replace persistent forward");
      std::this_thread::sleep_for(30ms);
      controller.update();
      require(controller.state() == State::stopped, "external command timeout must stop");
      rig.check_stopped();
      controller.forward(0.5);
      controller.stop();
      rig.feedback();
      controller.update();
      rig.check_stopped();
      require(controller.state() == State::stopped, "stop must not resume forward");
      controller.forward(0.5);
      rejects([&] {controller.set_velocity(std::numeric_limits<double>::quiet_NaN(), 0, 0);});
      require(controller.state() == State::faulted, "invalid command must fault and stop");
      require(!rig.power.back(), "fault must power off");
      rig.check_stopped();
      rejects([&] {controller.forward(0.5);});
    }
    require(
      rig.power == std::vector<bool>(
        {true,
          false}), "calibration-to-drive must not cycle power");

    Rig continuous;
    {
      auto config = continuous.config();
      config.command_timeout_s = 1.0;
      chassis::ChassisController controller(config, continuous.hardware());
      controller.initialize();
      controller.calibrate([&] {continuous.feedback(); return true;});
      auto tick = [&](double yaw) {
          for (auto & bus : continuous.buses) {
            bus.publish_feedback();
          }
          std::this_thread::sleep_for(20ms);
          controller.set_velocity(0.003, 0, yaw);
          controller.update();
        };
      for (unsigned step = 0; step < 5; ++step) {
        tick(0.0);
      }
      for (unsigned step = 0; step < 10; ++step) {
        const auto before = continuous.buses[0].axes[0].target_position;
        tick(0.001);
        require(
          continuous.buses[1].axes[0].velocity_raw > 0,
          "small steering changes must retain forward travel");
        require(
          std::abs(continuous.buses[0].axes[0].target_position - before) <= 1,
          "steering targets must be rate limited");
      }
      // A reported hard-stop crossing must fault even without a new motion command.
      continuous.buses[0].axes[1].position = continuous.buses[0].axes[1].positive_limit;
      continuous.buses[0].publish_feedback();
      std::this_thread::sleep_for(20ms);
      controller.set_velocity(0, 0, 0);
      rejects([&] {controller.update();});
      require(controller.state() == State::faulted, "unsafe measured steering must fault");
      require(!continuous.power.back(), "unsafe steering must power off");
      continuous.check_stopped();
    }

    Rig cancelled;
    {
      chassis::ChassisController controller(cancelled.config(), cancelled.hardware());
      controller.initialize();
      rejects([&] {controller.calibrate([] {return false;});});
      require(
        controller.state() == State::faulted,
        "cancel must fault rather than mark calibrated");
      require(!controller.config().steering_calibrated, "cancel must not persist success");
      cancelled.check_stopped();
      require(!cancelled.power.back(), "cancel must power off");
    }
    Rig partial;
    auto hardware = partial.hardware();
    hardware.power = [&partial](bool on) {
        partial.power.push_back(on);
        if (on) {throw std::runtime_error("injected power-on failure");}
      };
    {
      chassis::ChassisController controller(partial.config(), hardware);
      rejects([&] {controller.initialize();});
      require(controller.state() == State::faulted, "partial initialization failure must fault");
      partial.check_stopped();
      require(
        partial.power == std::vector<bool>(
          {true,
            false}), "failed power-on still needs power-off");
    }
    std::cout << "Chassis lifecycle, calibration gate, direction and fault checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
