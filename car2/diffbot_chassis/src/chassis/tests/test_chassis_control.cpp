#ifdef NDEBUG
#undef NDEBUG
#endif

#include "chassis/control/chassis_control.h"

#include "iswv/fake_transport.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using chassis::ChassisController;
using chassis::SteeringConfigFile;

void acknowledge_sdo(iswv::FakeTransport & transport, const iswv::CanFrame & request)
{
  if (request.id < 0x600U || request.id > 0x67fU || request.size != 8) {
    return;
  }

  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = request.data[0] == 0x40 ? 0x43 : 0x60;
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  transport.inject(response);
}

template<typename Exception, typename Function>
void expect_throw(Function action)
{
  bool caught = false;
  try {
    action();
  } catch (const Exception &) {
    caught = true;
  }
  assert(caught);
}

struct RecordingHardware
{
  std::array<std::shared_ptr<iswv::FakeTransport>, 3> transports{
    std::make_shared<iswv::FakeTransport>(),
    std::make_shared<iswv::FakeTransport>(),
    std::make_shared<iswv::FakeTransport>()};
  std::vector<unsigned int> opened_buses;
  std::vector<bool> power_events;

  ChassisController::Hardware make()
  {
    for (const auto & transport : transports) {
      transport->set_send_hook(
        [transport](const iswv::CanFrame & request) {
          acknowledge_sdo(*transport, request);
        });
    }

    ChassisController::Hardware hardware;
    hardware.power_settle = std::chrono::milliseconds(0);
    hardware.open = [this](unsigned int bus) {
        opened_buses.push_back(bus);
        return bus < transports.size() ? transports[bus] : nullptr;
      };
    hardware.power = [this](bool enabled) {
        power_events.push_back(enabled);
      };
    return hardware;
  }
};

SteeringConfigFile valid_config()
{
  SteeringConfigFile config;
  config.source_path = "offline-regression";
  return config;
}

void initial_state_rejects_commands_before_hardware_is_opened()
{
  RecordingHardware hardware;
  ChassisController controller(valid_config(), hardware.make());

  assert(controller.state() == ChassisController::State::uninitialized);
  assert(controller.config().source_path == "offline-regression");

  expect_throw<std::logic_error>([&] {controller.calibrate();});
  expect_throw<std::logic_error>([&] {controller.forward();});
  expect_throw<std::logic_error>([&] {controller.set_velocity(0.1, 0.0, 0.0);});
  expect_throw<std::logic_error>([&] {controller.update();});
  expect_throw<std::logic_error>([&] {controller.stop();});

  assert(controller.state() == ChassisController::State::uninitialized);
  assert(hardware.opened_buses.empty());
  assert(hardware.power_events.empty());
}

void initialization_cancelled_before_open_faults_without_touching_hardware()
{
  RecordingHardware hardware;
  ChassisController controller(valid_config(), hardware.make());

  expect_throw<std::runtime_error>([&] {controller.initialize([] {return false;});});

  assert(controller.state() == ChassisController::State::faulted);
  assert(hardware.opened_buses.empty());
  assert(hardware.power_events.empty());
  expect_throw<std::logic_error>([&] {controller.initialize();});
}

void transport_open_failure_faults_before_power_is_enabled()
{
  RecordingHardware hardware;
  hardware.transports[1]->close();
  ChassisController controller(valid_config(), hardware.make());

  expect_throw<std::runtime_error>([&] {controller.initialize();});

  assert(controller.state() == ChassisController::State::faulted);
  assert((hardware.opened_buses == std::vector<unsigned int>{0U, 1U}));
  assert(hardware.power_events.empty());
}

void cancellation_after_power_on_powers_down_once()
{
  RecordingHardware hardware;
  {
    ChassisController controller(valid_config(), hardware.make());
    std::size_t calls = 0;
    expect_throw<std::runtime_error>(
      [&] {
        controller.initialize(
          [&] {
            ++calls;
            return calls < 2;
          });
      });

    assert(controller.state() == ChassisController::State::faulted);
  }

  assert((hardware.opened_buses == std::vector<unsigned int>{0U, 1U, 2U}));
  assert((hardware.power_events == std::vector<bool>{true, false}));
}

void initialized_destructor_stops_and_powers_down_once()
{
  RecordingHardware hardware;
  {
    ChassisController controller(valid_config(), hardware.make());
    controller.initialize();
    assert(controller.state() == ChassisController::State::uninitialized);
    expect_throw<std::logic_error>([&] {controller.stop();});
  }

  assert((hardware.opened_buses == std::vector<unsigned int>{0U, 1U, 2U}));
  assert((hardware.power_events == std::vector<bool>{true, false}));
}

void initialization_is_single_use_even_without_calibration()
{
  RecordingHardware hardware;
  ChassisController controller(valid_config(), hardware.make());

  controller.initialize();
  expect_throw<std::logic_error>([&] {controller.initialize();});
  expect_throw<std::logic_error>([&] {controller.set_velocity(0.0, 0.0, 0.0);});

  assert(controller.state() == ChassisController::State::uninitialized);
}

}  // namespace

int main()
{
  initial_state_rejects_commands_before_hardware_is_opened();
  initialization_cancelled_before_open_faults_without_touching_hardware();
  transport_open_failure_faults_before_power_is_enabled();
  cancellation_after_power_on_powers_down_once();
  initialized_destructor_stops_and_powers_down_once();
  initialization_is_single_use_even_without_calibration();
  return 0;
}
