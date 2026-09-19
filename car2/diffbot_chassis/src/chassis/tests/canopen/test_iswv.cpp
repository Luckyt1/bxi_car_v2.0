#include "test.hpp"
#include "test_helpers.hpp"

#include "iswv/cia402/drive.hpp"
#include "iswv/iswv/module.hpp"
#include "iswv/iswv/steering_layout.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>

TEST_CASE("iSWV manual conversions match the manual examples")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  iswv::Axis axis(master.node(1), iswv::AxisConfiguration::manual_travel(1));

  auto velocity = axis.motor_rpm_to_raw(-100.0);
  CHECK(velocity);
  CHECK_EQ(velocity.value(), -1789570);
  CHECK_NEAR(axis.raw_to_motor_rpm(velocity.value()), -100.0, 0.001);

  auto acceleration = axis.acceleration_to_raw(100.0);
  CHECK(acceleration);
  CHECK_EQ(acceleration.value(), 107374U);
  CHECK_NEAR(axis.raw_to_acceleration(acceleration.value()), 100.0, 0.001);

  auto motor_rpm = axis.travel_mps_to_motor_rpm(1.9);
  CHECK(motor_rpm);
  CHECK_NEAR(motor_rpm.value(), 2791.2, 1.0);
}

TEST_CASE("CiA 402 status decoder ignores bit 5 where the standard marks it don't-care")
{
  using iswv::cia402::DriveState;
  using iswv::cia402::decode_state;

  CHECK_EQ(decode_state(0x0000), DriveState::not_ready_to_switch_on);
  CHECK_EQ(decode_state(0x0040), DriveState::switch_on_disabled);
  CHECK_EQ(decode_state(0x0031), DriveState::ready_to_switch_on);
  CHECK_EQ(decode_state(0x0033), DriveState::switched_on);
  CHECK_EQ(decode_state(0x0037), DriveState::operation_enabled);
  CHECK_EQ(decode_state(0x0007), DriveState::quick_stop_active);
  CHECK_EQ(decode_state(0x000F), DriveState::fault_reaction_active);
  CHECK_EQ(decode_state(0x0008), DriveState::fault);
  CHECK_EQ(decode_state(0x4238), DriveState::fault);
  CHECK_EQ(decode_state(0x4260), DriveState::switch_on_disabled);
  CHECK_EQ(decode_state(0x0022), DriveState::unknown);
}

TEST_CASE("CiA 402 transition accepts switch-on-disabled status with bit 5 set")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  auto node = master.node(1);
  iswv::cia402::Drive drive(node);
  std::uint16_t status = 0x0037;

  transport->set_send_hook(
    [transport, &status](const iswv::CanFrame & request) {
      const auto object = iswv::test::request_object(request);
      if (request.data[0] == 0x40 && object == iswv::objects::status_word.address) {
        transport->inject(iswv::test::sdo_upload_response(request, status));
        return;
      }
      if (object == iswv::objects::control_word.address) {
        const auto command = static_cast<std::uint16_t>(
          request.data[4] | (static_cast<std::uint16_t>(request.data[5]) << 8U));
        if (command == iswv::cia402::control::disable_voltage) {
          status = 0x4260;
        }
      }
      transport->inject(iswv::test::sdo_download_response(request));
    });

  CHECK(
    drive.transition_to(
      iswv::cia402::DriveState::switch_on_disabled,
      {std::chrono::milliseconds{300},
        std::chrono::milliseconds{1},
        {std::chrono::milliseconds{100}}}));
}

TEST_CASE("CiA 402 enable reports fault status with bit 5 set before issuing commands")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  auto node = master.node(1);
  iswv::cia402::Drive drive(node);
  unsigned control_writes = 0;

  transport->set_send_hook(
    [transport, &control_writes](const iswv::CanFrame & request) {
      const auto object = iswv::test::request_object(request);
      if (request.data[0] == 0x40 && object == iswv::objects::status_word.address) {
        transport->inject(iswv::test::sdo_upload_response(request, std::uint16_t{0x4238}));
        return;
      }
      if (object == iswv::objects::control_word.address) {
        ++control_writes;
      }
      transport->inject(iswv::test::sdo_download_response(request));
    });

  auto result = drive.enable(
    {std::chrono::milliseconds{300},
      std::chrono::milliseconds{1},
      {std::chrono::milliseconds{100}}});
  CHECK(!result);
  CHECK_EQ(result.error().code, iswv::ErrorCode::illegal_state);
  CHECK_EQ(control_writes, 0U);
}

TEST_CASE("CiA 402 enable follows 6, 7, F state sequence")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  auto node = master.node(1);
  iswv::cia402::Drive drive(node);
  std::mutex mutex;
  std::uint16_t status = 0x0040;

  transport->set_send_hook(
    [transport, &mutex, &status](const iswv::CanFrame & request) {
      const auto object = iswv::test::request_object(request);
      if (request.data[0] == 0x40 && object == iswv::objects::status_word.address) {
        std::lock_guard<std::mutex> lock(mutex);
        transport->inject(iswv::test::sdo_upload_response(request, status));
        return;
      }
      if (object == iswv::objects::control_word.address) {
        const auto command = static_cast<std::uint16_t>(
          request.data[4] | (static_cast<std::uint16_t>(request.data[5]) << 8U));
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (command == 0x0006) {
            status = 0x0031;
          } else if (command == 0x0007) {
            status = 0x0033;
          } else if (command == 0x000F) {
            status = 0x0037;
          }
        }
      }
      transport->inject(iswv::test::sdo_download_response(request));
    });

  CHECK(
    drive.enable(
      {std::chrono::milliseconds{300},
        std::chrono::milliseconds{1},
        {std::chrono::milliseconds{100}}}));
  auto final_state = drive.state();
  CHECK(final_state);
  CHECK_EQ(final_state.value(), iswv::cia402::DriveState::operation_enabled);
}

TEST_CASE("Module factory keeps travel and steering nodes distinct")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  auto module = iswv::Module::from_manual(master, 1, 2);
  CHECK_EQ(module->travel()->node()->id(), 1U);
  CHECK_EQ(module->steering()->node()->id(), 2U);
  CHECK_EQ(module->travel()->configuration().kind, iswv::AxisKind::travel);
  CHECK_EQ(module->steering()->configuration().kind, iswv::AxisKind::steering);

  bool rejected_duplicate = false;
  try {
    (void)iswv::Module::from_manual(master, 1, 1);
  } catch (const std::invalid_argument &) {
    rejected_duplicate = true;
  }
  CHECK(rejected_duplicate);
}

TEST_CASE("CAN0 steering layout maps four steering motors by wheel position")
{
  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster can0(transport);
  iswv::SteeringLayout steering(can0);

  CHECK_EQ(steering.at(iswv::WheelPosition::front_left)->node()->id(), 1U);
  CHECK_EQ(steering.at(iswv::WheelPosition::rear_left)->node()->id(), 2U);
  CHECK_EQ(steering.at(iswv::WheelPosition::rear_right)->node()->id(), 3U);
  CHECK_EQ(steering.at(iswv::WheelPosition::front_right)->node()->id(), 4U);
  CHECK_EQ(
    steering.at(iswv::WheelPosition::front_left)->configuration().kind,
    iswv::AxisKind::steering);
  CHECK_EQ(
    steering.at(iswv::WheelPosition::rear_left)->configuration().kind,
    iswv::AxisKind::steering);
  CHECK_EQ(
    steering.at(iswv::WheelPosition::rear_right)->configuration().kind,
    iswv::AxisKind::steering);
  CHECK_EQ(
    steering.at(iswv::WheelPosition::front_right)->configuration().kind,
    iswv::AxisKind::steering);
}

TEST_CASE("Steering layout config loads motor and wheel parameters")
{
  const std::string path = "/tmp/iswv-steering-test.ini";
  {
    std::ofstream output(path);
    output << "[motor]\nencoder_resolution=131072\ngear_ratio=9.5\n"
      "wheel_diameter_m=0.2\n[position]\nprofile_velocity_rpm=120\n"
      "acceleration_rps2=20\ndeceleration_rps2=30\n[pdo]\n"
      "tpdo_event_timer_ms=25\n[front_left]\nnode_id=11\ninverted=true\n"
      "[rear_left]\nnode_id=12\n[rear_right]\nnode_id=13\n"
      "[front_right]\nnode_id=14\n";
  }
  auto loaded = iswv::SteeringLayoutConfig::from_file(path);
  CHECK(loaded);
  CHECK_EQ(loaded.value().encoder_resolution, 131072U);
  CHECK_NEAR(loaded.value().gear_ratio, 9.5, 1e-9);
  CHECK_EQ(loaded.value().node_ids[0], 11U);
  CHECK(loaded.value().inverted[0]);
  CHECK_EQ(loaded.value().pdo.tpdo_event_timer_ms, 25U);
}

TEST_CASE("SafetyPolicy issues quick stop after supervision timeout")
{
  using namespace std::chrono_literals;

  auto transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master(transport);
  iswv::Axis axis(master.node(9), iswv::AxisConfiguration::manual_travel(9));
  auto throwing_state_observer = axis.on_state(
    [](const iswv::AxisState &) {
      throw std::runtime_error("observer failure");
    });

  std::promise<std::uint16_t> command_promise;
  auto command_future = command_promise.get_future();
  transport->set_send_hook(
    [&command_promise](const iswv::CanFrame & request) {
      if (request.id != 0x609U || request.data[0] != 0x2BU ||
      iswv::test::request_object(request) != iswv::objects::control_word.address)
      {
        return;
      }
      const auto command = static_cast<std::uint16_t>(
        request.data[4] | (static_cast<std::uint16_t>(request.data[5]) << 8U));
      command_promise.set_value(command);
    });

  iswv::SafetyPolicy policy;
  policy.enabled = true;
  policy.local_heartbeat_timeout = 15ms;
  policy.timeout_action = iswv::TimeoutAction::quick_stop;
  CHECK(axis.apply_safety_policy(policy));

  CHECK(command_future.wait_for(150ms) == std::future_status::ready);
  CHECK_EQ(command_future.get(), iswv::cia402::control::quick_stop);
}
