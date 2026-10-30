#include "chassis/control.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using iswv::CanFrame;
using iswv::canopen::ObjectAddress;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename T>
void encode(CanFrame & frame, std::size_t offset, T value)
{
  const auto bytes = iswv::canopen::encode_little_endian(value);
  std::copy(bytes.begin(), bytes.end(), frame.data.begin() + offset);
}

struct Write
{
  std::uint16_t index;
  std::uint32_t value;
};

enum class Scenario
{
  healthy, missing_initial_errors, drive_error, lost_current,
  initial_over_torque, moving_over_torque, complete
};

// Four device-side dictionaries respond to actual SDO requests/controlwords.
// PDO production is independent of reads, just as on the configured drives.
class Devices
{
public:
  explicit Devices(Scenario scenario) : scenario_(scenario)
  {
    for (auto & node : nodes_) {
      put(node, {0x6041, 0}, std::uint16_t{0x40});
      put(node, {0x6060, 0}, std::int8_t{0});
      put(node, {0x6063, 0}, std::int32_t{0});
      put(node, {0x6078, 0}, std::int16_t{0});
      put(node, {0x2601, 0}, std::uint16_t{0});
      put(node, {0x2602, 0}, std::uint16_t{0});
      put(node, {0x60F9, 1}, std::uint16_t{1});
      put(node, {0x60F9, 2}, std::uint16_t{0});
      put(node, {0x60F9, 5}, std::uint8_t{0});
      put(node, {0x60FB, 1}, std::uint16_t{1});
      put(node, {0x60FB, 5}, std::uint16_t{1});
    }
    transport->set_send_hook([this](const CanFrame & request) {respond(request);});
  }

  ~Devices() {stop();}

  void start()
  {
    producer_ = std::thread([this] {
      while (!stop_.load()) {
        publish();
        std::this_thread::sleep_for(5ms);
      }
    });
  }

  void stop()
  {
    stop_.store(true);
    if (producer_.joinable()) {producer_.join();}
  }

  unsigned motion_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return motion_count_;
  }

  void verify_stopped(bool expected_motion) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & node : nodes_) {
      require(node.moved == expected_motion, "unexpected nonzero search target");
      require(node.writes.size() >= 3, "axis cleanup must send all three stop commands");
      const auto end = node.writes.size();
      require(node.writes[end - 3].index == 0x60FF && node.writes[end - 3].value == 0,
        "axis cleanup must clear target velocity");
      require(node.writes[end - 2].index == 0x6040 &&
        node.writes[end - 2].value == iswv::cia402::control::quick_stop,
        "axis cleanup must quick stop");
      require(node.writes[end - 1].index == 0x6040 &&
        node.writes[end - 1].value == iswv::cia402::control::shutdown,
        "axis cleanup must disable operation");
    }
    require(moving_sdo_reads_ == 0, "moving calibration must not poll feedback over SDO");
  }

  void verify_centered(const chassis::SteeringCalibrationResult & result) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      const auto & node = nodes_[i];
      require(result.negative_limits[i] == -300 && result.positive_limits[i] == 300 &&
        result.midpoints[i] == 0, "calibration must find both limits and their midpoint");
      require(node.moved && node.reversed && node.position_started,
        "each axis must search, reverse and receive the midpoint command");
      require(node.position == 0 && node.position_target == 0 && node.mode == 1 && node.status == 0x27,
        "successful calibration must leave every axis enabled and holding the midpoint");
    }
    require(moving_sdo_reads_ == 0, "moving calibration must not poll feedback over SDO");
  }

  std::shared_ptr<iswv::FakeTransport> transport = std::make_shared<iswv::FakeTransport>();

private:
  struct Node
  {
    std::map<ObjectAddress, std::vector<std::uint8_t>> dictionary;
    std::vector<Write> writes;
    std::uint16_t status{0x40};
    bool operational{false};
    bool moved{false};
    bool reversed{false};
    bool position_started{false};
    std::int8_t mode{0};
    std::int32_t position{0};
    std::int32_t velocity_target{0};
    std::int32_t position_target{0};
  };

  template<typename T>
  static void put(Node & node, ObjectAddress object, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    node.dictionary[object] = {bytes.begin(), bytes.end()};
  }

  void respond(const CanFrame & request)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request.id == 0 && request.size == 2 && request.data[1] >= 1 && request.data[1] <= 4) {
      nodes_[request.data[1] - 1].operational = request.data[0] == 1;
      return;
    }
    if (request.id >= 0x201 && request.id <= 0x204 && request.size == 7) {
      auto & node = nodes_[request.id - 0x201];
      const auto velocity = iswv::canopen::decode_little_endian<std::int32_t>(
        request.data.data() + 3, 4).value();
      node.velocity_target = velocity;
      if (velocity != 0) {
        node.moved = true;
        if (velocity > 0) {node.reversed = true;}
        ++motion_count_;
      }
      return;
    }
    if (request.id >= 0x301 && request.id <= 0x304 && request.size == 6) {
      auto & node = nodes_[request.id - 0x301];
      node.position_target = iswv::canopen::decode_little_endian<std::int32_t>(
        request.data.data() + 2, 4).value();
      if ((request.data[0] & 0x10U) != 0) {node.position_started = true;}
      return;
    }
    if (request.id < 0x601 || request.id > 0x604 || request.size != 8) {return;}
    auto & node = nodes_[request.id - 0x601];
    const ObjectAddress object{
      static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8U)), request.data[3]};
    CanFrame reply;
    reply.id = request.id - 0x80;
    reply.size = 8;
    std::copy_n(request.data.begin() + 1, 3, reply.data.begin() + 1);
    if (request.data[0] == 0x40) {
      // Switching to midpoint control still verifies its configured mode.
      if (motion_count_ != 0 && object.index != 0x6060) {++moving_sdo_reads_;}
      const auto found = node.dictionary.find(object);
      if (found == node.dictionary.end()) {
        reply.data[0] = 0x80;
        encode(reply, 4, std::uint32_t{0x06020000});
      } else {
        reply.data[0] = static_cast<std::uint8_t>(0x43 | ((4 - found->second.size()) << 2));
        std::copy(found->second.begin(), found->second.end(), reply.data.begin() + 4);
      }
    } else if ((request.data[0] & 0xE3) == 0x23) {
      const auto size = 4U - ((request.data[0] >> 2U) & 3U);
      node.dictionary[object] = {request.data.begin() + 4, request.data.begin() + 4 + size};
      std::uint32_t value = 0;
      for (unsigned i = 0; i < size; ++i) {value |= std::uint32_t{request.data[4 + i]} << (i * 8U);}
      node.writes.push_back({object.index, value});
      if (object.index == 0x6060) {node.mode = static_cast<std::int8_t>(value);}
      if (object.index == 0x60FF) {node.velocity_target = static_cast<std::int32_t>(value);}
      if (object.index == 0x6040) {
        switch (value & 0x0F) {
          case 0x06: node.status = 0x21; break;
          case 0x07: node.status = 0x23; break;
          case 0x0F: node.status = 0x27; break;
          case 0x0B: node.status = 0x07; break;
          default: node.status = 0x40; break;
        }
        put(node, {0x6041, 0}, node.status);
      }
      reply.data[0] = 0x60;
    } else {
      reply.data[0] = 0x80;
      encode(reply, 4, std::uint32_t{0x05040001});
    }
    transport->inject(reply);
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool moving = std::all_of(nodes_.begin(), nodes_.end(), [](const Node & n) {return n.moved;});
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      auto & node = nodes_[i];
      CanFrame heartbeat;
      heartbeat.id = 0x701 + static_cast<std::uint32_t>(i);
      heartbeat.size = 1;
      heartbeat.data[0] = node.operational ? 5 : 127;
      transport->inject(heartbeat);
      if (!node.operational) {continue;}
      std::int16_t current = 0;
      if (scenario_ == Scenario::complete && node.status == 0x27) {
        // Test-only fixed-step plant, with stops at +/-300 encoder counts.
        if (node.mode == 3 && node.velocity_target != 0) {
          node.position = std::clamp(
            node.position + (node.velocity_target > 0 ? 50 : -50), -300, 300);
          if ((node.position == -300 && node.velocity_target < 0) ||
            (node.position == 300 && node.velocity_target > 0)) {current = 100;}
        } else if (node.mode == 1 && node.position_started) {
          node.position += std::clamp(node.position_target - node.position, -50, 50);
        }
      }
      if (i == 1 && (scenario_ == Scenario::initial_over_torque ||
        (moving && scenario_ == Scenario::moving_over_torque))) {current = 500;}
      const auto now = Clock::now();
      for (unsigned number = 1; number <= 3; ++number) {
        if (i == 1 && number == 3 && scenario_ == Scenario::missing_initial_errors) {continue;}
        if (i == 1 && number == 2 && moving && scenario_ == Scenario::lost_current) {continue;}
        CanFrame frame;
        frame.id = 0x180U + (number - 1) * 0x100U + static_cast<std::uint32_t>(i + 1);
        frame.size = number == 3 ? 8 : 6;
        frame.received_at = now;
        if (number == 1) {
          encode(frame, 0, node.status);
          encode(frame, 2, node.position);
        }
        if (number == 2) {encode(frame, 4, current);}
        if (i == 1 && number == 3 && moving && scenario_ == Scenario::drive_error) {
          encode(frame, 4, std::uint16_t{0x0800});
        }
        transport->inject(frame);
      }
    }
  }

  Scenario scenario_;
  mutable std::mutex mutex_;
  std::array<Node, 4> nodes_{};
  unsigned motion_count_{0};
  unsigned moving_sdo_reads_{0};
  std::atomic<bool> stop_{false};
  std::thread producer_;
};

void run_case(Scenario scenario, const std::string & expected_error, bool motion)
{
  chassis::SteeringConfigFile config;
  Devices devices(scenario);
  iswv::canopen::CanopenMaster master(devices.transport);
  chassis::SteeringAxes axes(master, config);
  devices.start();
  const auto deadline = Clock::now() + 3s;
  std::string error;
  try {
    (void)chassis::calibrate_steering_with_limits(axes, config, [&] {
      if (Clock::now() >= deadline) {return false;}
      return scenario != Scenario::healthy || devices.motion_count() < 16;
    });
  } catch (const std::runtime_error & failure) {
    error = failure.what();
  }
  devices.stop();
  if (error.find(expected_error) == std::string::npos) {
    throw std::runtime_error("unexpected calibration result: " + error);
  }
  require(Clock::now() < deadline, "test escaped only through its watchdog");
  devices.verify_stopped(motion);
}

void full_calibration_succeeds()
{
  chassis::SteeringConfigFile config;
  // A short simulated travel range with generous simulated speed headroom;
  // no deployment configuration or production safety threshold is changed.
  config.motor.encoder_resolution = 1000;
  config.motor.gear_ratio = 1;
  config.calibration_speed_rpm = 10000;
  config.calibration_position_epsilon_inc = 1;
  Devices devices(Scenario::complete);
  iswv::canopen::CanopenMaster master(devices.transport);
  chassis::SteeringAxes axes(master, config);
  devices.start();
  const auto deadline = Clock::now() + 3s;
  const auto result = chassis::calibrate_steering_with_limits(
    axes, config, [&] {return Clock::now() < deadline;});
  devices.stop();
  devices.verify_centered(result);
}
}  // namespace

int main()
{
  try {
    run_case(Scenario::healthy, "cancelled", true);
    run_case(Scenario::missing_initial_errors, "TPDO3", false);
    run_case(Scenario::drive_error, "error1=0x800", true);
    run_case(Scenario::lost_current, "校准反馈无效", true);
    run_case(Scenario::initial_over_torque, "达到安全上限", false);
    run_case(Scenario::moving_over_torque, "达到安全上限", true);
    full_calibration_succeeds();
    std::cout << "steering calibration integration passed (7 cases)\n";
  } catch (const std::exception & error) {
    std::cerr << "steering calibration integration: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
