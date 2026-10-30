#include "example/arm_position_remote.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class Dictionary
{
public:
  Dictionary()
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 0);
    put<std::int32_t>(0x606C, 0);
    put<std::int32_t>(0x60FF, 0);
    put<std::int16_t>(0x6071, 0);
    put<std::int16_t>(0x6078, 0);
    put<std::int16_t>(0x60B2, 0);
    put<std::int32_t>(0x2025, 1000);
    put<std::uint16_t>(0x26A2, 1);
    put<std::uint16_t>(0x26A3, 1);
  }

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  iswv::Result<SdoResponse> read(ObjectAddress object)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = values_.find(object);
    if (found == values_.end()) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    return iswv::Result<SdoResponse>::success(response);
  }

  iswv::Result<void> write(ObjectAddress object, const std::uint8_t * data, std::size_t size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    values_[object] = {data, data + size};
    if (object.index == 0x6060) {
      put<std::int8_t>(0x6061, static_cast<std::int8_t>(raw(data, size)));
    }
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (raw(data, size) & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (raw(data, size) & 0x10U) ? 0x1027 : 0x27; break;
        default: break;
      }
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

private:
  static std::uint32_t raw(const std::uint8_t * data, std::size_t size)
  {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {
      value |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    return value;
  }

  mutable std::mutex mutex_;
  std::map<ObjectAddress, std::vector<std::uint8_t>> values_;
};

class CoeNode final : public chassis::ethercat::Node
{
public:
  explicit CoeNode(std::uint16_t id) : id_(id) {}
  std::uint16_t id() const noexcept override {return id_;}

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    return dictionary.read(object);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    return dictionary.write(object, data, size);
  }

  std::uint16_t id_;
  Dictionary dictionary;
};

class Fixture
{
public:
  Fixture()
  {
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      nodes_[i] = std::make_shared<CoeNode>(static_cast<std::uint16_t>(i + 1));
    }
    nodes_[3]->dictionary.put<std::int32_t>(0x2025, 36000);
    nodes_[3]->dictionary.put<std::int32_t>(0x6064, 10000);
    chassis::ArmMotionConfig config;
    config.kinematics.upper_arm = 0.3;
    config.kinematics.forearm = 0.3;
    config.kinematics.tool = 0.1;
    controller = std::make_unique<chassis::ArmController>(
      std::array<std::shared_ptr<chassis::ethercat::Node>, 3>{nodes_[0], nodes_[1], nodes_[2]},
      std::function<void()>{}, nodes_[3], config);
    controller->initialize();
    node_ = std::make_shared<rclcpp::Node>("arm_position_remote_test",
      rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
        rclcpp::Parameter("arm_position_topic", "/arm_position_remote_test/jog")}));
    remote = std::make_unique<chassis::ArmPositionRemote>(*node_, *controller);
    publisher_ = node_->create_publisher<std_msgs::msg::Int8>(
      "/arm_position_remote_test/jog",
      rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)));
    executor_.add_node(node_);
  }

  ~Fixture()
  {
    executor_.remove_node(node_);
    controller->stop();
  }

  void send(int direction)
  {
    std_msgs::msg::Int8 message;
    message.data = static_cast<std::int8_t>(direction);
    publisher_->publish(message);
    executor_.spin_some();
    executor_.spin_some();
  }

  bool halted() const
  {
    const auto result = nodes_[3]->read(
      iswv::canopen::Object<std::uint16_t>{{0x6040, 0}, "control word"});
    require(bool(result), "fourth-axis control word must be readable");
    return (result.value() & 0x0100U) != 0;
  }

  std::unique_ptr<chassis::ArmController> controller;
  std::unique_ptr<chassis::ArmPositionRemote> remote;

private:
  std::array<std::shared_ptr<CoeNode>, 4> nodes_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void initial_hold_release_and_invalid_command()
{
  Fixture fixture;
  fixture.send(1);
  require(fixture.controller->fourth_axis_jog() == 0,
    "receiver must see a neutral command before initial position jog");
  fixture.send(0);
  fixture.send(1);
  require(fixture.controller->fourth_axis_jog() == 1, "armed receiver accepts positive jog");
  fixture.send(0);
  require(fixture.controller->fourth_axis_jog() == 0 && fixture.halted(),
    "release immediately halts and clears jog");
  fixture.send(-1);
  require(fixture.controller->fourth_axis_jog() == -1, "release rearms negative jog");
  fixture.send(2);
  require(fixture.controller->fourth_axis_jog() == 0 && fixture.halted(),
    "invalid direction immediately halts");
  fixture.send(-1);
  require(fixture.controller->fourth_axis_jog() == 0,
    "held commands cannot rearm after invalid input");
  fixture.send(0);
  fixture.send(-1);
  require(fixture.controller->fourth_axis_jog() == -1,
    "neutral then held command rearms after invalid input");
}

void timeout_cancels_handshake_and_requires_release()
{
  Fixture fixture;
  fixture.send(0);
  fixture.send(1);
  // Put the controller into a pending PP handshake without triggering a target.
  auto now = chassis::ArmPositionRemote::Clock::now() + 20ms;
  fixture.controller->update(now);
  fixture.controller->update(now + 20ms);
  fixture.remote->update(now + 400ms);
  require(fixture.controller->fourth_axis_jog() == 0 && fixture.halted(),
    "300 ms lease expiry immediately halts an in-flight position handshake");
  fixture.controller->update(now + 420ms);
  require(fixture.halted(), "controller update must not trigger a target after lease expiry");
  fixture.send(1);
  require(fixture.controller->fourth_axis_jog() == 0,
    "held refresh after timeout must not resume motion");
  fixture.send(0);
  fixture.send(1);
  require(fixture.controller->fourth_axis_jog() == 1,
    "release then press must rearm timed-out receiver");
}

void callback_cannot_renew_an_expired_lease()
{
  Fixture fixture;
  fixture.send(0);
  fixture.send(1);
  // Deliberately do not call update: a newly received command must notice expiry itself.
  std::this_thread::sleep_for(320ms);
  fixture.send(1);
  require(fixture.controller->fourth_axis_jog() == 0 && fixture.halted(),
    "a delayed callback must halt rather than renew an expired held command");
  fixture.send(0);
  fixture.send(-1);
  require(fixture.controller->fourth_axis_jog() == -1,
    "release remains sufficient to recover after a delayed executor");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    initial_hold_release_and_invalid_command();
    timeout_cancels_handshake_and_requires_release();
    callback_cannot_renew_an_expired_lease();
    std::cout << "arm position remote lease tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "arm position remote lease: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
