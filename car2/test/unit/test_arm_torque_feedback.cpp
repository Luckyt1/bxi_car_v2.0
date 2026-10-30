#include "arm/control.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;
constexpr std::array<std::uint32_t, 4> fourth_identity{123, 456, 789, 1011};

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
    for (std::uint8_t sub = 1; sub <= fourth_identity.size(); ++sub) {
      put<std::uint32_t>(0x1018, fourth_identity[sub - 1], sub);
    }
  }

  template<typename T>
  void put(std::uint16_t index, T value, std::uint8_t subindex = 0)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, subindex}] = {bytes.begin(), bytes.end()};
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

  std::vector<std::uint16_t> controls;
  std::vector<std::int32_t> targets;
  bool acknowledge{true};
  bool corrupt_target{false};

  iswv::Result<void> write(ObjectAddress object, const std::uint8_t * data, std::size_t size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    values_[object] = {data, data + size};
    if (object.index == 0x607A) {
      targets.push_back(static_cast<std::int32_t>(raw(data, size)));
      if (corrupt_target) {put<std::int32_t>(0x607A, 123);}
    }
    if (object.index == 0x6060) {
      put<std::int8_t>(0x6061, static_cast<std::int8_t>(raw(data, size)));
    }
    if (object.index == 0x6040) {
      controls.push_back(static_cast<std::uint16_t>(raw(data, size)));
      std::uint16_t status = 0x40;
      switch (raw(data, size) & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (acknowledge && (raw(data, size) & 0x10U)) ? 0x1027 : 0x27; break;
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

  std::optional<std::int16_t> cached_torque_permille() const override {return torque_permille;}
  std::optional<std::uint32_t> cached_rated_torque_millinewton_meter() const override
  {
    return rated_torque_mnm;
  }

  std::uint16_t id_;
  Dictionary dictionary;
  std::optional<std::int16_t> torque_permille;
  std::optional<std::uint32_t> rated_torque_mnm;
};

chassis::ArmMotionConfig torque_config(bool enabled)
{
  chassis::ArmMotionConfig config;
  config.monitor_torque = enabled;
  config.kinematics.upper_arm = 0.3;
  config.kinematics.forearm = 0.3;
  config.kinematics.tool = 0.1;
  return config;
}

void controller_reports_four_axis_optional_torque()
{
  std::array<std::shared_ptr<CoeNode>, 4> nodes{
    std::make_shared<CoeNode>(1), std::make_shared<CoeNode>(2),
    std::make_shared<CoeNode>(3), std::make_shared<CoeNode>(4)};
  nodes[0]->torque_permille = static_cast<std::int16_t>(10);
  nodes[0]->rated_torque_mnm = 1000U;
  nodes[1]->torque_permille = static_cast<std::int16_t>(-20);
  nodes[1]->rated_torque_mnm = 2000U;
  nodes[3]->torque_permille = static_cast<std::int16_t>(40);

  std::vector<chassis::ArmTorqueSample> samples;
  chassis::ArmController controller(
    {nodes[0], nodes[1], nodes[2]}, {}, nodes[3], torque_config(true));
  controller.set_torque_observer(
    [&](const chassis::ArmTorqueSample & sample) {
      samples.push_back(sample);
    });

  controller.initialize();
  require(!samples.empty(), "initialize/update publishes torque samples when enabled");
  const auto & sample = samples.back();
  require(sample.permille[0] == 10, "axis 1 torque is reported");
  require(sample.rated_mnm[0] == 1000U, "axis 1 rated torque is reported");
  require(sample.permille[1] == -20, "axis 2 signed torque preserves negative values");
  require(!sample.permille[2], "missing axis 3 torque stays unknown");
  require(sample.permille[3] == 40, "axis 4 torque is reported when present");
  require(!sample.rated_mnm[3], "missing axis 4 rated torque stays unknown");
  controller.stop();
}

void controller_observer_is_optional_and_isolated()
{
  std::array<std::shared_ptr<CoeNode>, 3> nodes{
    std::make_shared<CoeNode>(1), std::make_shared<CoeNode>(2),
    std::make_shared<CoeNode>(3)};
  unsigned calls = 0;
  chassis::ArmController controller({nodes[0], nodes[1], nodes[2]}, {}, nullptr, torque_config(true));
  controller.set_torque_observer(
    [&](const chassis::ArmTorqueSample &) {
      ++calls;
      throw std::runtime_error("observer failed");
    });
  controller.initialize();
  require(calls > 0, "observer was called");
  require(controller.fault_reason().empty(), "observer failure must not latch arm fault");
  controller.stop();
}

void disabled_monitor_does_not_call_observer()
{
  std::array<std::shared_ptr<CoeNode>, 3> nodes{
    std::make_shared<CoeNode>(1), std::make_shared<CoeNode>(2),
    std::make_shared<CoeNode>(3)};
  unsigned calls = 0;
  chassis::ArmController controller({nodes[0], nodes[1], nodes[2]}, {}, nullptr, torque_config(false));
  controller.set_torque_observer(
    [&](const chassis::ArmTorqueSample &) {
      ++calls;
    });
  controller.initialize();
  require(calls == 0, "disabled torque monitor does not publish samples");
  controller.stop();
}

struct FourthFixture
{
  std::array<std::shared_ptr<CoeNode>, 4> nodes{
    std::make_shared<CoeNode>(1), std::make_shared<CoeNode>(2),
    std::make_shared<CoeNode>(3), std::make_shared<CoeNode>(4)};
  bool power_off{false};
  std::size_t controls_at_power_off{0};
  chassis::ArmController controller;
  std::chrono::steady_clock::time_point now{};

  explicit FourthFixture(
    std::int32_t position = 60000, chassis::ArmMotionConfig config = torque_config(false),
    bool initialize = true)
  : controller({nodes[0], nodes[1], nodes[2]}, [this] {
      power_off = true;
      controls_at_power_off = nodes[3]->dictionary.controls.size();
    }, nodes[3], config)
  {
    nodes[3]->dictionary.put<std::int32_t>(0x2025, 36000);  // 100 counts/degree
    set_position(position);
    if (initialize) {controller.initialize();}
    now = std::chrono::steady_clock::now();
  }
  template<typename Predicate>
  void wait_until(Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate()) {
      require(std::chrono::steady_clock::now() < deadline, "controller progress timed out");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      const auto real_now = std::chrono::steady_clock::now();
      if (real_now <= now) {continue;}
      now = real_now;
      controller.update(now);
    }
  }
  Dictionary & drive() {return nodes[3]->dictionary;}
  void set_position(std::int32_t position) {drive().put<std::int32_t>(0x6064, position);}
  void ticks(int count = 1)
  {
    for (int i = 0; i < count; ++i) {
      now += std::chrono::milliseconds(20);
      controller.update(now);
    }
  }
  void require_hold()
  {
    require(controller.fourth_axis_jog() == 0, "zero jog after halt");
    require((drive().controls.back() & 0x11f) == 0x10f,
      "halt holds enable and clears set-point trigger");
  }
};

void controller_fourth_axis_position_limits()
{
  struct Case {std::int32_t position; int direction; std::int32_t target;};
  for (const auto & test : std::array<Case, 10>{{
      {119999, 1, 120000}, {120000, 1, 120000}, {120001, 1, 120001},
      {123000, 1, 123000}, {123000, -1, 119400}, {130000, -1, 126400},
      {-1, -1, -1}, {-5000, 1, -1400}, {0, -1, 0}, {1, -1, 0}}})
  {
    FourthFixture f(test.position);
    require(f.drive().targets.back() == test.position, "startup holds actual, never clamps or homes");
    f.require_hold();
    f.controller.set_fourth_axis_jog(test.direction);
    f.ticks(5);
    require(f.drive().targets.back() == test.target, "bounded PP target at/around software limits");
    if (test.position == test.target) {f.require_hold();}
  }
  FourthFixture f(119999);
  f.controller.set_fourth_axis_jog(1);
  f.ticks(2); // staged, but not yet triggered
  f.set_position(120001);
  f.ticks();
  f.require_hold();
  const auto diagnostic = f.controller.take_feedback_diagnostic(f.now + std::chrono::seconds(1));
  require(diagnostic.find("positive_inhibited=1") != std::string::npos, "upper limit reported");
  require(diagnostic.find("negative_inhibited=0") != std::string::npos, "upper allows recovery");
  f.controller.set_fourth_axis_jog(-1);
  f.ticks(5);
  require(f.drive().targets.back() < 120001, "reverse recovery accepted");
  f.set_position(-1);
  f.ticks();
  f.require_hold();
}

void controller_fourth_axis_cancels_and_does_not_accumulate()
{
  // Cancel idle/prepared/staged/triggered/resetting/accepted commands on release.
  for (int stage = 0; stage <= 5; ++stage) {
    FourthFixture f;
    f.controller.set_fourth_axis_jog(1);
    f.ticks(stage);
    f.controller.set_fourth_axis_jog(0);
    f.require_hold();
    const auto writes = f.drive().controls.size();
    f.ticks(10);
    require(f.drive().controls.size() == writes, "release never revives pending target");
    f.controller.set_fourth_axis_jog(-1);
    f.ticks(2);
    require((f.drive().controls.back() & 0x100) != 0, "reversal stages with halt retained");
    require(f.drive().targets.back() == 56400, "new target based on current feedback");
    f.ticks();
    require(f.drive().controls.back() == 0x3f, "only fresh target trigger clears halt");
  }
  FourthFixture f;
  f.controller.set_fourth_axis_jog(1);
  f.ticks(100);
  require(f.drive().targets.back() == 63600, "jam cannot accumulate targets ahead of feedback");
  require(f.drive().targets.size() == 3, "unchanged target is not repeatedly triggered"); // seed + hold + jog
  f.controller.set_fourth_axis_jog(-1);
  require((f.drive().controls.back() & 0x100) != 0, "direct reversal first halts");
  f.ticks(5);
  require(f.drive().targets.back() == 56400, "direct reversal discards old target");
}

void controller_fourth_axis_faults()
{
  for (int fault = 0; fault < 4; ++fault) {
    FourthFixture f;
    if (fault == 0) {f.drive().acknowledge = false;}
    if (fault == 1) {f.drive().corrupt_target = true;}
    if (fault == 2) {
      f.controller.set_fourth_axis_jog(1);
      f.drive().put<std::int8_t>(0x6061, 10);
    }
    if (fault == 3) {f.drive().put<std::int32_t>(0x606C, 36000);}
    const auto controls_before_fault = f.drive().controls.size();
    bool threw = false;
    try {
      if (fault != 2) {f.controller.set_fourth_axis_jog(1);}
      f.ticks(60);
    } catch (const std::exception &) {threw = true;}
    require(threw, "timeout, bad readback, wrong mode and overspeed fault");
    require(f.power_off, "fourth-axis fault cuts shared power");
    if (fault == 2) {
      require(f.controls_at_power_off == controls_before_fault,
        "feedback mode error powers off before any stop cleanup");
    }
    require(!f.controller.fault_reason().empty(), "fault stays latched");
  }
  for (const auto speed : {0.0, -1.0, 60.0, std::numeric_limits<double>::infinity()}) {
    auto config = torque_config(false);
    config.fourth_speed_rpm = speed;
    bool rejected = false;
    try {chassis::validate_arm_motion_config(config);} catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "invalid fourth speed rejected before hardware");
  }
}

struct MemoryDirectory
{
  MemoryDirectory()
  {
    char pattern[] = "/tmp/car2-arm-position-memory-XXXXXX";
    const auto directory = ::mkdtemp(pattern);
    if (!directory) {throw std::runtime_error("cannot create position-memory test directory");}
    path = directory;
    file = path + "/position.yaml";
  }
  ~MemoryDirectory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
  chassis::ArmMotionConfig config() const
  {
    auto result = torque_config(false);
    result.fourth_position_memory_file = file;
    result.ethercat_interface = "eth-test";
    return result;
  }
  void seed(std::int32_t logical = 120000) const
  {
    chassis::PositionMemory memory(file, "eth-test", 4, fourth_identity, 36000);
    memory.set_reference(logical, logical);
  }
  std::string path;
  std::string file;
};

std::size_t trigger_count(const Dictionary & dictionary)
{
  return static_cast<std::size_t>(std::count(
    dictionary.controls.begin(), dictionary.controls.end(), 0x3f));
}

void controller_fourth_axis_restores_turns_and_limits()
{
  MemoryDirectory directory;
  directory.seed();
  {
    FourthFixture f(12000, directory.config());
    require(std::all_of(f.drive().targets.begin(), f.drive().targets.end(),
      [](std::int32_t value) {return value == 12000;}),
      "restoring 1200 degrees must hold raw 120 degrees without a jump");
    f.require_hold();
    const auto diagnostic = f.controller.take_feedback_diagnostic(f.now + std::chrono::seconds(1));
    require(diagnostic.find("actual_position_degrees=1200.0000") != std::string::npos,
      "diagnostic reports restored cumulative angle");
    require(diagnostic.find("raw_position_pulses=12000") != std::string::npos,
      "diagnostic retains the distinct raw encoder coordinate");
    require(diagnostic.find("positive_inhibited=1") != std::string::npos,
      "restored cumulative upper limit inhibits positive jog");
    const auto triggers = trigger_count(f.drive());
    f.controller.set_fourth_axis_jog(1);
    f.ticks(5);
    f.require_hold();
    require(trigger_count(f.drive()) == triggers, "upper limit must not trigger a new move");
    f.controller.set_fourth_axis_jog(-1);
    f.ticks();
    require(trigger_count(f.drive()) == triggers,
      "PP target is not triggered in the journal reservation phase");
    require(f.drive().targets.back() == 12000,
      "PP target is not staged before journal completion is acknowledged");
    f.wait_until([&] {return trigger_count(f.drive()) > triggers;});
    require(f.drive().targets.back() == 8400,
      "1164 degree logical target maps to raw 84 degrees after recovery");
    // Feedback halfway through the reserved move must remain recoverable even
    // though no new target or periodic position snapshot was written there.
    f.set_position(10000);
    f.controller.set_fourth_axis_jog(0);
    f.require_hold();
  }
  FourthFixture rebooted(10000, directory.config());
  require(rebooted.drive().targets.back() == 10000, "second startup still holds the raw feedback");
  const auto diagnostic = rebooted.controller.take_feedback_diagnostic(
    rebooted.now + std::chrono::seconds(1));
  require(diagnostic.find("actual_position_degrees=1180.0000") != std::string::npos,
    "restart recovers intermediate actual position, not the prior target");
}

void controller_fourth_axis_cancels_journal_and_waits_for_stop()
{
  MemoryDirectory directory;
  directory.seed(60000);
  FourthFixture f(24000, directory.config());
  const auto triggers = trigger_count(f.drive());
  const auto targets = f.drive().targets.size();
  f.controller.set_fourth_axis_jog(1);
  f.ticks(); // Reservation is queued, but completion has not been acknowledged.
  f.controller.set_fourth_axis_jog(0);
  f.require_hold();
  const auto controls_after_release = f.drive().controls.size();
  const auto settle_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  f.wait_until([&] {return std::chrono::steady_clock::now() >= settle_until;});
  require(f.drive().controls.size() == controls_after_release,
    "completed disk write cannot revive a released command");
  require(f.drive().targets.size() == targets, "cancelled reservation never reaches target register");
  require(trigger_count(f.drive()) == triggers, "cancelled reservation never triggers motion");

  f.drive().put<std::int8_t>(0x2707, 0);
  f.controller.set_fourth_axis_jog(-1);
  f.ticks(5);
  require(f.drive().targets.size() == targets,
    "reverse jog waits while the motor-stopped flag is false");
  f.drive().put<std::int8_t>(0x2707, 1);
  f.drive().put<std::int32_t>(0x606C, 600); // 1 RPM despite the stopped flag.
  f.ticks(5);
  require(f.drive().targets.size() == targets,
    "reverse jog also requires near-zero measured velocity");
  f.drive().put<std::int32_t>(0x606C, 0);
  f.wait_until([&] {return trigger_count(f.drive()) > triggers;});
  require(f.drive().targets.back() == 20400,
    "after stopping, reverse target uses restored actual position and raw offset");
}

void controller_fourth_axis_rejects_untrusted_memory_before_enabling()
{
  for (const bool corrupt : {false, true}) {
    MemoryDirectory directory;
    if (corrupt) {
      std::ofstream record(directory.file);
      record << "version: [truncated\n";
      require(bool(record), "corrupt fixture was written");
    }
    FourthFixture f(12000, directory.config(), false);
    bool rejected = false;
    try {f.controller.initialize();} catch (const std::exception &) {rejected = true;}
    require(rejected, "missing or corrupt turn memory must reject initialization");
    require(f.power_off, "untrusted position requests shared power off");
    require(f.controls_at_power_off == 0, "power off precedes motor cleanup writes");
    require(!f.controller.fault_reason().empty(), "memory initialization fault stays latched");
    for (const auto & node : f.nodes) {
      require(node->dictionary.targets.empty(), "no axis receives a target with untrusted memory");
      require(std::none_of(node->dictionary.controls.begin(), node->dictionary.controls.end(),
        [](std::uint16_t control) {return (control & 0x0f) == 0x0f;}),
        "no axis is enabled before fourth-axis memory validation");
    }
  }
}
}  // namespace

int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests{
    {"Arm reports four-axis optional torque", controller_reports_four_axis_optional_torque},
    {"Arm torque observer is isolated", controller_observer_is_optional_and_isolated},
    {"Arm torque monitor can be disabled", disabled_monitor_does_not_call_observer},
    {"Fourth axis 0/1200 degree limits and reverse recovery", controller_fourth_axis_position_limits},
    {"Fourth axis cancels and bounds targets", controller_fourth_axis_cancels_and_does_not_accumulate},
    {"Fourth axis faults and speed validation", controller_fourth_axis_faults},
    {"Fourth axis restores turns, raw targets and software limits",
      controller_fourth_axis_restores_turns_and_limits},
    {"Fourth axis journal cancellation and stopped reversal",
      controller_fourth_axis_cancels_journal_and_waits_for_stop},
    {"Fourth axis rejects untrusted memory before enabling",
      controller_fourth_axis_rejects_untrusted_memory_before_enabling},
  };
  unsigned failed = 0;
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception & error) {
      ++failed;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  return failed == 0 ? 0 : 1;
}
