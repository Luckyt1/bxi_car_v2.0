#include "motor/yiyou/position_memory.h"

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

namespace
{
using chassis::PositionMemory;
constexpr std::array<std::uint32_t, 4> identity{123, 456, 789, 1011};
constexpr double scale = 36000;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void rejects(Function function)
{
  try {function();} catch (const std::exception &) {return;}
  throw std::runtime_error("expected rejection");
}

struct Directory
{
  Directory()
  {
    char pattern[] = "/tmp/car2-position-memory-XXXXXX";
    const auto created = ::mkdtemp(pattern);
    if (!created) {throw std::runtime_error("mkdtemp failed");}
    path = created;
    file = path + "/state.yaml";
  }
  ~Directory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
  std::string path;
  std::string file;
};

void complete(PositionMemory & memory)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!memory.poll()) {
    if (std::chrono::steady_clock::now() > deadline) {throw std::runtime_error("write did not finish");}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(!memory.pending(), "completed write still pending");
}

void seed(const std::string & path, std::int32_t raw = 120000)
{
  PositionMemory memory(path, "eth-test", 4, identity, scale);
  memory.set_reference(raw, raw);
}

void write_yaml(const std::string & path, const YAML::Node & node)
{
  std::ofstream file(path, std::ios::trunc);
  file << node;
  if (!file) {throw std::runtime_error("cannot write test fixture");}
}

void restore_lost_turns_and_unchanged_feedback()
{
  Directory directory;
  seed(directory.file);
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    require(memory.restore(120000) == 120000, "already retained turns were added twice");
    require(memory.raw_target(120000) == 120000, "unchanged raw target mismatch");
  }
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    require(memory.restore(12000) == 120000, "120 degrees did not recover 1200 degrees");
    require(memory.logical_position(12001) == 120001, "fixed offset mismatch");
    require(memory.raw_target(120000) == 12000, "restored target was not mapped to raw drive coordinates");
    rejects([&] {memory.logical_position(12000 - 36000);});
    rejects([&] {memory.logical_position(13000);});
  }
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    require(memory.restore(12000) == 120000, "second reboot lost the restored turn count");
  }
}

void restore_mid_motion_in_both_directions()
{
  Directory directory;
  seed(directory.file);
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    memory.restore(12000);
    memory.queue_target(12000, 110000);
    require(memory.pending(), "write was not retained for polling");
    rejects([&] {memory.raw_target(110000);});
    rejects([&] {memory.mark_target(110000);});
    rejects([&] {memory.invalidate();});
    rejects([&] {memory.set_reference(0, 0);});
    rejects([&] {memory.queue_target(12000, 111000);});
    complete(memory);
    require(memory.raw_target(110000) == 2000, "reverse target mapping failed");
    memory.mark_target(110000);
  }
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    require(memory.restore(5000) == 113000, "midway negative move failed to restore");
    memory.queue_target(5000, 125000);
    complete(memory);
    memory.mark_target(125000);
  }
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    require(memory.restore(9000) == 117000, "midway positive move failed to restore");
  }
}

void cancelled_reservation_retains_last_triggered_goal()
{
  Directory directory;
  seed(directory.file, 10000);
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    memory.restore(10000);
    memory.queue_target(10000, 20000);
    complete(memory);
    memory.mark_target(20000);
    // While still moving toward 20000, reserve 13000 but cancel before triggering.
    memory.queue_target(12000, 13000);
    complete(memory);
    // The next reservation must still include the possibly executing 20000.
    memory.queue_target(12000, 14000);
    complete(memory);
    require(memory.logical_position(19000) == 19000, "cancel erased the previous accepted target");
  }
  PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
  require(memory.restore(19000) == 19000, "cancelled reservation could not recover old motion");
}

void reject_ambiguous_or_unmatched_coordinates()
{
  Directory directory;
  seed(directory.file);
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    rejects([&] {memory.restore(20000);});
    memory.restore(12000);
    rejects([&] {memory.queue_target(12000, 160000);});
    require(!memory.pending(), "oversized interval created an async job");
  }
  auto record = YAML::LoadFile(directory.file);
  record["lower"] = 0;
  record["upper"] = 36000;
  write_yaml(directory.file, record);
  PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
  rejects([&] {memory.restore(0);});
  record["upper"] = 72000;
  write_yaml(directory.file, record);
  rejects([&] {memory.restore(0);});
}

void reject_changed_device_and_scale()
{
  Directory directory;
  seed(directory.file);
  const auto attempt = [&](std::string interface, std::uint16_t slave,
      std::array<std::uint32_t, 4> device, double counts) {
      PositionMemory memory(directory.file, std::move(interface), slave, device, counts);
      rejects([&] {memory.restore(12000);});
    };
  attempt("other-interface", 4, identity, scale);
  attempt("eth-test", 3, identity, scale);
  auto other = identity;
  for (std::size_t i = 0; i < other.size(); ++i) {
    other = identity;
    ++other[i];
    attempt("eth-test", 4, other, scale);
  }
  attempt("eth-test", 4, identity, scale + 1);
  for (const auto invalid : {0.0, -1.0, 1.0, 2.0, 36000.5,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    rejects([&] {PositionMemory memory(directory.file, "eth-test", 4, identity, invalid);});
  }
}

void reject_missing_corrupt_and_invalid_state_but_allow_explicit_recovery()
{
  Directory directory;
  PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
  rejects([&] {memory.restore(0);});
  rejects([&] {memory.logical_position(0);});
  rejects([&] {memory.queue_target(0, 1000);});
  for (const auto & content : std::vector<std::string>{"", "version: [", "version: 1\n", "[]"}) {
    {std::ofstream file(directory.file); file << content;}
    rejects([&] {memory.restore(0);});
  }
  memory.set_reference(12000, 120000);
  require(memory.logical_position(12000) == 120000, "explicit reference did not repair corrupt record");
  memory.invalidate();
  require(!YAML::LoadFile(directory.file)["valid"].as<bool>(), "zero did not persist invalid marker");
  rejects([&] {memory.logical_position(12000);});
  rejects([&] {memory.restore(12000);});
  memory.set_reference(0, 0);
  require(memory.restore(0) == 0, "explicit zero did not reestablish reference");
  rejects([&] {memory.set_reference(0, 1);});
  rejects([&] {memory.logical_position(0);});
}

void reject_invalid_schema()
{
  Directory directory;
  seed(directory.file);
  const auto original = YAML::LoadFile(directory.file);
  PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
  const auto malformed = [&](const std::function<void(YAML::Node &)> & change) {
      auto record = YAML::Clone(original);
      change(record);
      write_yaml(directory.file, record);
      rejects([&] {memory.restore(12000);});
    };
  malformed([](YAML::Node & record) {record["version"] = 2;});
  malformed([](YAML::Node & record) {record["valid"] = "maybe";});
  malformed([](YAML::Node & record) {record["slave"] = -1;});
  malformed([](YAML::Node & record) {record["identity"][0] = -1;});
  malformed([](YAML::Node & record) {record["counts_per_revolution"] = 36000.5;});
  malformed([](YAML::Node & record) {record["lower"] = 9999999;});
  malformed([](YAML::Node & record) {record["unknown"] = "ignored?";});
  malformed([](YAML::Node & record) {record.remove("upper");});
  malformed([](YAML::Node & record) {
      record["lower"] = std::numeric_limits<std::int64_t>::min();
      record["upper"] = std::numeric_limits<std::int64_t>::max();
    });
  write_yaml(directory.file, original);
  {std::ofstream file(directory.file, std::ios::app); file << "\nversion: 1\n";}
  rejects([&] {memory.restore(12000);});
  write_yaml(directory.file, original);
  {std::ofstream file(directory.file, std::ios::app); file << "\n---\nversion: 1\n";}
  rejects([&] {memory.restore(12000);});
}

void lifetime_lock_and_async_failure()
{
  Directory directory;
  seed(directory.file);
  {
    PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
    rejects([&] {PositionMemory other(directory.file, "eth-test", 4, identity, scale);});
    memory.restore(12000);
    // Fail atomically replacing the destination without relying on permissions/root status.
    std::filesystem::remove(directory.file);
    std::filesystem::create_directory(directory.file);
    memory.queue_target(12000, 121000);
    rejects([&] {complete(memory);});
    rejects([&] {memory.mark_target(121000);});
    rejects([&] {memory.queue_target(12000, 122000);});
    rejects([&] {memory.logical_position(12000);});
  }
  std::filesystem::remove(directory.file);
  seed(directory.file, 0);  // The stable lock is released on destruction.
  for (const auto & entry : std::filesystem::directory_iterator(directory.path)) {
    require(entry.path().filename().string().find(".tmp.") == std::string::npos,
      "failed write leaked a temporary state file");
  }
}

void extreme_positions_and_signed_feedback()
{
  Directory directory;
  PositionMemory memory(directory.file, "eth-test", 4, identity, scale);
  memory.set_reference(-12000, -120000);
  require(memory.restore(-12000) == -120000, "negative turn recovery failed");
  memory.queue_target(-12000, -110000);
  complete(memory);
  memory.mark_target(-110000);
  require(memory.raw_target(-110000) == -2000, "negative target conversion failed");
  rejects([&] {memory.set_reference(0, std::numeric_limits<std::int64_t>::min());});
  memory.set_reference(std::numeric_limits<std::int32_t>::max(),
    std::numeric_limits<std::int32_t>::max());
  rejects([&] {memory.queue_target(std::numeric_limits<std::int32_t>::max(),
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1);});
}
}  // namespace

int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests{
    {"lost turns and unchanged feedback", restore_lost_turns_and_unchanged_feedback},
    {"mid-motion recovery in both directions", restore_mid_motion_in_both_directions},
    {"cancel preserves last triggered goal", cancelled_reservation_retains_last_triggered_goal},
    {"ambiguous and unmatched coordinates", reject_ambiguous_or_unmatched_coordinates},
    {"device and scale binding", reject_changed_device_and_scale},
    {"corruption and explicit zero recovery", reject_missing_corrupt_and_invalid_state_but_allow_explicit_recovery},
    {"schema validation", reject_invalid_schema},
    {"lifetime lock and async write failure", lifetime_lock_and_async_failure},
    {"signed and extreme positions", extreme_positions_and_signed_feedback},
  };
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception & error) {
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
      return 1;
    }
  }
  return 0;
}
