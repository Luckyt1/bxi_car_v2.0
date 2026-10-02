#include "example/joystick_link_monitor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Predicate>
chassis::JoystickLinkMonitor::Snapshot wait_for(
  const chassis::JoystickLinkMonitor & monitor,
  Predicate predicate,
  const char * message,
  std::chrono::milliseconds timeout = 1500ms)
{
  const auto deadline = chassis::JoystickLinkMonitor::Clock::now() + timeout;
  while (chassis::JoystickLinkMonitor::Clock::now() < deadline) {
    const auto snapshot = monitor.snapshot();
    if (predicate(snapshot)) {return snapshot;}
    std::this_thread::sleep_for(5ms);
  }
  throw std::runtime_error(message);
}

void healthy_static_probe_reports_alive()
{
  chassis::JoystickLinkMonitor monitor([] {return true;});
  const auto snapshot = wait_for(
    monitor,
    [](const auto & state) {return state.healthy && state.generation == 1;},
    "static true probe did not become healthy");
  require(snapshot.reason == "ok", "healthy snapshot must report ok");
}

void stopped_answers_become_unhealthy()
{
  std::atomic<bool> answering{true};
  chassis::JoystickLinkMonitor monitor([&] {return answering.load();});
  const auto healthy = wait_for(
    monitor, [](const auto & state) {return state.healthy;},
    "probe did not become healthy before stop");
  answering.store(false);
  const auto unhealthy = wait_for(
    monitor,
    [&healthy](const auto & state) {
      return !state.healthy && state.generation == healthy.generation;
    },
    "probe did not become unhealthy after stop");
  require(unhealthy.reason == "probe returned false", "false probe reason must be readable");
}

void blocking_probe_does_not_block_snapshot()
{
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false;
  bool release = false;
  chassis::JoystickLinkMonitor monitor(
    [&] {
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      cv.notify_all();
      cv.wait(lock, [&] {return release;});
      return false;
    });

  {
    std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock, 500ms, [&] {return entered;}), "probe did not start");
  }

  const auto before = chassis::JoystickLinkMonitor::Clock::now();
  const auto snapshot = monitor.snapshot();
  const auto elapsed = chassis::JoystickLinkMonitor::Clock::now() - before;
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  cv.notify_all();
  require(elapsed < 50ms, "snapshot must not wait for the blocking probe");
  require(!snapshot.healthy, "blocked first probe must not allow motion");
}

void generation_changes_on_recovery()
{
  std::atomic<bool> answering{true};
  chassis::JoystickLinkMonitor monitor([&] {return answering.load();});
  const auto first = wait_for(
    monitor, [](const auto & state) {return state.healthy;},
    "probe did not become healthy");
  answering.store(false);
  wait_for(
    monitor, [](const auto & state) {return !state.healthy;},
    "probe did not become unhealthy");
  answering.store(true);
  const auto recovered = wait_for(
    monitor,
    [&first](const auto & state) {return state.healthy && state.generation > first.generation;},
    "probe did not recover with a new generation");
  require(recovered.reason == "ok", "recovered snapshot must report ok");
}

void late_success_does_not_revive_link()
{
  std::atomic<bool> slow{false};
  chassis::JoystickLinkMonitor monitor(
    [&] {
      if (slow.load()) {
        std::this_thread::sleep_for(chassis::JoystickLinkMonitor::kProbeTimeout + 50ms);
      }
      return true;
    });
  const auto first = wait_for(
    monitor, [](const auto & state) {return state.healthy;},
    "probe did not become healthy before slow probe");
  slow.store(true);
  wait_for(
    monitor,
    [](const auto & state) {
      return !state.healthy && state.reason == "last successful HID probe is stale";
    },
    "stale snapshot did not fail while probe was blocked");
  wait_for(
    monitor,
    [](const auto & state) {
      return !state.healthy && state.reason == "HID probe exceeded 500 ms";
    },
    "late successful probe was not rejected as a timeout");
  slow.store(false);
  wait_for(
    monitor,
    [&first](const auto & state) {return state.healthy && state.generation > first.generation;},
    "quick success after timeout did not recover with a new generation");
}
}  // namespace

int main()
{
  try {
    healthy_static_probe_reports_alive();
    stopped_answers_become_unhealthy();
    blocking_probe_does_not_block_snapshot();
    generation_changes_on_recovery();
    late_success_does_not_revive_link();
    std::cout << "joystick link monitor tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "joystick link monitor: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
