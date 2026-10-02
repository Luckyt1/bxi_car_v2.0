#include "example/joystick_link_monitor.hpp"

#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace chassis
{
namespace
{
constexpr unsigned kBattleDragonBus = 0x0005;      // BUS_BLUETOOTH
constexpr unsigned kBattleDragonVendor = 0x20d6;
constexpr unsigned kBattleDragonProduct = 0x400d;
constexpr unsigned char kInputReportId = 1;
constexpr std::size_t kInputReportSize = 17;

class FdGuard
{
public:
  explicit FdGuard(int fd) : fd_(fd) {}
  ~FdGuard()
  {
    if (fd_ >= 0) {::close(fd_);}
  }
  FdGuard(const FdGuard &) = delete;
  FdGuard & operator=(const FdGuard &) = delete;

private:
  int fd_;
};

std::string errno_message(const char * action, const std::filesystem::path & path)
{
  std::ostringstream message;
  message << action << ' ' << path << ": " << std::strerror(errno);
  return message.str();
}

std::optional<std::string> read_file(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {return std::nullopt;}
  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

bool parse_battle_dragon_hid_id(const std::string & uevent)
{
  std::istringstream lines(uevent);
  std::string line;
  while (std::getline(lines, line)) {
    constexpr const char * prefix = "HID_ID=";
    if (line.rfind(prefix, 0) != 0) {continue;}
    unsigned bus = 0;
    unsigned vendor = 0;
    unsigned product = 0;
    if (std::sscanf(
        line.c_str() + std::strlen(prefix), "%x:%x:%x", &bus, &vendor, &product) != 3)
    {
      return false;
    }
    return bus == kBattleDragonBus && vendor == kBattleDragonVendor &&
           product == kBattleDragonProduct;
  }
  return false;
}

bool is_verified_battle_dragon_hid(const std::filesystem::path & path)
{
  const auto uevent = read_file(path / "uevent");
  return uevent && parse_battle_dragon_hid_id(*uevent);
}

std::optional<std::filesystem::path> hidraw_node_under(const std::filesystem::path & hid_path)
{
  const auto hidraw_dir = hid_path / "hidraw";
  std::error_code error;
  if (!std::filesystem::is_directory(hidraw_dir, error)) {return std::nullopt;}
  for (const auto & entry : std::filesystem::directory_iterator(hidraw_dir, error)) {
    if (error) {return std::nullopt;}
    const auto name = entry.path().filename().string();
    if (name.rfind("hidraw", 0) == 0) {return std::filesystem::path("/dev") / name;}
  }
  return std::nullopt;
}

std::optional<std::filesystem::path> locate_hidraw_from_joystick(
  const std::filesystem::path & joystick_device,
  std::string & reason)
{
  struct stat status {};
  if (::stat(joystick_device.c_str(), &status) != 0) {
    reason = errno_message("stat joystick", joystick_device);
    return std::nullopt;
  }
  if (!S_ISCHR(status.st_mode)) {
    reason = joystick_device.string() + " is not a character device";
    return std::nullopt;
  }

  const auto sys_char = std::filesystem::path("/sys/dev/char") /
    (std::to_string(major(status.st_rdev)) + ":" + std::to_string(minor(status.st_rdev)));
  std::error_code error;
  auto current = std::filesystem::canonical(sys_char, error);
  if (error) {
    reason = "cannot resolve " + sys_char.string() + ": " + error.message();
    return std::nullopt;
  }

  while (!current.empty() && current != current.root_path()) {
    if (is_verified_battle_dragon_hid(current)) {
      auto hidraw = hidraw_node_under(current);
      if (hidraw) {return hidraw;}
      reason = "Battle Dragon HID found at " + current.string() + " but no hidraw child exists";
      return std::nullopt;
    }
    current = current.parent_path();
  }

  reason = joystick_device.string() +
    " is not the verified Battle Dragon BLE HID device (bus 0005 vendor 20d6 product 400d)";
  return std::nullopt;
}

bool verify_hidraw_identity(int fd, const std::filesystem::path & hidraw, std::string & reason)
{
  hidraw_devinfo info {};
  if (::ioctl(fd, HIDIOCGRAWINFO, &info) != 0) {
    reason = errno_message("HIDIOCGRAWINFO", hidraw);
    return false;
  }
  if (static_cast<unsigned>(info.bustype) != kBattleDragonBus ||
    static_cast<unsigned short>(info.vendor) != kBattleDragonVendor ||
    static_cast<unsigned short>(info.product) != kBattleDragonProduct)
  {
    std::ostringstream message;
    message << hidraw << " identity mismatch: bus=0x" << std::hex
            << static_cast<unsigned>(info.bustype) << " vendor=0x"
            << static_cast<unsigned short>(info.vendor) << " product=0x"
            << static_cast<unsigned short>(info.product);
    reason = message.str();
    return false;
  }
  return true;
}

bool readonly_input_probe(const std::filesystem::path & joystick_device, std::string & reason)
{
  const auto hidraw = locate_hidraw_from_joystick(joystick_device, reason);
  if (!hidraw) {return false;}

  const int fd = ::open(hidraw->c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    reason = errno_message("open hidraw read-only", *hidraw);
    return false;
  }
  FdGuard guard(fd);
  if (!verify_hidraw_identity(fd, *hidraw, reason)) {return false;}

  unsigned char report[kInputReportSize] {};
  report[0] = kInputReportId;
  if (::ioctl(fd, HIDIOCGINPUT(sizeof(report)), report) < 0) {
    reason = errno_message("HIDIOCGINPUT report 1", *hidraw);
    return false;
  }
  reason = "ok";
  return true;
}

std::int64_t to_nanoseconds(JoystickLinkMonitor::Clock::time_point time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    time.time_since_epoch()).count();
}
}  // namespace

class JoystickLinkMonitor::Impl
{
public:
  explicit Impl(std::string device_path)
  : device_path_(std::move(device_path))
  {
    if (device_path_.empty()) {throw std::invalid_argument("joystick device path is empty");}
    start();
  }

  explicit Impl(Probe probe)
  : probe_(std::move(probe))
  {
    if (!probe_) {throw std::invalid_argument("joystick link probe is empty");}
    start();
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(stop_mutex_);
      stop_ = true;
    }
    stop_cv_.notify_all();
    if (thread_.joinable()) {thread_.join();}
  }

  Snapshot snapshot() const
  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    Snapshot result;
    result.generation = generation_;
    result.reason = reason_;
    if (!healthy_) {
      result.healthy = false;
      if (result.reason.empty()) {result.reason = "no successful HID probe yet";}
      return result;
    }
    const auto last_success = Clock::time_point(std::chrono::nanoseconds(last_success_ns_));
    if (now - last_success >= kProbeTimeout) {
      result.healthy = false;
      result.reason = "last successful HID probe is stale";
      return result;
    }
    result.healthy = true;
    if (result.reason.empty()) {result.reason = "ok";}
    return result;
  }

private:
  void start()
  {
    thread_ = std::thread([this] {run();});
  }

  void run()
  {
    while (true) {
      {
        std::lock_guard<std::mutex> lock(stop_mutex_);
        if (stop_) {return;}
      }

      const auto started = Clock::now();
      std::string failure_reason;
      bool success = false;
      try {
        if (probe_) {
          success = probe_();
          if (!success) {failure_reason = "probe returned false";}
        } else {
          success = readonly_input_probe(device_path_, failure_reason);
        }
      } catch (const std::exception & error) {
        failure_reason = error.what();
      } catch (...) {
        failure_reason = "probe threw unknown exception";
      }
      const auto finished = Clock::now();

      if (finished - started >= kProbeTimeout) {
        mark_failure("HID probe exceeded 500 ms");
      } else if (success) {
        mark_success(finished);
      } else {
        mark_failure(failure_reason.empty() ? "HID probe failed" : failure_reason);
      }

      std::unique_lock<std::mutex> lock(stop_mutex_);
      if (stop_cv_.wait_until(lock, started + kProbePeriod, [this] {return stop_;})) {return;}
    }
  }

  void mark_success(Clock::time_point now)
  {
    const auto now_ns = to_nanoseconds(now);
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool recovering = !healthy_ || last_success_ns_ < 0 ||
      now - Clock::time_point(std::chrono::nanoseconds(last_success_ns_)) >= kProbeTimeout;
    if (recovering) {++generation_;}
    healthy_ = true;
    last_success_ns_ = now_ns;
    reason_ = "ok";
  }

  void mark_failure(std::string reason)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    healthy_ = false;
    reason_ = std::move(reason);
  }

  std::string device_path_;
  Probe probe_;
  mutable std::mutex state_mutex_;
  bool healthy_{false};
  std::uint64_t generation_{0};
  std::int64_t last_success_ns_{-1};
  std::string reason_{"no successful HID probe yet"};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stop_{false};
  std::thread thread_;
};

JoystickLinkMonitor::JoystickLinkMonitor(std::string device_path)
: impl_(std::make_unique<Impl>(std::move(device_path))) {}

JoystickLinkMonitor::JoystickLinkMonitor(Probe probe)
: impl_(std::make_unique<Impl>(std::move(probe))) {}

JoystickLinkMonitor::~JoystickLinkMonitor() = default;

JoystickLinkMonitor::Snapshot JoystickLinkMonitor::snapshot() const
{
  return impl_->snapshot();
}

}  // namespace chassis
