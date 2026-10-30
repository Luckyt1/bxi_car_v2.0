#include "motor/iswv/diagnostic_log.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace iswv::canopen
{
namespace
{
using Clock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;
namespace fs = std::filesystem;

constexpr std::size_t kRingLimit = 4096;
constexpr std::size_t kCaptureLimit = 8192;
constexpr std::size_t kActiveCaptureLimit = 2;
constexpr auto kPostTriggerWindow = std::chrono::seconds(2);
constexpr auto kRetention = std::chrono::hours(168);
constexpr auto kCleanupPeriod = std::chrono::hours(1);

struct Event
{
  std::uint64_t sequence{0};
  SystemClock::time_point wall_time{};
  Clock::time_point monotonic_time{};
  std::string message;
};

struct Capture
{
  std::uint64_t id{0};
  SystemClock::time_point wall_time{};
  Clock::time_point trigger_time{};
  Clock::time_point deadline{};
  std::string reason;
  std::vector<Event> events;
  std::size_t dropped_events{0};
};

std::string sanitize_channel(const std::string & channel)
{
  std::string result;
  result.reserve(channel.size());
  for (const auto ch : channel) {
    const auto value = static_cast<unsigned char>(ch);
    if (std::isalnum(value) || ch == '-' || ch == '_') {
      result.push_back(static_cast<char>(ch));
    } else {
      result.push_back('_');
    }
  }
  return result.empty() ? "can" : result;
}

std::string timestamp_name(SystemClock::time_point time)
{
  const auto raw = SystemClock::to_time_t(time);
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
    time.time_since_epoch()).count() % 1000000;
  std::tm tm{};
  localtime_r(&raw, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%d-%H%M%S") << '-'
      << std::setw(6) << std::setfill('0') << micros;
  return out.str();
}

std::string timestamp_line(SystemClock::time_point time)
{
  const auto raw = SystemClock::to_time_t(time);
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
    time.time_since_epoch()).count() % 1000000;
  std::tm tm{};
  localtime_r(&raw, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.'
      << std::setw(6) << std::setfill('0') << micros;
  return out.str();
}

bool all_digits(const std::string & value)
{
  return !value.empty() && std::all_of(value.begin(), value.end(), [](char ch) {
    return std::isdigit(static_cast<unsigned char>(ch)) != 0;
  });
}

bool valid_channel_char(char ch)
{
  const auto value = static_cast<unsigned char>(ch);
  return std::isalnum(value) || ch == '-' || ch == '_';
}

bool is_component_file(const fs::directory_entry & entry)
{
  std::error_code error;
  if (entry.symlink_status(error).type() != fs::file_type::regular || error) {
    return false;
  }
  const auto name = entry.path().filename().string();
  const std::string prefix = "can-diagnostic-";
  const std::string suffix = ".log";
  if (name.rfind(prefix, 0) != 0 || name.size() <= prefix.size() + suffix.size()) {
    return false;
  }
  if (name.substr(name.size() - suffix.size()) != suffix) {
    return false;
  }
  const auto stem = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= stem.size()) {
    const auto end = stem.find('-', start);
    parts.push_back(stem.substr(start, end == std::string::npos ? end : end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (parts.size() < 6) {
    return false;
  }
  const auto micros = parts.back();
  parts.pop_back();
  const auto time = parts.back();
  parts.pop_back();
  const auto date = parts.back();
  parts.pop_back();
  const auto id = parts.back();
  parts.pop_back();
  const auto pid = parts.back();
  parts.pop_back();
  if (!all_digits(pid) || !all_digits(id) ||
    date.size() != 8 || !all_digits(date) ||
    time.size() != 6 || !all_digits(time) ||
    micros.size() != 6 || !all_digits(micros))
  {
    return false;
  }
  const auto channel = std::accumulate(
    std::next(parts.begin()), parts.end(), parts.front(),
    [](std::string left, const std::string & right) {
      left.push_back('-');
      left += right;
      return left;
    });
  if (channel.empty()) {
    return false;
  }
  return std::all_of(
    channel.begin(), channel.end(), valid_channel_char);
}

SystemClock::duration file_age(const fs::file_time_type & file_time, SystemClock::time_point now)
{
  const auto file_system_time =
    now + (file_time - fs::file_time_type::clock::now());
  return now - file_system_time;
}

}  // namespace

std::atomic<std::uint64_t> global_capture_id{1};

class DiagnosticLog::Impl
{
public:
  Impl(std::string directory, std::string channel)
  : directory_(std::move(directory)),
    channel_(sanitize_channel(channel))
  {
    worker_ = std::thread([this] {run();});
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;

  void record(std::string message) noexcept
  {
    try {
      const auto now = Clock::now();
      Event event;
      event.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
      event.wall_time = SystemClock::now();
      event.monotonic_time = now;
      event.message = std::move(message);

      std::lock_guard<std::mutex> lock(mutex_);
      ring_.push_back(event);
      while (ring_.size() > kRingLimit) {
        ring_.pop_front();
      }
      for (auto & capture : active_) {
        if (now > capture.deadline) {
          continue;
        }
        if (capture.events.size() < kCaptureLimit) {
          capture.events.push_back(event);
        } else {
          ++capture.dropped_events;
        }
      }
    } catch (...) {
    }
  }

  void trigger(std::string reason) noexcept
  {
    try {
      Capture capture;
      const auto now = Clock::now();
      capture.id = global_capture_id.fetch_add(1, std::memory_order_relaxed);
      capture.wall_time = SystemClock::now();
      capture.trigger_time = now;
      capture.deadline = now + kPostTriggerWindow;
      capture.reason = std::move(reason);

      std::lock_guard<std::mutex> lock(mutex_);
      if (active_.size() >= kActiveCaptureLimit) {
        ++dropped_captures_;
        Event marker;
        marker.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        marker.wall_time = SystemClock::now();
        marker.monotonic_time = now;
        marker.message = "diagnostic capture dropped: too many active triggers";
        ring_.push_back(marker);
        while (ring_.size() > kRingLimit) {
          ring_.pop_front();
        }
        return;
      }
      capture.events.assign(ring_.begin(), ring_.end());
      active_.push_back(std::move(capture));
    } catch (...) {
      return;
    }
    wake_.notify_one();
  }

private:
  void run() noexcept
  {
    try {
      info("enabled directory=" + directory_ + " retention_hours=168");
      cleanup();
      auto next_cleanup = Clock::now() + kCleanupPeriod;
      std::unique_lock<std::mutex> lock(mutex_);
      while (true) {
        const auto next_capture_time = next_deadline_locked();
        const auto wake_time = std::min(next_cleanup, next_capture_time);
        if (!stopping_) {
          wake_.wait_until(lock, wake_time);
        }

        auto now = Clock::now();
        std::vector<Capture> ready;
        for (auto iterator = active_.begin(); iterator != active_.end();) {
          if (stopping_ || now >= iterator->deadline) {
            ready.push_back(std::move(*iterator));
            iterator = active_.erase(iterator);
          } else {
            ++iterator;
          }
        }
        std::size_t dropped_captures = 0;
        if (!ready.empty()) {
          dropped_captures = dropped_captures_;
          dropped_captures_ = 0;
        }
        const auto should_stop = stopping_;
        lock.unlock();

        if (now >= next_cleanup) {
          cleanup();
          next_cleanup = now + kCleanupPeriod;
        }
        for (auto & capture : ready) {
          write_capture(capture, dropped_captures);
        }

        lock.lock();
        if (should_stop && active_.empty()) {
          break;
        }
      }
    } catch (...) {
    }
  }

  Clock::time_point next_deadline_locked() const
  {
    auto result = Clock::time_point::max();
    for (const auto & capture : active_) {
      result = std::min(result, capture.deadline);
    }
    return result;
  }

  fs::path capture_path(const Capture & capture) const
  {
    std::ostringstream name;
    name << "can-diagnostic-" << channel_ << '-'
         << static_cast<long long>(::getpid()) << '-'
         << capture.id << '-' << timestamp_name(capture.wall_time) << ".log";
    return fs::path(directory_) / name.str();
  }

  void cleanup() noexcept
  {
    try {
      std::error_code error;
      fs::create_directories(directory_, error);
      if (error) {
        warn("failed to create CAN diagnostic directory during cleanup: " + error.message());
        return;
      }
      const auto now = SystemClock::now();
      for (const auto & entry : fs::directory_iterator(directory_, error)) {
        if (error) {
          warn("failed to iterate CAN diagnostic directory: " + error.message());
          return;
        }
        if (!is_component_file(entry)) {
          continue;
        }
        const auto modified = entry.last_write_time(error);
        if (error) {
          warn("failed to read CAN diagnostic log timestamp: " + error.message());
          error.clear();
          continue;
        }
        if (file_age(modified, now) > kRetention) {
          fs::remove(entry.path(), error);
          if (error) {
            warn("failed to remove expired CAN diagnostic log: " + error.message());
          }
          error.clear();
        }
      }
    } catch (...) {
    }
  }

  void write_capture(const Capture & capture, std::size_t dropped_captures) noexcept
  {
    try {
      std::error_code error;
      fs::create_directories(directory_, error);
      if (error) {
        warn("failed to create CAN diagnostic directory: " + error.message());
        return;
      }
      const auto path = capture_path(capture);
      std::ostringstream out;
      out << "channel=" << channel_ << "\n";
      out << "pid=" << static_cast<long long>(::getpid()) << "\n";
      out << "trigger_wall=" << timestamp_line(capture.wall_time) << "\n";
      out << "reason=" << capture.reason << "\n";
      out << "events=" << capture.events.size() << "\n";
      out << "dropped_events=" << capture.dropped_events << "\n";
      out << "dropped_captures=" << dropped_captures << "\n";
      out << "\n";
      for (const auto & event : capture.events) {
        const auto delta = std::chrono::duration_cast<std::chrono::microseconds>(
          event.monotonic_time - capture.trigger_time).count();
        out << event.sequence << ' '
            << "wall=" << timestamp_line(event.wall_time) << ' '
            << "trigger_us=" << delta << ' '
            << event.message << "\n";
      }
      const auto content = out.str();
      const int fd = ::open(
        path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0644);
      if (fd < 0) {
        warn(std::string("failed to create CAN diagnostic log: ") + std::strerror(errno));
        return;
      }
      info("writing CAN diagnostic log: " + path.string());
      const char * data = content.data();
      std::size_t remaining = content.size();
      while (remaining > 0) {
        const auto written = ::write(fd, data, remaining);
        if (written < 0) {
          if (errno == EINTR) {
            continue;
          }
          warn(std::string("failed to write CAN diagnostic log: ") + std::strerror(errno));
          ::close(fd);
          return;
        }
        if (written == 0) {
          warn("failed to write CAN diagnostic log: write returned 0");
          ::close(fd);
          return;
        }
        data += written;
        remaining -= static_cast<std::size_t>(written);
      }
      if (::close(fd) != 0) {
        warn(std::string("failed to close CAN diagnostic log: ") + std::strerror(errno));
      }
    } catch (...) {
      warn("unexpected CAN diagnostic write failure");
    }
  }

  void warn(const std::string & message) noexcept
  {
    try {
      const auto count = error_warnings_.fetch_add(1, std::memory_order_relaxed);
      if (count < 8) {
        std::cerr << "[can-diagnostic] " << message << "\n";
      }
    } catch (...) {
    }
  }

  void info(const std::string & message) noexcept
  {
    try {
      const auto count = info_messages_.fetch_add(1, std::memory_order_relaxed);
      if (count < 32) {
        std::cerr << "[can-diagnostic] " << message << "\n";
      }
    } catch (...) {
    }
  }

  const std::string directory_;
  const std::string channel_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Event> ring_;
  std::vector<Capture> active_;
  bool stopping_{false};
  std::size_t dropped_captures_{0};
  std::atomic<std::uint64_t> next_sequence_{1};
  std::atomic<std::size_t> error_warnings_{0};
  std::atomic<std::size_t> info_messages_{0};
  std::thread worker_;
};

DiagnosticLog::DiagnosticLog(std::string directory, std::string channel)
{
  if (!directory.empty()) {
    impl_ = std::make_unique<Impl>(std::move(directory), std::move(channel));
  }
}

DiagnosticLog::~DiagnosticLog() = default;
DiagnosticLog::DiagnosticLog(DiagnosticLog &&) noexcept = default;
DiagnosticLog & DiagnosticLog::operator=(DiagnosticLog &&) noexcept = default;

void DiagnosticLog::record(std::string message) noexcept
{
  if (impl_) {
    impl_->record(std::move(message));
  }
}

void DiagnosticLog::trigger(std::string reason) noexcept
{
  if (impl_) {
    impl_->trigger(std::move(reason));
  }
}

}  // namespace iswv::canopen
