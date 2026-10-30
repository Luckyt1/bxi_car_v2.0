#pragma once

#include "arm/control.h"

#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace chassis
{

class ArmTorqueLog
{
public:
  explicit ArmTorqueLog(std::string directory);
  ~ArmTorqueLog();

  ArmTorqueLog(const ArmTorqueLog &) = delete;
  ArmTorqueLog & operator=(const ArmTorqueLog &) = delete;

  void record(const ArmTorqueSample & sample) noexcept;
  void finish() noexcept;

  const std::string & directory() const noexcept {return directory_;}
  std::string finished_path() const;

private:
  struct Event
  {
    ArmTorqueSample sample{};
    double wall_time_s{0.0};
    std::size_t dropped_samples{0};
    bool gap{false};
  };

  void run() noexcept;
  bool ensure_open();
  void write_header();
  void write_event(const Event & event);
  void flush();
  void close_and_finalize();
  bool open_unique_session();
  void warn_once(const std::string & message) noexcept;

  std::string directory_;
  std::filesystem::path part_path_;
  std::filesystem::path complete_path_;
  FILE * file_{nullptr};
  int fd_{-1};
  bool disabled_{false};
  bool stopping_{false};
  bool finished_{false};
  bool writing_failed_{false};
  std::optional<std::chrono::steady_clock::time_point> first_sample_;
  std::size_t dropped_pending_{0};
  std::size_t warning_count_{0};
  std::deque<Event> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
};

}  // namespace chassis
