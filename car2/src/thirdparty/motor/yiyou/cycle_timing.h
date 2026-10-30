#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>
#include <thread>

namespace chassis::ethercat
{
// Elapsed wall time from steady_clock, including preemption/driver delays.
// These are not measurements of wire propagation time.
struct CycleTiming
{
  std::uint64_t cycle{0};
  std::int64_t start_us{0};
  std::int64_t interval_us{0}, wake_late_us{0}, lock_wait_us{0};
  std::int64_t send_us{0}, receive_us{0}, al_total_us{0}, al_max_us{0};
  std::int64_t work_us{0}, feedback_age_us{0}, overrun_us{0};
  std::uint16_t al_max_slave{0};
  unsigned al_polls{0};
  unsigned pdo_failure_streak{0}, al_failure_streak{0}, al_timeouts{0}, al_recoveries{0};
  std::uint16_t al_failed_slave{0};
  int wkc{0}, expected_wkc{0};
  bool fault{false}, pdo_recovered{false};
};

std::string cycle_timing_text(const CycleTiming & sample);

// One producer (the cyclic thread). No formatting, I/O, allocation or waiting
// in record(). Contended samples are counted; the existing fault line also
// carries the exact failing sample independently of this recorder.
class CycleTimingLog
{
public:
  explicit CycleTimingLog(
    std::ostream & output,
    std::chrono::milliseconds interval = std::chrono::seconds(5));
  ~CycleTimingLog();
  bool record(const CycleTiming & sample);
  void stop();  // Call after the producer has stopped. Idempotent.

private:
  struct Window
  {
    std::uint64_t count{0}, first_cycle{0}, overruns{0}, interval_count{0};
    std::uint64_t pdo_invalid_cycles{0}, pdo_recoveries{0}, al_timeouts{0}, al_recoveries{0};
    CycleTiming sums{}, maxima{}, last{};
    CycleTiming last_warning{};
    bool have_warning{false};
  };
  void run();
  std::ostream & output_;
  std::chrono::milliseconds interval_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::atomic<std::uint64_t> skipped_{0};
  Window window_{};
  std::array<CycleTiming, 32> history_{};
  std::size_t history_count_{0}, history_next_{0};
  bool stopping_{false}, frozen_{false}, fault_pending_{false};
  std::thread thread_;
};
}  // namespace chassis::ethercat
