#include "motor/yiyou/cycle_timing.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace chassis::ethercat
{
namespace
{
struct Metric
{
  const char * name;
  std::int64_t CycleTiming::* value;
};
constexpr Metric metrics[] = {
  {"interval_us", &CycleTiming::interval_us},
  {"wake_late_us", &CycleTiming::wake_late_us},
  {"lock_wait_us", &CycleTiming::lock_wait_us},
  {"send_us", &CycleTiming::send_us},
  {"receive_us", &CycleTiming::receive_us},
  {"al_total_us", &CycleTiming::al_total_us},
  {"al_max_us", &CycleTiming::al_max_us},
  {"work_us", &CycleTiming::work_us},
  {"feedback_age_us", &CycleTiming::feedback_age_us},
  {"overrun_us", &CycleTiming::overrun_us},
};
}  // namespace

std::string cycle_timing_text(const CycleTiming & sample)
{
  std::ostringstream out;
  out << "cycle=" << sample.cycle << " start_mono_us=" << sample.start_us;
  for (const auto & metric : metrics) {
    out << ' ' << metric.name << '=' << sample.*(metric.value);
  }
  out << " al_max_slave=" << sample.al_max_slave << " al_polls=" << sample.al_polls
      << " wkc=" << sample.wkc << " expected_wkc=" << sample.expected_wkc
      << " pdo_failure_streak=" << sample.pdo_failure_streak
      << " pdo_recovered=" << sample.pdo_recovered
      << " al_timeouts=" << sample.al_timeouts << " al_failure_streak=" << sample.al_failure_streak
      << " al_failed_slave=" << sample.al_failed_slave << " al_recoveries=" << sample.al_recoveries
      << " fault_latched=" << sample.fault;
  return out.str();
}

CycleTimingLog::CycleTimingLog(std::ostream & output, std::chrono::milliseconds interval)
: output_(output), interval_(interval)
{
  if (interval <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("EtherCAT timing log interval must be positive");
  }
  thread_ = std::thread([this] {run();});
}

CycleTimingLog::~CycleTimingLog() {stop();}

bool CycleTimingLog::record(const CycleTiming & sample)
{
  std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
  if (!guard.owns_lock()) {++skipped_; return false;}
  if (stopping_ || frozen_) {return false;}
  if (window_.count++ == 0) {window_.first_cycle = sample.cycle;}
  window_.overruns += sample.overrun_us > 0 ? 1 : 0;
  window_.interval_count += sample.interval_us > 0 ? 1 : 0;
  window_.pdo_invalid_cycles += sample.wkc != sample.expected_wkc ? 1 : 0;
  window_.pdo_recoveries += sample.pdo_recovered ? 1 : 0;
  window_.al_timeouts += sample.al_timeouts;
  window_.al_recoveries += sample.al_recoveries;
  if (sample.wkc != sample.expected_wkc || sample.al_timeouts > 0) {
    window_.last_warning = sample;
    window_.have_warning = true;
  }
  if (sample.al_polls > 0 && sample.al_max_us >= window_.maxima.al_max_us) {
    window_.maxima.al_max_slave = sample.al_max_slave;
  }
  for (const auto & metric : metrics) {
    window_.sums.*(metric.value) += sample.*(metric.value);
    window_.maxima.*(metric.value) = std::max(
      window_.maxima.*(metric.value), sample.*(metric.value));
  }
  window_.last = sample;
  history_[history_next_] = sample;
  history_next_ = (history_next_ + 1) % history_.size();
  history_count_ = std::min(history_count_ + 1, history_.size());
  if (sample.fault) {
    frozen_ = fault_pending_ = true;  // Preserve the pre-cut history.
    changed_.notify_one();
  }
  return true;
}

void CycleTimingLog::stop()
{
  {
    std::lock_guard<std::mutex> guard(mutex_);
    stopping_ = true;
  }
  changed_.notify_one();
  if (thread_.joinable()) {thread_.join();}
}

void CycleTimingLog::run()
{
  try {
    std::unique_lock<std::mutex> guard(mutex_);
    for (;;) {
      changed_.wait_for(guard, interval_, [this] {return stopping_ || fault_pending_;});
      const auto window = window_;
      window_ = {};
      const bool fault = fault_pending_;
      fault_pending_ = false;
      const bool stopping = stopping_;
      const auto history = history_;
      const auto count = history_count_;
      const auto next = history_next_;
      const auto skipped = skipped_.load();
      guard.unlock();  // Stream formatting and output never hold the producer lock.
      if (window.count > 0) {
        std::ostringstream out;
        out << "EtherCAT timing: reason=" << (fault ? "fault" : stopping ? "shutdown" : "periodic")
            << " samples=" << window.count << " first_cycle=" << window.first_cycle
            << " last_cycle=" << window.last.cycle << " overruns=" << window.overruns
            << " skipped_samples_total=" << skipped
            << " pdo_invalid_cycles=" << window.pdo_invalid_cycles
            << " pdo_recoveries=" << window.pdo_recoveries
            << " al_timeouts=" << window.al_timeouts << " al_recoveries=" << window.al_recoveries;
        for (const auto & metric : metrics) {
          const auto n = metric.value == &CycleTiming::interval_us ?
            window.interval_count : window.count;
          out << ' ' << metric.name << "_avg=" << (n ? window.sums.*(metric.value) / n : 0)
              << ' ' << metric.name << "_max=" << window.maxima.*(metric.value);
        }
        out << " al_max_slave=" << window.maxima.al_max_slave
            << " last_wkc=" << window.last.wkc << " expected_wkc=" << window.last.expected_wkc
            << '\n';
        output_ << out.str();
        if (window.have_warning) {
          output_ << "EtherCAT timing warning: " << cycle_timing_text(window.last_warning) << '\n';
        }
      }
      if (fault) {
        for (std::size_t i = 0; i < count; ++i) {
          output_ << "EtherCAT timing history: " << cycle_timing_text(
            history[(next + history.size() - count + i) % history.size()]) << '\n';
        }
      }
      if (stopping) {break;}
      guard.lock();
    }
  } catch (const std::exception & error) {
    std::cerr << "EtherCAT timing logger failed: " << error.what() << '\n';
  }
}
}  // namespace chassis::ethercat
