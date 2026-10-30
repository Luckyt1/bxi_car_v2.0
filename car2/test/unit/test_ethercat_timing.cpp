#include "motor/yiyou/cycle_timing.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <thread>

using namespace std::chrono_literals;
using chassis::ethercat::CycleTiming;
using chassis::ethercat::CycleTimingLog;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void contains(const std::string & text, const std::string & expected)
{
  if (text.find(expected) == std::string::npos) {
    throw std::runtime_error("missing log field: " + expected);
  }
}
void record(CycleTimingLog & log, const CycleTiming & sample)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!log.record(sample)) {
    require(std::chrono::steady_clock::now() < deadline, "sample was never accepted");
    std::this_thread::yield();
  }
}

void shutdown_flushes_measurements()
{
  std::ostringstream output;
  CycleTimingLog log(output);
  CycleTiming sample;
  sample.cycle = 1;
  sample.receive_us = 200;
  sample.wkc = sample.expected_wkc = 12;
  record(log, sample);
  sample.cycle = 2;
  sample.interval_us = 2400;
  sample.receive_us = 1400;
  sample.overrun_us = 400;
  sample.al_polls = 4;
  sample.al_max_slave = 3;
  sample.al_max_us = 1200;
  record(log, sample);
  log.stop();
  log.stop();
  const auto text = output.str();
  contains(text, "reason=shutdown samples=2 first_cycle=1 last_cycle=2 overruns=1");
  contains(text, "receive_us_avg=800 receive_us_max=1400");
  contains(text, "interval_us_avg=2400 interval_us_max=2400");
  contains(text, "al_max_slave=3 last_wkc=12 expected_wkc=12");
}

void fault_preserves_recent_history_once()
{
  std::ostringstream output;
  CycleTimingLog log(output);
  for (unsigned i = 1; i <= 40; ++i) {
    CycleTiming sample;
    sample.cycle = i;
    sample.start_us = 100000 + i * 2000;
    sample.interval_us = i > 1 ? 2000 : 0;
    sample.wkc = i == 40 ? -1 : 12;
    sample.expected_wkc = 12;
    sample.fault = i == 40;
    record(log, sample);
  }
  CycleTiming after_cut;
  after_cut.cycle = 41;
  after_cut.fault = true;
  require(!log.record(after_cut), "post-cut samples must not overwrite first-fault history");
  log.stop();
  const auto text = output.str();
  contains(text, "reason=fault samples=40");
  contains(text, "timing history: cycle=9 start_mono_us=118000");
  contains(text, "timing history: cycle=40 start_mono_us=180000");
  contains(text, "wkc=-1 expected_wkc=12");
  contains(text, "fault_latched=1");
  require(text.find("timing history: cycle=8 ") == std::string::npos, "history was not bounded");
  require(text.find("timing history: cycle=41 ") == std::string::npos, "history was overwritten");
  std::size_t count = 0, pos = 0;
  while ((pos = text.find("EtherCAT timing history:", pos)) != std::string::npos) {++count; ++pos;}
  require(count == 32, "fault history must be dumped exactly once");
}

void transient_warnings_survive_recovery_without_freezing_history()
{
  std::ostringstream output;
  CycleTimingLog log(output);
  CycleTiming sample;
  sample.cycle = 1;
  sample.expected_wkc = 12;
  sample.wkc = -1;
  sample.pdo_failure_streak = 1;
  sample.al_timeouts = 1;
  sample.al_failure_streak = 1;
  sample.al_failed_slave = 4;
  record(log, sample);
  sample.cycle = 2;
  sample.wkc = 12;
  sample.pdo_failure_streak = sample.al_failure_streak = sample.al_timeouts = 0;
  sample.pdo_recovered = true;
  sample.al_recoveries = 1;
  record(log, sample);
  log.stop();
  const auto text = output.str();
  contains(text, "reason=shutdown samples=2");
  contains(text, "pdo_invalid_cycles=1 pdo_recoveries=1 al_timeouts=1 al_recoveries=1");
  contains(text, "EtherCAT timing warning: cycle=1");
  contains(text, "pdo_failure_streak=1");
  contains(text, "al_failed_slave=4");
  contains(text, "last_wkc=12 expected_wkc=12");
  require(text.find("reason=fault") == std::string::npos, "transient warning must not freeze recorder");
}

class BlockedOutput : public std::streambuf
{
public:
  bool wait_for_writer()
  {
    std::unique_lock<std::mutex> guard(mutex_);
    return changed_.wait_for(guard, 2s, [this] {return entered_;});
  }
  void release()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    released_ = true;
    changed_.notify_all();
  }
protected:
  std::streamsize xsputn(const char *, std::streamsize n) override
  {
    std::unique_lock<std::mutex> guard(mutex_);
    entered_ = true;
    changed_.notify_all();
    changed_.wait(guard, [this] {return released_;});
    return n;
  }
private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool entered_{false}, released_{false};
};

void slow_output_does_not_block_cyclic_producer()
{
  BlockedOutput buffer;
  std::ostream output(&buffer);
  CycleTimingLog log(output, 10ms);
  CycleTiming sample;
  sample.cycle = 1;
  record(log, sample);
  const bool writer_started = buffer.wait_for_writer();
  if (!writer_started) {buffer.release();}
  require(writer_started, "periodic summary was not emitted");
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for (unsigned i = 2; i < 10002; ++i) {sample.cycle = i; log.record(sample);}
    done = true;
  });
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!done && std::chrono::steady_clock::now() < deadline) {std::this_thread::yield();}
  const bool completed = done;
  buffer.release();
  producer.join();
  log.stop();
  require(completed, "blocked output prevented cyclic recording");
}
}  // namespace

int main()
{
  try {
    shutdown_flushes_measurements();
    fault_preserves_recent_history_once();
    transient_warnings_survive_recovery_without_freezing_history();
    slow_output_does_not_block_cyclic_producer();
    std::cout << "EtherCAT timing diagnostics passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
