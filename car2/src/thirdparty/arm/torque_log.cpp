#include "arm/torque_log.h"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>
#include <unistd.h>

#include <sys/file.h>
#include <sys/stat.h>

namespace chassis
{
namespace
{
constexpr std::size_t kQueueCapacity = 4096;
constexpr std::chrono::milliseconds kFlushPeriod{250};

double unix_time_seconds() noexcept
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

std::string timestamp_name()
{
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto micros =
    std::chrono::duration_cast<std::chrono::microseconds>(now - seconds).count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream out;
  out << std::put_time(&local, "%Y%m%d-%H%M%S") << '-'
      << std::setw(6) << std::setfill('0') << micros << '-' << ::getpid();
  return out.str();
}

std::string csv_number(double value, int precision = 6)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

template<typename T>
void append_optional_int(std::ostringstream & out, const std::optional<T> & value)
{
  if (value) {out << *value;}
}

}  // namespace

ArmTorqueLog::ArmTorqueLog(std::string directory)
: directory_(std::move(directory))
{
  if (directory_.empty()) {
    disabled_ = true;
    finished_ = true;
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(directory_, error);
  if (error) {
    disabled_ = true;
    finished_ = true;
    warn_once("cannot create arm torque log directory '" + directory_ + "': " + error.message());
    return;
  }
  worker_ = std::thread([this] {run();});
}

ArmTorqueLog::~ArmTorqueLog()
{
  finish();
}

void ArmTorqueLog::record(const ArmTorqueSample & sample) noexcept
{
  try {
    if (disabled_) {return;}
    const auto wall_time = unix_time_seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || finished_) {return;}
    const std::size_t required = dropped_pending_ > 0 ? 2 : 1;
    if (queue_.size() + required > kQueueCapacity) {
      ++dropped_pending_;
      return;
    }
    if (dropped_pending_ > 0) {
      Event gap;
      gap.sample.sampled_at = sample.sampled_at;
      gap.wall_time_s = wall_time;
      gap.dropped_samples = dropped_pending_;
      gap.gap = true;
      queue_.push_back(gap);
      dropped_pending_ = 0;
    }
    Event event;
    event.sample = sample;
    event.wall_time_s = wall_time;
    queue_.push_back(event);
    condition_.notify_one();
  } catch (...) {
  }
}

void ArmTorqueLog::finish() noexcept
{
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (finished_) {return;}
      stopping_ = true;
      if (dropped_pending_ > 0 && queue_.size() < kQueueCapacity) {
        Event gap;
        gap.sample.sampled_at = std::chrono::steady_clock::now();
        gap.wall_time_s = unix_time_seconds();
        gap.dropped_samples = dropped_pending_;
        gap.gap = true;
        queue_.push_back(gap);
        dropped_pending_ = 0;
      }
    }
    condition_.notify_one();
    if (worker_.joinable()) {worker_.join();}
    std::lock_guard<std::mutex> lock(mutex_);
    finished_ = true;
  } catch (...) {
  }
}

std::string ArmTorqueLog::finished_path() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return complete_path_.string();
}

void ArmTorqueLog::run() noexcept
{
  try {
    auto next_flush = std::chrono::steady_clock::now() + kFlushPeriod;
    while (true) {
      std::deque<Event> events;
      bool should_stop = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_until(lock, next_flush, [this] {
          return stopping_ || !queue_.empty();
        });
        if (stopping_ && queue_.empty() && dropped_pending_ > 0) {
          Event gap;
          gap.sample.sampled_at = std::chrono::steady_clock::now();
          gap.wall_time_s = unix_time_seconds();
          gap.dropped_samples = dropped_pending_;
          gap.gap = true;
          queue_.push_back(gap);
          dropped_pending_ = 0;
        }
        events.swap(queue_);
        should_stop = stopping_ && queue_.empty() && dropped_pending_ == 0;
      }
      for (const auto & event : events) {
        if (!ensure_open()) {continue;}
        write_event(event);
      }
      if (std::chrono::steady_clock::now() >= next_flush || should_stop) {
        flush();
        next_flush = std::chrono::steady_clock::now() + kFlushPeriod;
      }
      if (should_stop) {break;}
    }
  } catch (const std::exception & error) {
    writing_failed_ = true;
    warn_once("background writer failed: " + std::string(error.what()));
  } catch (...) {
    writing_failed_ = true;
    warn_once("background writer failed with unknown error");
  }
  try {
    close_and_finalize();
  } catch (const std::exception & error) {
    warn_once("finalize failed: " + std::string(error.what()));
  } catch (...) {
    warn_once("finalize failed with unknown error");
  }
}

bool ArmTorqueLog::ensure_open()
{
  if (file_ || writing_failed_) {return file_ != nullptr;}
  if (!open_unique_session()) {
    writing_failed_ = true;
    return false;
  }
  write_header();
  flush();
  return file_ != nullptr;
}

bool ArmTorqueLog::open_unique_session()
{
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto timestamp = timestamp_name();
    const auto stem = "arm-torque-" + timestamp + ".csv";
    const auto complete = std::filesystem::path(directory_) / stem;
    const auto part = complete.string() + ".part";
    const auto temporary = (std::filesystem::path(directory_) /
      (".arm-torque-" + timestamp + ".csv.part.tmp")).string();
    const int flags = O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC;
    const int opened = ::open(temporary.c_str(), flags, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (opened < 0) {
      if (errno == EEXIST) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        continue;
      }
      warn_once("cannot open arm torque log '" + temporary + "': " + std::strerror(errno));
      return false;
    }
    if (::flock(opened, LOCK_EX) != 0) {
      warn_once("cannot lock arm torque log '" + temporary + "': " + std::strerror(errno));
      ::close(opened);
      ::unlink(temporary.c_str());
      return false;
    }
    if (::link(temporary.c_str(), part.c_str()) != 0) {
      const int saved_errno = errno;
      ::flock(opened, LOCK_UN);
      ::close(opened);
      ::unlink(temporary.c_str());
      if (saved_errno == EEXIST) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        continue;
      }
      warn_once("cannot publish arm torque log '" + part + "': " + std::strerror(saved_errno));
      return false;
    }
    if (::unlink(temporary.c_str()) != 0) {
      const int saved_errno = errno;
      ::unlink(part.c_str());
      ::flock(opened, LOCK_UN);
      ::close(opened);
      warn_once(
        "cannot remove temporary arm torque log '" + temporary + "': " +
        std::strerror(saved_errno));
      return false;
    }
    FILE * stream = ::fdopen(opened, "w");
    if (!stream) {
      warn_once("cannot attach stream to arm torque log '" + part + "': " + std::strerror(errno));
      ::flock(opened, LOCK_UN);
      ::close(opened);
      ::unlink(part.c_str());
      return false;
    }
    fd_ = opened;
    file_ = stream;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      part_path_ = part;
      complete_path_ = complete;
    }
    return true;
  }
  warn_once("cannot create unique arm torque log name in '" + directory_ + "'");
  return false;
}

void ArmTorqueLog::write_header()
{
  if (!file_) {return;}
  if (std::fputs(
      "elapsed_s,wall_time_s,"
      "motor1_permille,motor1_rated_mnm,motor1_nm,"
      "motor2_permille,motor2_rated_mnm,motor2_nm,"
      "motor3_permille,motor3_rated_mnm,motor3_nm,"
      "motor4_permille,motor4_rated_mnm,motor4_nm,"
      "dropped_samples\n", file_) < 0)
  {
    warn_once("cannot write arm torque CSV header: " + std::string(std::strerror(errno)));
    writing_failed_ = true;
  }
}

void ArmTorqueLog::write_event(const Event & event)
{
  if (!file_ || writing_failed_) {return;}
  if (!first_sample_) {first_sample_ = event.sample.sampled_at;}
  const double elapsed =
    std::chrono::duration<double>(event.sample.sampled_at - *first_sample_).count();
  std::ostringstream row;
  row << csv_number(elapsed) << ',' << csv_number(event.wall_time_s);
  for (std::size_t index = 0; index < event.sample.permille.size(); ++index) {
    row << ',';
    if (!event.gap) {
      append_optional_int(row, event.sample.permille[index]);
    }
    row << ',';
    if (!event.gap) {
      append_optional_int(row, event.sample.rated_mnm[index]);
    }
    row << ',';
    if (!event.gap && event.sample.permille[index] && event.sample.rated_mnm[index] &&
      *event.sample.rated_mnm[index] > 0)
    {
      const double nm = static_cast<double>(*event.sample.permille[index]) *
        static_cast<double>(*event.sample.rated_mnm[index]) / 1000000.0;
      row << csv_number(nm);
    }
  }
  row << ',' << event.dropped_samples << '\n';
  const auto text = row.str();
  if (std::fwrite(text.data(), 1, text.size(), file_) != text.size()) {
    warn_once("cannot write arm torque CSV row: " + std::string(std::strerror(errno)));
    writing_failed_ = true;
  }
}

void ArmTorqueLog::flush()
{
  if (!file_ || writing_failed_) {return;}
  if (std::fflush(file_) != 0) {
    warn_once("cannot flush arm torque CSV: " + std::string(std::strerror(errno)));
    writing_failed_ = true;
  }
}

void ArmTorqueLog::close_and_finalize()
{
  if (!file_) {return;}
  bool failed = writing_failed_;
  if (std::fflush(file_) != 0) {
    warn_once("cannot flush arm torque CSV before close: " + std::string(std::strerror(errno)));
    failed = true;
  }
  if (!failed && ::fsync(fd_) != 0) {
    warn_once("cannot fsync arm torque CSV: " + std::string(std::strerror(errno)));
    failed = true;
  }
  const int lock_fd = ::dup(fd_);
  if (lock_fd < 0) {
    warn_once("cannot keep arm torque CSV locked through close: " + std::string(std::strerror(errno)));
    failed = true;
  }
  FILE * stream = file_;
  file_ = nullptr;
  fd_ = -1;
  if (std::fclose(stream) != 0) {
    warn_once("cannot close arm torque CSV: " + std::string(std::strerror(errno)));
    failed = true;
  }
  if (failed) {
    warn_once("arm torque CSV left as .part because writing failed");
  } else if (::link(part_path_.c_str(), complete_path_.c_str()) != 0) {
    warn_once(
      "cannot finalize arm torque CSV '" + complete_path_.string() + "': " +
      std::strerror(errno));
  } else if (::unlink(part_path_.c_str()) != 0) {
    warn_once(
      "cannot remove finalized arm torque .part '" + part_path_.string() + "': " +
      std::strerror(errno));
  }
  if (lock_fd >= 0) {
    ::flock(lock_fd, LOCK_UN);
    ::close(lock_fd);
  }
}

void ArmTorqueLog::warn_once(const std::string & message) noexcept
{
  if (warning_count_ >= 3) {return;}
  ++warning_count_;
  try {
    std::cerr << "Arm torque logging: " << message << '\n';
  } catch (...) {
  }
}

}  // namespace chassis
