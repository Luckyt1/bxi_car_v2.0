#include "arm/torque_log.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
namespace fs = std::filesystem;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

fs::path unique_dir(const std::string & name)
{
  auto path = fs::temp_directory_path() /
    ("arm-torque-log-test-" + name + "-" + std::to_string(::getpid()));
  std::error_code error;
  fs::remove_all(path, error);
  fs::create_directories(path);
  return path;
}

std::vector<fs::path> files_with_suffix(const fs::path & dir, const std::string & suffix)
{
  std::vector<fs::path> result;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {continue;}
    const auto name = entry.path().filename().string();
    if (name.rfind("arm-torque-", 0) == 0 &&
      name.size() >= suffix.size() &&
      name.substr(name.size() - suffix.size()) == suffix)
    {
      result.push_back(entry.path());
    }
  }
  return result;
}

std::string read_file(const fs::path & path)
{
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::string wait_for_part_content(
  const fs::path & dir,
  const std::string & expected,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string latest;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto parts = files_with_suffix(dir, ".csv.part");
    for (const auto & part : parts) {
      latest = read_file(part);
      if (latest.find(expected) != std::string::npos) {return latest;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return latest;
}

chassis::ArmTorqueSample sample_at(std::chrono::steady_clock::time_point at)
{
  chassis::ArmTorqueSample sample;
  sample.sampled_at = at;
  return sample;
}

void expect_contains(const std::string & haystack, const std::string & needle)
{
  if (haystack.find(needle) == std::string::npos) {
    throw std::runtime_error("missing expected text: " + needle);
  }
}

void empty_session_does_not_create_csv()
{
  auto dir = unique_dir("empty");
  {
    chassis::ArmTorqueLog log(dir.string());
    log.finish();
    log.finish();
  }
  require(files_with_suffix(dir, ".csv").empty(), "empty session should not create csv");
  require(files_with_suffix(dir, ".csv.part").empty(), "empty session should not create part");
  fs::remove_all(dir);
}

void writes_four_axis_signed_values_and_nm_conversion()
{
  auto dir = unique_dir("values");
  fs::path finished;
  {
    chassis::ArmTorqueLog log(dir.string());
    auto sample = sample_at(std::chrono::steady_clock::now());
    sample.permille = {500, -250, std::nullopt, 100};
    sample.rated_mnm = {2000, 4000, 3000, 0};
    log.record(sample);
    log.finish();
    log.finish();
    finished = log.finished_path();
  }
  const auto csvs = files_with_suffix(dir, ".csv");
  require(csvs.size() == 1, "expected one completed csv");
  require(finished == csvs.front(), "finished_path should point at completed csv");
  const auto content = read_file(csvs.front());
  expect_contains(
    content,
    "elapsed_s,wall_time_s,motor1_permille,motor1_rated_mnm,motor1_nm,"
    "motor2_permille,motor2_rated_mnm,motor2_nm,"
    "motor3_permille,motor3_rated_mnm,motor3_nm,"
    "motor4_permille,motor4_rated_mnm,motor4_nm,dropped_samples");
  expect_contains(content, "0.000000");
  expect_contains(content, ",500,2000,1.000000,-250,4000,-1.000000,,3000,,100,0,,0\n");
  fs::remove_all(dir);
}

void worker_flushes_part_before_finish()
{
  auto dir = unique_dir("flush");
  {
    chassis::ArmTorqueLog log(dir.string());
    auto sample = sample_at(std::chrono::steady_clock::now());
    sample.permille[0] = 123;
    sample.rated_mnm[0] = 1000;
    log.record(sample);
    const auto content = wait_for_part_content(dir, "123,1000,0.123000");
    require(!content.empty(), "worker should flush .part while log is still active");
  }
  require(files_with_suffix(dir, ".csv").size() == 1, "destructor should finalize csv");
  fs::remove_all(dir);
}

void active_part_is_locked()
{
  auto dir = unique_dir("lock");
  fs::path part;
  {
    chassis::ArmTorqueLog log(dir.string());
    auto sample = sample_at(std::chrono::steady_clock::now());
    sample.permille[0] = 1;
    log.record(sample);
    require(!wait_for_part_content(dir, "1").empty(), "expected active part file");
    const auto parts = files_with_suffix(dir, ".csv.part");
    require(parts.size() == 1, "expected one active part");
    part = parts.front();
    const int fd = ::open(part.c_str(), O_RDWR | O_CLOEXEC);
    require(fd >= 0, "test should open active part");
    const int locked = ::flock(fd, LOCK_EX | LOCK_NB);
    require(locked != 0, "active part should hold exclusive lock");
    ::close(fd);
    log.finish();
  }
  require(!fs::exists(part), "finished session should rename part away");
  fs::remove_all(dir);
}

void repeated_sessions_do_not_overwrite()
{
  auto dir = unique_dir("unique");
  {
    chassis::ArmTorqueLog first(dir.string());
    first.record(sample_at(std::chrono::steady_clock::now()));
    first.finish();
    chassis::ArmTorqueLog second(dir.string());
    second.record(sample_at(std::chrono::steady_clock::now()));
    second.finish();
  }
  require(files_with_suffix(dir, ".csv").size() == 2, "sessions should use unique names");
  fs::remove_all(dir);
}

void finalize_does_not_overwrite_existing_completed_csv()
{
  auto dir = unique_dir("no-overwrite");
  fs::path part;
  fs::path completed;
  {
    chassis::ArmTorqueLog log(dir.string());
    auto sample = sample_at(std::chrono::steady_clock::now());
    sample.permille[0] = 77;
    log.record(sample);
    require(!wait_for_part_content(dir, "77").empty(), "expected active part file");
    const auto parts = files_with_suffix(dir, ".csv.part");
    require(parts.size() == 1, "expected one active part");
    part = parts.front();
    const auto part_name = part.string();
    completed = part_name.substr(0, part_name.size() - std::string(".part").size());
    {
      std::ofstream out(completed);
      out << "existing";
    }
    log.finish();
  }
  require(read_file(completed) == "existing", "completed csv must not be overwritten");
  require(fs::exists(part), "part should remain for recovery when final name already exists");
  fs::remove_all(dir);
}

void unusable_path_does_not_throw_from_record_or_finish()
{
  auto dir = unique_dir("bad-path");
  const auto regular_file = dir / "not-a-directory";
  {
    std::ofstream out(regular_file);
    out << "x";
  }
  chassis::ArmTorqueLog log(regular_file.string());
  log.record(sample_at(std::chrono::steady_clock::now()));
  log.finish();
  require(read_file(regular_file) == "x", "regular file should be left untouched");
  fs::remove_all(dir);
}

}  // namespace

int main()
{
  try {
    empty_session_does_not_create_csv();
    writes_four_axis_signed_values_and_nm_conversion();
    worker_flushes_part_before_finish();
    active_part_is_locked();
    repeated_sessions_do_not_overwrite();
    finalize_does_not_overwrite_existing_completed_csv();
    unusable_path_does_not_throw_from_record_or_finish();
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
