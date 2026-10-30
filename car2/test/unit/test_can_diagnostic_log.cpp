#include "motor/iswv/diagnostic_log.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
    ("can-diagnostic-test-" + name + "-" + std::to_string(::getpid()));
  std::error_code error;
  fs::remove_all(path, error);
  fs::create_directories(path);
  return path;
}

std::vector<fs::path> diagnostic_files(const fs::path & dir)
{
  std::vector<fs::path> result;
  for (const auto & entry : fs::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    if (name.rfind("can-diagnostic-", 0) == 0 && name.size() > 4 &&
      name.substr(name.size() - 4) == ".log")
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

std::string wait_for_content(
  const fs::path & dir,
  const std::string & expected,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(3500))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string latest;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto & file : diagnostic_files(dir)) {
      latest = read_file(file);
      if (latest.find(expected) != std::string::npos) {
        return latest;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return latest;
}

void write_file(const fs::path & path, const std::string & content = "x")
{
  std::ofstream out(path);
  out << content;
}

void expect_contains(const std::string & haystack, const std::string & needle)
{
  if (haystack.find(needle) == std::string::npos) {
    throw std::runtime_error("missing expected text: " + needle);
  }
}

void captures_pre_and_post_events()
{
  auto dir = unique_dir("pre-post");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("before-a");
    log.record("before-b");
    log.trigger("sdo timeout");
    log.record("after-a");
    log.record("after-b");
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 1, "expected one diagnostic file");
  const auto content = read_file(files.front());
  expect_contains(content, "reason=sdo timeout");
  expect_contains(content, "before-a");
  expect_contains(content, "before-b");
  expect_contains(content, "after-a");
  expect_contains(content, "after-b");
  fs::remove_all(dir);
}

void destructor_flushes_active_capture()
{
  auto dir = unique_dir("flush");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("kept-before-dtor");
    log.trigger("shutdown flush");
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 1, "destructor must flush active capture");
  expect_contains(read_file(files.front()), "kept-before-dtor");
  fs::remove_all(dir);
}

void active_capture_writes_after_window_without_destruction()
{
  auto dir = unique_dir("auto-window");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("window-before");
    log.trigger("automatic flush");
    log.record("window-after");
    const auto content = wait_for_content(dir, "window-after");
    require(!content.empty(), "active capture should write complete content after post-trigger window");
    expect_contains(content, "reason=automatic flush");
    expect_contains(content, "window-before");
    expect_contains(content, "window-after");
  }
  fs::remove_all(dir);
}

void cleanup_retains_recent_and_unrelated_entries()
{
  auto dir = unique_dir("cleanup");
  const auto old_log = dir / "can-diagnostic-can0-123-1-20000101-000000-000000.log";
  const auto recent_log = dir / "can-diagnostic-can0-123-2-20000101-000001-000000.log";
  const auto other_channel = dir / "can-diagnostic-can1-123-1-20000101-000000-000000.log";
  const auto loose_prefix = dir / "can-diagnostic-can0-important.log";
  const auto unrelated = dir / "notes.log";
  const auto nested = dir / "can-diagnostic-can0-dir.log";
  const auto symlink = dir / "can-diagnostic-can0-123-3-20000101-000002-000000.log";
  const auto symlink_target = dir / "target.log";

  write_file(old_log);
  write_file(recent_log);
  write_file(other_channel);
  write_file(loose_prefix);
  write_file(unrelated);
  fs::create_directory(nested);
  write_file(symlink_target);
  std::error_code error;
  fs::create_symlink(symlink_target, symlink, error);

  const auto now = fs::file_time_type::clock::now();
  fs::last_write_time(old_log, now - std::chrono::hours(169));
  fs::last_write_time(recent_log, now - std::chrono::hours(167));
  fs::last_write_time(other_channel, now - std::chrono::hours(169));
  fs::last_write_time(loose_prefix, now - std::chrono::hours(169));

  {
    iswv::canopen::DiagnosticLog log(dir.string(), "retention");
  }

  require(!fs::exists(old_log), "old matching log should be removed");
  require(fs::exists(recent_log), "recent matching log should be retained");
  require(!fs::exists(other_channel), "old matching log from any channel should be removed");
  require(fs::exists(loose_prefix), "loosely matching prefix file should be retained");
  require(fs::exists(unrelated), "unrelated file should be retained");
  require(fs::exists(nested), "matching directory should be retained");
  if (!error) {
    require(fs::is_symlink(symlink), "matching symlink should be retained");
  }
  fs::remove_all(dir);
}

void repeated_construction_does_not_overwrite_logs()
{
  auto dir = unique_dir("no-overwrite");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("first capture");
    log.trigger("first");
  }
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("second capture");
    log.trigger("second");
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 2, "same-process same-channel captures should not overwrite");
  std::string combined;
  for (const auto & file : files) {
    combined += read_file(file);
  }
  expect_contains(combined, "first capture");
  expect_contains(combined, "second capture");
  fs::remove_all(dir);
}

void ring_buffer_keeps_latest_4096_events()
{
  auto dir = unique_dir("ring-limit");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    for (int index = 0; index < 4100; ++index) {
      log.record("ring-event-" + std::to_string(10000 + index));
    }
    log.trigger("ring limit");
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 1, "ring limit should write one file");
  const auto content = read_file(files.front());
  expect_contains(content, "events=4096");
  require(
    content.find("ring-event-10000") == std::string::npos,
    "oldest ring event should be evicted");
  expect_contains(content, "ring-event-10004");
  expect_contains(content, "ring-event-14099");
  fs::remove_all(dir);
}

void capture_limit_records_dropped_events()
{
  auto dir = unique_dir("capture-limit");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    for (int index = 0; index < 4096; ++index) {
      log.record("pre-" + std::to_string(index));
    }
    log.trigger("capture limit");
    for (int index = 0; index < 5000; ++index) {
      log.record("post-" + std::to_string(index));
    }
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 1, "capture limit should write one file");
  const auto content = read_file(files.front());
  expect_contains(content, "events=8192");
  expect_contains(content, "dropped_events=904");
  expect_contains(content, "pre-0");
  expect_contains(content, "post-4095");
  require(
    content.find("post-4096") == std::string::npos,
    "events beyond capture limit should be dropped");
  fs::remove_all(dir);
}

void excess_trigger_count_is_saved()
{
  auto dir = unique_dir("trigger-limit");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("trigger-base");
    log.trigger("first");
    log.trigger("second");
    log.trigger("third should drop");
  }
  const auto files = diagnostic_files(dir);
  require(files.size() == 2, "only two active captures should be retained");
  std::string combined;
  for (const auto & file : files) {
    combined += read_file(file);
  }
  expect_contains(combined, "dropped_captures=1");
  fs::remove_all(dir);
}

void no_fault_does_not_write_bulk_events()
{
  auto dir = unique_dir("quiet");
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    for (int index = 0; index < 200; ++index) {
      log.record("quiet event " + std::to_string(index));
    }
  }
  require(diagnostic_files(dir).empty(), "events without trigger should not create files");
  fs::remove_all(dir);
}

void disabled_and_unwritable_paths_do_not_throw()
{
  {
    iswv::canopen::DiagnosticLog disabled("", "can0");
    disabled.record("ignored");
    disabled.trigger("ignored");
  }

  auto dir = unique_dir("unwritable");
  fs::permissions(
    dir,
    fs::perms::owner_read | fs::perms::owner_exec,
    fs::perm_options::replace);
  {
    iswv::canopen::DiagnosticLog log(dir.string(), "can0");
    log.record("will fail to write");
    log.trigger("permission denied");
  }
  fs::permissions(
    dir,
    fs::perms::owner_all,
    fs::perm_options::replace);
  fs::remove_all(dir);
}

}  // namespace

int main()
{
  try {
    captures_pre_and_post_events();
    destructor_flushes_active_capture();
    active_capture_writes_after_window_without_destruction();
    cleanup_retains_recent_and_unrelated_entries();
    repeated_construction_does_not_overwrite_logs();
    ring_buffer_keeps_latest_4096_events();
    capture_limit_records_dropped_events();
    excess_trigger_count_is_saved();
    no_fault_does_not_write_bulk_events();
    disabled_and_unwritable_paths_do_not_throw();
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
