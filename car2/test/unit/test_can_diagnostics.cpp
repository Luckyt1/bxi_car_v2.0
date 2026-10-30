#include "motor/iswv/communication.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoOptions;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class TempDirectory
{
public:
  explicit TempDirectory(const char * pattern_prefix)
  {
    std::string pattern = std::string("/tmp/") + pattern_prefix + "-XXXXXX";
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    const char * created = ::mkdtemp(writable.data());
    require(created != nullptr, "cannot create temporary diagnostic directory");
    path_ = created;
  }

  ~TempDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept {return path_;}

private:
  std::filesystem::path path_;
};

iswv::canopen::MasterOptions diagnostic_options(const std::filesystem::path & directory)
{
  iswv::canopen::MasterOptions options;
  options.timer_resolution = 1ms;
  options.diagnostic_log_directory = directory.string();
  return options;
}

iswv::CanFrame upload_response(std::uint8_t node_id, ObjectAddress object, std::uint32_t value)
{
  iswv::CanFrame frame;
  frame.id = 0x580U + node_id;
  frame.size = 8;
  frame.data[0] = 0x43;
  frame.data[1] = static_cast<std::uint8_t>(object.index & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>(object.index >> 8U);
  frame.data[3] = object.subindex;
  frame.data[4] = static_cast<std::uint8_t>(value & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  frame.data[6] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  frame.data[7] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
  return frame;
}

iswv::CanFrame abort_response(
  std::uint8_t node_id,
  ObjectAddress object,
  std::uint32_t abort_code)
{
  auto frame = upload_response(node_id, object, 0);
  frame.data[0] = 0x80;
  frame.data[4] = static_cast<std::uint8_t>(abort_code & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((abort_code >> 8U) & 0xFFU);
  frame.data[6] = static_cast<std::uint8_t>((abort_code >> 16U) & 0xFFU);
  frame.data[7] = static_cast<std::uint8_t>((abort_code >> 24U) & 0xFFU);
  return frame;
}

ObjectAddress request_object(const iswv::CanFrame & request)
{
  return ObjectAddress{
    static_cast<std::uint16_t>(
      request.data[1] | (static_cast<std::uint16_t>(request.data[2]) << 8U)),
    request.data[3]};
}

bool contains(const std::string & text, const std::string & expected)
{
  return text.find(expected) != std::string::npos;
}

std::vector<std::filesystem::path> diagnostic_files(const std::filesystem::path & directory)
{
  std::vector<std::filesystem::path> files;
  if (!std::filesystem::exists(directory)) {
    return files;
  }
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.find("can-diagnostic-") == 0 && entry.path().extension() == ".log") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream input(path);
  require(input.good(), "cannot open diagnostic log");
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string read_all_logs(const std::filesystem::path & directory)
{
  std::ostringstream buffer;
  for (const auto & file : diagnostic_files(directory)) {
    buffer << read_file(file) << '\n';
  }
  return buffer.str();
}

template<typename Predicate>
std::string wait_for_log(
  const std::filesystem::path & directory,
  Predicate predicate,
  const char * message)
{
  const auto deadline = std::chrono::steady_clock::now() + 1500ms;
  std::string last;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!diagnostic_files(directory).empty()) {
      last = read_all_logs(directory);
      if (predicate(last)) {
        return last;
      }
    }
    std::this_thread::sleep_for(10ms);
  }
  throw std::runtime_error(message);
}

void require_all(const std::string & text, std::initializer_list<const char *> snippets)
{
  for (const char * snippet : snippets) {
    if (!contains(text, snippet)) {
      throw std::runtime_error(std::string("missing diagnostic snippet: ") + snippet);
    }
  }
}

bool line_contains_all(const std::string & text, std::initializer_list<const char *> snippets)
{
  std::istringstream input(text);
  std::string line;
  while (std::getline(input, line)) {
    bool matched = true;
    for (const char * snippet : snippets) {
      matched = matched && contains(line, snippet);
    }
    if (matched) {
      return true;
    }
  }
  return false;
}

void timeout_log_preserves_prior_successes_and_frames()
{
  TempDirectory directory("can-diagnostics-timeout");
  auto transport = std::make_shared<iswv::FakeTransport>();
  auto * transport_ptr = transport.get();
  transport->set_send_hook(
    [transport_ptr](const iswv::CanFrame & request) {
      if (request.id != 0x601 || request.size != 8 || request.data[0] != 0x40) {
        return;
      }
      const auto object = request_object(request);
      if (object.index == 0x6063) {
        return;
      }
      transport_ptr->inject(upload_response(1, object, 0x12345678));
    });

  {
    iswv::canopen::CanopenMaster master(transport, diagnostic_options(directory.path()));
    auto node = master.node(1);
    SdoOptions sdo;
    sdo.timeout = 20ms;

    require(static_cast<bool>(node->upload_blocking({0x6041, 0x00}, sdo)),
      "status read must succeed");
    require(static_cast<bool>(node->upload_blocking({0x2601, 0x00}, sdo)),
      "error1 read must succeed");
    require(static_cast<bool>(node->upload_blocking({0x2602, 0x00}, sdo)),
      "error2 read must succeed");
    const auto failed = node->upload_blocking({0x6063, 0x00}, sdo);
    require(!failed, "position read must time out");
    require(failed.error().code == iswv::ErrorCode::timeout, "failure must be a timeout");
  }

  const auto log = wait_for_log(
    directory.path(),
    [](const std::string & text) {
      return contains(text, "SDO_TIMEOUT") && contains(text, "object=0x6063:00");
    },
    "timeout diagnostic log was not written");

  require_all(
    log,
    {"SDO_TIMEOUT", "node=1", "object=0x6063:00", "elapsed_ms=",
      "receive_queue_drops=", "event_queue_drops=", "sdo_timeouts=", "CAN_TX_ATTEMPT",
      "CAN_TX_QUEUED", "CAN_RX"});
  require(line_contains_all(log, {"SDO_OK", "object=0x6041:00"}),
    "status read must have an SDO_OK line");
  require(line_contains_all(log, {"SDO_OK", "object=0x2601:00"}),
    "error1 read must have an SDO_OK line");
  require(line_contains_all(log, {"SDO_OK", "object=0x2602:00"}),
    "error2 read must have an SDO_OK line");
  require(contains(log, "id=0x601") || contains(log, "cob_id=0x601"),
    "diagnostic log must include request frames");
  require(contains(log, "id=0x581") || contains(log, "cob_id=0x581"),
    "diagnostic log must include response frames");
}

void late_reply_after_timeout_is_logged_as_unmatched()
{
  TempDirectory directory("can-diagnostics-late");
  auto transport = std::make_shared<iswv::FakeTransport>();
  std::mutex mutex;
  std::condition_variable condition;
  bool late_response_dispatched = false;

  {
    iswv::canopen::CanopenMaster master(transport, diagnostic_options(directory.path()));
    auto subscription = master.on_raw_frame(
      [&](const iswv::CanFrame & frame) {
        if (frame.id == 0x581 && frame.size == 8 && request_object(frame) == ObjectAddress{0x6063, 0x00}) {
          std::lock_guard<std::mutex> lock(mutex);
          late_response_dispatched = true;
          condition.notify_one();
        }
      });
    auto node = master.node(1);
    SdoOptions sdo;
    sdo.timeout = 20ms;

    const auto failed = node->upload_blocking({0x6063, 0x00}, sdo);
    require(!failed, "position read must time out");
    require(failed.error().code == iswv::ErrorCode::timeout, "failure must be a timeout");

    transport->inject(upload_response(1, {0x6063, 0x00}, 42));
    std::unique_lock<std::mutex> lock(mutex);
    require(
      condition.wait_for(lock, 500ms, [&] {return late_response_dispatched;}),
      "late SDO response must be dispatched before shutdown");
  }

  const auto log = wait_for_log(
    directory.path(),
    [](const std::string & text) {
      return contains(text, "CAN_RX") && contains(text, "SDO_UNMATCHED");
    },
    "late SDO response was not captured");
  require_all(log, {"SDO_TIMEOUT", "CAN_RX", "SDO_UNMATCHED", "object=0x6063:00"});
}

void normal_success_does_not_write_packet_log()
{
  TempDirectory directory("can-diagnostics-normal");
  auto transport = std::make_shared<iswv::FakeTransport>();
  auto * transport_ptr = transport.get();
  transport->set_send_hook(
    [transport_ptr](const iswv::CanFrame & request) {
      if (request.id != 0x601 || request.size != 8 || request.data[0] != 0x40) {
        return;
      }
      transport_ptr->inject(upload_response(1, request_object(request), 0x00000027));
    });

  {
    iswv::canopen::CanopenMaster master(transport, diagnostic_options(directory.path()));
    auto node = master.node(1);
    SdoOptions sdo;
    sdo.timeout = 20ms;
    require(static_cast<bool>(node->upload_blocking({0x6041, 0x00}, sdo)),
      "normal SDO read must succeed");
    require(static_cast<bool>(node->upload_blocking({0x2601, 0x00}, sdo)),
      "normal error read must succeed");
  }

  require(diagnostic_files(directory.path()).empty(), "successful traffic must not write a log file");
}

void send_failure_triggers_diagnostic_log()
{
  TempDirectory directory("can-diagnostics-send-failure");
  auto transport = std::make_shared<iswv::FakeTransport>();

  {
    iswv::canopen::CanopenMaster master(transport, diagnostic_options(directory.path()));
    auto node = master.node(1);
    transport->close();
    SdoOptions sdo;
    sdo.timeout = 20ms;
    const auto failed = node->upload_blocking({0x6041, 0x00}, sdo);
    require(!failed, "closed transport must fail the SDO");
    require(failed.error().code == iswv::ErrorCode::transport_closed,
      "closed transport must report transport_closed");
  }

  const auto log = wait_for_log(
    directory.path(),
    [](const std::string & text) {
      return contains(text, "SDO_FAIL") && contains(text, "transport");
    },
    "send failure diagnostic log was not written");
  require_all(log, {"SDO_FAIL", "CAN_TX_ATTEMPT", "CAN_TX_FAILED", "node=1",
      "object=0x6041:00"});
}

void sdo_abort_triggers_diagnostic_log()
{
  TempDirectory directory("can-diagnostics-abort");
  auto transport = std::make_shared<iswv::FakeTransport>();
  auto * transport_ptr = transport.get();
  transport->set_send_hook(
    [transport_ptr](const iswv::CanFrame & request) {
      if (request.id != 0x601 || request.size != 8 || request.data[0] != 0x40) {
        return;
      }
      transport_ptr->inject(abort_response(1, request_object(request), 0x06020000));
    });

  {
    iswv::canopen::CanopenMaster master(transport, diagnostic_options(directory.path()));
    auto node = master.node(1);
    SdoOptions sdo;
    sdo.timeout = 20ms;
    const auto failed = node->upload_blocking({0x2601, 0x00}, sdo);
    require(!failed, "abort response must fail the SDO");
    require(failed.error().code == iswv::ErrorCode::sdo_abort, "failure must be SDO abort");
  }

  const auto log = wait_for_log(
    directory.path(),
    [](const std::string & text) {
      return contains(text, "SDO_FAIL") && contains(text, "object=0x2601:00");
    },
    "SDO abort diagnostic log was not written");
  require_all(log, {"SDO_FAIL", "CAN_TX_ATTEMPT", "CAN_TX_QUEUED", "CAN_RX",
      "object=0x2601:00", "sdo_aborts="});
}

}  // namespace

int main()
{
  try {
    timeout_log_preserves_prior_successes_and_frames();
    late_reply_after_timeout_is_logged_as_unmatched();
    normal_success_does_not_write_packet_log();
    send_failure_triggers_diagnostic_log();
    sdo_abort_triggers_diagnostic_log();
    std::cout << "CAN diagnostic tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "CAN diagnostic tests: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
