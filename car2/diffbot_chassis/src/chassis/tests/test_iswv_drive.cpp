#include "chassis/iswv_drive.hpp"

#include "iswv/canopen/types.hpp"
#include "iswv/cia402/drive.hpp"
#include "iswv/fake_transport.hpp"
#include "iswv/iswv/object_dictionary.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <thread>

namespace
{

using namespace std::chrono_literals;

struct TestCase
{
  std::string name;
  std::function<void()> function;
};

std::vector<TestCase> & tests()
{
  static std::vector<TestCase> all;
  return all;
}

struct TestRegistration
{
  TestRegistration(std::string name, std::function<void()> function)
  {
    tests().push_back(TestCase{std::move(name), std::move(function)});
  }
};

void fail(const char * expression, const char * file, int line)
{
  std::ostringstream message;
  message << file << ':' << line << ": check failed: " << expression;
  throw std::runtime_error(message.str());
}

iswv::canopen::ObjectAddress request_object(const iswv::CanFrame & request)
{
  return iswv::canopen::ObjectAddress{
    static_cast<std::uint16_t>(request.data[1] |
    (static_cast<std::uint16_t>(request.data[2]) << 8U)),
    request.data[3]};
}

iswv::CanFrame sdo_download_response(const iswv::CanFrame & request)
{
  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = 0x60;
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  return response;
}

template<typename T>
iswv::CanFrame sdo_upload_response(const iswv::CanFrame & request, T value)
{
  const auto bytes = iswv::canopen::encode_little_endian(value);
  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = sizeof(T) == 1 ? 0x4F : (sizeof(T) == 2 ? 0x4B : 0x43);
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  std::copy(bytes.begin(), bytes.end(), response.data.begin() + 4);
  return response;
}

iswv::CanFrame sdo_abort_response(const iswv::CanFrame & request)
{
  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = 0x80;
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  response.data[4] = 0x00;
  response.data[5] = 0x00;
  response.data[6] = 0x02;
  response.data[7] = 0x06;
  return response;
}

std::int64_t decode_request_value(const iswv::CanFrame & request)
{
  switch (request.data[0]) {
    case 0x2F:
      return static_cast<std::int8_t>(request.data[4]);
    case 0x2B:
      return static_cast<std::int16_t>(
        request.data[4] | (static_cast<std::uint16_t>(request.data[5]) << 8U));
    case 0x23:
      return static_cast<std::int32_t>(
        request.data[4] |
        (static_cast<std::uint32_t>(request.data[5]) << 8U) |
        (static_cast<std::uint32_t>(request.data[6]) << 16U) |
        (static_cast<std::uint32_t>(request.data[7]) << 24U));
    default:
      return 0;
  }
}

struct Write
{
  iswv::canopen::ObjectAddress object;
  std::int64_t value{0};
};

class IswvDriveHarness
{
public:
  IswvDriveHarness()
  : transport(std::make_shared<iswv::FakeTransport>()),
    master(transport)
  {
    transport->set_send_hook(
      [this](const iswv::CanFrame & request) {
        sent.push_back(request);
        if (request.id == 0) {
          return;
        }

        const auto object = request_object(request);
        if (request.data[0] == 0x40) {
          respond_upload(request, object);
          return;
        }

        writes.push_back(Write{object, decode_request_value(request)});
        if (object == iswv::objects::control_word.address) {
          update_status(static_cast<std::uint16_t>(decode_request_value(request)));
        }
        if (abort_next_download || object == abort_on_download_object) {
          abort_next_download = false;
          abort_on_download_object = {};
          transport->inject(sdo_abort_response(request));
          return;
        }
        transport->inject(sdo_download_response(request));
      });
  }

  chassis::IswvDrive make_drive(
    double ratio = 10.0,
    std::uint32_t resolution = 65536)
  {
    auto config = iswv::AxisConfiguration::manual_travel(7);
    config.gear_ratio = ratio;
    config.encoder_resolution = resolution;
    config.inverted = true;
    return chassis::IswvDrive(master, config);
  }

  unsigned upload_count(iswv::canopen::ObjectAddress object) const
  {
    unsigned count = 0;
    for (const auto & frame : sent) {
      if (frame.id != 0 && frame.data[0] == 0x40 &&
        request_object(frame) == object)
      {
        ++count;
      }
    }
    return count;
  }

  std::vector<iswv::CanFrame> sent;
  std::vector<Write> writes;
  std::shared_ptr<iswv::FakeTransport> transport;
  iswv::canopen::CanopenMaster master;
  std::uint16_t status{0x0040};
  std::uint16_t error_status{0};
  std::uint16_t error_status_2{0};
  std::int8_t mode{static_cast<std::int8_t>(
      iswv::cia402::OperationMode::profile_velocity)};
  std::int32_t actual_velocity_raw{0};
  bool abort_next_download{false};
  iswv::canopen::ObjectAddress abort_on_download_object{};

private:
  void respond_upload(
    const iswv::CanFrame & request,
    iswv::canopen::ObjectAddress object)
  {
    if (object == iswv::objects::status_word.address) {
      transport->inject(sdo_upload_response(request, status));
    } else if (object == iswv::objects::mode.address) {
      transport->inject(sdo_upload_response(request, mode));
    } else if (object == iswv::objects::actual_velocity.address) {
      transport->inject(sdo_upload_response(request, actual_velocity_raw));
    } else if (object == iswv::objects::error_status.address) {
      transport->inject(sdo_upload_response(request, error_status));
    } else if (object == iswv::objects::error_status_2.address) {
      transport->inject(sdo_upload_response(request, error_status_2));
    } else {
      transport->inject(sdo_upload_response(request, std::uint32_t{0}));
    }
  }

  void update_status(std::uint16_t control)
  {
    if (control == iswv::cia402::control::shutdown) {
      status = 0x0031;
    } else if (control == iswv::cia402::control::switch_on) {
      status = 0x0033;
    } else if (control == iswv::cia402::control::enable_operation) {
      status = 0x0037;
    } else if (control == iswv::cia402::control::disable_voltage) {
      status = 0x0040;
    } else if (control == iswv::cia402::control::quick_stop) {
      status = 0x0007;
    }
  }
};

std::int64_t last_write_to(
  const std::vector<Write> & writes,
  iswv::canopen::ObjectAddress object)
{
  for (auto it = writes.rbegin(); it != writes.rend(); ++it) {
    if (it->object == object) {
      return it->value;
    }
  }
  fail("last write exists", __FILE__, __LINE__);
  return 0;
}

unsigned count_writes_to(
  const std::vector<Write> & writes,
  iswv::canopen::ObjectAddress object)
{
  unsigned count = 0;
  for (const auto & write : writes) {
    if (write.object == object) {
      ++count;
    }
  }
  return count;
}

std::vector<iswv::canopen::ObjectAddress> tail_write_objects(
  const std::vector<Write> & writes, std::size_t count)
{
  std::vector<iswv::canopen::ObjectAddress> objects;
  const auto start = writes.size() > count ? writes.size() - count : 0;
  for (std::size_t i = start; i < writes.size(); ++i) {
    objects.push_back(writes[i].object);
  }
  return objects;
}

}  // namespace

#define CONCAT_INNER(a, b) a ## b
#define CONCAT(a, b) CONCAT_INNER(a, b)
#define TEST_CASE(name) \
  static void CONCAT(test_function_, __LINE__)(); \
  static TestRegistration CONCAT(test_registration_, __LINE__)( \
    name, CONCAT(test_function_, __LINE__)); \
  static void CONCAT(test_function_, __LINE__)()
#define CHECK(expression) \
  do { \
    if (!(expression)) { \
      fail(#expression, __FILE__, __LINE__); \
    } \
  } while (false)
#define CHECK_EQ(lhs, rhs) \
  do { \
    const auto check_lhs = (lhs); \
    const auto check_rhs = (rhs); \
    if (!(check_lhs == check_rhs)) { \
      fail(#lhs " == " #rhs, __FILE__, __LINE__); \
    } \
  } while (false)
#define CHECK_NEAR(lhs, rhs, tolerance) \
  do { \
    const auto check_lhs = (lhs); \
    const auto check_rhs = (rhs); \
    const auto check_tolerance = (tolerance); \
    if (!((check_lhs >= check_rhs - check_tolerance) && \
      (check_lhs <= check_rhs + check_tolerance))) { \
      fail(#lhs " ~= " #rhs, __FILE__, __LINE__); \
    } \
  } while (false)

TEST_CASE("IswvDrive is neither copyable nor movable")
{
  CHECK(!std::is_copy_constructible<chassis::IswvDrive>::value);
  CHECK(!std::is_copy_assignable<chassis::IswvDrive>::value);
  CHECK(!std::is_move_constructible<chassis::IswvDrive>::value);
  CHECK(!std::is_move_assignable<chassis::IswvDrive>::value);
}

TEST_CASE("IswvDrive rejects invalid reduction or encoder configuration")
{
  IswvDriveHarness harness;

  bool rejected_zero_ratio = false;
  try {
    (void)harness.make_drive(0.0);
  } catch (const std::invalid_argument &) {
    rejected_zero_ratio = true;
  }
  CHECK(rejected_zero_ratio);

  bool rejected_nan_ratio = false;
  try {
    (void)harness.make_drive(std::numeric_limits<double>::quiet_NaN());
  } catch (const std::invalid_argument &) {
    rejected_nan_ratio = true;
  }
  CHECK(rejected_nan_ratio);

  bool rejected_zero_resolution = false;
  try {
    (void)harness.make_drive(10.0, 0);
  } catch (const std::invalid_argument &) {
    rejected_zero_resolution = true;
  }
  CHECK(rejected_zero_resolution);
}

TEST_CASE("IswvDrive converts output RPM commands through the gear ratio")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive(10.0);

  drive.set_velocity_rpm(120.0);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::target_velocity.address),
    21474836LL);

  drive.set_velocity_rpm(-30.0);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::target_velocity.address),
    -5368709LL);
}

TEST_CASE("IswvDrive set_velocity stops the axis before rethrowing on failure")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive();
  harness.abort_next_download = true;

  bool threw = false;
  try {
    drive.set_velocity_rpm(1.0);
  } catch (const std::runtime_error &) {
    threw = true;
  }

  CHECK(threw);
  CHECK_EQ(
    count_writes_to(harness.writes, iswv::objects::target_velocity.address),
    2U);
  const auto tail = tail_write_objects(harness.writes, 3);
  CHECK_EQ(tail[0], iswv::objects::target_velocity.address);
  CHECK_EQ(tail[1], iswv::objects::control_word.address);
  CHECK_EQ(tail[2], iswv::objects::control_word.address);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::control_word.address),
    static_cast<std::int64_t>(iswv::cia402::control::disable_voltage));
}

TEST_CASE("IswvDrive enable uses output-axis acceleration units and startup sequence")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive(10.0);

  drive.enable_rpm(600.0, 300.0);

  CHECK_EQ(harness.sent[0].id, 0U);
  CHECK_EQ(
    harness.sent[0].data[0], static_cast<std::uint8_t>(
      iswv::canopen::NmtCommand::enter_pre_operational));
  CHECK_EQ(harness.sent[0].data[1], 7U);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::profile_acceleration.address),
    107374LL);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::profile_deceleration.address),
    53687LL);
  CHECK_EQ(last_write_to(harness.writes, iswv::objects::direction.address), 0LL);
  CHECK_EQ(
    count_writes_to(harness.writes, iswv::objects::heartbeat_producer_time.address),
    1U);
  CHECK_EQ(
    count_writes_to(
      harness.writes,
      iswv::objects::communication_interruption_mode.address),
    1U);

  bool read_mode = false;
  bool started = false;
  for (const auto & frame : harness.sent) {
    if (frame.id == 0x607U && frame.data[0] == 0x40 &&
      request_object(frame) == iswv::objects::mode.address)
    {
      read_mode = true;
    }
    if (frame.id == 0 && frame.data[0] == static_cast<std::uint8_t>(
        iswv::canopen::NmtCommand::start) &&
      frame.data[1] == 7U)
    {
      started = true;
    }
  }
  CHECK(read_mode);
  CHECK(started);
}

TEST_CASE("IswvDrive enable stops the axis before rethrowing on failure")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive();
  harness.abort_on_download_object = iswv::objects::profile_deceleration.address;

  bool threw = false;
  try {
    drive.enable_rpm(600.0, 300.0);
  } catch (const std::runtime_error &) {
    threw = true;
  }

  CHECK(threw);
  const auto tail = tail_write_objects(harness.writes, 3);
  CHECK_EQ(tail[0], iswv::objects::target_velocity.address);
  CHECK_EQ(tail[1], iswv::objects::control_word.address);
  CHECK_EQ(tail[2], iswv::objects::control_word.address);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::control_word.address),
    static_cast<std::int64_t>(iswv::cia402::control::disable_voltage));
}

TEST_CASE("IswvDrive stop works before enable and attempts every cleanup command")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive();
  harness.abort_next_download = true;

  bool threw = false;
  try {
    drive.stop();
  } catch (const std::runtime_error &) {
    threw = true;
  }

  CHECK(threw);
  CHECK_EQ(
    count_writes_to(harness.writes, iswv::objects::target_velocity.address),
    1U);
  CHECK_EQ(
    last_write_to(harness.writes, iswv::objects::control_word.address),
    static_cast<std::int64_t>(iswv::cia402::control::disable_voltage));
  CHECK_EQ(
    count_writes_to(harness.writes, iswv::objects::control_word.address),
    2U);
}

TEST_CASE("IswvDrive reads only status errors and velocity for actual output RPM")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive(10.0);
  harness.actual_velocity_raw = 17895697;

  CHECK_NEAR(drive.actual_velocity_rpm(), 100.0, 0.001);
  CHECK_EQ(harness.upload_count(iswv::objects::status_word.address), 1U);
  CHECK_EQ(harness.upload_count(iswv::objects::error_status.address), 1U);
  CHECK_EQ(harness.upload_count(iswv::objects::error_status_2.address), 1U);
  CHECK_EQ(harness.upload_count(iswv::objects::actual_velocity.address), 1U);
  CHECK_EQ(harness.upload_count(iswv::objects::actual_position.address), 0U);
  CHECK_EQ(harness.upload_count(iswv::objects::actual_current.address), 0U);
  CHECK_EQ(harness.upload_count(iswv::objects::digital_inputs.address), 0U);
}

TEST_CASE("IswvDrive actual velocity requires operation enabled after enable")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive();
  drive.enable_rpm(600.0, 300.0);
  harness.status = 0x0033;

  bool threw = false;
  try {
    (void)drive.actual_velocity_rpm();
  } catch (const std::runtime_error &) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE("IswvDrive treats any non-zero error status as a fault")
{
  IswvDriveHarness harness;
  chassis::IswvDrive drive = harness.make_drive();
  harness.error_status_2 = 0x0020;

  bool threw = false;
  try {
    (void)drive.actual_velocity_rpm();
  } catch (const std::runtime_error &) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE("PDO feedback is nonblocking and each required stream must remain fresh")
{
  IswvDriveHarness harness;
  auto drive = harness.make_drive();
  drive.enable_rpm(600.0, 300.0);
  auto rejected = [&] {
      try {
        (void)drive.actual_velocity_pdo_rpm();
      } catch (const std::runtime_error &) {
        return true;
      }
      return false;
    };
  CHECK(rejected());  // Missing data must never look like zero RPM.
  auto inject = [&](unsigned number, std::uint16_t error = 0) {
      iswv::CanFrame frame;
      frame.id = 0x180U + (number - 1) * 0x100U + 7;
      frame.size = number == 3 ? 8 : 6;
      if (number == 1) {frame.data[0] = 0x37;}
      if (number == 3) {
        frame.data[4] = error & 0xFF;
        frame.data[5] = error >> 8;
      }
      harness.transport->inject(frame);
    };
  for (unsigned i = 1; i <= 3; ++i) {
    inject(i);
  }
  std::this_thread::sleep_for(20ms);
  const auto sent_before = harness.sent.size();
  CHECK_EQ(drive.actual_velocity_pdo_rpm(), 0.0);
  CHECK_EQ(harness.sent.size(), sent_before);  // No SDO request on feedback reads.
  inject(3, 0x800);
  std::this_thread::sleep_for(20ms);
  CHECK(rejected());
  std::this_thread::sleep_for(260ms);
  inject(1);
  inject(3);
  std::this_thread::sleep_for(20ms);
  CHECK(rejected());  // Fresh status/errors cannot hide stale velocity.
  inject(2);
  std::this_thread::sleep_for(20ms);
  CHECK_EQ(drive.actual_velocity_pdo_rpm(), 0.0);
}

int main()
{
  unsigned failures = 0;
  for (const auto & test : tests()) {
    try {
      test.function();
      std::cout << "[PASS] " << test.name << '\n';
    } catch (const std::exception & error) {
      ++failures;
      std::cerr << "[FAIL] " << test.name << ": " << error.what() << '\n';
    } catch (...) {
      ++failures;
      std::cerr << "[FAIL] " << test.name << ": unknown exception\n";
    }
  }
  std::cout << tests().size() << " tests, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
