#include "motor/iswv/motor.h"
#include "motor/yiyou/motor.h"
#include "motor/bxi/communication.h"
#include "../support/bxi_fake_transport.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Exception = std::exception, typename Function>
std::string rejects(Function function)
{
  try {
    function();
  } catch (const Exception & error) {
    return error.what();
  }
  throw std::runtime_error("expected operation to be rejected");
}

struct Access
{
  bool write;
  std::uint16_t index;
  std::uint32_t value;
};

// A device-side object dictionary, shared by CAN SDO and EtherCAT CoE fakes.
// State changes are responses to controlwords, not prescribed read sequences.
class Dictionary
{
public:
  Dictionary()
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 12345);
    put<std::int32_t>(0x60FF, 2000);
    put<std::int16_t>(0x6071, 500);
    put<std::int16_t>(0x60B2, 100);
  }

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  iswv::Result<SdoResponse> read(ObjectAddress object)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = values_.find(object);
    if (found == values_.end()) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    if (fail_read_ == object.index) {
      fail_read_ = 0;
      accesses_.push_back({false, object.index, 0});
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x08000020, "injected device read failure"));
    }
    SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    accesses_.push_back({false, object.index, raw(response.data.data(), response.size)});
    return iswv::Result<SdoResponse>::success(response);
  }

  iswv::Result<void> write(ObjectAddress object, const std::uint8_t * data, std::size_t size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto value = raw(data, size);
    accesses_.push_back({true, object.index, value});
    if (fail_write_ == object.index) {
      fail_write_ = 0;
      return iswv::Result<void>::failure(
        iswv::make_sdo_abort(0x08000020, "injected device write failure"));
    }
    values_[object] = {data, data + size};
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(value));}
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (value & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (value & 0x10) ? 0x1027 : 0x27; break;
        default: break;
      }
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  void fail_next_read(std::uint16_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_read_ = index;
  }

  void fail_next_write(std::uint16_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_write_ = index;
  }

  std::vector<Access> accesses() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return accesses_;
  }

  std::vector<Access> writes() const
  {
    auto result = accesses();
    result.erase(
      std::remove_if(
        result.begin(), result.end(),
        [](const Access & access) {return !access.write;}), result.end());
    return result;
  }

private:
  static std::uint32_t raw(const std::uint8_t * data, std::size_t size)
  {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {
      value |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    return value;
  }

  mutable std::mutex mutex_;
  std::map<ObjectAddress, std::vector<std::uint8_t>> values_;
  std::vector<Access> accesses_;
  std::uint16_t fail_read_{0};
  std::uint16_t fail_write_{0};
};

bool is_write(const Access & access, std::uint16_t index, std::uint32_t value)
{
  return access.write && access.index == index && access.value == value;
}

bool has_write(const std::vector<Access> & accesses, std::uint16_t index, std::uint32_t value)
{
  return std::any_of(
    accesses.begin(), accesses.end(), [ = ](const Access & access) {
      return is_write(access, index, value);
    });
}

std::vector<Access>::const_iterator find_write(
  const std::vector<Access> & accesses, std::uint16_t index, std::uint32_t value)
{
  return std::find_if(
    accesses.begin(), accesses.end(), [ = ](const Access & access) {
      return is_write(access, index, value);
    });
}

std::vector<Access>::const_iterator find_read(
  const std::vector<Access> & accesses, std::uint16_t index, std::uint32_t value)
{
  return std::find_if(
    accesses.begin(), accesses.end(), [ = ](const Access & access) {
      return !access.write && access.index == index && access.value == value;
    });
}

class CanDevice
{
public:
  CanDevice()
  {
    transport->set_send_hook(
      [this](const iswv::CanFrame & request) {
        if (request.id != 0x601 || request.size != 8) {return;}
        const ObjectAddress object{
          static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8U)),
          request.data[3]};
        iswv::CanFrame response;
        response.id = 0x581;
        response.size = 8;
        std::copy_n(request.data.begin() + 1, 3, response.data.begin() + 1);
        if (request.data[0] == 0x40) {
          const auto result = dictionary.read(object);
          if (result) {
            response.data[0] = static_cast<std::uint8_t>(0x43 | ((4 - result.value().size) << 2));
            std::copy(
              result.value().data.begin(), result.value().data.end(),
              response.data.begin() + 4);
          } else {
            abort_reply(response, result.error());
          }
        } else if ((request.data[0] & 0xe3) == 0x23) {
          const auto size = 4U - ((request.data[0] >> 2U) & 3U);
          const auto result = dictionary.write(object, request.data.data() + 4, size);
          if (result) {response.data[0] = 0x60;} else {abort_reply(response, result.error());}
        } else {
          abort_reply(response, iswv::make_sdo_abort(0x05040001, "unsupported test SDO"));
        }
        transport->inject(response);
      });
  }

  Dictionary dictionary;
  std::shared_ptr<iswv::FakeTransport> transport = std::make_shared<iswv::FakeTransport>();

private:
  static void abort_reply(iswv::CanFrame & response, const iswv::Error & error)
  {
    response.data[0] = 0x80;
    const auto bytes = iswv::canopen::encode_little_endian(
      error.sdo_abort_code.value_or(0x08000000));
    std::copy(bytes.begin(), bytes.end(), response.data.begin() + 4);
  }
};

class CoeNode final : public chassis::ethercat::Node
{
public:
  std::uint16_t id() const noexcept override {return 1;}

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    return dictionary.read(object);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    return dictionary.write(object, data, size);
  }

  Dictionary dictionary;
};

void require_iswv_stopped(const Dictionary & dictionary)
{
  const auto writes = dictionary.writes();
  require(writes.size() >= 3, "cleanup must attempt all three stop commands");
  require(is_write(writes[writes.size() - 3], 0x60FF, 0), "cleanup clears target velocity");
  require(is_write(writes[writes.size() - 2], 0x6040, 0x0b), "cleanup sends quick stop");
  require(is_write(writes.back(), 0x6040, 0), "cleanup disables voltage last");
}

void require_yiyou_mode_confirmed_before_enable(
  const std::vector<Access> & accesses, std::int8_t mode, const char * label)
{
  const auto set_mode = find_write(accesses, 0x6060, static_cast<std::uint32_t>(mode));
  const auto displayed = find_read(accesses, 0x6061, static_cast<std::uint32_t>(mode));
  const auto enable = find_write(accesses, 0x6040, 0x0f);
  require(set_mode != accesses.end(), label);
  require(displayed != accesses.end(), label);
  require(enable != accesses.end(), label);
  require(set_mode < displayed && displayed < enable, label);
}

void bxi_transport_rejects_invalid_frames()
{
  bxi_test::FakeTransport transport;
  const std::array<std::uint8_t, 8> payload{1, 2, 3, 4, 5, 6, 7, 8};
  require(!bxi::send_payload(transport, 0x800, {}, payload), "reject nonstandard CAN ID");
  require(!bxi::send_payload(transport, 1, {false, true}, payload), "reject BRS on classic CAN");
  require(transport.sent_frames().empty(), "invalid frames must not reach the transport");
  require(
    bxi::send_payload(transport, 0x7ff, {false, false}, payload).has_value(),
    "valid classic CAN frame reaches the transport");
  const auto frames = transport.sent_frames();
  require(
    frames.size() == 1 && frames[0].id == 0x7ff && frames[0].size == 8 &&
    !frames[0].fd && !frames[0].bitrate_switch &&
    std::equal(payload.begin(), payload.end(), frames[0].data.begin()),
    "communication preserves payload, ID and frame flags");
}

void iswv_tuning_order()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  unsigned callbacks = 0;
  motor.enable_rpm(
    60, 60, [&](iswv::Axis & axis) {
      ++callbacks;
      const auto accesses = device.dictionary.accesses();
      require(accesses.size() >= 3, "tuning requires disable/zero/status confirmation");
      require(is_write(accesses[0], 0x6040, 0), "disable voltage precedes tuning");
      require(is_write(accesses[1], 0x60FF, 0), "zero velocity precedes tuning");
      require(
        !accesses.back().write && accesses.back().index == 0x6041 &&
        accesses.back().value == 0x40, "tuning requires confirmed disabled state");
      require(
        axis.node()->write(iswv::objects::quick_stop_option, 2).has_value(),
        "tuning callback can issue a real SDO transaction");
    });
  require(callbacks == 1, "tuning callback runs exactly once");
  require(has_write(device.dictionary.writes(), 0x6040, 0x0f), "normal enable completes");
  motor.stop();
  require_iswv_stopped(device.dictionary);
}

void iswv_no_callback_enable_clears_zero_and_enables()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  motor.enable_rpm(60, 60);
  const auto accesses = device.dictionary.accesses();
  const auto disable = find_write(accesses, 0x6040, 0);
  const auto zero = find_write(accesses, 0x60FF, 0);
  const auto mode = find_write(
    accesses, 0x6060,
    static_cast<std::uint32_t>(iswv::cia402::OperationMode::profile_velocity));
  const auto mode_readback = find_read(
    accesses, 0x6060,
    static_cast<std::uint32_t>(iswv::cia402::OperationMode::profile_velocity));
  const auto enable = find_write(accesses, 0x6040, 0x0f);
  require(disable != accesses.end(), "two-argument enable disables voltage first");
  require(zero != accesses.end(), "two-argument enable clears target velocity");
  require(mode != accesses.end(), "two-argument enable selects PV mode");
  require(mode_readback != accesses.end(), "two-argument enable verifies PV mode");
  require(enable != accesses.end(), "two-argument enable completes CiA402 enable");
  require(
    disable < zero && zero < mode && mode < mode_readback && mode_readback < enable,
    "two-argument enable preserves zero/mode/readback order");
  motor.stop();
  require_iswv_stopped(device.dictionary);
}

void iswv_callback_failure_cleans_up()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  const auto message = rejects<std::runtime_error>(
    [&] {
      motor.enable_rpm(60, 60, [](iswv::Axis &) {throw std::runtime_error("tuning failed");});
    });
  require(message == "tuning failed", "original callback failure must propagate");
  require_iswv_stopped(device.dictionary);
  require(!has_write(device.dictionary.writes(), 0x6040, 0x0f), "failed tuning must not enable");
}

void iswv_failed_clear_never_tunes()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  device.dictionary.fail_next_write(0x60FF);
  bool tuned = false;
  const auto message = rejects<std::runtime_error>(
    [&] {
      motor.enable_rpm(60, 60, [&](iswv::Axis &) {tuned = true;});
    });
  require(!tuned, "failed zero command must prevent tuning");
  require(message.find("clear target velocity") != std::string::npos, "SDO error retains context");
  require_iswv_stopped(device.dictionary);
}

void iswv_no_callback_failed_zero_cleans_up()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  device.dictionary.fail_next_write(0x60FF);
  const auto message = rejects<std::runtime_error>([&] {motor.enable_rpm(60, 60);});
  require(message.find("clear target velocity") != std::string::npos, "SDO error retains context");
  require_iswv_stopped(device.dictionary);
  require(!has_write(device.dictionary.writes(), 0x6040, 0x0f), "failed zero must not enable");
}

void yiyou_pv_zero_mode_confirm_enable()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable(1000, 2000);
  const auto accesses = node->dictionary.accesses();
  const auto zero = find_write(accesses, 0x60FF, 0);
  const auto enable = find_write(accesses, 0x6040, 0x0f);
  require(zero != accesses.end(), "PV enable clears target velocity");
  require(enable != accesses.end(), "PV enable completes CiA402 enable");
  require(zero < enable, "PV zero target is written before enable");
  require_yiyou_mode_confirmed_before_enable(
    accesses, static_cast<std::int8_t>(iswv::cia402::OperationMode::profile_velocity),
    "PV mode display is confirmed before enable");
  motor.stop();
}

void yiyou_pp_hold_handshake_stop()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  const auto startup_accesses = node->dictionary.accesses();
  auto writes = node->dictionary.writes();
  const auto hold = std::find_if(
    writes.begin(), writes.end(), [](const Access & access) {
      return is_write(access, 0x607A, 12345);
    });
  const auto enable = std::find_if(
    writes.begin(), writes.end(), [](const Access & access) {
      return is_write(access, 0x6040, 0x0f);
    });
  require(hold < enable && enable != writes.end(), "PP seeds current position before enable");
  require(!has_write(writes, 0x6040, 0x3f), "PP enable must not trigger a motion target");
  require_yiyou_mode_confirmed_before_enable(
    startup_accesses, static_cast<std::int8_t>(iswv::cia402::OperationMode::profile_position),
    "PP mode display is confirmed before enable");
  motor.prepare_position_command(12500);
  require(motor.stage_position_command(), "PP stage waits for cleared acknowledgment");
  motor.trigger_position_command();
  require(!motor.finish_position_command(), "PP completion first clears acknowledged set-point");
  require(motor.finish_position_command(), "PP command completes after acknowledgment clears");
  require(has_write(node->dictionary.writes(), 0x607A, 12500), "staged position reaches device");
  rejects<std::logic_error>([&] {motor.set_velocity(1);});
  motor.stop();
  writes = node->dictionary.writes();
  require(is_write(writes[writes.size() - 2], 0x6040, 0x010f), "PP stops with halt");
  require(is_write(writes.back(), 0x6040, 0), "PP disables voltage after halt");
}

void yiyou_cst_zero_limits_stop()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_cst();
  const auto accesses = node->dictionary.accesses();
  const auto enable = std::find_if(
    accesses.begin(), accesses.end(), [](const Access & access) {
      return is_write(access, 0x6040, 0x0f);
    });
  for (const auto index : {0x6071, 0x60B2}) {
    const auto confirmed = std::find_if(
      accesses.begin(), accesses.end(), [ = ](const Access & access) {
        return !access.write && access.index == index && access.value == 0;
      });
    require(confirmed < enable && enable != accesses.end(), "CST confirms zero before enable");
  }
  require_yiyou_mode_confirmed_before_enable(
    accesses, 10,
    "CST mode display is confirmed before enable");
  motor.set_current_permille(1000);
  const auto count = node->dictionary.writes().size();
  rejects<std::out_of_range>([&] {motor.set_current_permille(1001);});
  rejects<std::out_of_range>([&] {motor.set_current_permille(-1001);});
  rejects<std::logic_error>([&] {motor.set_velocity(1);});
  require(node->dictionary.writes().size() == count, "invalid commands must not transmit");
  motor.stop();
  const auto writes = node->dictionary.writes();
  require(is_write(writes[writes.size() - 2], 0x6071, 0), "CST clears current before disabling");
  require(is_write(writes.back(), 0x6040, 0), "CST disables voltage");
}

void yiyou_stop_continues_after_sdo_failure()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_cst();
  node->dictionary.fail_next_write(0x6071);
  const auto message = rejects<std::runtime_error>([&] {motor.stop();});
  require(message.find("clear 0x6071") != std::string::npos, "stop preserves first failure");
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0),
    "failed zero-current command must still disable voltage");
}

void yiyou_failed_mode_display_read_cleans_up()
{
  for (const auto mode : {1, 3, 10}) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    node->dictionary.fail_next_read(0x6061);
    const auto message = rejects<std::runtime_error>(
      [&] {
        if (mode == 1) {motor.enable_position(1000, 2000, 2000);} else if (mode == 3) {
          motor.enable(1000, 2000);
        } else {motor.enable_cst();}
      });
    const auto context = mode == 10 ? "read 0x6061 while waiting for CST mode" :
      "read 0x6061 while waiting for operation mode";
    require(
      message.find(context) != std::string::npos,
      "mode-display read failure keeps operation context");
    require(
      message.find("injected device read failure") != std::string::npos,
      "mode-display read failure keeps device error");
    const auto writes = node->dictionary.writes();
    require(!has_write(writes, 0x6040, 0x0f), "failed mode read must not enable");
    require(is_write(writes.back(), 0x6040, 0), "failed mode read cleanup disables voltage");
  }
}

void yiyou_invalid_enable_does_not_write()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::invalid_argument>([&] {motor.enable_position(0, 1, 1);});
  rejects<std::invalid_argument>([&] {motor.enable(0, 1);});
  rejects<std::logic_error>([&] {motor.set_current_permille(1);});
  require(node->dictionary.writes().empty(), "invalid initial commands must not write objects");
}
}  // namespace

int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests{
    {"BXI invalid transport frames", bxi_transport_rejects_invalid_frames},
    {"ISWV tuning order", iswv_tuning_order},
    {"ISWV no-callback enable order", iswv_no_callback_enable_clears_zero_and_enables},
    {"ISWV callback failure cleanup", iswv_callback_failure_cleans_up},
    {"ISWV failed zero blocks tuning", iswv_failed_clear_never_tunes},
    {"ISWV no-callback failed zero cleanup", iswv_no_callback_failed_zero_cleans_up},
    {"Yiyou PV zero mode enable", yiyou_pv_zero_mode_confirm_enable},
    {"Yiyou PP hold handshake stop", yiyou_pp_hold_handshake_stop},
    {"Yiyou CST zero limits stop", yiyou_cst_zero_limits_stop},
    {"Yiyou failed stop still disables", yiyou_stop_continues_after_sdo_failure},
    {"Yiyou failed mode-display read cleanup", yiyou_failed_mode_display_read_cleans_up},
    {"Yiyou invalid enable", yiyou_invalid_enable_does_not_write},
  };
  unsigned failed = 0;
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception & error) {
      ++failed;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  return failed == 0 ? 0 : 1;
}
