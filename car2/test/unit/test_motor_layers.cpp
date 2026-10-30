#include "motor/iswv/motor.h"
#include "motor/yiyou/motor.h"
#include "motor/bxi/communication.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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
  explicit CoeNode(std::uint16_t node_id = 1) : id_(node_id) {}
  std::uint16_t id() const noexcept override {return id_;}

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions options) override
  {
    if (object.index == 0x607A) {target_read_timeout = options.timeout;}
    return dictionary.read(object);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions options) override
  {
    if (object.index == 0x607A) {
      target_write_timeout = options.timeout;
      if (options.timeout < target_write_delay) {
        return iswv::Result<void>::failure(iswv::ErrorCode::timeout, "simulated PDO recovery delay");
      }
    }
    return dictionary.write(object, data, size);
  }

  std::optional<std::int16_t> cached_torque_permille() const override
  {
    return torque_permille;
  }

  std::optional<std::uint32_t> cached_rated_torque_millinewton_meter() const override
  {
    return rated_torque_mnm;
  }

  std::uint16_t id_;
  Dictionary dictionary;
  std::optional<std::int16_t> torque_permille;
  std::optional<std::uint32_t> rated_torque_mnm;
  std::chrono::milliseconds target_write_delay{0}, target_write_timeout{0}, target_read_timeout{0};
};

void require_iswv_stopped(const Dictionary & dictionary)
{
  const auto writes = dictionary.writes();
  require(writes.size() >= 3, "cleanup must attempt all three stop commands");
  require(is_write(writes[writes.size() - 3], 0x60FF, 0), "cleanup clears target velocity");
  require(is_write(writes[writes.size() - 2], 0x6040, 0x0b), "cleanup sends quick stop");
  require(is_write(writes.back(), 0x6040, 0), "cleanup disables voltage last");
}

void bxi_transport_rejects_invalid_frames()
{
  iswv::FakeTransport transport;
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

void yiyou_pp_hold_handshake_stop()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
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

void yiyou_pp_stage_has_bounded_recovery_budget()
{
  using namespace std::chrono_literals;
  for (const auto delay : {6ms, 11ms}) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    node->target_write_delay = delay;
    motor.prepare_position_command(12500);
    if (delay == 6ms) {
      require(motor.stage_position_command(), "two lost cycles followed by valid delivery fit budget");
      require(node->target_read_timeout == 5ms, "mailbox verification remains bounded to 5 ms");
    } else {
      rejects<std::runtime_error>([&] {motor.stage_position_command();});
      rejects<std::logic_error>([&] {motor.trigger_position_command();});
    }
    require(node->target_write_timeout == 10ms, "PDO staging wait is bounded to 10 ms");
    require(!has_write(node->dictionary.writes(), 0x6040, 0x3f), "staging alone never triggers motion");
    motor.stop();
  }
}

void yiyou_pp_halt_cancels_every_handshake_stage()
{
  // idle (with an accepted target), clearing, staged, triggered, resetting.
  for (unsigned phase = 0; phase < 5; ++phase) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    motor.move_to_position(12500);
    if (phase >= 1) {motor.prepare_position_command(13000);}
    if (phase >= 2) {require(motor.stage_position_command(), "stage old command");}
    if (phase >= 3) {motor.trigger_position_command();}
    if (phase == 4) {require(!motor.finish_position_command(), "reset old acknowledge");}
    if (phase == 1 || phase == 4) {
      node->dictionary.put<std::uint16_t>(0x6041, 0x1027);  // Delayed acknowledge clear.
    } else if (phase == 3) {
      node->dictionary.put<std::uint16_t>(0x6041, 0x0027);  // Delayed acknowledge set.
    }

    const auto before_halt = node->dictionary.writes().size();
    motor.halt_position();
    auto writes = node->dictionary.writes();
    require(writes.size() == before_halt + 1, "halt must only write its controlword");
    require(is_write(writes.back(), 0x6040, 0x012f), "halt preserves enable and clears bit 4");
    node->dictionary.put<std::uint16_t>(0x6041, 0x1427);  // Late old ACK and target-reached.
    require(motor.finish_position_command(), "cancelled handshake is idle despite late ACK");
    rejects<std::logic_error>([&] {motor.stage_position_command();});
    rejects<std::logic_error>([&] {motor.trigger_position_command();});
    require(!motor.position_reached(), "cancelled target must not become reached");
    require(
      node->dictionary.writes().size() == writes.size(),
      "late completion must not write a controlword or revive the cancelled target");

    motor.prepare_position_command(12000);
    require(
      is_write(node->dictionary.writes().back(), 0x6040, 0x012f),
      "preparing a replacement must not resume the previous target");
    node->dictionary.put<std::uint16_t>(0x6041, 0x1027);
    const auto waiting_writes = node->dictionary.writes().size();
    require(!motor.stage_position_command(), "replacement waits for old ACK to clear");
    require(
      node->dictionary.writes().size() == waiting_writes,
      "waiting for old ACK must leave halt and target unchanged");
    node->dictionary.put<std::uint16_t>(0x6041, 0x0027);
    require(motor.stage_position_command(), "replacement stages after ACK clears");
    writes = node->dictionary.writes();
    require(is_write(writes.back(), 0x607A, 12000), "replacement target is staged under halt");
    require(
      !has_write(std::vector<Access>(writes.begin() + before_halt, writes.end()), 0x6040, 0x003f),
      "staging must not release halt");
    motor.trigger_position_command();
    require(
      is_write(node->dictionary.writes().back(), 0x6040, 0x003f),
      "only the verified replacement trigger releases halt");
    require(!motor.finish_position_command(), "replacement acknowledge resets");
    require(motor.finish_position_command(), "replacement handshake completes");
    motor.stop();
  }
}

void yiyou_pp_blocking_replacement_preserves_halt()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  motor.move_to_position(13000);
  motor.halt_position();
  const auto before = node->dictionary.accesses().size();
  motor.move_to_position(12000);
  const auto accesses = node->dictionary.accesses();
  bool confirmed = false;
  bool triggered = false;
  for (auto it = accesses.begin() + before; it != accesses.end(); ++it) {
    if (!it->write && it->index == 0x607A && it->value == 12000) {confirmed = true;}
    if (it->write && it->index == 0x6040) {
      if (it->value == 0x003f) {
        require(confirmed, "replacement target must be read back before releasing halt");
        triggered = true;
      } else if (!triggered) {
        require(it->value == 0x012f, "blocking prepare must preserve halt");
      }
    }
  }
  require(triggered, "blocking replacement sends a new trigger");
  motor.stop();
}

void yiyou_pp_failed_halt_cancels_and_mode_reset_clears_halt()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::logic_error>([&] {motor.halt_position();});
  motor.enable_position(1000, 2000, 2000);
  motor.prepare_position_command(13000);
  require(motor.stage_position_command(), "stage before failed halt");
  node->dictionary.fail_next_write(0x6040);
  rejects<std::runtime_error>([&] {motor.halt_position();});
  const auto count = node->dictionary.writes().size();
  require(motor.finish_position_command(), "failed halt still cancels local handshake");
  rejects<std::logic_error>([&] {motor.trigger_position_command();});
  require(node->dictionary.writes().size() == count, "failed halt leaves no stale trigger");
  motor.prepare_position_command(12000);
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0x012f),
    "failed halt must still be preserved by replacement preparation");
  require(motor.stage_position_command(), "stage replacement after failed halt");
  node->dictionary.fail_next_write(0x6040);
  rejects<std::runtime_error>([&] {motor.trigger_position_command();});
  motor.halt_position();
  motor.stop();
  motor.enable_position(1000, 2000, 2000);
  motor.prepare_position_command(12500);
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0x002f),
    "new mode enable clears the previous halted state");
  motor.stop();
  motor.enable_cst();
  const auto cst_count = node->dictionary.writes().size();
  rejects<std::logic_error>([&] {motor.halt_position();});
  require(node->dictionary.writes().size() == cst_count, "PP halt must reject CST without writes");
  motor.stop();
}

void yiyou_pp_health_failure_has_no_stop_side_effects()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::logic_error>([&] {motor.check_position_mode();});
  require(node->dictionary.writes().empty(), "disabled PP health check must not write");
  motor.enable_position(1000, 2000, 2000);
  motor.check_position_mode();
  const auto before = node->dictionary.writes().size();
  node->dictionary.put<std::int8_t>(0x6061, 3);
  rejects<std::runtime_error>([&] {motor.check_position_mode();});
  require(node->dictionary.writes().size() == before, "wrong mode must not initiate stop");
  node->dictionary.put<std::int8_t>(0x6061, 1);
  for (const auto status : {0x0040, 0x0008, 0x0827, 0x2027}) {
    node->dictionary.put<std::uint16_t>(0x6041, static_cast<std::uint16_t>(status));
    rejects<std::runtime_error>([&] {motor.check_position_mode();});
    require(
      node->dictionary.writes().size() == before,
      "invalid PP state must propagate without stop so owner can power off first");
  }
  node->dictionary.put<std::uint16_t>(0x6041, 0x0027);
  motor.check_position_mode();
  motor.stop();
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

void yiyou_invalid_enable_does_not_write()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::invalid_argument>([&] {motor.enable_position(0, 1, 1);});
  rejects<std::invalid_argument>([&] {motor.enable(0, 1);});
  rejects<std::logic_error>([&] {motor.set_current_permille(1);});
  require(node->dictionary.writes().empty(), "invalid initial commands must not write objects");
}

void yiyou_torque_feedback_is_cached_optional()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  require(!motor.actual_torque_permille(), "missing cached torque stays unknown");
  require(!motor.rated_torque_millinewton_meter(), "missing cached rated torque stays unknown");
  require(!motor.actual_torque_newton_meter(), "missing rating prevents Nm conversion");
  require(node->dictionary.accesses().empty(), "cached torque getters must not issue SDO reads");

  node->torque_permille = static_cast<std::int16_t>(-125);
  node->rated_torque_mnm = 2400U;
  require(motor.actual_torque_permille() == -125, "cached signed torque preserves negative values");
  require(
    motor.rated_torque_millinewton_meter() == 2400U,
    "cached rated torque returns motor-side mNm");
  const auto nm = motor.actual_torque_newton_meter();
  require(nm && std::abs(*nm + 0.3) < 1e-12, "Nm conversion uses raw * rated_mNm / 1e6");
  require(node->dictionary.accesses().empty(), "torque conversion must not issue SDO reads");

  node->rated_torque_mnm = 0U;
  require(!motor.actual_torque_newton_meter(), "zero rated torque is treated as unknown");
}

void yiyou_torque_feedback_does_not_change_motion_output()
{
  auto node = std::make_shared<CoeNode>();
  node->torque_permille = static_cast<std::int16_t>(321);
  node->rated_torque_mnm = 1000U;
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  const auto writes_before = node->dictionary.writes().size();
  require(motor.actual_torque_permille() == 321, "enabled PP motor exposes cached torque");
  require(motor.actual_torque_newton_meter().has_value(), "enabled PP motor converts cached torque");
  require(
    node->dictionary.writes().size() == writes_before,
    "reading cached torque must not write motion objects");
  motor.stop();
}
}  // namespace

int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests{
    {"BXI invalid transport frames", bxi_transport_rejects_invalid_frames},
    {"ISWV tuning order", iswv_tuning_order},
    {"ISWV callback failure cleanup", iswv_callback_failure_cleans_up},
    {"ISWV failed zero blocks tuning", iswv_failed_clear_never_tunes},
    {"Yiyou PP hold handshake stop", yiyou_pp_hold_handshake_stop},
    {"Yiyou PP bounded recovery budget", yiyou_pp_stage_has_bounded_recovery_budget},
    {"Yiyou PP halt cancels every handshake stage", yiyou_pp_halt_cancels_every_handshake_stage},
    {"Yiyou PP blocking replacement preserves halt", yiyou_pp_blocking_replacement_preserves_halt},
    {"Yiyou PP failed halt cancellation and mode reset", yiyou_pp_failed_halt_cancels_and_mode_reset_clears_halt},
    {"Yiyou PP health failure has no stop side effects", yiyou_pp_health_failure_has_no_stop_side_effects},
    {"Yiyou CST zero limits stop", yiyou_cst_zero_limits_stop},
    {"Yiyou failed stop still disables", yiyou_stop_continues_after_sdo_failure},
    {"Yiyou invalid enable", yiyou_invalid_enable_does_not_write},
    {"Yiyou torque feedback cached optional", yiyou_torque_feedback_is_cached_optional},
    {"Yiyou torque feedback leaves output unchanged", yiyou_torque_feedback_does_not_change_motion_output},
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
