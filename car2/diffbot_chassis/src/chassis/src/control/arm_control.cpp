#include "chassis/control/arm_control.h"
#include "chassis/motor/bxi_pci_transport.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace chassis
{
struct ArmController::FeedbackState
{
  struct Sample
  {
    std::uint64_t count{0};
    std::chrono::steady_clock::time_point last_received{};
    iswv::CanFrame frame{};
    bxi::Feedback feedback{};
  };

  std::mutex mutex;
  std::array<Sample, 3> samples{};
  std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_report{};
  std::uint64_t unmatched_count{0};
  std::uint64_t invalid_count{0};
  std::uint64_t transport_error_count{0};
  bool reply_pending{false};
  bool active{true};
  bool fault_latched{false};
  std::uint32_t fault_motor_id{0};
  double fault_velocity_rpm{0.0};
  double fault_limit_rpm{60.0};
  std::string fault_reason;
  std::string power_off_error;
  std::function<void()> power_off;
  std::array<double, 3> target_degrees{};
  std::chrono::steady_clock::time_point first_hold_started{};
  bool has_unmatched_frame{false};
  iswv::CanFrame last_unmatched_frame{};
  std::string last_transport_error;
};

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kCan3SpeedLimitRpm = 60.0;
constexpr double kCan3SpeedLimitRadPerSecond = kCan3SpeedLimitRpm * 2.0 * kPi / 60.0;
constexpr double kRadiansPerDegree = kPi / 180.0;
constexpr double kVelocityRpmScale = 60.0 / (2.0 * kPi);

std::string format_fixed(double value, int precision)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string fault_message(std::uint32_t id, double velocity_rpm, double limit_rpm)
{
  return "CAN3 motor " + std::to_string(id) + " feedback speed " +
         format_fixed(velocity_rpm, 2) + " rpm reaches limit " +
         format_fixed(limit_rpm, 2) + " rpm; motor power off requested";
}

std::string feedback_loss_message(std::uint32_t id)
{
  return "CAN3 motor " + std::to_string(id) +
         " feedback missing for more than 1s; motor power off requested";
}

void append_raw_frame(std::ostream & out, const iswv::CanFrame & frame)
{
  out << "can_id=0x" << std::hex << std::setfill('0') << std::setw(3) << frame.id
      << std::dec << " dlc=" << static_cast<unsigned>(frame.size)
      << " fd=" << frame.fd << " brs=" << frame.bitrate_switch
      << " extended=" << frame.extended << " rtr=" << frame.remote
      << " error=" << frame.error << " data=";
  for (std::size_t i = 0; i < std::min<std::size_t>(frame.size, frame.data.size()); ++i) {
    if (i != 0) {out << ' ';}
    out << std::hex << std::setw(2) << static_cast<unsigned>(frame.data[i]);
  }
  out << std::dec;
}

std::shared_ptr<iswv::ICanTransport> checked_transport(
  std::shared_ptr<iswv::ICanTransport> transport)
{
  if (!transport || !transport->is_open()) {
    throw std::invalid_argument("BXI CAN3 requires an open transport");
  }
  return transport;
}

std::size_t motor_index(std::uint32_t id)
{
  if (id < 1 || id > 3) {
    throw std::invalid_argument("BXI CAN3 motor ID must be 1, 2 or 3");
  }
  return id - 1;
}

void require_success(const iswv::Result<void> & result, std::uint32_t id, const char * action)
{
  if (!result) {
    throw std::runtime_error(
            "BXI CAN3 ID " + std::to_string(id) + " " + action + ": " + result.error().message);
  }
}

}  // namespace

ArmController::ArmController()
: transport_(std::make_shared<BxiPciTransport>(3)),
  feedback_state_(std::make_shared<FeedbackState>()),
  motors_{bxi::Motor(*transport_, 1), bxi::Motor(*transport_, 2), bxi::Motor(*transport_, 3)}
{
  auto pci_transport = std::dynamic_pointer_cast<BxiPciTransport>(transport_);
  std::weak_ptr<BxiPciTransport> weak_transport = pci_transport;
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    feedback_state_->power_off =
      [weak_transport] {
        auto transport = weak_transport.lock();
        if (!transport) {throw std::runtime_error("BXI CAN3 transport is no longer available");}
        auto result = transport->set_motor_power(false);
        if (!result) {throw std::runtime_error(result.error().message);}
      };
  }
  install_handlers();
}

ArmController::ArmController(
  std::shared_ptr<iswv::ICanTransport> transport, std::function<void()> power_off)
: transport_(checked_transport(std::move(transport))),
  feedback_state_(std::make_shared<FeedbackState>()),
  motors_{bxi::Motor(*transport_, 1), bxi::Motor(*transport_, 2), bxi::Motor(*transport_, 3)}
{
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    feedback_state_->power_off = std::move(power_off);
  }
  install_handlers();
}

void ArmController::install_handlers()
{
  // 接收线程可能已复制回调；捕获共享状态，避免析构后访问 ArmController 本体。
  transport_->set_receive_handler(
    [state = feedback_state_](const iswv::CanFrame & frame) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->active) {return;}
        const bool standard_data = iswv::valid_frame(frame) && !frame.extended &&
        !frame.remote && !frame.error;
        const bool expected_id = standard_data && frame.id >= 0x11 && frame.id <= 0x13;
        if (!standard_data || !expected_id || frame.size != 8 || frame.data[0] != frame.id - 0x10) {
          if (!standard_data || expected_id) {++state->invalid_count;} else {
            ++state->unmatched_count;
          }
          state->has_unmatched_frame = true;
          state->last_unmatched_frame = frame;
          return;
        }
        // 官方普通 MIT 回复为 8 字节，data[0] 为电机 ID；不能将命令回显算作回复。
        const auto decoded = bxi::decode_feedback(
          frame, bxi::encoding_ranges(
            bxi::Model::BXI8515_19));
        if (!decoded) {++state->invalid_count; return;}
        auto & sample = state->samples[frame.id - 0x11];
        const auto received = std::chrono::steady_clock::now();
        if (sample.count == 0 || received - sample.last_received >= std::chrono::seconds(1)) {
          state->reply_pending = true;
        }
        ++sample.count;
        sample.last_received = received;
        sample.frame = frame;
        sample.feedback = decoded.value();
        const auto velocity_rpm = static_cast<double>(sample.feedback.velocity) * kVelocityRpmScale;
        if (std::abs(static_cast<double>(sample.feedback.velocity)) >=
        kCan3SpeedLimitRadPerSecond)
        {
          ArmController::latch_fault_locked(
            *state, frame.data[0], velocity_rpm,
            fault_message(frame.data[0], std::abs(velocity_rpm), kCan3SpeedLimitRpm));
        }
      }
    });
  transport_->set_error_handler(
    [state = feedback_state_](const iswv::Error & error) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->active) {return;}
      ++state->transport_error_count;
      state->last_transport_error = error.message;
    });
}

ArmController::~ArmController()
{
  transport_->set_receive_handler({});
  transport_->set_error_handler({});
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    feedback_state_->active = false;
    feedback_state_->power_off = {};
  }
  stop_noexcept();
}

void ArmController::initialize()
{
  throw_if_fault_latched();
  initialized_ = false;
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    feedback_state_->samples = {};
    feedback_state_->started = std::chrono::steady_clock::now();
    feedback_state_->next_report = {};
    feedback_state_->unmatched_count = 0;
    feedback_state_->invalid_count = 0;
    feedback_state_->transport_error_count = 0;
    feedback_state_->reply_pending = false;
    feedback_state_->has_unmatched_frame = false;
    feedback_state_->last_transport_error.clear();
    feedback_state_->first_hold_started = {};
    feedback_state_->target_degrees = {};
  }
  stop();
  initialized_ = true;
}

void ArmController::save_zero_positions()
{
  throw_if_fault_latched();
  if (!initialized_ ||
    std::any_of(enabled_.begin(), enabled_.end(), [](bool value) {return value;}))
  {
    throw std::logic_error("initialize and disable all BXI CAN3 motors before saving zero positions");
  }
  zero_save_commands_sent_ = false;
  try {
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      require_success(
        motors_[i].save_zero_position(), static_cast<std::uint32_t>(i + 1), "save zero position");
    }
    // 这里只确认发送提交成功，不代表已收到设备的零点保存应答。
    zero_save_commands_sent_ = true;
    next_zero_hold_command_ = {};
  } catch (...) {
    initialized_ = false;
    stop_noexcept();
    throw;
  }
}

void ArmController::update(std::chrono::steady_clock::time_point now)
{
  throw_if_fault_latched();
  if (!initialized_ || !zero_save_commands_sent_) {
    throw std::logic_error("send all BXI CAN3 save-zero commands before holding zero");
  }
  if (now < next_zero_hold_command_) {return;}
  latch_feedback_loss_if_needed(now);
  throw_if_fault_latched();
  std::array<double, 3> target_degrees;
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    target_degrees = feedback_state_->target_degrees;
  }
  for (std::uint32_t id = 1; id <= 3; ++id) {
    // enable() 对已使能轴不重复发帧；任一发送失败会全组失能并抛出异常。
    enable(id);
    const bxi::Command target{
      static_cast<float>(target_degrees[id - 1] * kRadiansPerDegree), 0.0F, 200.0F, 4.0F, 0.0F};
    command(id, target);
  }
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    if (feedback_state_->first_hold_started == std::chrono::steady_clock::time_point{}) {
      feedback_state_->first_hold_started = now;
    }
  }
  next_zero_hold_command_ = now + std::chrono::milliseconds(20);
}

void ArmController::enable(std::uint32_t id)
{
  const auto index = motor_index(id);
  if (!initialized_) {throw std::logic_error("initialize BXI CAN3 after power-on first");}
  std::unique_lock<std::mutex> lock(feedback_state_->mutex);
  if (feedback_state_->fault_latched) {throw std::logic_error(feedback_state_->fault_reason);}
  if (enabled_[index]) {return;}
  try {
    require_success(motors_[index].enter_motor_mode(), id, "enable");
    enabled_[index] = true;
  } catch (...) {
    lock.unlock();
    initialized_ = false;
    stop_noexcept();
    throw;
  }
}

void ArmController::command(std::uint32_t id, const bxi::Command & command)
{
  const auto index = motor_index(id);
  if (!initialized_ || !enabled_[index]) {
    throw std::logic_error("enable the BXI CAN3 motor before sending commands");
  }
  auto target = command;
  // CAN3 的 ID 1/2/3 均为 BXI8515-19，统一使用 MOTOR_85 编码范围。
  target.ranges = bxi::encoding_ranges(bxi::Model::BXI8515_19);
  std::unique_lock<std::mutex> lock(feedback_state_->mutex);
  if (feedback_state_->fault_latched) {throw std::logic_error(feedback_state_->fault_reason);}
  try {
    require_success(motors_[index].command(target), id, "command");
  } catch (...) {
    lock.unlock();
    initialized_ = false;
    stop_noexcept();
    throw;
  }
}

std::string ArmController::take_feedback_diagnostic(std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(feedback_state_->mutex);
  auto & state = *feedback_state_;
  if (now < state.next_report && !state.reply_pending) {return {};}
  state.reply_pending = false;
  state.next_report = now + std::chrono::seconds(1);
  const auto age_ms = [now](std::chrono::steady_clock::time_point received) {
      return std::max<std::int64_t>(
        0, std::chrono::duration_cast<std::chrono::milliseconds>(now - received).count());
    };
  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  for (std::size_t i = 0; i < state.samples.size(); ++i) {
    const auto & sample = state.samples[i];
    const char * status = sample.count == 0 ?
      (age_ms(state.started) < 1000 ? "WAITING" : "NO_REPLY") :
      (age_ms(sample.last_received) < 1000 ? "REPLIED" : "STALE");
    out << "CAN3_FEEDBACK motor_id=" << i + 1 << " reply_id=0x"
        << std::hex << std::setfill('0') << std::setw(3) << 0x11 + i << std::dec
        << " state=" << status << " rx_count=" << sample.count;
    if (sample.count != 0) {
      const auto velocity_rpm = static_cast<double>(sample.feedback.velocity) * kVelocityRpmScale;
      out << " age_ms=" << age_ms(sample.last_received)
          << " position_rad=" << sample.feedback.position
          << " velocity_rad_s=" << sample.feedback.velocity
          << " velocity_rpm=" << velocity_rpm
          << " target_deg=" << state.target_degrees[i] << ' ';
      append_raw_frame(out, sample.frame);
    } else {
      out << " waiting_ms=" << age_ms(state.started)
          << " target_deg=" << state.target_degrees[i];
    }
    out << '\n';
  }
  out << "CAN3_RX_DIAGNOSTIC unmatched_count=" << state.unmatched_count
      << " invalid_count=" << state.invalid_count
      << " transport_error_count=" << state.transport_error_count
      << " (reply confirms communication only; enable/zero-save not confirmed)";
  if (state.has_unmatched_frame) {
    out << "\nCAN3_RX_UNMATCHED ";
    append_raw_frame(out, state.last_unmatched_frame);
  }
  if (!state.last_transport_error.empty()) {
    out << "\nCAN3_RX_ERROR " << state.last_transport_error;
  }
  if (state.fault_latched) {
    out << "\nCAN3_FAULT " << state.fault_reason;
    if (!state.power_off_error.empty()) {
      out << " power_off_error=" << state.power_off_error;
    }
  }
  return out.str();
}

void ArmController::set_target_degrees(std::uint32_t id, double degrees)
{
  const auto index = motor_index(id);
  if (!initialized_ || !zero_save_commands_sent_ || !enabled_[index]) {
    throw std::logic_error(
            "initialize, save zero, and enable the BXI CAN3 motor before changing target");
  }
  if (!std::isfinite(degrees)) {
    throw std::invalid_argument("BXI CAN3 target degrees must be finite");
  }
  const auto radians = degrees * kRadiansPerDegree;
  const auto ranges = bxi::encoding_ranges(bxi::Model::BXI8515_19);
  if (radians < ranges.position.min || radians > ranges.position.max) {
    throw std::invalid_argument("BXI CAN3 target exceeds protocol position encoding range");
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(feedback_state_->mutex);
  if (feedback_state_->fault_latched) {throw std::logic_error(feedback_state_->fault_reason);}
  const auto & sample = feedback_state_->samples[index];
  if (sample.count == 0 ||
    now - sample.last_received >= std::chrono::seconds(1))
  {
    throw std::logic_error("recent CAN3 feedback is required before changing target");
  }
  feedback_state_->target_degrees[index] = degrees;
}

double ArmController::target_degrees(std::uint32_t id) const
{
  const auto index = motor_index(id);
  std::lock_guard<std::mutex> lock(feedback_state_->mutex);
  return feedback_state_->target_degrees[index];
}

std::string ArmController::fault_reason() const
{
  std::lock_guard<std::mutex> lock(feedback_state_->mutex);
  if (!feedback_state_->fault_latched) {return {};}
  auto reason = feedback_state_->fault_reason;
  if (!feedback_state_->power_off_error.empty()) {
    reason += " (power_off_error=" + feedback_state_->power_off_error + ")";
  }
  return reason;
}

void ArmController::stop()
{
  zero_save_commands_sent_ = false;
  next_zero_hold_command_ = {};
  std::exception_ptr first_error;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    enabled_[i] = false;
    try {
      require_success(motors_[i].exit_motor_mode(), static_cast<std::uint32_t>(i + 1), "stop");
    } catch (...) {
      if (!first_error) {first_error = std::current_exception();}
    }
  }
  if (first_error) {
    initialized_ = false;
    std::rethrow_exception(first_error);
  }
}

void ArmController::stop_noexcept() noexcept
{
  try {
    stop();
  } catch (...) {
  }
}

void ArmController::throw_if_fault_latched() const
{
  const auto reason = fault_reason();
  if (!reason.empty()) {
    throw std::logic_error(reason);
  }
}

void ArmController::latch_feedback_loss_if_needed(std::chrono::steady_clock::time_point now)
{
  {
    std::lock_guard<std::mutex> lock(feedback_state_->mutex);
    if (feedback_state_->fault_latched ||
      feedback_state_->first_hold_started == std::chrono::steady_clock::time_point{})
    {
      return;
    }
    for (std::size_t i = 0; i < feedback_state_->samples.size(); ++i) {
      const auto & sample = feedback_state_->samples[i];
      const auto deadline = sample.count == 0 ? feedback_state_->first_hold_started :
        sample.last_received;
      if (now - deadline >= std::chrono::seconds(1)) {
        latch_fault_locked(
          *feedback_state_, static_cast<std::uint32_t>(i + 1), 0.0,
          feedback_loss_message(static_cast<std::uint32_t>(i + 1)));
        break;
      }
    }
  }
}

void ArmController::latch_fault_locked(
  FeedbackState & state, std::uint32_t id, double velocity_rpm, const std::string & reason)
{
  if (state.fault_latched) {return;}
  state.fault_latched = true;
  state.fault_motor_id = id;
  state.fault_velocity_rpm = velocity_rpm;
  state.fault_limit_rpm = kCan3SpeedLimitRpm;
  state.fault_reason = reason;
  if (!state.power_off) {
    state.power_off_error = "no CAN3 motor power-off callback configured";
  } else {
    try {
      state.power_off();
    } catch (const std::exception & ex) {
      state.power_off_error = ex.what();
    } catch (...) {
      state.power_off_error = "unknown exception while requesting CAN3 motor power off";
    }
  }
}

}  // namespace chassis
