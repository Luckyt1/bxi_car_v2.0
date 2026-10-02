#include "arm/control.h"

#include <cmath>
#include <algorithm>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace chassis
{
namespace
{
std::array<std::shared_ptr<ethercat::Node>, 3> selected_nodes(
  const std::shared_ptr<ethercat::Master> & master, const std::array<std::uint16_t, 3> & positions)
{
  if (!master) {throw std::invalid_argument("Yiyou requires an EtherCAT master");}
  if (positions[0] == positions[1] || positions[0] == positions[2] ||
    positions[1] == positions[2])
  {
    throw std::invalid_argument("Yiyou arm slave positions must be unique");
  }
  return {master->node(positions[0]), master->node(positions[1]), master->node(positions[2])};
}

void check_motor_id(std::uint32_t id)
{
  if (id < 1 || id > 3) {throw std::invalid_argument("EtherCAT motor ID must be 1, 2 or 3");}
}
}  // namespace

arm::Configuration load_arm_kinematics(const std::string & path)
{
  arm::Configuration config;
  if (path.empty()) {return config;}
  const auto document = YAML::LoadFile(path);
  config.upper_arm = document["upper_arm"].as<double>();
  config.forearm = document["forearm"].as<double>();
  config.tool = document["tool"].as<double>();
  const auto limits = document["joint_limits"];
  const auto offsets = document["zero_offsets"];
  if (!limits.IsSequence() || limits.size() != 3 ||
    (offsets && (!offsets.IsSequence() || offsets.size() != 3)))
  {
    throw std::invalid_argument("arm config requires three joint limits and zero offsets");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!limits[i].IsSequence() || limits[i].size() != 2) {
      throw std::invalid_argument("each arm joint limit must contain min and max radians");
    }
    config.limits[i] = {limits[i][0].as<double>(), limits[i][1].as<double>()};
    // Match the simulation's old-format JSON behavior.
    config.zero_offsets[i] = offsets ? offsets[i].as<double>() : 0.0;
  }
  (void)arm::Kinematics(config);
  return config;
}

void validate_arm_motion_config(const ArmMotionConfig & motion)
{
  (void)arm::Kinematics(motion.kinematics);
  if (!std::isfinite(motion.speed_rpm) || motion.speed_rpm <= 0 || motion.speed_rpm >= 60 ||
    !std::isfinite(motion.acceleration_rpm_s) || motion.acceleration_rpm_s <= 0)
  {
    throw std::invalid_argument("invalid arm profile speed/acceleration");
  }
  for (const auto direction : motion.motor_directions) {
    if (direction != -1 && direction != 1) {
      throw std::invalid_argument("arm motor directions must be -1 or 1");
    }
  }
}

ArmController::ArmController(
  std::shared_ptr<ethercat::Master> master, std::array<std::uint16_t, 3> positions,
  std::function<void()> power_off, std::optional<std::uint16_t> current_position,
  const ArmMotionConfig & motion)
: ArmController(selected_nodes(master, positions), std::move(power_off),
    current_position && master ? master->node(*current_position) : nullptr, motion)
{
  master_ = std::move(master);
}

ArmController::ArmController(
  std::array<std::shared_ptr<ethercat::Node>, 3> nodes, std::function<void()> power_off,
  std::shared_ptr<ethercat::Node> current_node, const ArmMotionConfig & motion)
: motors_{YiyouMotor(nodes[0]), YiyouMotor(nodes[1]), YiyouMotor(nodes[2])},
  motion_config_(motion), kinematics_(motion.kinematics),
  power_off_(std::move(power_off))
{
  validate_arm_motion_config(motion);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      if (nodes[i]->id() == nodes[j]->id()) {
        throw std::invalid_argument("arm slave positions must be unique");
      }
    }
  }
  if (current_node) {
    for (const auto & node : nodes) {
      if (node->id() == current_node->id()) {
        throw std::invalid_argument("CST fourth axis must have a distinct slave position");
      }
    }
    current_motor_ = std::make_unique<YiyouMotor>(std::move(current_node));
  }
}

ArmController::~ArmController()
{
  stop_noexcept();
}

void ArmController::initialize()
{
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (initialized_) {throw std::logic_error("EtherCAT already holds its startup positions");}
  try {
    // 先读取全部轴，任一轴失联时不开始使能。原始脉冲位置可以非零或为负。
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      position_scales_[i] = motors_[i].configure_rpm_units();
      hold_positions_[i] = motors_[i].actual_position();
      actual_positions_[i] = hold_positions_[i];
    }
    if (current_motor_) {
      current_axis_position_scale_ = current_motor_->configure_rpm_units();
      (void)current_motor_->actual_current_permille();
      current_axis_position_pulses_ = current_motor_->actual_position();
    }
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      auto & motor = motors_[i];
      const auto speed = static_cast<std::uint32_t>(
        motor.velocity_from_rpm(motion_config_.speed_rpm));
      const auto acceleration = static_cast<std::uint32_t>(
        motor.velocity_from_rpm(motion_config_.acceleration_rpm_s));
      motor.enable_position(speed, acceleration, acceleration);
      motor.move_to_position(hold_positions_[i]);
    }
    if (current_motor_) {
      current_motor_->enable_cst();
      target_current_ = 0;
    }
    initialized_ = true;
    accepted_joints_ = actual_joints();
    elbow_branch_ = std::sin(accepted_joints_[1] + motion_config_.kinematics.zero_offsets[1]) >
      1e-10 ? 1 : -1;
    next_feedback_ = {};
    next_report_ = {};
    update();
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
}

void ArmController::return_to_zero(
  const std::function<bool()> & keep_running, std::chrono::milliseconds timeout)
{
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT arm before returning to zero");}
  if (position_command_pending() || timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("return to zero requires idle commands and a positive timeout");
  }
  try {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    if (keep_running && !keep_running()) {
      throw std::runtime_error("arm return to zero cancelled");
    }
    update();
    const auto actual = actual_joints();
    for (std::size_t i = 0; i < actual.size(); ++i) {
      const auto & limit = motion_config_.kinematics.limits[i];
      if (actual[i] < limit.min - 1e-12 || actual[i] > limit.max + 1e-12 ||
        std::abs(actual[i]) > arm::kPi + 1e-9)
      {
        throw std::invalid_argument(
                "cannot return to zero: measured joint outside limits or travel exceeds 180 degrees");
      }
    }
    // Exact calibrated joint zero, not an alternate IK solution at the same tool pose.
    // Reuse the all-axis staged/readback/acknowledgement path and configured PP limits.
    pending_positions_ = position_targets({0, 0, 0});
    pending_joints_ = {0, 0, 0};
    position_phase_ = PositionPhase::queued;
    while (true) {
      if (keep_running && !keep_running()) {
        throw std::runtime_error("arm return to zero cancelled");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("arm return to zero timed out before all axes settled");
      }
      update();
      if (!position_command_pending()) {
        bool settled = true;
        for (auto & motor : motors_) {
          if (!motor.position_reached() || !motor.motion_stopped()) {settled = false;}
        }
        if (settled) {
          // Joint-space startup homing may change the elbow branch. Lock runtime IK
          // to the branch at the confirmed zero position from this point onward.
          elbow_branch_ = std::sin(motion_config_.kinematics.zero_offsets[1]) > 1e-10 ? 1 : -1;
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
}

void ArmController::update(std::chrono::steady_clock::time_point now)
{
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT position hold first");}
  if (now < next_feedback_) {return;}
  try {
    if (master_) {master_->check_health();}
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      auto & motor = motors_[i];
      // 不把实时反馈重新设为目标；启动回零或显式末端请求才修改目标。
      (void)motor.position_reached();
      actual_positions_[i] = motor.actual_position();
      actual_rpm_[i] = motor.actual_velocity_rpm();
      if (std::abs(actual_rpm_[i]) >= 60.0) {
        throw std::runtime_error(
                "EtherCAT motor " + std::to_string(i + 1) +
                " feedback speed reaches 60 RPM; motor power off requested");
      }
    }
    if (current_motor_) {
      current_motor_->check_current_mode();
      actual_current_ = current_motor_->actual_current_permille();
      current_axis_position_pulses_ = current_motor_->actual_position();
      current_axis_rpm_ = current_motor_->actual_velocity_rpm();
      if (std::abs(current_axis_rpm_) >= 60.0) {
        throw std::runtime_error(
                "EtherCAT motor 4 feedback speed reaches 60 RPM; motor power off requested");
      }
      if (current_direction_blocked(target_current_)) {
        current_motor_->set_current_permille(0);
        target_current_ = 0;
      }
    }
    advance_position_command(now);
    next_feedback_ = now + std::chrono::milliseconds(20);
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
}

std::string ArmController::take_feedback_diagnostic(std::chrono::steady_clock::time_point now)
{
  if (!initialized_ || now < next_report_) {return {};}
  next_report_ = now + std::chrono::seconds(1);
  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (i != 0) {out << '\n';}
    out << "ETHERCAT_YIYOU_HOLD motor_id=" << i + 1
        << " target_position_pulses=" << hold_positions_[i]
        << " actual_position_pulses=" << actual_positions_[i]
        << " velocity_rpm=" << actual_rpm_[i];
  }
  if (current_motor_) {
    out << '\n' << "ETHERCAT_YIYOU_CST motor_id=4"
        << " target_current_permille=" << target_current_
        << " actual_current_permille=" << actual_current_
        << " actual_position_pulses=" << current_axis_position_pulses_
        << " actual_position_degrees=" <<
      static_cast<double>(current_axis_position_pulses_) * 360.0 / current_axis_position_scale_
        << " velocity_rpm=" << current_axis_rpm_
        << " negative_inhibited=" << current_direction_blocked(-1)
        << " positive_inhibited=" << current_direction_blocked(1);
  }
  const auto pose = actual_pose();
  out << '\n' << "ETHERCAT_YIYOU_POSE x_m=" << pose.x << " z_m=" << pose.z
      << " pitch_rad=" << pose.pitch << " pending=" << position_command_pending();
  return out.str();
}

void ArmController::set_target_degrees(std::uint32_t id, double degrees)
{
  check_motor_id(id);
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT position hold first");}
  if (!std::isfinite(degrees) || degrees != 0.0) {
    throw std::invalid_argument(
            "EtherCAT Yiyou motors only hold startup positions; angle control disabled");
  }
}

double ArmController::target_degrees(std::uint32_t id) const
{
  check_motor_id(id);
  return 0.0;
}

arm::JointAngles ArmController::actual_joints() const
{
  if (!initialized_) {throw std::logic_error("initialize EtherCAT arm first");}
  arm::JointAngles joints{};
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto counts = static_cast<std::int64_t>(actual_positions_[i]) -
      motion_config_.zero_counts[i];
    joints[i] = motion_config_.motor_directions[i] * static_cast<double>(counts) *
      2.0 * arm::kPi / position_scales_[i];
  }
  return joints;
}

arm::Pose ArmController::actual_pose() const
{
  return kinematics_.forward(actual_joints());
}

bool ArmController::position_command_pending() const noexcept
{
  return position_phase_ != PositionPhase::idle;
}

std::array<std::int32_t, 3> ArmController::position_targets(const arm::JointAngles & joints) const
{
  std::array<std::int32_t, 3> positions{};
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const double raw = std::round(
      motion_config_.motor_directions[i] * joints[i] *
      position_scales_[i] / (2.0 * arm::kPi)) + motion_config_.zero_counts[i];
    if (!std::isfinite(raw) || raw < std::numeric_limits<std::int32_t>::min() ||
      raw > std::numeric_limits<std::int32_t>::max())
    {
      throw std::invalid_argument("arm position target exceeds int32 encoder range");
    }
    const auto counts = static_cast<std::int64_t>(raw);
    const double quantized = motion_config_.motor_directions[i] *
      static_cast<double>(counts - motion_config_.zero_counts[i]) * 2.0 * arm::kPi /
      position_scales_[i];
    const auto & limit = motion_config_.kinematics.limits[i];
    if (quantized < limit.min - 1e-12 || quantized > limit.max + 1e-12) {
      throw std::invalid_argument("rounded encoder target exceeds arm joint limit");
    }
    positions[i] = static_cast<std::int32_t>(counts);
  }
  return positions;
}

arm::JointAngles ArmController::validate_target_pose(const arm::Pose & pose) const
{
  if (!fault_reason_.empty()) {throw std::invalid_argument(fault_reason_);}
  if (!initialized_) {throw std::invalid_argument("initialize EtherCAT arm first");}
  if (position_command_pending()) {
    throw std::invalid_argument("arm position handshake is pending");
  }
  const auto actual = actual_joints();
  for (std::size_t i = 0; i < actual.size(); ++i) {
    const auto & limit = motion_config_.kinematics.limits[i];
    if (actual[i] < limit.min - 1e-12 || actual[i] > limit.max + 1e-12) {
      throw std::invalid_argument("measured arm joint is outside limits; check calibrated zeros");
    }
  }
  const auto result = kinematics_.inverse(pose, accepted_joints_);
  if (result.status != arm::InverseStatus::Success) {
    throw std::invalid_argument("arm target is nonfinite, unreachable or outside joint limits");
  }
  for (const auto & solution : result.solutions) {
    const double branch = std::sin(solution.joints[1] + motion_config_.kinematics.zero_offsets[1]);
    if (elbow_branch_ * branch < -1e-10 || solution.singular) {continue;}
    bool continuous = true;
    for (std::size_t i = 0; i < actual.size(); ++i) {
      continuous = continuous &&
        std::abs(solution.joints[i] - accepted_joints_[i]) <= arm::kPi + 1e-9 &&
        std::abs(solution.joints[i] - actual[i]) <= arm::kPi + 1e-9;
    }
    if (!continuous) {continue;}
    (void)position_targets(solution.joints);
    return solution.joints;
  }
  throw std::invalid_argument(
          "arm target changes elbow branch, is singular or requires >180 degrees");
}

void ArmController::set_target_pose(const arm::Pose & pose)
{
  const auto joints = validate_target_pose(pose);
  const auto positions = position_targets(joints);
  pending_joints_ = joints;
  pending_positions_ = positions;
  position_phase_ = PositionPhase::queued;
}

void ArmController::advance_position_command(std::chrono::steady_clock::time_point now)
{
  if (position_phase_ == PositionPhase::idle) {return;}
  if (position_phase_ != PositionPhase::queued && now >= position_deadline_) {
    throw std::runtime_error("three-axis PP command acknowledgement timed out");
  }
  switch (position_phase_) {
    case PositionPhase::queued:
      for (std::size_t i = 0; i < motors_.size(); ++i) {
        motors_[i].prepare_position_command(pending_positions_[i]);
      }
      staging_axis_ = 0;
      position_deadline_ = now + std::chrono::seconds(1);
      position_phase_ = PositionPhase::staging;
      break;
    case PositionPhase::staging:
      // Only one mailbox readback per tick; all three must pass before any trigger.
      if (motors_[staging_axis_].stage_position_command()) {++staging_axis_;}
      if (staging_axis_ == motors_.size()) {position_phase_ = PositionPhase::triggering;}
      break;
    case PositionPhase::triggering:
      for (auto & motor : motors_) {
        motor.trigger_position_command();
      }
      position_phase_ = PositionPhase::acknowledging;
      break;
    case PositionPhase::acknowledging: {
        bool acknowledged = true;
        for (auto & motor : motors_) {
          if (!motor.finish_position_command()) {acknowledged = false;}
        }
        if (acknowledged) {
          accepted_joints_ = pending_joints_;
          hold_positions_ = pending_positions_;
          position_phase_ = PositionPhase::idle;
        }
        break;
      }
    case PositionPhase::idle:
      break;
  }
}

bool ArmController::current_direction_blocked(std::int64_t current) const
{
  const double degrees =
    static_cast<double>(current_axis_position_pulses_) * 360.0 / current_axis_position_scale_;
  return (degrees < 0.0 && current < 0) || (degrees > 1300.0 && current > 0);
}

void ArmController::set_current_permille(std::int64_t current)
{
  if (current < -1000 || current > 1000) {
    throw std::invalid_argument("fourth-axis current must be in [-1000, 1000] permille");
  }
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT arm first");}
  if (!current_motor_) {throw std::logic_error("no fourth CST axis configured");}
  try {
    if (master_) {master_->check_health();}
    // Check fresh cached PDO position for every command, including reversals.
    current_axis_position_pulses_ = current_motor_->actual_position();
    if (current_direction_blocked(current)) {current = 0;}
    current_motor_->set_current_permille(static_cast<std::int16_t>(current));
    target_current_ = static_cast<std::int16_t>(current);
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
}

std::string ArmController::fault_reason() const
{
  return fault_reason_;
}

void ArmController::stop()
{
  initialized_ = false;
  position_phase_ = PositionPhase::idle;
  std::exception_ptr first_error;
  if (current_motor_) {
    try {
      current_motor_->stop();
      target_current_ = 0;
    } catch (...) {
      first_error = std::current_exception();
    }
  }
  for (auto & motor : motors_) {
    try {
      motor.stop();
    } catch (...) {
      if (!first_error) {first_error = std::current_exception();}
    }
  }
  if (first_error) {std::rethrow_exception(first_error);}
}

void ArmController::stop_noexcept() noexcept
{
  try {
    stop();
  } catch (...) {
  }
}

void ArmController::latch_fault(const std::string & reason)
{
  if (!fault_reason_.empty()) {return;}
  fault_reason_ = "EtherCAT Yiyou arm failed: " + reason;
  if (power_off_) {
    try {
      power_off_();
    } catch (const std::exception & error) {
      fault_reason_ += " (power_off_error=" + std::string(error.what()) + ")";
    } catch (...) {
      fault_reason_ += " (unknown power-off error)";
    }
  }
  stop_noexcept();
}

}  // namespace chassis
