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
  if (!std::isfinite(motion.fourth_speed_rpm) || motion.fourth_speed_rpm <= 0 ||
    motion.fourth_speed_rpm >= 60 || !std::isfinite(motion.fourth_acceleration_rpm_s) ||
    motion.fourth_acceleration_rpm_s <= 0)
  {
    throw std::invalid_argument("invalid fourth-axis profile speed/acceleration");
  }
  for (const auto direction : motion.motor_directions) {
    if (direction != -1 && direction != 1) {
      throw std::invalid_argument("arm motor directions must be -1 or 1");
    }
  }
}

ArmController::ArmController(
  std::shared_ptr<ethercat::Master> master, std::array<std::uint16_t, 3> positions,
  std::function<void()> power_off, std::optional<std::uint16_t> fourth_position,
  const ArmMotionConfig & motion)
: ArmController(selected_nodes(master, positions), std::move(power_off),
    fourth_position && master ? master->node(*fourth_position) : nullptr, motion)
{
  master_ = std::move(master);
  if (fourth_motor_ && motion.fourth_position_memory_file.empty()) {
    throw std::invalid_argument("fourth-axis position memory file is required on hardware");
  }
}

ArmController::ArmController(
  std::array<std::shared_ptr<ethercat::Node>, 3> nodes, std::function<void()> power_off,
  std::shared_ptr<ethercat::Node> fourth_node, const ArmMotionConfig & motion)
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
  if (fourth_node) {
    for (const auto & node : nodes) {
      if (node->id() == fourth_node->id()) {
        throw std::invalid_argument("PP fourth axis must have a distinct slave position");
      }
    }
    fourth_node_ = std::move(fourth_node);
    fourth_motor_ = std::make_unique<YiyouMotor>(fourth_node_);
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
    if (fourth_motor_) {
      fourth_axis_position_scale_ = fourth_motor_->configure_rpm_units();
      fourth_raw_position_ = fourth_motor_->actual_position();
      if (!motion_config_.fourth_position_memory_file.empty()) {
        std::array<std::uint32_t, 4> identity{};
        for (std::uint8_t sub = 1; sub <= identity.size(); ++sub) {
          const auto value = fourth_node_->read(
            iswv::canopen::Object<std::uint32_t>{{0x1018, sub}, "position memory identity"});
          if (!value) {throw std::runtime_error("cannot read fourth-axis identity for position memory");}
          identity[sub - 1] = value.value();
        }
        fourth_memory_ = std::make_unique<PositionMemory>(
          motion_config_.fourth_position_memory_file, motion_config_.ethercat_interface,
          fourth_node_->id(), identity, fourth_axis_position_scale_);
        fourth_axis_position_pulses_ = fourth_memory_->restore(fourth_raw_position_);
      } else {
        fourth_axis_position_pulses_ = fourth_raw_position_;
      }
      const double upper = std::floor(fourth_axis_position_scale_ * 1200.0 / 360.0);
      if (upper < 1 || upper > std::numeric_limits<std::int32_t>::max()) {
        throw std::invalid_argument("fourth-axis 1200 degree limit exceeds encoder range");
      }
      fourth_upper_pulses_ = static_cast<std::int32_t>(upper);
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
    if (fourth_motor_) {
      const auto speed = static_cast<std::uint32_t>(
        fourth_motor_->velocity_from_rpm(motion_config_.fourth_speed_rpm));
      const auto acceleration = static_cast<std::uint32_t>(
        fourth_motor_->velocity_from_rpm(motion_config_.fourth_acceleration_rpm_s));
      fourth_motor_->enable_position(speed, acceleration, acceleration);
      // Seed/accept the measured position, including an already-outside startup.
      read_fourth_position();
      fourth_motor_->move_to_position(fourth_raw_position_);
      halt_fourth_axis();
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
  if (position_command_pending() || planar_halt_pending_ ||
    timeout <= std::chrono::milliseconds::zero())
  {
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
    planar_halted_ = false;
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
    if (planar_halt_pending_) {
      bool stopped = true;
      for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (std::abs(actual_rpm_[i]) >= 0.1 || !motors_[i].motion_stopped()) {
          stopped = false;
        }
      }
      if (stopped) {
        accepted_joints_ = actual_joints();
        hold_positions_ = actual_positions_;
        planar_halt_pending_ = false;
      }
    }
    if (fourth_motor_) {
      fourth_motor_->check_position_mode();
      if (fourth_memory_) {(void)fourth_memory_->poll();}
      read_fourth_position();
      fourth_axis_rpm_ = fourth_motor_->actual_velocity_rpm();
      if (std::abs(fourth_axis_rpm_) >= 60.0) {
        throw std::runtime_error(
                "EtherCAT motor 4 feedback speed reaches 60 RPM; motor power off requested");
      }
      if (fourth_direction_blocked(fourth_direction_)) {halt_fourth_axis();}
      advance_fourth_position(now);
    }
    advance_position_command(now);
    publish_torque_sample(now);
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
  if (fourth_motor_) {
    out << '\n' << "ETHERCAT_YIYOU_PP motor_id=4"
        << " jog_direction=" << fourth_direction_
        << " target_position_pulses=" << fourth_target_.value_or(fourth_axis_position_pulses_)
        << " profile_speed_rpm=" << motion_config_.fourth_speed_rpm
        << " actual_position_pulses=" << fourth_axis_position_pulses_
        << " raw_position_pulses=" << fourth_raw_position_
        << " position_memory=" << bool(fourth_memory_)
        << " actual_position_degrees=" <<
      static_cast<double>(fourth_axis_position_pulses_) * 360.0 / fourth_axis_position_scale_
        << " velocity_rpm=" << fourth_axis_rpm_
        << " negative_inhibited=" << fourth_direction_blocked(-1)
        << " positive_inhibited=" << fourth_direction_blocked(1);
  }
  const auto pose = actual_pose();
  out << '\n' << "ETHERCAT_YIYOU_POSE x_m=" << pose.x << " z_m=" << pose.z
      << " pitch_rad=" << pose.pitch << " pending=" << position_command_pending();
  return out.str();
}

void ArmController::set_torque_observer(std::function<void(const ArmTorqueSample &)> observer)
{
  torque_observer_ = std::move(observer);
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

void ArmController::halt_planar_position()
{
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT arm first");}
  if (planar_halted_) {return;}
  // Cancel the group before touching any drive, including staged but untriggered work.
  position_phase_ = PositionPhase::idle;
  planar_halted_ = true;
  planar_halt_pending_ = true;
  next_feedback_ = {};
  try {
    std::exception_ptr first_error;
    for (auto & motor : motors_) {
      try {
        motor.halt_position();
      } catch (...) {
        if (!first_error) {first_error = std::current_exception();}
      }
    }
    if (first_error) {std::rethrow_exception(first_error);}
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
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
  if (planar_halt_pending_) {
    throw std::invalid_argument("arm planar halt is settling");
  }
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
  planar_halted_ = false;
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

bool ArmController::fourth_direction_blocked(int direction) const
{
  return (fourth_axis_position_pulses_ <= 0 && direction < 0) ||
         (fourth_axis_position_pulses_ >= fourth_upper_pulses_ && direction > 0);
}

void ArmController::halt_fourth_axis()
{
  fourth_direction_ = 0;
  fourth_phase_ = PositionPhase::idle;
  fourth_target_.reset();
  fourth_motor_->halt_position();
  fourth_waiting_for_stop_ = true;
}

void ArmController::set_fourth_axis_jog(int direction)
{
  if (!fault_reason_.empty()) {throw std::logic_error(fault_reason_);}
  if (!initialized_) {throw std::logic_error("initialize EtherCAT arm first");}
  if (!fourth_motor_) {throw std::logic_error("no fourth PP axis configured");}
  try {
    if (direction < -1 || direction > 1) {
      halt_fourth_axis();
      throw std::invalid_argument("fourth-axis jog direction must be -1, 0 or 1");
    }
    if (master_) {master_->check_health();}
    read_fourth_position();
    if (fourth_direction_blocked(direction)) {direction = 0;}
    if (direction == fourth_direction_) {return;}
    // Cancel an in-flight handshake on release or reversal. Preparation keeps
    // halt asserted until a newly staged/read-back target is safe to trigger.
    halt_fourth_axis();
    fourth_direction_ = direction;
  } catch (const std::exception & error) {
    latch_fault(error.what());
    throw;
  }
}

void ArmController::advance_fourth_position(std::chrono::steady_clock::time_point now)
{
  if (fourth_direction_ == 0) {return;}
  // Preserve the old motion envelope during halt deceleration. A new direction
  // may replace its target only after the drive confirms that motion stopped.
  if (fourth_waiting_for_stop_) {
    if (std::abs(fourth_axis_rpm_) >= 0.1 || !fourth_motor_->motion_stopped()) {return;}
    fourth_waiting_for_stop_ = false;
  }
  if (fourth_phase_ != PositionPhase::idle && now >= fourth_deadline_) {
    throw std::runtime_error("fourth-axis PP command acknowledgement timed out");
  }
  switch (fourth_phase_) {
    case PositionPhase::idle: {
        // A cancelled write can still complete. Never start a second reservation
        // until it has been acknowledged, including after release/reversal.
        if (fourth_memory_ && fourth_memory_->pending()) {return;}
        // Half a second of travel ahead of measured position, never an accumulated
        // target. A stalled motor cannot build an arbitrarily long queued move.
        const auto step = std::max<std::int64_t>(1, std::llround(
          fourth_axis_position_scale_ * motion_config_.fourth_speed_rpm / 120.0));
        const auto actual = static_cast<std::int64_t>(fourth_axis_position_pulses_);
        auto target = actual + fourth_direction_ * step;
        if (fourth_direction_ > 0) {target = std::min<std::int64_t>(target, fourth_upper_pulses_);}
        else {target = std::max<std::int64_t>(target, 0);}
        // Outside starts move inward in bounded steps; never jump to a boundary.
        if (fourth_target_ && *fourth_target_ == target) {return;}
        fourth_target_ = target;
        fourth_deadline_ = now + std::chrono::seconds(1);
        if (fourth_memory_) {
          fourth_memory_->queue_target(fourth_raw_position_, target);
          fourth_phase_ = PositionPhase::queued;
        } else {
          fourth_motor_->prepare_position_command(static_cast<std::int32_t>(target));
          fourth_phase_ = PositionPhase::staging;
        }
        break;
      }
    case PositionPhase::queued:
      if (!fourth_memory_->pending()) {
        fourth_motor_->prepare_position_command(fourth_memory_->raw_target(*fourth_target_));
        fourth_phase_ = PositionPhase::staging;
      }
      break;
    case PositionPhase::staging:
      if (fourth_motor_->stage_position_command()) {fourth_phase_ = PositionPhase::triggering;}
      break;
    case PositionPhase::triggering:
      if (fourth_memory_) {fourth_memory_->mark_target(*fourth_target_);}
      fourth_motor_->trigger_position_command();
      fourth_phase_ = PositionPhase::acknowledging;
      break;
    case PositionPhase::acknowledging:
      if (fourth_motor_->finish_position_command()) {fourth_phase_ = PositionPhase::idle;}
      break;
  }
}

void ArmController::read_fourth_position()
{
  fourth_raw_position_ = fourth_motor_->actual_position();
  fourth_axis_position_pulses_ = fourth_memory_ ?
    fourth_memory_->logical_position(fourth_raw_position_) : fourth_raw_position_;
}

std::string ArmController::fault_reason() const
{
  return fault_reason_;
}

void ArmController::stop()
{
  initialized_ = false;
  position_phase_ = PositionPhase::idle;
  planar_halted_ = false;
  planar_halt_pending_ = false;
  std::exception_ptr first_error;
  if (fourth_motor_) {
    try {
      fourth_motor_->stop();
      fourth_direction_ = 0;
      fourth_phase_ = PositionPhase::idle;
      fourth_target_.reset();
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

void ArmController::publish_torque_sample(std::chrono::steady_clock::time_point now) noexcept
{
  if (!motion_config_.monitor_torque || !torque_observer_) {return;}
  try {
    ArmTorqueSample sample;
    sample.sampled_at = now;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      sample.permille[i] = motors_[i].actual_torque_permille();
      sample.rated_mnm[i] = motors_[i].rated_torque_millinewton_meter();
    }
    if (fourth_motor_) {
      sample.permille[3] = fourth_motor_->actual_torque_permille();
      sample.rated_mnm[3] = fourth_motor_->rated_torque_millinewton_meter();
    }
    torque_observer_(sample);
  } catch (...) {
  }
}

}  // namespace chassis
