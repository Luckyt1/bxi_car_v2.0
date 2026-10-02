#include "arm/kinematics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace arm
{
namespace
{

constexpr double kTwoPi = 2.0 * kPi;
constexpr long double kAngleLimitTolerance = 1.0e-12L;
constexpr long double kLongTwoPi = 2.0L * static_cast<long double>(kPi);

bool finite(double value)
{
  return std::isfinite(value);
}

double normalize_angle(double angle)
{
  if (!finite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double wrapped = std::remainder(angle, kTwoPi);
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  } else if (wrapped > kPi) {
    wrapped -= kTwoPi;
  }
  return wrapped;
}

bool valid_limit(const JointLimit & limit)
{
  return finite(limit.min) && finite(limit.max) && limit.min <= limit.max;
}

bool within_limit(double value, const JointLimit & limit)
{
  return value >= limit.min && value <= limit.max;
}

long double clamp_long_double(long double value, long double minimum, long double maximum)
{
  return std::min(std::max(value, minimum), maximum);
}

std::optional<double> equivalent_angle_in_limit(
  double angle, const JointLimit & limit, double reference)
{
  if (!finite(angle) || !valid_limit(limit)) {
    return std::nullopt;
  }

  const long double base = static_cast<long double>(angle);
  const long double minimum = static_cast<long double>(limit.min);
  const long double maximum = static_cast<long double>(limit.max);
  const long double first = std::ceil((minimum - base - kAngleLimitTolerance) / kLongTwoPi);
  const long double last = std::floor((maximum - base + kAngleLimitTolerance) / kLongTwoPi);
  if (last < first) {
    return std::nullopt;
  }

  const long double desired =
    std::round((static_cast<long double>(reference) - base) / kLongTwoPi);
  const long double center = clamp_long_double(desired, first, last);

  std::optional<double> best;
  long double best_distance = std::numeric_limits<long double>::infinity();
  for (const long double offset : {-2.0L, -1.0L, 0.0L, 1.0L, 2.0L}) {
    const long double k = center + offset;
    if (k < first || k > last) {
      continue;
    }
    long double long_value = base + k * kLongTwoPi;
    if (!std::isfinite(long_value) ||
      long_value < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
      long_value > static_cast<long double>(std::numeric_limits<double>::max()))
    {
      continue;
    }
    if (long_value < minimum && minimum - long_value <= kAngleLimitTolerance) {
      long_value = minimum;
    }
    if (long_value > maximum && long_value - maximum <= kAngleLimitTolerance) {
      long_value = maximum;
    }
    const double value = static_cast<double>(long_value);
    if (within_limit(value, limit)) {
      const long double distance =
        std::abs(long_value - static_cast<long double>(reference));
      if (!best || distance < best_distance) {
        best = value;
        best_distance = distance;
      }
    }
  }
  return best;
}

long double distance_to_seed(const JointAngles & joints, const JointAngles & seed)
{
  const long double d0 = static_cast<long double>(joints[0]) - static_cast<long double>(seed[0]);
  const long double d1 = static_cast<long double>(joints[1]) - static_cast<long double>(seed[1]);
  const long double d2 = static_cast<long double>(joints[2]) - static_cast<long double>(seed[2]);
  return d0 * d0 + d1 * d1 + d2 * d2;
}

long double distance_to_zero(const JointAngles & joints)
{
  const long double d0 = static_cast<long double>(joints[0]);
  const long double d1 = static_cast<long double>(joints[1]);
  const long double d2 = static_cast<long double>(joints[2]);
  return d0 * d0 + d1 * d1 + d2 * d2;
}

bool same_solution(const JointAngles & lhs, const JointAngles & rhs)
{
  return std::abs(lhs[0] - rhs[0]) < 1.0e-7 &&
         std::abs(lhs[1] - rhs[1]) < 1.0e-7 &&
         std::abs(lhs[2] - rhs[2]) < 1.0e-7;
}

void append_solution(
  std::vector<Solution> & solutions, const JointAngles & joints, bool singular)
{
  const auto duplicate = std::find_if(
    solutions.begin(), solutions.end(),
    [&](const Solution & solution) {return same_solution(solution.joints, joints);});
  if (duplicate == solutions.end()) {
    solutions.push_back(Solution{joints, singular});
  }
}

void append_limited_solution(
  std::vector<Solution> & solutions,
  const Configuration & configuration,
  const JointAngles & raw_joints,
  const JointAngles & reference,
  bool singular)
{
  const auto shoulder =
    equivalent_angle_in_limit(
    normalize_angle(raw_joints[0] - normalize_angle(configuration.zero_offsets[0])),
    configuration.limits[0], reference[0]);
  const auto elbow =
    equivalent_angle_in_limit(
    normalize_angle(raw_joints[1] - normalize_angle(configuration.zero_offsets[1])),
    configuration.limits[1], reference[1]);
  const auto wrist =
    equivalent_angle_in_limit(
    normalize_angle(raw_joints[2] - normalize_angle(configuration.zero_offsets[2])),
    configuration.limits[2], reference[2]);
  if (shoulder && elbow && wrist) {
    append_solution(solutions, JointAngles{*shoulder, *elbow, *wrist}, singular);
  }
}

std::optional<JointAngles> folded_continuum_solution(
  const Configuration & configuration, double pitch, const JointAngles & reference)
{
  const auto elbow = equivalent_angle_in_limit(
    normalize_angle(kPi - normalize_angle(configuration.zero_offsets[1])),
    configuration.limits[1], reference[1]);
  if (!elbow) {
    return std::nullopt;
  }

  const long double sum_base =
    static_cast<long double>(pitch) - static_cast<long double>(*elbow) -
    static_cast<long double>(normalize_angle(configuration.zero_offsets[0])) -
    static_cast<long double>(normalize_angle(configuration.zero_offsets[1])) -
    static_cast<long double>(normalize_angle(configuration.zero_offsets[2]));
  const long double shoulder_min = static_cast<long double>(configuration.limits[0].min);
  const long double shoulder_max = static_cast<long double>(configuration.limits[0].max);
  const long double wrist_min = static_cast<long double>(configuration.limits[2].min);
  const long double wrist_max = static_cast<long double>(configuration.limits[2].max);
  const long double first =
    std::ceil((shoulder_min + wrist_min - sum_base - kAngleLimitTolerance) / kLongTwoPi);
  const long double last =
    std::floor((shoulder_max + wrist_max - sum_base + kAngleLimitTolerance) / kLongTwoPi);
  if (last < first) {
    return std::nullopt;
  }

  const long double reference_sum =
    static_cast<long double>(reference[0]) + static_cast<long double>(reference[2]);
  const long double desired = std::round((reference_sum - sum_base) / kLongTwoPi);
  const long double center = clamp_long_double(desired, first, last);
  std::optional<JointAngles> best;
  long double best_distance = std::numeric_limits<long double>::infinity();

  for (const long double offset : {-2.0L, -1.0L, 0.0L, 1.0L, 2.0L}) {
    const long double k = center + offset;
    if (k < first || k > last) {
      continue;
    }
    const long double total = sum_base + k * kLongTwoPi;
    const long double interval_min = std::max(shoulder_min, total - wrist_max);
    const long double interval_max = std::min(shoulder_max, total - wrist_min);
    if (interval_max < interval_min) {
      continue;
    }

    const long double unconstrained =
      (static_cast<long double>(reference[0]) + total - static_cast<long double>(reference[2])) /
      2.0L;
    const long double shoulder = clamp_long_double(unconstrained, interval_min, interval_max);
    const long double wrist = total - shoulder;
    if (!std::isfinite(shoulder) || !std::isfinite(wrist) ||
      shoulder < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
      shoulder > static_cast<long double>(std::numeric_limits<double>::max()) ||
      wrist < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
      wrist > static_cast<long double>(std::numeric_limits<double>::max()))
    {
      continue;
    }

    const JointAngles candidate{
      static_cast<double>(shoulder), *elbow, static_cast<double>(wrist)};
    const long double d0 = shoulder - static_cast<long double>(reference[0]);
    const long double d1 = static_cast<long double>(*elbow) -
      static_cast<long double>(reference[1]);
    const long double d2 = wrist - static_cast<long double>(reference[2]);
    const long double distance = d0 * d0 + d1 * d1 + d2 * d2;
    if (!best || distance < best_distance) {
      best = candidate;
      best_distance = distance;
    }
  }
  return best;
}

}  // namespace

Kinematics::Kinematics(const Configuration & configuration)
: configuration_(configuration)
{
  if (!finite(configuration_.upper_arm) || !finite(configuration_.forearm) ||
    !finite(configuration_.tool) || configuration_.upper_arm <= 0.0 ||
    configuration_.forearm <= 0.0 || configuration_.tool < 0.0 ||
    !finite(configuration_.upper_arm + configuration_.forearm + configuration_.tool))
  {
    throw std::invalid_argument(
            "upper arm and forearm must be positive; tool length must be non-negative");
  }

  if (!std::all_of(configuration_.limits.begin(), configuration_.limits.end(), valid_limit)) {
    throw std::invalid_argument("joint limits must be finite and ordered");
  }
  if (!std::all_of(
      configuration_.zero_offsets.begin(), configuration_.zero_offsets.end(),
      finite))
  {
    throw std::invalid_argument("joint zero offsets must be finite");
  }
}

Pose Kinematics::forward(const JointAngles & joints) const
{
  if (!std::all_of(joints.begin(), joints.end(), finite)) {
    throw std::invalid_argument("joint angles must be finite");
  }

  JointAngles geometric{};
  for (std::size_t i = 0; i < joints.size(); ++i) {
    geometric[i] = normalize_angle(
      normalize_angle(joints[i]) + normalize_angle(configuration_.zero_offsets[i]));
  }
  const double shoulder_angle = geometric[0];
  const double elbow_angle = normalize_angle(shoulder_angle + geometric[1]);
  const double pitch_angle = normalize_angle(elbow_angle + geometric[2]);
  const long double shoulder = static_cast<long double>(shoulder_angle);
  const long double elbow = static_cast<long double>(elbow_angle);
  const long double pitch =
    static_cast<long double>(pitch_angle);
  const long double x =
    static_cast<long double>(configuration_.upper_arm) * std::cos(shoulder) +
    static_cast<long double>(configuration_.forearm) * std::cos(elbow) +
    static_cast<long double>(configuration_.tool) * std::cos(pitch);
  const long double z =
    static_cast<long double>(configuration_.upper_arm) * std::sin(shoulder) +
    static_cast<long double>(configuration_.forearm) * std::sin(elbow) +
    static_cast<long double>(configuration_.tool) * std::sin(pitch);

  return Pose{
    static_cast<double>(x),
    static_cast<double>(z),
    static_cast<double>(pitch)};
}

InverseResult Kinematics::inverse(
  const Pose & target, const std::optional<JointAngles> & seed) const
{
  if (!finite(target.x) || !finite(target.z) || !finite(target.pitch)) {
    return InverseResult{InverseStatus::InvalidInput, {}};
  }
  if (seed) {
    if (!std::all_of(seed->begin(), seed->end(), finite)) {
      return InverseResult{InverseStatus::InvalidInput, {}};
    }
  }

  const double pitch = normalize_angle(target.pitch);
  const long double long_pitch = static_cast<long double>(pitch);
  const long double wrist_x = static_cast<long double>(target.x) -
    static_cast<long double>(configuration_.tool) * std::cos(long_pitch);
  const long double wrist_z = static_cast<long double>(target.z) -
    static_cast<long double>(configuration_.tool) * std::sin(long_pitch);
  const long double wrist_radius_sq = wrist_x * wrist_x + wrist_z * wrist_z;
  const long double wrist_radius = std::sqrt(wrist_radius_sq);

  const long double upper = static_cast<long double>(configuration_.upper_arm);
  const long double forearm = static_cast<long double>(configuration_.forearm);
  const long double max_reach = upper + forearm;
  const long double min_reach = std::abs(upper - forearm);
  const long double reach_tolerance =
    64.0L * static_cast<long double>(std::numeric_limits<double>::epsilon()) *
    max_reach;

  if (wrist_radius > max_reach + reach_tolerance ||
    wrist_radius < min_reach - reach_tolerance)
  {
    return InverseResult{InverseStatus::Unreachable, {}};
  }

  std::vector<Solution> solutions;
  const JointAngles reference = seed ? *seed : JointAngles{0.0, 0.0, 0.0};
  const bool equal_link_origin =
    wrist_radius <= reach_tolerance && configuration_.upper_arm == configuration_.forearm;

  if (equal_link_origin) {
    const auto folded = folded_continuum_solution(configuration_, pitch, reference);
    if (folded) {
      append_solution(solutions, *folded, true);
    }
  } else {
    long double cos_elbow =
      (wrist_radius_sq - upper * upper - forearm * forearm) /
      (2.0L * upper * forearm);
    cos_elbow = clamp_long_double(cos_elbow, -1.0L, 1.0L);
    const long double elbow_abs = std::acos(cos_elbow);
    const bool singular = std::abs(std::sin(elbow_abs)) <= 1.0e-7L;

    for (const long double elbow : {elbow_abs, -elbow_abs}) {
      const long double shoulder = std::atan2(wrist_z, wrist_x) -
        std::atan2(forearm * std::sin(elbow), upper + forearm * std::cos(elbow));
      const long double wrist = static_cast<long double>(pitch) - shoulder - elbow;
      append_limited_solution(
        solutions, configuration_,
        JointAngles{
          normalize_angle(static_cast<double>(shoulder)),
          normalize_angle(static_cast<double>(elbow)),
          normalize_angle(static_cast<double>(wrist))},
        reference,
        singular);
    }
  }

  if (solutions.empty()) {
    return InverseResult{InverseStatus::JointLimits, {}};
  }

  if (seed) {
    std::sort(
      solutions.begin(), solutions.end(),
      [&](const Solution & lhs, const Solution & rhs) {
        return distance_to_seed(lhs.joints, *seed) < distance_to_seed(rhs.joints, *seed);
      });
  } else {
    std::sort(
      solutions.begin(), solutions.end(),
      [&](const Solution & lhs, const Solution & rhs) {
        const double offset = normalize_angle(configuration_.zero_offsets[1]);
        const bool lhs_positive = std::sin(normalize_angle(lhs.joints[1]) + offset) >= -1.0e-12;
        const bool rhs_positive = std::sin(normalize_angle(rhs.joints[1]) + offset) >= -1.0e-12;
        if (lhs_positive != rhs_positive) {
          return lhs_positive;
        }
        const long double lhs_score = distance_to_zero(lhs.joints);
        const long double rhs_score = distance_to_zero(rhs.joints);
        if (std::abs(lhs_score - rhs_score) > 1.0e-18L) {
          return lhs_score < rhs_score;
        }
        return lhs.joints[1] > rhs.joints[1];
      });
  }

  return InverseResult{InverseStatus::Success, solutions};
}

}  // namespace arm
