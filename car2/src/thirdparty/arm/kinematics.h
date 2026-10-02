#pragma once

#include <array>
#include <optional>
#include <vector>

namespace arm
{

inline constexpr double kPi = 3.14159265358979323846;
// Shoulder, elbow, wrist output angles relative to their calibrated zeros; radians.
using JointAngles = std::array<double, 3>;

struct JointLimit
{
  double min{-kPi};
  double max{kPi};
};

struct Configuration
{
  // Link lengths: 300 mm / 400 mm.
  double upper_arm{0.30};  // Shoulder to elbow, metres; must be positive.
  double forearm{0.40};    // Elbow to wrist, metres; must be positive.
  double tool{0.0};       // Wrist to tool tip, along the tool's pitch direction.
  // Limits use calibrated coordinates. These preserve geometric ranges
  // shoulder [0, 180], elbow [-180, 180], wrist [-180, 180] degrees.
  std::array<JointLimit, 3> limits{
    JointLimit{-5.0 * kPi / 6.0, kPi - 5.0 * kPi / 6.0},
    JointLimit{-kPi + 5.0 * kPi / 6.0, kPi + 5.0 * kPi / 6.0}, JointLimit{}};
  // Geometric relative angles = joints + zero_offsets. At joints = [0, 0, 0],
  // the upper arm is at 150 degrees from +X, forearm and tool point horizontally forward.
  JointAngles zero_offsets{5.0 * kPi / 6.0, -5.0 * kPi / 6.0, 0.0};
};

// Shoulder frame: +X forward, +Z up. Positive pitch rotates +X towards +Z.
// Pitch is the sum of joints[i] + zero_offsets[i] (modulo 2*pi).
// There is no yaw or lateral (Y) degree of freedom.
struct Pose
{
  double x{0.0};
  double z{0.0};
  double pitch{0.0};
};

enum class InverseStatus
{
  Success,
  InvalidInput,
  Unreachable,
  JointLimits
};

struct Solution
{
  JointAngles joints{};
  bool singular{false};  // Fully extended/folded: Cartesian motion loses a DOF.
};

struct InverseResult
{
  InverseStatus status{InverseStatus::Unreachable};
  // At most one representative per elbow branch; nearest seed first.
  // Without a seed, prefer angles close to zero, positive elbow branch first.
  // At equal-link full folding, return one feasible representative of the continuum.
  std::vector<Solution> solutions;
};

class Kinematics
{
public:
  // Throws std::invalid_argument for invalid dimensions, limits or nonfinite offsets.
  explicit Kinematics(const Configuration & configuration);

  const Configuration & configuration() const noexcept {return configuration_;}

  // Throws std::invalid_argument for nonfinite joint angles. Does not enforce limits:
  // forward kinematics can also describe measured joints outside the commanded range.
  // Output pitch is normalized to [-pi, pi].
  Pose forward(const JointAngles & joints) const;

  // Inputs are metres and radians. Rejects unreachable targets outside roundoff tolerance.
  // Each solution respects limits, allowing equivalent angles differing by 2*pi.
  // Seed uses the same calibrated joint coordinates, and need not be within limits.
  // Motor direction/reduction and encoder counts are handled outside this solver.
  InverseResult inverse(
    const Pose & target, const std::optional<JointAngles> & seed = std::nullopt) const;

private:
  Configuration configuration_;
};

}  // namespace arm
