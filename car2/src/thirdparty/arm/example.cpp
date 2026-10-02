// 纯运动学函数示例，可复制到自己的 .cpp 中；不提供 main，不加入默认编译。
// 本工程链接 chassis_arm_kinematics；安装后链接 chassis::chassis_arm_kinematics。

#include "arm/kinematics.h"

arm::Pose arm_forward(
  const arm::JointAngles & joints, const arm::Configuration & config = {})
// 输入：joints=[肩角,肘角,腕角]，相对标定零位，单位 rad；config 为可选模型配置。
// 输出：Pose{x,z,pitch}，肩坐标系 X 向前、Z 向上，单位 m、m、rad，pitch 正向为抬头。
// 配置默认上臂 0.30 m、前臂 0.40 m，包含标定偏置；正解不检查关节限位。
// 无效配置或非有限关节角会抛出 std::invalid_argument。
{
  return arm::Kinematics(config).forward(joints);
}

arm::InverseResult arm_inverse(
  const arm::Pose & target, const arm::JointAngles & current_joints,
  const arm::Configuration & config = {})
// 输入：target={x,z,pitch}，单位 m、m、rad；current_joints 为当前肩/肘/腕角（rad）。
//       config 为可选模型配置，与正解保持一致；当前关节角用于选择附近的解。
// 输出：status 表示成功或失败；成功时 solutions 按接近 current_joints 的顺序排列。
//       每个解的 joints 是肩/肘/腕角（rad），singular 表示是否为奇异位形。
// 失败时检查 status（InvalidInput / Unreachable / JointLimits），不要访问 front()。
// 无效模型配置会抛出 std::invalid_argument。
{
  return arm::Kinematics(config).inverse(target, current_joints);
}

// 在自己的函数中调用：
// const arm::JointAngles joints{0.0, 0.0, 0.0};
// const auto pose = arm_forward(joints);
// const auto result = arm_inverse({0.15, 0.15, 0.0}, joints);
// if (result.status == arm::InverseStatus::Success) {
//   const auto & q = result.solutions.front().joints;  // q[0]/q[1]/q[2] 为肩/肘/腕角。
// }
// 自定义模型时，把同一个 config 作为 arm_forward 的第二参数、arm_inverse 的第三参数。
