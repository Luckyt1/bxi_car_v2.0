#include "arm/control.h"
#include "arm/kinematics.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: arm_solver --config arm_kinematics.json\n"
      "离线解算，不初始化硬件。输入 1 q1 q2 q3 做 FK（rad）；"
      "2 x z pitch 做 IK（m, m, rad）；0 或 EOF 退出。\n";
    return 0;
  }
  if (argc != 3 || std::string(argv[1]) != "--config") {
    std::cerr << "Usage: arm_solver --config arm_kinematics.json\n";
    return 2;
  }
  try {
    const arm::Kinematics solver(chassis::load_arm_kinematics(argv[2]));
    std::cout << "机械臂离线解算（校准后的关节角，单位 rad）\n"
      "1 q1 q2 q3：正解 FK；2 x z pitch：逆解 IK（m, m, rad）；0：退出\n";
    std::string line;
    while (std::cout << "> " << std::flush, std::getline(std::cin, line)) {
      std::istringstream input(line);
      std::string operation;
      input >> operation;
      if (operation == "0") {return 0;}
      double first, second, third;
      std::string extra;
      if ((operation != "1" && operation != "2") || !(input >> first >> second >> third) ||
        (input >> extra) || !std::isfinite(first) || !std::isfinite(second) ||
        !std::isfinite(third))
      {
        std::cerr << "输入无效：使用 1/2 加三个有限数值。\n";
        continue;
      }
      if (operation == "1") {
        const auto pose = solver.forward({first, second, third});
        std::cout << "x=" << pose.x << " z=" << pose.z << " pitch=" << pose.pitch << '\n';
      } else {
        const auto result = solver.inverse({first, second, third});
        if (result.status != arm::InverseStatus::Success) {
          std::cout << "无可行逆解（不可达或超出关节限位），status="
                    << static_cast<int>(result.status) << '\n';
          continue;
        }
        for (const auto & solution : result.solutions) {
          std::cout << "q=" << solution.joints[0] << ' ' << solution.joints[1] << ' '
                    << solution.joints[2] << " singular=" << solution.singular << '\n';
        }
      }
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Arm solver: " << error.what() << '\n';
    return 1;
  }
}
