# 按范围实施与验证

## 只移植机械臂解算

复制 `src/thirdparty/arm/kinematics.h` 和 `kinematics.cpp`，保留 `arm/kinematics.h` 的 include 关系。同时保留 `docs/arm_kinematics_origin.md` 的算法来源记录及适用的 `LICENSE`。例如目标项目沿用 `src/thirdparty/arm/` 时：

```cmake
# 接入目标 CMake；空项目先添加 cmake_minimum_required 和 project。
add_library(arm_kinematics STATIC src/thirdparty/arm/kinematics.cpp)
target_compile_features(arm_kinematics PUBLIC cxx_std_17)
target_include_directories(arm_kinematics PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/thirdparty")
# target_link_libraries(your_application PRIVATE arm_kinematics)
```

调用 `arm::Kinematics(configuration).forward(joints)` / `inverse(pose, seed)`；参考 `arm/example.cpp`。实际尺寸、限位、标定零偏由调用方构造；不要只改连杆长度却无意沿用旧零偏。需要配置文件时接入目标已有解析方式，避免为加载 JSON 链接整个硬件控制层。

离线验证以正解后逆解再正解的位姿一致性为主，角度按周期比较，并覆盖不可达目标或无效输入。逆解可能有多个分支，不要求所有解都等于原关节角。测试使用明确的模型参数，避免依赖本车已校准数据。

## 移植单种电机或板卡

1. 检查目标的通信实现；已有 CAN 接口时可实现/适配 `iswv::ICanTransport`。没有使用 BXI 板卡的需求时，不因接口命名含 ISWV 就引入厂家 PCI 库。
2. 按 [模块地图](project-map.md) 和原 CMake 拷贝对应源码、内部头和协议依赖。ISWV 协议实现在 `protocol/`，意优还需要 `detail/drive.h` 与本地 SOEM；不能只拿公开的两对文件。
3. CMake 明确列出调用层、通信层和协议库，设置 include 根、C++17 和线程链接。含 SOEM 时启用 C 语言并保留 Linux OSAL/OSHW、`rt` 等当前依赖。不要把其他平台假定成兼容 Linux 驱动。
4. 接入具体 transport/master 的生命周期和错误处理；若系统已有共享总线，用现有对象传入电机，不创建竞争的控制者。
5. 调用示例分清初始化、周期目标输入、反馈和退出。保留设备支持的模式：BXI 是 MIT；ISWV 简单调用层是速度模式；意优支持 PV/PP/CST，详情以头文件为准。

当前 BXI MIT 的最小生产依赖已通过独立 C++17 消费者验证：`motor/bxi/` 的两对通信/调用文件，加 `motor/iswv/protocol/include/iswv/` 中的 `transport.hpp`、`can_frame.hpp`、`result.hpp`。接入自有 transport 时无需链接 CANopen 实现或 BXI 板卡库。此裁剪只适用于 BXI MIT，不适用于 ISWV 驱动；源码版本变化后重新确认头文件依赖。

BXI 验证可另带 `iswv/fake_transport.hpp` 和 `protocol/src/fake_transport.cpp`，使用 FakeTransport 检查编码、发帧与错误传递，不创建板卡。ISWV 可沿用 `test/unit/test_motor_layers.cpp` 中模拟通信的相关检查。EtherCAT 没有实机时，验证构建、接口、单位转换及可离线执行的状态逻辑，明确实际链路未测；不要编造模拟器或把未运行的循环通信写成通过。

移植到不同架构时，先检查厂家静态库。二进制不兼容且无源码时，完成其余可验证部分并说明需要目标平台版本，不通过忽略链接错误宣称完成。

## 移植机械臂、底盘或完整 ROS 主控

硬件控制模块不直接要求 ROS；按实际选定模块接入依赖。完整 ROS 模式再携带 `src/example/` 中需要的文件、`arm/joystick.hpp` 等传递头、`package.xml` 所需依赖和安装规则。

- 保持 `main()` 简短，按配置读取、硬件初始化、ROS 接口创建、周期控制、退出处理划分函数；不为每行调用额外建类。
- 新应用与已有主循环整合，不重复启动控制同一设备的节点。底盘与机械臂共享电源时由一处负责，保持当前启动与销毁顺序。
- 保留用户需要的话题、消息类型、参数及单位。修改命名或新增超时行为应明确说明，不能把目录整理变成隐藏的接口修改。
- 若沿用原 `main.cpp`，适配 `CHASSIS_CONTROL_MODE`、`CHASSIS_SOURCE_CONFIG_DIR` 和默认配置查找策略。若沿用安装包，链接安装导出的 target，不写开发机绝对库路径。
- 迁移启动脚本时检查它们对 `test/debug.py`、构建和安装目录的依赖。目标已有 launch/启动器时直接接入，不额外复制一套不用的入口。

按目标项目既有约定组织文件。空项目可采用 `src/thirdparty/{bxi,motor,arm,chassis}`、`src/example/`、`src/config/`、`test/`；已有项目不必整体重排。

## 配置迁移

先分清可保留的接口约定和必须重新提供的实机参数：

| 类别 | 实施要求 |
| --- | --- |
| 通信 | 核对 CAN 通道、Node-ID/标准 ID、FD/BRS、EtherCAT 网卡与物理站位 |
| 机械 | 核对轮序、尺寸、连杆、编码器分辨率、减速比、方向、关节限位 |
| 标定 | 不把参考零偏、零位脉冲或 `steering_calibrated: true` 宣称为新机器标定结果 |
| 启动 | 确认自动回零、校准、CST 站位和共享电源安排；未知时交付为待配置，先做离线验证 |
| 运行 | 保留速度/加速度/电流等单位；目标值需要设备依据，不凭空生成“安全”数值 |
| 部署 | 适配配置路径、写回位置、手柄映射、目标构建目录；不依赖原开发机 cwd |

保留拆分 YAML 的引用和覆盖语义。换包名或节点名时同步调整 YAML 的节点键、安装位置和默认查找代码。ROS 显式参数应覆盖文件默认值；验证时注意多个 `--ros-args` 参数组及节点限定。

新硬件参数暂缺不阻止源码和构建移植，但最终必须区分“离线可构建”与“实机参数已确认”。不要默认运行回零、校准或 EEPROM 写入来补齐信息。

## 离线验收与交付

选取能证明这次移植正确的最小验证集：

| 移植范围 | 验证证据 |
| --- | --- |
| 纯解算 | 独立 C++17 消费者构建；FK/IK 位姿回环；不可达或无效输入 |
| 通信/电机 | 编译链接；可用假通信下的编码、错误传递或单位转换；明确未测的真实总线 |
| 底盘/机械臂控制 | 对应算法回归；配置加载/相对引用；目标公共接口的消费者构建 |
| ROS 集成 | 所选程序构建；离线参数/话题契约；默认配置与显式覆盖；启动器 `--help`/`--dry-run` |

参考现有 `test/unit/regression.cpp`、`test_motor_layers.cpp`、`test_steering_config.cpp` 和三个 Python 测试，按所选范围迁移有效断言。不要为了两文件解算器引入整套 ROS 测试。

没有用户指定构建目录时，可使用工作区外临时目录：

```bash
port_build_dir=$(mktemp -d /tmp/robot-port-build.XXXXXX)
cmake -S /path/to/target -B "$port_build_dir" -DBUILD_TESTING=ON
cmake --build "$port_build_dir" --parallel 4
ctest --test-dir "$port_build_dir" --output-on-failure
```

命令用于已接好相应 CMake/CTest 的目标项目。ROS 项目先加载目标发行版环境；使用 colcon 时把 build/install/log 也放在目标约定目录或工作区外，不生成新的 `build-refactor` 等临时目录到源码根。

执行前读清所选测试的设备访问行为。纯算法、FakeTransport、菜单预览、已确认的 `--help` 可离线运行；硬件示例和默认主程序启动不属于离线测试。

交付说明包含：迁移后的文件/target、最短调用片段、必要配置、实际执行的验证及结果、尚未验证的硬件部分。缺少环境或厂家库时明确限制，不把“源码已复制”表述为“可在新机器直接运行”。
