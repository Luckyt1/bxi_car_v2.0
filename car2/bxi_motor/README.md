# BXI 本体电机 · MIT 控制

从 [bxirobotics/bxi_car](https://github.com/bxirobotics/bxi_car) 提取电机控制代码，补全先前只收录退出命令的模块。提供独立 C++17 库，通过 `iswv::ICanTransport` 发送原始 CAN/CAN FD 帧，无 ROS 依赖。

## 已包含

| 接口 | 用途 |
| --- | --- |
| `bxi::Motor::enter_motor_mode()` | 进入电机模式：`FF FF FF FF FF FF FF FC` |
| `bxi::Motor::exit_motor_mode()` | 退出电机模式：`FF FF FF FF FF FF FF FD` |
| `bxi::Motor::command(Command)` | 下发位置、速度、kp、kd、前馈力矩五参数 |
| `bxi::pack_command(Command)` | 只编码 8 字节数据，便于接入自己的传输接口 |
| `bxi::decode_feedback(CanFrame, EncodingRanges)` | 按指定范围解析位置、速度反馈，范围可省略以使用默认值 |

头文件为 [include/bxi/motor.hpp](include/bxi/motor.hpp)，实现为 [src/motor.cpp](src/motor.cpp)，CMake target 为 `bxi::motor`。需要编译并链接实现文件。

共享电机供电、PCI 初始化和风扇控制见 [bxi_hardware](../bxi_hardware/README.md)。本模块构造和析构都不发送电机指令，应用负责显式退出电机模式，再关闭共享电源。

## 编码范围和协议约定

| `Command` 字段 | 默认编码范围 | 位宽 |
| --- | --- | --- |
| `position` | -12.5 ～ 12.5 | 16 |
| `velocity` | -45 ～ 45 | 12 |
| `kp` | 0 ～ 500 | 12 |
| `kd` | 0 ～ 5 | 12 |
| `torque` | -40 ～ 40 | 12 |

这些是源码中的编码范围，并非所接电机的额定范围。上游源码没有明确注明各物理量单位，接入具体型号前需要核对；接口不会把底盘线速度换算成电机速度，也不把这些数值当作 ISWV/EYOU 的输出轴 RPM。

不同型号可以通过 `Command::ranges` 分别设置五个参数的编码范围，每个范围包含 `min` 和 `max`。范围参与数值校验和字节编码，必须与设备协议一致，不是运行时运动限位。默认值保持上表范围，原有 `Command{position, velocity, kp, kd, torque}` 写法仍可使用。

```cpp
bxi::Command command;
// 以下仅为配置写法示例，不代表具体型号参数；按各型号手册填写。
command.ranges.position = {-12.5F, 12.5F};
command.ranges.velocity = {-60.0F, 60.0F};
command.ranges.kp = {0.0F, 500.0F};
command.ranges.kd = {0.0F, 10.0F};
command.ranges.torque = {-20.0F, 20.0F};
command.velocity = 30.0F;
command.kd = 6.0F;
auto sent = motor.command(command);
auto feedback = bxi::decode_feedback(incoming, command.ranges);
```

同型号命令可以复制同一份 `EncodingRanges`；不同型号各自持有配置，互不影响。反馈解码需要传入对应电机的范围，否则使用默认范围；目前只使用并校验位置和速度范围。

合法值沿用上游的量化截断方式；范围端点必须有限且 `min < max`。无效范围、越界、NaN、Inf 返回 `invalid_argument`，不发送帧。普通参数编码结果若与已知进入/退出指令重合，也会被拒绝。上游示例曾传入 `kd=6`，在默认 `[0,5]` 范围下仍被拒绝；只有实际型号支持且配置了相应编码范围时才可使用。

默认帧为标准 CAN ID、8 字节 CAN FD+BRS，沿用原 `motor_test.c` 的明确设置；可用 `FrameOptions{false, false}` 选择经典 CAN。上游 `chassis.cpp` 没有初始化帧 flags，不能据此判断真实线上的帧类型。本封装明确初始化全部帧字段，不自动修改板卡波特率。

反馈至少需要 5 字节：位置取 `data[1]`、`data[2]`，速度取 `data[3]` 和 `data[4]` 高半字节。`prefix` 原样保留 `data[0]`，不推定其含义。源代码没有解析力矩、电流和温度反馈，因此接口不虚构这些字段。无效帧、短帧及非标准数据帧返回 `protocol_error`。

上游底盘使用 CAN bus 2、指令/反馈 ID 1 和 2；早期延迟样例使用 CAN0、指令 ID 1、监听 ID 0x11。调用方应按实际接线及电机协议路由反馈，不套用未经确认的 ID 偏移公式。`decode_feedback()` 不安装或覆盖接收回调。

## 离线示例

```cpp
#include "bxi/motor.hpp"
#include "iswv/fake_transport.hpp"

int main()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  if (!motor.enter_motor_mode()) { return 1; }
  const auto command = motor.command({0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
  const auto exit = motor.exit_motor_mode();
  return command && exit ? 0 : 1;
}
```

完整的双电机离线发送与模拟反馈示例见 [examples/offline.cpp](examples/offline.cpp)。使用真实 transport 时，各发送方法成功只表示传输层接受发送，未等待设备确认；退出命令不构成物理停机已完成的证明。应用应管理运动状态、失联动作和硬件停机条件。

从工具库根目录执行：

```bash
python3 tools.py build bxi_motor
python3 tools.py test bxi_motor
python3 tools.py run bxi_motor
```

`run bxi_motor` 仅运行 FakeTransport 示例。也可直接构建：

```bash
cmake -S Can/bxi_motor -B /tmp/bxi-motor-build
cmake --build /tmp/bxi-motor-build -j2
ctest --test-dir /tmp/bxi-motor-build --output-on-failure
/tmp/bxi-motor-build/bxi_motor_offline_example
```

接入其他 CMake 项目：

```cmake
add_subdirectory(path/to/bxi_motor)
target_link_libraries(my_app PRIVATE bxi::motor)
```

如果父项目没有提供 `iswv::canopen`，默认使用相邻的 `ISWV`；可通过 `ISWV_SOURCE_DIR` 指定其位置。测试和示例分别由 `BXI_MOTOR_BUILD_TESTS`、`BXI_MOTOR_BUILD_EXAMPLES` 控制。

## 来源及适配

固定来源提交：[`a02e41f`](https://github.com/bxirobotics/bxi_car/tree/a02e41f59373b36e38686ed8be83f3421f1959a4)。原始电机逻辑在 [chassis.cpp](https://github.com/bxirobotics/bxi_car/blob/a02e41f59373b36e38686ed8be83f3421f1959a4/diffbot_chassis/src/chassis/src/chassis.cpp)，本地完整保留在 [reference/upstream_chassis.cpp](reference/upstream_chassis.cpp)，许可证见 [LICENSE](LICENSE)。原始示例未加入构建，因为它包含实际硬件操作。

适配保留合法参数的字节编码，增加输入和反馈长度检查，修复未初始化 CAN 帧，并避免原退出路径“仅初始化一个元素却发送两个元素”的问题。原有 [reference/motor_test.c](reference/motor_test.c) 保留为初次收录依据。

测试包含上游编码对照、边界与随机合法值、越界/非有限值拒绝、模式指令、反馈解码及传输失败传播。当前验证为离线协议和构建测试；具体型号单位、机械参数、反馈完整字段及实机运行仍需按设备补充。
