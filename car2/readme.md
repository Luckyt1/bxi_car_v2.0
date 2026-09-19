# car2：四舵轮底盘说明

底盘工程位于 `diffbot_chassis/src/chassis`。原先分散在 `car2/bxi_motor`、`car2/bxi_pci_drv`、`car2/ISWV` 和 `car2/libcanfd` 中仍需要的代码和二进制依赖已经收拢到 `chassis` 包内；四个旧目录不再作为构建输入保留。

## 接线与电机

轮位顺序固定为 `[前左、后左、后右、前右]`。

| 用途 | CAN 通道 | Node-ID | 驱动接口 |
| --- | --- | --- | --- |
| 四个转向轴 | CAN0 | 1、2、3、4 | `iswv::Axis`，由 `chassis::ChassisController` 组织 |
| 前左、后左行进轴 | CAN1 | 1、2 | `chassis::IswvDrive` |
| 后右、前右行进轴 | CAN2 | 3、4 | `chassis::IswvDrive`，镜像安装，默认 `drive_inverted=true` |
| CAN3 机械臂 | CAN3 | 1、2、3 | `chassis::ArmController` 管理三个 BXI8515-19 |
| 意优上装电机 | 按调用方指定 | 1..127 | `chassis::YiyouMotor`，当前不由主底盘入口自动创建 |

CAN0/1/2/3 是 BXI PCI 板卡通道编号，不是 Linux SocketCAN 网卡名。不同 CAN 通道可以复用 Node-ID，同一通道不得重复。行进 RPM 均指输出轴 RPM；转向与行进减速比、编码器分辨率由 `config/steering.yaml` 配置。

## 构建与测试

需要 ROS 2 Humble、C++17 编译器、ament/colcon、`rclcpp`、`geometry_msgs` 和 `yaml-cpp`。BXI PCI 厂商静态库在 `chassis/vendor/bxi_pci/lib/libbxi_pci_drv.a`，头文件在 `chassis/vendor/bxi_pci/include/bxi_pci_drv.h`。

```bash
source /opt/ros/humble/setup.bash
cd car2/diffbot_chassis
colcon build --symlink-install --packages-select chassis
source install/setup.bash
colcon test --packages-select chassis
colcon test-result --verbose
```

`chassis` 包直接编译内置 CANopen 核心，不再读取 `ISWV_SOURCE_DIR`，也不再支持指向外部独立 ISWV 源码树。公开目标 `iswv::canopen` 仍会被导出，供 `chassis` 的公共 API 和外部用户链接。

当前 CTest 覆盖 CAN3 机械臂、遥控器、底盘控制边界、BXI 协议、BXI 离线示例、CANopen 原始回归和入口 `--help`。这些测试不访问真实电机。

## 真实硬件入口

以下命令在 `car2/` 下执行。除 `--help` 和只读手柄检查外，均可能访问真实板卡或控制电机；运行前确认机构已固定或架空、机械限位可用、硬件急停有效，且没有其他程序占用 BXI 板卡。

| 命令 | 用途 |
| --- | --- |
| `sudo -E bash start_remote_control.sh` | 旧双进程方式：底盘节点 + `start_chassis:=false` 纯遥控节点 |
| `ros2 run chassis key_control --ros-args -p steering_config_file:=/path.yaml` | 推荐自启动入口：等待 button14 后同进程启动底盘、CAN3 和遥控 |
| `ros2 run chassis robot_control --ros-args -p steering_config_file:=/path.yaml` | 同上，保留统一入口名称 |
| `ros2 run chassis key_control --ros-args -p start_chassis:=false` | 纯遥控发布模式，用于已有底盘节点 |
| `ros2 run chassis chassis ...` | 只启动底盘和 CAN3 机械臂，等待 `/cmd_vel_car` |
| `sudo -E bash start_motor_power_hold.sh` | 仅保持共享电机电源，不使能电机 |
| `bash start_can3_control.sh` | 第二终端 CAN3 调角界面，复用主程序参数服务 |
| `sudo ./inspect_joystick.py` | 只读取手柄事件，不启动 ROS 或电机 |

旧启动脚本默认使用 `diffbot_chassis/src/chassis/config/steering.yaml`，可用 `CHASSIS_CONFIG` 指定现场 YAML；直接运行 ROS 节点时，用 `steering_config_file` 参数指定配置。主节点订阅 `/cmd_vel_car`，读取 `linear.x`、`linear.y`（m/s）和 `angular.z`（rad/s）。停止发布速度不会自动停车；发送零速度、触发故障或退出进程才会停机。

## 遥控器按键启动

自启动程序直接运行 ROS 节点即可，按键等待逻辑已内置：

```bash
ros2 run chassis key_control --ros-args \
  -p steering_config_file:="$(pwd)/diffbot_chassis/src/chassis/config/steering.yaml"
```

`key_control` 和 `robot_control` 默认先创建遥控节点，等待手柄启动键 **button14**。节点会先确认急停 **button11** 已释放、启动键已释放，再接受一次真实按下；连接时已经按住启动键不会启动，必须松开后再按。等待和校准期间不发布底盘运动指令，也不提交 CAN3 调角请求，但急停仍有效。

启动键触发后，同进程初始化底盘和 CAN3 机械臂，执行转向校准，再进入遥控运行。初始化或校准失败会退出进程，不自动重试；校准期间遥控器断连会取消启动。按钮编号从 0 开始，与 `inspect_joystick.py` 一致。

`steering_config_file` 必须显式传入。它属于 `chassis` 节点参数，普通全局 CLI override（`-p steering_config_file:=...`）即可生效。`start_chassis` 和 `start_button` 属于 `remote_ctrl` 节点；设置 `-p start_chassis:=false` 时，`key_control` 保持纯遥控发布模式，适用于已经单独启动底盘节点的旧双进程流程。`start_remote_control.sh` 已按这个模式传参。

## 主程序入口

同一份 `src/control/main.cpp` 构建三个 ROS 可执行文件：

| 可执行文件 | 行为 |
| --- | --- |
| `chassis` | 启动底盘和 CAN3 机械臂；继续接收独立遥控节点或其他程序的 `/cmd_vel_car` |
| `key_control` | 默认等待 button14 后同进程启动底盘、CAN3 和遥控；`start_chassis:=false` 时为纯遥控 |
| `robot_control` | 与 `key_control` 默认行为相同，保留统一入口名称 |

`key_control` / `robot_control` 在初始化和转向校准期间会轮询遥控器，手柄急停可以取消启动；底盘完成校准后才创建底盘速度订阅和主控制定时器。正常退出时先停止 CAN3 机械臂，再由底盘控制器断共享电源。

示例：

```bash
ros2 run chassis key_control --ros-args \
  -p steering_config_file:="$(pwd)/diffbot_chassis/src/chassis/config/steering.yaml" \
  -p post_calibration_rpm:=0.0
```

三个入口的 `--help` 都不打开 CAN 设备或发送电机命令。

## 遥控器与 CAN3 机械臂

遥控器默认节点名为 `remote_ctrl`，底盘节点名为 `chassis`。默认前后轴为 3，横向轴为 0，旋转轴为 6；axis7 和 button3/button1 留给 CAN3 机械臂。底盘运动轴不能配置为 7，使能/停止/急停按钮不能配置为 1 或 3。

CAN3 机械臂由 `ArmController` 管理 ID 1、2、3 三个 BXI8515-19。启动时先退出电机模式，再各发送一次保存当前位置为零点，然后按 Kp=200、Kd=4 保持目标位置。初始目标为 0°；button3/button1 切换电机，axis7 每次回中后允许目标角 ±1°。调角通过参数服务提交，每次只允许一个在途请求，忙时不排队。

CAN3 反馈按普通 MIT 回复匹配：电机 1/2/3 的默认回复 ID 为 `0x11/0x12/0x13`，标准数据帧、8 字节、`data[0]` 匹配电机 ID。任一电机反馈速度绝对值达到 60 rpm，或首次保持后反馈缺失超过 1 秒，会锁存故障并请求关闭整车共享电源。日志里的断电提交成功不等于已测得母线电压消失，实际时延需要现场验证。

第二终端可用 `bash start_can3_control.sh` 通过同一参数服务调角；它不单独打开 PCI 板卡。

## 代码分工

业务源码分成 `motor` 和 `control`，公开接口放在 `include/chassis/`：

```text
diffbot_chassis/src/chassis/
├── include/
│   ├── chassis/motor/
│   │   ├── bxi_motor.h
│   │   ├── bxi_pci_transport.h
│   │   ├── iswv_motor.h
│   │   └── yiyou_motor.h
│   ├── chassis/control/
│   │   ├── arm_control.h
│   │   ├── chassis_control.h
│   │   └── remote_control.h
│   └── iswv/                  # 原 ISWV 公共头，保持 include 层次
├── src/
│   ├── motor/
│   │   ├── bxi_motor.cpp
│   │   ├── bxi_pci_transport.cpp
│   │   ├── iswv_motor.cpp
│   │   ├── yiyou_motor.cpp
│   │   └── canopen/            # 原 ISWV 8 个核心 cpp
│   └── control/
│       ├── arm_control.cpp
│       ├── chassis_control.cpp
│       ├── remote_control.cpp
│       └── main.cpp
├── tests/
│   ├── test_bxi_motor.cpp
│   ├── test_bxi_motor_commands.cpp
│   └── canopen/                # 原 ISWV 6 个测试
├── vendor/bxi_pci/
│   ├── include/bxi_pci_drv.h
│   └── lib/libbxi_pci_drv.a
└── docs/
    ├── dependencies.md
    └── iswv/
        ├── protocol.md
        └── user_manual.pdf
```

| 模块 | 公开头文件 | 主要接口 | 安装后的 CMake 目标 |
| --- | --- | --- | --- |
| BXI 单电机 | `chassis/motor/bxi_motor.h` | `bxi::Motor`、编码/反馈函数 | `chassis::chassis_bxi_motor` |
| BXI PCI | `chassis/motor/bxi_pci_transport.h` | `chassis::BxiPciTransport` | `chassis::chassis_bxi_transport` |
| ISWV 行进轴 | `chassis/motor/iswv_motor.h` | `chassis::IswvDrive` | `chassis::chassis_iswv_motor` |
| 意优 | `chassis/motor/yiyou_motor.h` | `chassis::YiyouMotor` | `chassis::chassis_yiyou_motor` |
| CANopen 核心 | `include/iswv/...` | `iswv::canopen`、`iswv::Axis` 等 | `iswv::canopen` |
| 底盘 | `chassis/control/chassis_control.h` | `chassis::ChassisController` | `chassis::chassis_controller` |
| CAN3 机械臂 | `chassis/control/arm_control.h` | `chassis::ArmController` | `chassis::chassis_arm_control` |
| 遥控器 | `chassis/control/remote_control.h` | `chassis::make_remote_control()` | `chassis::chassis_remote_control` |

迁移旧调用时，使用上表的 `.h` 路径；`EyouMotor` 改为 `YiyouMotor`，`BxiCan3` 改为 `ArmController`，旧 `hold_zero()` 对应现在的 `update()`。

## C++ 调用示例

底盘控制器本身不依赖 ROS 消息或 executor，调用方串行调用即可：

```cpp
#include "chassis/control/chassis_control.h"

auto config = chassis::load_steering_config("steering.yaml");
chassis::ChassisController controller(config);
controller.initialize(keep_running);
controller.calibrate(keep_running);
controller.set_velocity(0.0, 0.0, 0.0);
while (keep_running()) {
  controller.update();
}
controller.stop();
```

CAN3 机械臂可直接和底盘一起使用：

```cpp
#include "chassis/control/arm_control.h"

chassis::ArmController arm;
arm.initialize();
arm.save_zero_positions();
arm.update();
arm.set_target_degrees(1, 1.0);
arm.update();
arm.stop();
```

单独调用 BXI 协议时包含规范头并链接 `chassis::chassis_bxi_motor`；若通过 PCI 板卡发送，再链接 `chassis::chassis_bxi_transport`。离线示例在 `examples/bxi_motor_offline.cpp`，不访问真实硬件。

```cpp
#include "chassis/motor/bxi_motor.h"
#include "iswv/fake_transport.hpp"

iswv::FakeTransport transport;
bxi::Motor motor(transport, 1);
motor.enter_motor_mode();
bxi::Command command{0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
command.ranges = bxi::encoding_ranges(bxi::Model::BXI5014_19);
motor.command(command);
motor.exit_motor_mode();
```

安装后其他工程可按需链接：

```cmake
find_package(chassis REQUIRED)
add_executable(your_main main.cpp)
target_link_libraries(your_main PRIVATE
  chassis::chassis_controller
  chassis::chassis_arm_control)
```

只使用 CANopen 核心时仍可链接导出的 `iswv::canopen`，但源码已随 `chassis` 包构建，不再有独立 `ISWV` 源码目录入口。

## 文档与来源

- 依赖收拢和来源：[dependencies.md](diffbot_chassis/src/chassis/docs/dependencies.md)
- ISWV 协议整理：[docs/iswv/protocol.md](diffbot_chassis/src/chassis/docs/iswv/protocol.md)
- ISWV 用户手册：[docs/iswv/user_manual.pdf](diffbot_chassis/src/chassis/docs/iswv/user_manual.pdf)
- BXI 协议代码基于 Apache-2.0 来源，许可证见 `diffbot_chassis/src/chassis/LICENSE`。

离线测试使用 FakeTransport、参数服务或协议编码断言，不证明实车机械安装、现场波特率、急停、断电时延或力矩阈值正确。
