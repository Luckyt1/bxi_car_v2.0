# 底盘与机械臂控制

ROS 2 Humble / C++17，包名仍为 `chassis`。主程序保留原有 ROS 话题、参数、启动校准和故障停机流程。

## 目录

```text
CMakeLists.txt            # 根目录统一构建入口
start.sh                  # 日常启动菜单：意优设零、仅底盘、整机
start_yiyou_jog.sh         # 意优点动 / 设置零位快捷入口
start_chassis_only.sh      # 底盘手柄启动快捷入口
start_chassis_arm.sh       # 整机手柄启动快捷入口
src/
  thirdparty/
    bxi/                  # board.h/.cpp：BXI 板卡、CAN 收发、共享电机电源
      vendor/             # 厂家头文件和静态库
    motor/
      bxi/                # communication.h/.cpp + motor.h/.cpp：MIT 电机
      yiyou/              # communication.h/.cpp + motor.h/.cpp：EtherCAT 电机
        detail/           # CiA402 辅助逻辑
        vendor/soem/      # 原有 SOEM 源码
      iswv/               # communication.h/.cpp + motor.h/.cpp：CANopen 电机
        protocol/         # PDO、Axis、对象字典等协议内部实现
    arm/                  # kinematics.h/.cpp：正逆解；control.h/.cpp：机械臂控制
    chassis/              # control.h/.cpp：底盘控制；转向校准、限位和运动学
  example/
    main.cpp              # ROS 主入口，调用各模块
    remote_control.h/.cpp # 手柄与 ROS 接口
    arm_parameters.h      # 机械臂 ROS 参数适配
  config/                 # 原有机械尺寸、电机方向和站位配置
  cmake/                  # 安装包导出模板
test/
  debug.py               # 唯一面向使用者的调试菜单
  tools/                 # 菜单调用的 C++ 后端和 Python 辅助模块
  unit/                  # 离线回归与模拟通信测试
docs/                    # 协议与机械臂说明
```

`src/thirdparty` 按本项目约定包含自有模块和外部依赖。`bxi/board` 与 `motor/bxi` 分别控制板卡和电机。通信层负责收发、编解码或总线会话，调用层提供电机操作；底盘调参在确认电机失能后由底盘层回调执行。

## 构建与测试

在项目根目录直接执行 `colcon build`，产物使用标准的 `build/`、`install/`、`log/`。
启动脚本、调试工具和整车服务都使用这里生成的程序。

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

工控机 `/home/bxi/tang/car2` 编译成功后，用
`sudo systemctl restart ros_elf_launch.service` 加载新程序。只改 YAML 时无需编译。
需要自定义构建目录时，仍可设置 `CHASSIS_BUILD_DIR` 和 `CHASSIS_INSTALL_DIR`。
`src/cmake/` 中的 `.cmake.in` 是安装包导出模板，属于构建源码，需要保留。

## 日常启动脚本

参考 car2 的操作方式，在源码根目录执行：

```bash
sudo -E bash start.sh
```

菜单选择 `1` 意优点动/设置零位、`2` 底盘单独启动、`3` 整机启动、`0` 退出。
也可以直接使用与 car2 同名的快捷脚本：

```bash
# 意优前三轴点动、设置零位；按实际网口和站位修改。
sudo -E bash start_yiyou_jog.sh --interface enp86s0 --slaves 1,2,3
# 仅底盘：等待手柄启动键，不初始化机械臂。
sudo -E bash start_chassis_only.sh
# 整机：等待手柄启动键，按配置机械臂回零后校准底盘。
sudo -E bash start_chassis_arm.sh
```

意优界面中，左右键选轴，上下键每次点动 1°，`z` 将当前轴当前位置写成 EEPROM 零位，
`q` 退出。只有按 `z` 才设零并保存该轴全部可持久化参数；记录写入项目根目录的
`yiyou_ethercat_zero_calibration.json`。这与启动时返回已保存零位是两个操作。
菜单默认使用网口 `enp86s0`、站位 `1,2,3`，可以用 `YIYOU_INTERFACE`、`YIYOU_SLAVES`
修改；快捷脚本的 `--interface`、`--slaves` 参数优先。

底盘和整机默认松开后按 **button14** 开始，**button11** 急停，`Ctrl-C` 退出。
整机调用 `robot_control`，底盘调用 `chassis_only_remote`；其余控制参数沿用当前程序和
`src/config` 的配置。可用 `CHASSIS_CONFIG`、`YIYOU_CONFIG` 覆盖配置文件，
`JOYSTICK_DEVICE` 覆盖手柄设备，末尾追加 ROS 参数作临时覆盖：

```bash
sudo -E bash start_chassis_arm.sh --ros-args -p yiyou_arm_home_on_start:=false
# 以下只预览命令，不加载 ROS 环境或启动设备，也不需要 sudo。
bash start.sh --dry-run
bash start_yiyou_jog.sh --dry-run
bash start_chassis_only.sh --dry-run
bash start_chassis_arm.sh --dry-run
```

脚本可从任意工作目录调用，按项目根目录定位 `build/` 和 `install/`，支持
`CHASSIS_BUILD_DIR` / `CHASSIS_INSTALL_DIR` 覆盖目录。
底盘/整机脚本自动加载 ROS 环境；脚本不自动提权或启用网卡。
运行前需有设备访问权限、EtherCAT 网口已启用，并避免多个程序同时控制同一设备。
`--help` 查看脚本参数。日常启动脚本保留在源码根目录，调试菜单仍使用下面的入口。

## 底盘日常调参

修改 `src/config/steering.yaml` 后重启程序生效，底盘和遥控器共同读取它。

| 调节内容 | 参数 | 当前值 |
| --- | --- | --- |
| 四轮输出轴最高转速 | `drive_max_output_rpm` | 105 RPM |
| 四轮轮缘加速度 / 减速度 | `drive_acceleration_m_s2` / `drive_deceleration_m_s2` | 2 / 2.4 m/s² |
| 舵轮转向速度 / 整车 yaw 变化率 | `steering_rate_limit_rad_s` / `command_yaw_acceleration_rad_s2` | 4 rad/s / 0.8 rad/s² |
| 整车 x / y / yaw 速度上限 | `max_velocity_x_m_s` / `max_velocity_y_m_s` / `max_velocity_yaw_rad_s` | 0 / 0 / 0 |
| 遥控满杆 x / y / yaw 速度 | `scale_linear` / `scale_lateral` / `scale_angular` | 0.6 m/s / 0.6 m/s / 0.4 rad/s |
| 遥控 x/y 正向 / 负向指令变化率 | `max_accel` / `max_decel` | 1.2 / 2.4 m/s² |

整车上限为 `0` 时保留原有轮速限制；设为正数后，超限指令的 x/y/yaw 同比例缩小。
这些上限适用于 `/cmd_vel_car`、遥控器和控制器直接调用，自动前进也受 x 上限约束。
遥控变化率沿用原算法，按指令差值正负选择，不按车速绝对值的加速/减速分类。
原配置中的逐轮设置实际将轮速限制为 105 RPM；整理后直接显示该有效值。

`hardware_config_file` 指向同目录的 `steering_hardware.yaml`，保存 CAN、机械尺寸、
方向、校准和驱动器参数。驱动器 `null` 参数的自动回填留在所属文件，日常文件不会
因此展开。日常文件同名参数优先；旧单文件 YAML 和显式逐轮 `modules` 仍可加载，
旧配置的逐轮字段仍优先于全局字段。复制配置时同时复制这两个文件。
通过无节点限定的 `-p steering_config_file:=/path/steering.yaml` 同时选择底盘和遥控配置；
遥控的显式 ROS 参数（例如 `-p scale_linear:=0.3`）优先于 YAML 中的默认值。

## 统一调试入口

```bash
python3 test/debug.py
# 安装后也可以：
ros2 run chassis debug.py
```

- `1` 电机调试，再选择 `1 BXI / 2 意优 / 3 ISWV`。
- `2` 机械臂离线正逆解，使用 `src/config/arm_kinematics.json`，不连接电机。
- `3` 底盘调试，选择直接启动或遥控启动。
- `0` 退出。

预览菜单将运行的命令：

```bash
python3 test/debug.py --dry-run
python3 test/debug.py --mode yiyou --interface enp86s0 --slaves 1,2,3 --dry-run
python3 test/debug.py --mode iswv --bus 1 --node 1 --dry-run
python3 test/debug.py --mode arm
```

电机、底盘模式会实际打开设备；`--dry-run` 和 `--help` 不打开设备。意优网口通过 `--interface` 指定，站位通过 `--slaves` 指定。调试工具默认机械参数需与实机一致，使用 `--help` 查看接口单位与可选项。不要同时启动多个控制同一设备的程序。

## 主控制程序

只有一份 `src/example/main.cpp`，保留原来的可执行名称供现有 ROS 调用继续使用：

`main()` 负责模式选择、ROS 初始化和程序退出；`run_control()` 管理遥控等待和节点调度。
节点内部按 `read_startup_config()`、`initialize_hardware()`、`setup_ros_interfaces()` 顺序启动，
运行期间由 `update_control()` 统一处理机械臂、底盘和反馈。

| 程序 | 启动方式 |
| --- | --- |
| `chassis` | 底盘与机械臂直接启动 |
| `key_control` / `robot_control` | 等待手柄启动键，再启动底盘与机械臂 |
| `chassis_only` | 只启动底盘 |
| `chassis_only_remote` | 手柄启动，只控制底盘 |

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run chassis robot_control
```

程序默认读取本项目 `src/config/steering.yaml`；含机械臂的模式还会读取
`src/config/yiyou_ethercat.yaml`。查找不依赖终端当前目录，在本项目中修改配置后重启程序即可生效。
源码配置目录不存在时，使用可执行程序所在安装前缀的 `share/chassis/config/`。
手柄模式仍等待 14 号启动键，再按配置进行机械臂回零和底盘校准。

临时使用其他配置或参数时，仍可覆盖默认值：

```bash
ros2 run chassis robot_control --ros-args \
  -p steering_config_file:=/absolute/path/steering.yaml \
  --params-file /absolute/path/yiyou_ethercat.yaml \
  -p yiyou_arm_home_on_start:=false
```

底盘接收 `/cmd_vel_car`；机械臂使用 `yiyou_arm_target=[x,z,pitch]` 和 `/arm_pose`。
底盘默认在 300 ms 没有新速度指令后下发零速，每 20 ms 控制周期检查一次。
可在启动参数中用 `-p chassis:command_timeout_sec:=0.3` 设置正数超时值；不要用单次非零消息保持行驶。

Battle Dragon 蓝牙手柄（20d6:400d）的链路保护每 100 ms 主动读取一次 HID 输入报告，
500 ms 没有成功应答即禁止运动，在下一个 50 ms 遥控周期持续发布零速度。
查询在独立线程执行，蓝牙读操作卡住不会阻塞遥控停车；摇杆保持不动不要求产生新事件。
断连、设备拔除、读取失败或权限不足均停止输出；恢复后重新读取轴状态，必须先回中再操作。
目前主动探测只验证过该型号的蓝牙连接：其他设备或 USB 接收器连接会保持零速并记录原因，
需先验证对应接收器的无线连接检测协议才能开放控制。服务以 root 运行可读取 hidraw；
手动运行时也需有对应 hidraw 设备的读取权限。

以上时间是零速度指令的检测/下发时限，实际停车还需要驱动器减速时间。
两层保护依赖控制进程正常调度，不能替代硬件急停。机械臂方向配置保持原值。

## 模块调用示例

各模块的 `example.cpp` 提供可复制的调用片段，顶部注明对象创建和通信前提。
ISWV 与意优示例直接调用 motor 的使能、目标设置和停止方法；单位写在对应调用旁，
失败时尝试停车并保留原始异常。意优的位置、电流调用列在同一文件中，按需选用。
BXI 电机示例是可编辑的 `main()` 框架；三种电机示例均不加入默认编译。

可用 `cmake --build build/chassis --target iswv_motor_example yiyou_motor_example bxi_motor_example`
检查这些示例的编译；这条命令不会运行电机，编辑器也会使用对应的头文件路径。

| 模块 | 示例 | 展示内容 |
| --- | --- | --- |
| BXI 板卡 | [bxi/example.cpp](src/thirdparty/bxi/example.cpp) | 打开通道、共享电源、发帧、接收回调 |
| BXI 电机 | [motor/bxi/example.cpp](src/thirdparty/motor/bxi/example.cpp) | MIT 使能、位置/速度/力矩调用、退出模式 |
| ISWV 电机 | [motor/iswv/example.cpp](src/thirdparty/motor/iswv/example.cpp) | 速度模式初始化、速度目标、反馈、停止 |
| 意优电机 | [motor/yiyou/example.cpp](src/thirdparty/motor/yiyou/example.cpp) | 速度、位置、电流模式及反馈、停止 |
| 机械臂运动学 | [arm/example.cpp](src/thirdparty/arm/example.cpp) | 正解、逆解、结果状态与单位 |
| 底盘 | [chassis/example.cpp](src/thirdparty/chassis/example.cpp) | 舵轮解算、配置、启动、速度目标、周期更新、停车 |

电机和通信对象由调用方长期持有，初始化与周期目标更新分开。
各文件顶部注明所需链接库；错误处理保留原接口的 Result 或异常语义。
机械臂示例只依赖 chassis_arm_kinematics。

| 头文件 | 调用方式与单位 |
| --- | --- |
| `bxi/board.h` | `chassis::BxiPciTransport(bus)`；`set_motor_power(true/false)` |
| `motor/bxi/motor.h` | `bxi::Motor`：`set_position()`、`set_velocity()`、`set_torque()` 均须显式传入 `bxi::Model`；MIT 参数为 rad、rad/s、Nm 和 Kp/Kd |
| `motor/iswv/bxi_transport.h` | `iswv::BxiTransport(board)`：将 BXI 板卡接入 ISWV CANopen 主站 |
| `motor/yiyou/motor.h` | `chassis::YiyouMotor`：`enable_rpm()`、`enable_position()`、`enable_cst()`、目标输入、`stop()`；分别为输出轴 RPM、编码器脉冲、额定电流千分比 |
| `motor/iswv/motor.h` | `chassis::IswvDrive`：`enable_rpm()`、`set_velocity_rpm()`、`stop()`；转向位置和其他支持的模式复用 `iswv::Axis` / `axis.drive()` |

BXI 的接口提供 MIT 模式，不虚构 CANopen 式模式切换；ISWV 与意优保留各自协议支持的模式。`stop()` 在意优和 ISWV 中包含停车及失能。构造板卡/主站、上电、校准和退出顺序由应用管理，正逆解可以独立使用 `arm::Kinematics`。

安装后 CMake 使用 `find_package(chassis REQUIRED)`，链接例如 `chassis::chassis_bxi_motor`、`chassis::chassis_arm_kinematics`；CANopen 仍导出 `iswv::canopen`。供应商来源与许可范围见 [docs/dependencies.md](docs/dependencies.md)。

BXI 板卡和电机使用自己的 `bxi::Result`、`bxi::CanFrame` 和 `bxi::ICanTransport`，
其源码及两个库均不依赖 ISWV。ISWV 保留自己的同类接口；使用 BXI 板卡驱动 ISWV 电机时，
额外链接 `chassis::chassis_iswv_bxi_transport`，并将
`std::make_shared<iswv::BxiTransport>(board)` 传给 `iswv::canopen::CanopenMaster`。
这个适配器负责类型转换，并与主站一起持有板卡；共享电源仍通过 `board->set_motor_power()` 管理。

## 作为示例工程移植

本项目可作为板卡、电机、机械臂或底盘模块的移植参考。仓库内提供
[port-robot-modules Skill](skills/port-robot-modules/SKILL.md)，包含模块依赖、通信与调用层边界、配置迁移和离线验证方法。
Skill 不打包驱动源码，使用时需要能访问本项目；可以只提取所需模块，不必复制整个工程。

将 `skills/port-robot-modules` 整个目录放到 Codex 的技能目录
`${CODEX_HOME:-$HOME/.codex}/skills/` 后，在目标项目的新会话中使用，例如：

```text
使用 $port-robot-modules，参考 /path/to/car2.1，
把机械臂正逆解移植到当前项目，使用 C++17，不使用 ROS 2。
```

```text
使用 $port-robot-modules，参考 /path/to/car2.1，
把 ISWV 电机模块和 BXI 板卡接入当前 ROS 2 项目，
分开通信层与调用层，提供调用示例并完成离线构建验证。
```

完整 ROS 控制可以复用应用层；纯 C++ 移植需要给选定模块单独接入 CMake，
因为参考工程根 CMake 仍依赖 ROS/ament。新设备的网卡、ID、方向、零偏、限位和启动回零须另行配置；移植验证默认不操作硬件。
