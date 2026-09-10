# BXI 四舵轮底盘

基于 ROS 2 Humble、BXI PCI 板卡和 ISWV CANopen 驱动的四舵轮底盘工程，包含底盘控制、手柄遥控、转向校准及独立电机诊断工具。

## 目录

| 路径 | 内容 |
| --- | --- |
| `car2/diffbot_chassis/src/chassis/` | ROS 2 底盘包、控制器、配置与测试 |
| `car2/start_*.sh` | 底盘、遥控、校准和诊断启动入口 |
| `car2/bxi_pci_drv/` | BXI PCI 厂商头文件、静态库及测试样例 |
| `car2/libcanfd/` | 厂商 CAN FD 库与样例 |
| `ISWV/` | C++17 CANopen / CiA402 驱动、示例、测试及协议资料 |

`car2` 和 `ISWV` 均直接包含源码，无需初始化 Git 子模块。保持两者同级；底盘构建会自动编译 `ISWV`，无需预先安装该库。厂商 `.a` 文件是构建输入，应随仓库保留；构建产物、日志及本地编辑器缓存由 `.gitignore` 排除。

## 使用了哪些电机

当前包含两类电机驱动，底盘共使用 **4 个转向轴 + 4 个行进轴**；意优电机是独立上装工具，不参与底盘运动控制。

| 电机系列 / 用途 | 协议与驱动 | 当前连接 | 参数来源 |
| --- | --- | --- | --- |
| Kinco（步科）iSWV-AA 立式舵轮：4 个转向轴 | CANopen / CiA402，`iswv::Axis` | BXI CAN0，Node-ID 1、2、3、4 | `steering.yaml`：编码器 65536 count/rev，转向减速比约 9.02527 |
| Kinco（步科）iSWV 立式舵轮：4 个行进轴 | CANopen / CiA402，`chassis::IswvDrive` | BXI CAN1：Node-ID 1、2；CAN2：Node-ID 3、4 | `steering.yaml`：编码器 65536 count/rev，行进减速比 10，轮径 0.130 m |
| 意优 EYOU PHU/RHU 系列：上装电机 | CANopen / CiA402 PV 速度模式，`chassis::EyouMotor` | 试转工具固定 BXI CAN2，Node-ID 由参数指定，菜单默认 1 | 运行时从驱动器读取编码器分辨率及减速比 |

上述系列依据 [ISWV 说明](ISWV/README.md) 和 [意优说明](car2/diffbot_chassis/src/chassis/EYOU.md)。转向配置中的电流参数参考手册型号 `iGMH06809-00627`，仓库没有完整实装电机 BOM，也没有指定意优的具体型号后缀；更换电机时应以铭牌和驱动器参数核对，不能把这些配置值当作所有型号的通用规格。当前底盘不包含 Modbus 升降柱控制。

轮号在行进测试和单转向轴测试中统一如下，实际映射可在 [steering.yaml](car2/diffbot_chassis/src/chassis/config/steering.yaml) 中调整：

| 轮号 | 轮位 | 转向通道 / Node-ID | 行进通道 / Node-ID | 行进方向反转 `drive_inverted` |
| --- | --- | --- | --- | --- |
| 1 | 前左 | CAN0 / 1 | CAN1 / 1 | `false` |
| 2 | 后左 | CAN0 / 2 | CAN1 / 2 | `false` |
| 3 | 后右 | CAN0 / 3 | CAN2 / 3 | `true` |
| 4 | 前右 | CAN0 / 4 | CAN2 / 4 | `true` |

CAN0/1/2 是 BXI 板卡的零起始通道编号，不是 Linux SocketCAN 网卡名。两类驱动都发送经典 CAN 标准帧；使用 CAN FD 板卡不意味着电机协议是 CAN FD。同一通道上的 Node-ID 必须唯一，意优接入 CAN2 时也不能与行进轴 3、4 重号。板卡与该通道上所有电机的波特率必须一致；意优手册默认 1 Mbps，本项目 BXI 接口不会自动配置波特率。

## 构建与测试

需要 Linux、C++17 编译器、CMake、ROS 2 Humble、ament/colcon、`rclcpp`、`geometry_msgs` 和 `yaml-cpp`。启动脚本使用 `/opt/ros/humble/setup.bash`；厂商静态库须与目标平台兼容。

在仓库根目录执行底盘构建及离线测试：

```bash
source /opt/ros/humble/setup.bash
cd car2/diffbot_chassis
colcon build --symlink-install --packages-select chassis
colcon test --packages-select chassis
colcon test-result --verbose
```

单独验证 ISWV 核心库无需 ROS 或实车，在仓库根目录执行：

```bash
cmake -S ISWV -B ISWV/build -DCMAKE_BUILD_TYPE=Debug
cmake --build ISWV/build -j2
ctest --test-dir ISWV/build --output-on-failure
```

默认底盘路径已指向仓库内的 `ISWV`。使用外部驱动源码时可给 `colcon build` 增加 `--cmake-args -DISWV_SOURCE_DIR=/绝对路径/ISWV`。ISWV 的 Zqwl 和 ROS 2 传输扩展默认关闭，启用方式见其独立文档。

## 实车入口

以下启动命令在 `car2/` 下执行。底盘启动会搜索机械限位并转动车轮，需准备机械限位和硬件急停；同一时间只运行一个占用 BXI 板卡的程序。

| 命令 | 用途 |
| --- | --- |
| `sudo -E bash start_remote_control.sh` | 校准后由手柄控制底盘 |
| `sudo -E bash start_chassis.sh` | 仅启动底盘，默认校准后停车等待指令 |
| `sudo -E bash start_steering_calibration.sh` | 单独校准并保持转向中位 |
| `sudo -E bash start_drive_wheel_test.sh 3 0.5` | 以 0.5 输出轴 RPM 测试后右行进轮 |
| `sudo -E bash start_motor_power_hold.sh` | 保持共享电机电源，不使能或转动电机 |
| `sudo ./inspect_joystick.py` | 只读取手柄事件 |

配置文件为 `car2/diffbot_chassis/src/chassis/config/steering.yaml`，可用 `CHASSIS_CONFIG` 指定其他可写配置；实车校准会更新中位数据。主节点订阅 `/cmd_vel_car`，读取 `linear.x`、`linear.y`（m/s）和 `angular.z`（rad/s）。

## 单独使用电机

先完成上面的 `chassis` 构建。以下各小节均在 **`car2/` 目录**执行；从仓库根目录进入并加载环境：

```bash
cd car2
source /opt/ros/humble/setup.bash
source diffbot_chassis/install/setup.bash
```

这些工具可以独立运行，无需启动底盘节点或遥控节点。先停止其他占用 BXI 的程序（包括 `motor_power_hold`），固定并架空待测机构、准备硬件急停。工具使用共享电机电源，上电后等待约 4 秒；“单轴测试”指只向所选轴发送使能和运动指令，供电仍可能覆盖其他电机。退出时会尝试停机、失能并关闭共享电源。

### 1. ISWV 行进电机：选择一个轮子持续试转

```bash
# 交互选择轮号与速度，速度默认 0.5 输出轴 RPM
sudo -E bash start_drive_wheel_test.sh

# 后右行进轮（CAN2 / Node-ID 3），底盘前进方向 0.5 输出轴 RPM
sudo -E bash start_drive_wheel_test.sh 3 0.5
```

参数为 `[轮号 1..4] [输出轴 RPM]`。程序只打开所选轮的 CAN 通道、只使能该行进轴，**不执行转向校准**，持续打印速度反馈，按 Ctrl+C 停机。RPM 必须大于 0 且不超过 `drive_max_output_rpm`（当前为 105）；此工具不接受负数。方向由 `drive_inverted` 自动换算，所以后右、前右轮不需要手动传负号。上限是软件配置值，首次测试使用小转速。

修改接线或传动后，先核对 `drive_can_buses`、`drive_node_ids`、`drive_inverted`、`drive_encoder_resolution` 和 `drive_gear_ratio`。脚本支持用 `CHASSIS_CONFIG` 指定替代 YAML。直接调用同一个程序的等效命令为：

```bash
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/drive_wheel_test \
  ./diffbot_chassis/src/chassis/config/steering.yaml 3 0.5
```

### 2. ISWV 转向电机：单轴小角度点动

独立程序 `steering_direction_test` 只点动所选转向轴，不搜索机械限位，也不自动回中。它要求 YAML 已有有效校准（`steering_calibrated: true` 和对应 `zero_offset_inc`），且当前编码器位置距保存中位不超过 10°。仓库模板默认未校准，不能仅手动改为 `true` 来跳过标定。

需要先标定时运行以下命令；**这一步会依次校准全部四个转向轴并保持中位**，不是单电机操作。校准完成后按 Ctrl+C 退出，再运行点动工具：

```bash
sudo -E bash start_steering_calibration.sh
```

确认保存中位在重新上电后仍有效，再测试单轴：

```bash
# 前左转向轴，从当前位置正向点动 2°
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/steering_direction_test \
  ./diffbot_chassis/src/chassis/config/steering.yaml 1 2

# 后右转向轴，从当前位置反向点动 2°
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/steering_direction_test \
  ./diffbot_chassis/src/chassis/config/steering.yaml 3 -2
```

参数为 `CONFIG WHEEL DELTA_DEG`；角度带符号、非零且绝对值不超过 3°，方向沿用转向 `inverted` 配置。点动速度为输出轴 0.2 RPM，到达目标后停机、失能并断电，不自动返回起点，不改写配置。位置检查不通过时应重新确认/校准中位；裸电机若没有有效校准，需按下文 API 接入。

`start_steering_direction_test.sh` 是另一套方向确认流程：先两两校准四轴，再逐轮以 2 输出轴 RPM 点动 +50° 并记录人工观察。它不接受轮号参数，不能替代上述单轴小角度命令，详见 [方向确认说明](car2/diffbot_chassis/README.md)。

### 3. 意优上装电机：读取信息、定时或持续试转

将目标意优电机接到 BXI CAN2，确认 Node-ID、波特率、STO 和抱闸条件后，先读取信息：

```bash
sudo -E bash start_motor_info.sh 1
```

菜单选择 `1`（默认）只读取信息，选择 `2` 则按输入的输出轴 RPM 持续试转。两种功能都会上电，退出时关闭共享电源；读取信息不发送使能、速度、模式切换或清故障指令。日志保存在 `car2/logs/motor_info_*.log`，最近一次为 `car2/logs/latest_motor_info.log`（路径相对仓库根目录）。

也可跳过菜单直接调用；以下命令需逐条运行：

```bash
# CAN2 上 Node-ID 1：读取状态、故障、模式、编码器、减速比和遥测
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/eyou_motor_info 1

# 输出轴正向 1 RPM，运行 5 秒后停机
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/eyou_motor_test 1 1 5

# 输出轴反向 1 RPM，运行 5 秒，加减速度均为 10 RPM/s
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/eyou_motor_test 1 -1 5 10 10

# 持续运行，Ctrl+C 停机
sudo -E ./diffbot_chassis/install/chassis/lib/chassis/eyou_motor_test 1 1 --continuous
```

试转参数为 `NODE_ID RPM [SECONDS=5|--continuous] [ACCEL_RPM_S=10] [DECEL_RPM_S=10]`。节点范围 1..127，输出轴速度范围 ±30 RPM，定时运行范围 1–60 秒；负 RPM 反转。这些是工具限制，不是电机额定规格。`eyou_motor_info [NODE_ID=1] [CAN_BUS=2]` 可以另传读取通道；试转工具和菜单固定使用 CAN2。

工具读取编码器和减速比后换算输出轴 RPM，并切换到 CANopen PV 速度模式再使能；底层 `0x60FF/0x606C` 的单位是 pulse/s，不能直接套用 ISWV 的速度原始值。程序不会自动清故障或解除 STO。意优工具尚未配置设备侧断线看门狗，通信断开或进程被强杀时无法保证软件停机送达，持续运行前需确认设备失联动作与硬件急停。更多控制权、单位换算和遥测说明见 [EYOU.md](car2/diffbot_chassis/src/chassis/EYOU.md)。

### 4. 在自己的 C++ 程序中控制单个电机

单电机控制不必实例化完整的 `ChassisController`。现有接口与可复用的完整程序如下：

| 对象 | 使用顺序 | 完整源码参考 |
| --- | --- | --- |
| ISWV 行进轴 `chassis::IswvDrive` | 根据目标节点创建 `AxisConfiguration::manual_travel(node_id)`，填入编码器与减速比；`enable_rpm(accel, decel)` → `set_velocity_rpm(rpm)` → 循环 `actual_velocity_rpm()` → `stop()` | [drive_wheel_test.cpp](car2/diffbot_chassis/src/chassis/src/drive_wheel_test.cpp) |
| ISWV 转向轴 `iswv::Axis` | `AxisConfiguration::manual_steering(node_id)` → 配置位置模式和方向 → 校验当前位置、目标与保护参数 → NMT start / `drive().enable()` → `move_absolute_inc()` → 监测状态 → `quick_stop()` / `disable()` | [steering_direction_test.cpp](car2/diffbot_chassis/src/chassis/src/steering_direction_test.cpp) |
| 意优 `chassis::EyouMotor` | `EyouMotor(master, node_id)` → `configure_rpm_units()` → `enable_rpm(accel, decel)` → `set_velocity_rpm(rpm)` → 循环读取反馈 → `stop()` | [eyou_motor_test.cpp](car2/diffbot_chassis/src/chassis/src/eyou_motor_test.cpp) |

这些是调用顺序索引，完整初始化、状态检查和异常停机应参照源码保留。`IswvDrive` / `EyouMotor` 的 RPM 接口使用输出轴单位；`Axis::configure_position_mode()` 的速度使用电机轴 RPM，`move_absolute_inc()` 使用绝对编码器计数。`IswvDrive` 本身不替调用者应用底盘的 `drive_inverted`，行进测试程序在下发速度前完成该方向换算。

BXI 接入使用 `BxiPciTransport(bus)` 和 `CanopenMaster`，由应用管理共享电源和上电等待。每条总线复用 transport/master，各电机实例通过 Node-ID 区分；master 与 transport 必须比电机实例存活更久，电机 API 由同一线程串行调用。上述工具是普通 C++ 可执行程序，现有 `chassis` 包仍需通过 ROS 2/ament 构建；底盘包内分别链接 `chassis_iswv_drive`、`chassis_eyou_motor` 和 `chassis_bxi_transport`。意优库当前没有作为独立安装库导出，复用方式可参考 [CMakeLists.txt](car2/diffbot_chassis/src/chassis/CMakeLists.txt)。

只使用 ISWV 核心库时可完全脱离 ROS：按前面的独立 CMake 命令构建，安装后用 `find_package(iswv_canopen CONFIG REQUIRED)` 和 `iswv::canopen` 链接，再接入自己的 `ICanTransport`。默认 [basic.cpp](ISWV/examples/basic.cpp) 使用 `FakeTransport`，不会驱动实物；另有 [Zqwl 慢速试转示例](ISWV/examples/zqwl_slow_rotate.cpp)，需要单独启用 Zqwl 适配器，固定通道 0 / 500 kbit/s / Node-ID 1，以**电机轴 30 RPM**运行最多 5 秒，不能作为 BXI 启动程序直接使用。构建与运行参数见 [ISWV README](ISWV/README.md)。

## 详细说明

- [接线、轮位、控制流程与参数](car2/readme.md)
- [底盘控制实现](car2/diffbot_chassis/README.md)
- [ISWV 驱动与示例](ISWV/README.md)
- [ISWV CANopen 协议](ISWV/iSWV_CANopen_通讯协议总结.md)
- [BXI PCI 驱动样例](car2/bxi_pci_drv/README.md)
- [意优上装电机工具](car2/diffbot_chassis/src/chassis/EYOU.md)

离线测试使用模拟通信或参数检查，不证明实车接线、机械限位、力矩参数和急停功能有效。
