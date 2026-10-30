# BXI 小车控制工程

ROS 2 `chassis` 包，包含 BXI 板卡与 CAN 通信、ISWV 舵轮底盘、意优 EtherCAT 机械臂、运动学解算、遥控器和 App 接口。此目录可独立编译，包含离线测试与 OTA 制包工具；编译产物、日志和实机状态记录不随源码发布。

## 目录

| 路径 | 用途 |
| --- | --- |
| `src/example/` | ROS 主程序、遥控器与断连保护 |
| `src/thirdparty/` | 板卡、电机、机械臂、底盘模块及必要供应商依赖 |
| `src/config/` | 底盘硬件、机械臂及运动学配置 |
| `tools/` | 意优点动/设零工具、机械臂力矩报告 |
| `test/` | 模拟通信回归、离线测试与数字调试菜单 |
| `scripts/` | App 启动、配置迁移、安装检查与 OTA 制包 |
| `start*.sh` | 日常启动入口 |
| `deploy/` | systemd 部署示例 |
| `docs/` | 控制接口、依赖来源和许可说明 |

## 编译

发布构建环境为 Ubuntu 22.04 x86_64、ROS 2 Humble、C++17、CMake 和 Python 3。依赖 `rclcpp`、`rcl_interfaces`、`rclpy`、`geometry_msgs`、`std_msgs`、`yaml_cpp_vendor` / yaml-cpp；力矩图表使用 Python matplotlib。所需 BXI PCI 静态库和 Linux SOEM 源码已随工程保留，构建时无需下载它们。

在已配置 ROS 2 和 rosdep 的环境中，从本目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --base-paths . --packages-select chassis --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select chassis
colcon test-result --verbose
source install/setup.bash
```

也可直接使用 CMake，测试只使用模拟通信、测试 ROS 节点或程序 `--help`，不会启动真实电机：

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

使用的 BXI PCI 设备及系统驱动需在目标机器上可用；本仓库只包含用户态接口和供应商静态库。更换 CPU 架构时需使用匹配架构的供应商库。

## App 与 OTA

现有 App 通过支持 `car2` 后端的 `bxi_rc_ros2` 网关启动本包。机器人上的
`/etc/bxi/control_backend.env` 选择 `BXI_CONTROL_BACKEND=car2`；
`BXI_CAR2_ENABLE_ARM=0` 仅启动底盘，`1` 启动底盘及机械臂。
App 入口继承网关的 ROS domain/DDS 配置，收到启动操作后才初始化硬件和校准。
App 与手柄主控不能同时占用设备；启用 App 管理后应停用旧手柄自启动服务。

OTA 程序安装到 `/opt/bxi/car2`，每台机器的配置、标定和第四轴位置记忆保存在
`/var/lib/bxi/car2`，升级保留已有内容。网关默认等待启动完成 900 秒；启动期间可在 App 停止。
升级时控制程序必须停止，安装程序不会自动启动电机。

仓库根目录的 GitHub Actions 工作流负责 `main` 分支的编译、离线测试和发布；
`scripts/package_ota.py` 生成 `car2_vYYMMDD-N.tar.gz` 及同名 `.sha256` 文件，
x86_64 发布目录为 BXI 下载站的 `/ELF3/car2`。
其他机器人需要兼容硬件、ROS Humble 和已启用 `car2` 后端的网关，才能使用该升级项。
具体接口与首次部署见 [App 接入说明](docs/app_integration.md) 和 [OTA 说明](docs/car2_ota.md)。

## 转向校准反馈

四轴校准循环读取驱动器持续上报的 TPDO1/2/3，分别检查状态/位置、电流和故障字段。
配置完成后保持零目标，最多等待 250 ms 收齐新反馈；运动期间任一帧年龄达到 100 ms，
或位置与电流帧接收时间相差超过 20 ms，均中止校准。
故障、超速、力矩上限、硬限位间距和各阶段超时保护保留。
主要校准阶段各有 230 秒上限，阶段切换后重新计时；这些软件检查不承诺固定的物理停机延迟。

## 配置与启动

先核对 `src/config/steering.yaml`、其引用的 `steering_hardware.yaml` 和 `yiyou_ethercat.yaml`。其中网口、站位、轴方向、几何尺寸和零位参数来自现有小车，应按目标机器配置。

默认 EtherCAT 网口为 `enp86s0`，手柄为 `/dev/input/js0`。EtherCAT 网口需处于 UP 状态，进程需要访问 BXI 设备及原始网口的权限。手柄可用 `JOYSTICK_DEVICE` 覆盖。整机网口在机械臂 YAML 中设置，`YIYOU_CONFIG` 可指定该 YAML；`YIYOU_INTERFACE` 仅设置点动/设零工具的网口选择默认值。完整说明见 `bash start.sh --help`。

```bash
bash start.sh robot --dry-run  # 预览命令，不访问硬件
sudo -E bash start.sh          # 菜单：设零、底盘、整机
sudo -E bash start_chassis_only.sh
sudo -E bash start_chassis_arm.sh
```

底盘/整机启动后，松开再按 `button14` 开始，`button11` 急停，`Ctrl-C` 退出。整机默认先让前三轴回到配置中的已保存零位，再校准底盘；启动不会重写 EEPROM 零点。第四轴保持实际启动位置。

启动脚本依次查找 `build/chassis/`、`build/`、`install/chassis/lib/chassis/`、`install/lib/chassis/` 下的程序。自定义构建目录可设置 `CHASSIS_BUILD_DIR`、`CHASSIS_INSTALL_DIR`。移动源码目录后需重新编译，以更新程序内默认配置和状态文件路径。

## 意优点动与第四轴位置记录

```bash
sudo -E bash start_yiyou_jog.sh --interface enp86s0 --slaves 4
```

左右切换站位，上下每次点动 100° 输出轴角度，`z` 将所选轴的当前位置保存为 EEPROM 零点，`q` 退出。点动工具默认 20 RPM，可用 `--speed-rpm` 调整；它与遥控运行的速度配置独立。**只在已确认机械位置就是所需零位时按 `z`**，此操作也会保存该轴其他持久化参数。

第四轴遥控采用 PP 位置控制：`button0` / `button4` 正反移动，松手减速停止并保持；速度 12 RPM（72°/秒），加减速度 24 RPM/s，软件目标范围为累计输出角 0°～1200°。软件限位不能保证实际运动无超调；PP 出力由驱动器位置环及其原有电流/力矩限制决定。

源码启动时，第四轴多圈角度默认保存在 `state/yiyou_motor4_position.yaml`；App 部署使用 `/var/lib/bxi/car2/state/yiyou_motor4_position.yaml`。该实机记录不随代码发布，新机器/新目录缺少可信记录时会禁止第四轴移动；应在已知机械零位通过上述工具显式设零，创建对应记录。调试工具与遥控程序必须共用同一记录；单机同时只运行一个控制程序。

恢复累计角度要求断电期间轴没有移动，且运动均由本工程的受控入口执行。记录丢失、损坏、身份不符或无法唯一恢复位置时程序拒绝运动。不要通过复制其他机器记录或删除记录来解除保护。详细接口与边界见 [机械臂控制说明](docs/arm_cartesian_control.md)。

## 日志与部署

CAN 诊断写入 `log/can/`，机械臂力矩数据和图表写入 `log/arm_torque/`。`tools/arm_torque_report.py` 是运行日志的出图工具，已保留。

`deploy/ros_elf_launch.service` 是可选自启动示例，使用 `/opt/bxi_car/car2`、`enp86s0` 和 `/dev/input/js0`；安装前应修改为实际部署路径和设备。默认启动后等待手柄启动键。源码复制本身不会安装或启用服务。

`.gitignore` 排除构建目录、日志、多圈位置状态、零位标定 JSON、缓存和本地开发配置。

## 依赖与许可

本项目许可见 [LICENSE](LICENSE)。第三方组件沿用各自许可和来源说明，供应商静态库是必需依赖。详见 [依赖来源](docs/dependencies.md) 和 [机械臂解算来源](docs/arm_kinematics_origin.md)。
