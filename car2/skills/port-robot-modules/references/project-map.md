# 参考项目模块地图

所有源码路径相对参考项目根目录。目录名称与 target 是当前实现的索引，移植时以实际头文件和 CMake 为准。

## 目录与构建依赖

```text
src/
  thirdparty/
    bxi/                 板卡电源和 CAN transport，含厂家头文件及静态库
    motor/
      bxi/               MIT 通信与调用层
      iswv/              CANopen 通信与调用层，protocol/ 保存协议实现
      yiyou/             EtherCAT 通信与调用层，vendor/soem/ 保存本地 SOEM
    arm/                 kinematics 正逆解、control 硬件控制
    chassis/             底盘控制、解算和配置辅助头
  example/               ROS 主控与遥控接入
  config/                底盘、机械臂和电机配置
  cmake/                 安装包配置模板
test/
  debug.py               统一调试菜单
  tools/                 调试后端
  unit/                  离线回归与接口检查
```

下表列出当前 CMake 的直接依赖，选定 target 后继续跟随其依赖，直到源码、头文件和链接库齐全。表中模块均位于 `src/thirdparty/`，ROS 应用除外。

| target | 主要源码 | 直接依赖 |
| --- | --- | --- |
| `iswv_canopen` / `iswv::canopen` | `motor/iswv/communication.cpp`；`protocol/src/{fake_transport,pdo,drive,axis,faults,module,steering_layout}.cpp` | `protocol/include`、Threads |
| `chassis_bxi_transport` | `bxi/board.cpp` | `iswv::canopen`、`bxi/vendor/include`、`bxi/vendor/lib/libbxi_pci_drv.a` |
| `chassis_bxi_motor` | `motor/bxi/{communication,motor}.cpp` | `iswv::canopen` |
| `chassis_iswv_motor` | `motor/iswv/motor.cpp` | `iswv::canopen` |
| `chassis_soem` | `motor/yiyou/vendor/soem/soem/*.c`、Linux osal/oshw 源文件 | Threads、`rt`；详见根 CMake |
| `chassis_ethercat` | `motor/yiyou/communication.cpp` | `iswv::canopen`、`chassis_soem` |
| `chassis_yiyou_motor` | `motor/yiyou/motor.cpp` | `chassis_ethercat`，还需本目录 `detail/drive.h` |
| `chassis_arm_kinematics` | `arm/kinematics.cpp` | C++17 标准库 |
| `chassis_arm_control` | `arm/control.cpp` | 解算、意优电机、BXI transport、yaml-cpp |
| `chassis_controller` | `chassis/control.cpp` 及同目录辅助头 | ISWV 电机、BXI transport、yaml-cpp |
| `chassis_remote_control` | `src/example/remote_control.cpp` | rclcpp、rcl_interfaces、geometry_msgs、std_msgs、yaml-cpp |

- 公共 include 根通常是 `src/thirdparty`；CANopen 另有 `motor/iswv/protocol/include`。同时复制对应公开头与内部辅助头，不只复制两个 `.cpp`。
- 上表保留现有链接关系；进一步裁剪头文件或链接依赖时要用独立消费者构建证明，不能仅凭目录名删除依赖。
- 意优复用 ISWV 的对象字典/CiA402 类型，但实际通信是网卡上的 EtherCAT CoE。它不因使用这些类型就需要 CAN 板卡；板卡只用于目标所需的 CAN 或共享电源。
- 安装后的模块 target 使用 `chassis::chassis_*`，CANopen 使用 `iswv::canopen`。当前根 CMake 与完整安装包均依赖 ROS/ament。
- BXI 厂家库是二进制静态库，移植到不同 CPU/ABI 时核对兼容性；没有源代码可直接重编。SOEM 保留 `LICENSE`、`ORIGIN.md` 和 `LOCAL_PATCHES.md`。

## 入口、单位和生命周期

### BXI 板卡与 MIT 电机

- `bxi/board.h`：`chassis::BxiPciTransport(bus)` 打开通道；`set_motor_power()` 管共享电源。构造并非离线操作，最后一个通道销毁会清理共享电源。
- `motor/bxi/motor.h`：`bxi::Motor(transport, id)`；显式 `enter_motor_mode()` → 周期 `command()` → `exit_motor_mode()`。电机构造和析构不自动发使能/失能帧。
- MIT 输入是位置 rad、速度 rad/s、Kp/Kd、力矩 Nm；编码范围由型号决定。CAN ID 是标准帧 ID，不是 CANopen Node-ID。`FrameOptions` 的 FD/BRS 要与设备一致；BRS 表示 CAN FD 数据段切换波特率。
- 此接口没有 CANopen 式 PV/PP 切换。`decode_feedback()` 解析反馈；发帧成功不等于目标完成。`save_zero_position()` 不是常规启动步骤。

### ISWV CANopen

- `motor/iswv/motor.h`：`chassis::IswvDrive(master, configuration)`；`enable_rpm()` → `set_velocity_rpm()` → `actual_velocity_rpm()` 或 `actual_feedback_pdo()` → `stop()`。
- 调用层速度是输出轴 RPM，加减速度是输出轴 RPM/s。底层 `iswv::Axis` 的电机轴单位、脉冲及力矩百分比应另查接口，不混用。
- 同总线一个 CANopen master，transport/master 均须比电机长寿；不要用额外接收回调覆盖主站的回调。
- 反馈未收到或超过 250 ms 会报错；保留掉线处理。`stop()` 尝试零速、快停及失能，但不切共享电源，也不保证机械已经停稳。

### 意优 EtherCAT

- `motor/yiyou/communication.h` 提供主站；`motor/yiyou/motor.h` 的 `chassis::YiyouMotor` 提供调用层。同链路共享主站，内部周期通信，应用串行调用并检查 `check_health()`。
- 使用 RPM API 前先 `configure_rpm_units()`；`enable_rpm()` 加减速度为输出轴 RPM/s，`set_velocity_rpm()` 为输出轴 RPM。
- 原生 PV/PP 参数使用 pulse/s、pulse/s²；`move_to_position()` 为绝对脉冲位置，返回成功仅完成命令交握。到位需检查 `position_reached()`，调用方设置超时。
- 模式切换前 `stop()`。CST 站位须在创建主站前通过 `Options::cst_slave` 指定；电流目标为额定电流千分比 `[-1000,1000]`，50 表示 5%。零电流不提供位置保持。
- `save_zero_position()` 持久化零位，须保留驱动要求的 EEPROM 保存时间。回到已保存零位与重新设置零位是两个动作。

### 机械臂与底盘

- `arm/kinematics.h/.cpp` 是三自由度平面正逆解：X 向前、Z 向上，m/rad，无 Y/yaw 自由度。`arm::Configuration` 包含尺寸、限位与标定零偏；默认值只对应参考模型。
- `forward()` 不验证关节限位；`inverse()` 检查 `status` 后再取 `solutions`，结果包含奇异标记。电机方向、减速比和编码器转换位于求解器之外。
- `load_arm_kinematics()` 实现在 `arm/control.cpp`，会引入硬件控制依赖；纯解算不要为读取 JSON 直接搬用它，可由调用方构造 `Configuration`。
- 机械臂硬件控制：初始化 → 按配置回已保存零位 → 周期 `update()`；共享断电由应用协调。
- 底盘：`initialize()` → `calibrate()` → `set_velocity()`/`update()` → `stop()`。速度为 m/s、m/s、rad/s；`update()` 至少每 100 ms 一次，当前 ROS 周期 20 ms。停止发送指令不会自动停车；调用 `stop()` 后继续更新并观察反馈。

六处 `bxi/example.cpp`、`motor/{bxi,iswv,yiyou}/example.cpp`、`arm/example.cpp`、`chassis/example.cpp` 当前均为无 `main()` 的函数片段，未加入默认构建或头文件安装。除纯解算外，调用示例可能操作真实设备。

## ROS 应用的五个文件

| 文件（`src/example/`） | 职责 |
| --- | --- |
| `main.cpp` | 模式、默认配置、硬件生命周期、ROS 接口、周期调度和退出 |
| `remote_control.h/.cpp` | 手柄读取、启动/急停与底盘遥控 |
| `arm_parameters.h` | 机械臂 ROS 参数适配 |
| `arm_current_remote.h` | 第四轴电流遥控接入 |

同一份 main 编译出 `chassis`、`key_control`、`robot_control`、`chassis_only`、`chassis_only_remote`。裁剪应用时可保留用户实际需要的入口。

- 核心接口：`/cmd_vel_car` 的 Twist 输入；`yiyou_arm_target=[x_m,z_m,pitch_rad]`；`/arm_pose` 反馈；第四轴 `yiyou_motor_4_current_permille`。
- 启动顺序：底盘初始化/共享上电 → 机械臂初始保持与可选 CST → 可选机械臂回零 → 底盘校准 → 开放控制。启动阶段仍处理遥控急停和取消。
- 退出先停机械臂，再清理底盘和共享电源。不要将通信对象先于执行器销毁。
- 当前底盘保持最后一次速度；若目标项目要求失联停车，需要明确实现并验证超时策略。遥控第四轴电流有过期处理，直接设置 ROS 电流参数仍持续生效。

## 配置与启动索引

| 文件 | 迁移注意点 |
| --- | --- |
| `src/config/steering.yaml` | 日常速度、限速、`remote_ctrl` 遥控参数；引用 `hardware_config_file` |
| `src/config/steering_hardware.yaml` | CAN/Node、FL/RL/RR/FR 轮序、尺寸、方向、编码器、减速比、零偏、校准和驱动参数 |
| `src/config/yiyou_ethercat.yaml` | 网卡、站位、方向、回零、速度、第四轴电流等 ROS 参数 |
| `src/config/arm_kinematics.json` | 机械臂模型、零偏、限位，用于解算及硬件控制配置 |

- 两份 steering YAML 一起迁移；硬件文件相对主 YAML 所在目录解析，只支持一层引用。主文件同名设置覆盖硬件文件；旧逐轮 `modules` 优先于全局设置。
- 驱动参数的读取回填写入参数所属文件，部署需考虑文件可写性。读取配置不代表所有运行过程均只读。
- 默认查找优先使用编译宏 `CHASSIS_SOURCE_CONFIG_DIR` 指向的现存源码目录，再按可执行位置找安装目录 `share/chassis/config`；不依赖终端当前目录。移植时更新宏、包名和安装路径，或重做等价策略。
- 含臂模式自动加载意优 YAML；用户显式 ROS 参数应保持优先。共享 `steering_config_file` 需同时提供给底盘与遥控节点；保留当前多组 ROS 参数的覆盖处理。
- 参考硬件的 `enp86s0`、站位 `[1,2,3,4]`、`/dev/input/js0`、启动/急停按钮 14/11、零偏与已校准标记不是新机器的默认事实。`yiyou_arm_home_on_start: true` 会运动。
- 根 `start.sh` 菜单是 `1 零点/点动、2 底盘、3 整车`；`test/debug.py` 菜单是 `1 电机、2 离线机械臂、3 底盘`。不要混为同一菜单。
- `start_yiyou_jog.sh`、`start_chassis_only.sh`、`start_chassis_arm.sh` 委派 `start.sh`，后者依赖 `test/debug.py` 查找程序。默认使用项目根目录 `colcon build` 生成的 `build/` 和 `install/`，可通过环境变量覆盖；`--dry-run` 只预览。
- `deploy/ros_elf_launch.service` 是本机部署示例，仅在用户需要服务部署时适配；不随模块移植自动启用。
