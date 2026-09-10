# car2：ISWV 四舵轮底盘

底盘八个轴统一使用 ISWV CANopen 驱动。意优电机只用于上装，保留为独立诊断/试转工具，不参与底盘控制。

## 接线与方向

轮位顺序固定为 `[前左、后左、后右、前右]`。

| 用途 | CAN 通道 | Node-ID | 底盘向前时方向 |
|---|---|---|---|
| 四个转向轴 | CAN0 | 1、2、3、4 | 由转向 `inverted` 配置 |
| 前左、后左行进轴 | CAN1 | 1、2 | 正 |
| 后右、前右行进轴 | CAN2 | 3、4 | 反（镜像安装） |

`drive_inverted` 定义底盘前进与电机原始转速的对应关系；现场按实际安装确认。不同 CAN 通道可以使用相同 Node-ID，同一通道不得重复。

转向 `gear_ratio` 与行进 `drive_gear_ratio` 分开配置。行进默认减速比 10、编码器分辨率 65536 来自本地 ISWV 驱动的 `manual_travel()`，必须与实际电机一致。所有行进 RPM 均指输出轴。

## 控制流程

1. 打开 CAN0/1/2、上电，先将四个行进轴停机并禁用。
2. CAN0 先校准前左、后左，再校准后右、前右；任一时刻最多两个转向轴运动。
3. 每组先搜索负限位，再搜索正限位，校验两端间距并分别计算中位。
4. 每组切换位置模式并回中；第一组保持中位时继续校准第二组。
5. 使能 CAN1/2 的四个 ISWV 行进轴，默认保持停车并等待遥控速度指令。

普通 CAN 帧依次发送，同组两轴在同一搜索阶段运行，并非硬件同步触发。估算输出力矩达到接触阈值后立即判定限位，不再等待位置停滞；第二端仍要求最小限位间距，若过早达到接触力矩会明确报告硬限位异常。搜索超时或驱动器报告过载时，日志会提示检查硬限位、机械卡滞、电流反馈与阈值。软件力矩上限始终优先中止，不能替代驱动器限矩或硬件急停。

故障、通信异常、超速、有限超时或取消会停止所有轴；主控制器退出时关闭共享电机电源。控制器校准与行驶使用同一进程、同一组 CAN master，期间不关闭再开启电源。每次启动重新校准，成功后保存 `zero_offset_inc` 与 `steering_calibrated`。

## 构建

需要 ROS 2 Humble、C++17、ament/colcon、yaml-cpp、BXI 静态库，以及与 car2 同级的 `ISWV` 源码；也可通过 `-DISWV_SOURCE_DIR=/绝对路径/ISWV` 指定。无需 libmodbus 或 pygame。

```bash
source /opt/ros/humble/setup.bash
cd diffbot_chassis
colcon build --symlink-install --packages-select chassis
source install/setup.bash
```

## 启动底盘

以下命令会实际校准并转动车轮。BXI 驱动需要访问硬件的权限；在已准备好机械限位和急停的设备上运行。

遥控启动会同时运行底盘和 Linux joystick 节点。完成两两校准并回到中位后，行进轮保持停止；右摇杆前推/后拉控制前进/后退，方向键左/右控制左转/右转。任一进程退出时脚本会停止两个进程，底盘节点随后停机、失能并断电。

```bash
sudo -E bash start_remote_control.sh
```

默认手柄为 `/dev/input/js0`：实测右摇杆上下为轴 3（前推值 `-1`），方向键左右为轴 6（左转值 `-1`），因此两个遥控输入轴默认都反转；急停为按钮 11。实车四个 CAN0 转向电机的机械方向也全部与控制坐标相反，`steering.yaml` 中的 `inverted` 独立设置为 `[true, true, true, true]`；这不会改变 CAN1/2 行进轮的 `drive_inverted`。线速度上限 `0.6 m/s`、角速度上限 `0.4 rad/s`。满速前进同时旋转时最外侧轮约需 101 输出轴 RPM，因此当前 `drive_max_output_rpm` 设置为 105 RPM。可用 `JOYSTICK_DEVICE`、`JOYSTICK_LINEAR_AXIS`、`JOYSTICK_ANGULAR_AXIS`、`JOYSTICK_INVERT_LINEAR`、`JOYSTICK_INVERT_ANGULAR`、`JOYSTICK_LINEAR_SCALE`、`JOYSTICK_ANGULAR_SCALE`、`JOYSTICK_DEADZONE` 和 `JOYSTICK_ESTOP_BUTTON` 调整。

如果手柄轴编号或方向不确定，先运行 `sudo ./inspect_joystick.py`。该脚本只读取 `/dev/input/js0`，不会启动 ROS、上电或控制电机；依次操作右摇杆前后、方向键左右和急停按钮，把输出中的 `axis`、正负 `value` 与 `button` 编号记录下来即可。其他设备可传路径，例如 `sudo ./inspect_joystick.py /dev/input/js1`；摇杆抖动较大时可用 `--threshold 3000` 提高输出阈值。

只启动底盘节点：

```bash
sudo -E bash start_chassis.sh
```

默认校准后停车。仍可显式传入输出轴 RPM 进行旧的正转等待测试，例如 `sudo -E bash start_chassis.sh 0.5`。上限由 `drive_max_output_rpm` 控制；当前配置为 105 RPM。可用环境变量 `CHASSIS_CONFIG` 指定配置路径。

等效 ROS 入口：

```bash
ros2 run chassis chassis --ros-args \
  -p steering_config_file:=/绝对路径/steering.yaml \
  -p post_calibration_rpm:=0.0
```

`steering_config_file` 必须显式指定可写 YAML；安装目录内配置仅作为模板，不会默认改写。

主节点接收 `/cmd_vel_car`（`geometry_msgs/msg/Twist`），使用 `linear.x`、`linear.y`（m/s）和 `angular.z`（rad/s）。收到首个指令后退出“正转等待”，按外部指令控制；指令超过 `command_timeout_s`（默认 0.5 秒）未更新则停车，不会自动恢复正转。发送零速度可停车，Ctrl-C 停机退出。

手柄节点也可单独运行：`ros2 run chassis key_control`。右摇杆上下轴与方向键左右轴均反转，使前推对应正向前进、左方向键对应左转。若手柄轴定义不同，可通过 ROS 参数 `axis_linear`、`axis_angular`、`invert_linear_axis`、`invert_angular_axis` 修改。按钮 0 为普通停车键，仅在按住时发送零速；按钮 11 为紧急杀停键，按下后立即发送零速并退出遥控节点。通过 `start_remote_control.sh` 运行时，遥控节点退出会触发底盘进程停机、全部轴失能并断电。拔出手柄会发布零速。事件超时默认关闭，避免摇杆保持不动时因 Linux joystick 不重复上报事件而误停车。

转向校准中位对应的行进轮正方向实测为车体右侧，配置中的 `steering_forward_offset_rad: 1.5707963267948966` 会把车体前进方向换算为转向轴 `+90°`。若首次架空测试发现轮组从中位转向的方向与预期相反，应将该值改为 `-1.5707963267948966` 后再测试。

仅校准并保持转向中位：`sudo -E bash start_steering_calibration.sh`。这是独立调试入口，不应与底盘进程同时占用板卡。

仅保持共享电机电源开启：`sudo -E bash start_motor_power_hold.sh`。该入口不会使能或转动任何电机，进程会持续运行以保持上电；按下 Ctrl+C、收到 SIGTERM/SIGHUP 或终端关闭时会先断电再退出。不要与底盘、标定或其它 BXI 程序同时运行。

单独测试某个行进轮：`sudo -E bash start_drive_wheel_test.sh`。脚本会显示前左、后左、后右、前右四个选项，并询问底盘前进方向的输出轴 RPM；只打开所选轮对应的 CAN 通道，只使能该行进电机，并持续运行到 Ctrl+C，随后停止、失能并断电。也可直接传参，例如 `sudo -E bash start_drive_wheel_test.sh 3 0.5` 测试后右轮。RPM 必须大于 0 且不超过 `drive_max_output_rpm`。

## 主程序直接调用

`ChassisController` 不依赖 ROS 消息或 ROS 执行器，主程序串行调用即可：

```cpp
#include "chassis/chassis_controller.hpp"

auto config = chassis::load_steering_config("steering.yaml");
chassis::ChassisController controller(config);
controller.initialize(keep_running);       // 打开硬件、禁用行进轴
controller.calibrate(keep_running);        // 找两端、回中、确认，最后使能行进轴
controller.forward(0.5);                   // 持续正转等待
while (keep_running()) {
  controller.update();                    // 至少每 100 ms 调用一次，持续检查状态
  // 下一步操作：controller.set_velocity(-0.01, 0.0, 0.0);
  // 停车：controller.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
controller.stop();                        // 析构还会禁用轴并断电
```

完整可编译示例是 `examples/chassis_main.cpp`，生成 `chassis_main`：

```bash
./diffbot_chassis/install/chassis/lib/chassis/chassis_main /绝对路径/steering.yaml 0.5
```

API 必须由同一线程调用，或由主程序加锁串行化。`forward()` 为显式持续运行模式；`set_velocity()` 为需要持续刷新指令的模式。异常后控制器进入 `faulted`，不会自动重新使能。直接调用示例需包含 `<thread>` 和 `<chrono>`，并提供取消回调 `keep_running`。

同一个 CMake 工程链接 `chassis_controller`；安装后使用：

```cmake
find_package(chassis REQUIRED)
add_executable(your_main main.cpp)
target_link_libraries(your_main PRIVATE chassis::chassis_controller)
```

导出的目标传递 ISWV、yaml-cpp、线程库与 BXI 静态库依赖；也支持 `ament_target_dependencies(your_main chassis)`。

## 代码分工

- `include/chassis/chassis_controller.hpp`：主程序控制 API。
- `src/chassis_controller.cpp`：硬件生命周期、状态、运动学和停车保护。
- `src/steering_calibration_core.cpp`：不依赖 ROS 的四轴校准。
- `src/iswv_drive.cpp`：ISWV 行进轴模式、转速单位与故障处理。
- `src/chassis.cpp`：ROS 参数、消息与定时循环。
- `src/bxi_pci_transport.cpp`：BXI PCI 通信适配。
- `src/eyou_motor*.cpp`、`EYOU.md`：独立上装电机工具。

已移除未接入的 Modbus 升降柱、旧 CAN 头文件、Pygame 重复遥控、自动前进再后退演示和混合电机菜单。厂商库与原始驱动样例保留。

## 验证

底盘运行时，CAN1/2 行进轮的状态、实际转速与错误寄存器通过 TPDO1/2/3 更新，配置周期为 10 ms。底盘定时器及行进轮健康检查周期为 20 ms；读取反馈不再发送 SDO。每组 PDO 独立检查时间戳，任一组缺失或超过 250 ms 未更新就停机，日志包含轮名、CAN 通道、Node-ID 和 PDO 编号。启动时等待首组完整反馈，最长 250 ms，期间行进目标保持零速。

该周期是调度目标，不是硬实时保证。运行中的转向位置/状态与行进反馈使用 PDO 缓存，行进速度和转向位置指令使用 RPDO；初始化、校准及独立测试的 SDO 诊断接口保留。连续解算和实测限位保护见 [控制说明](diffbot_chassis/README.md)。PDO 更新不代表物理链路丢帧已修复，也不修改板卡波特率。

```bash
cd diffbot_chassis
colcon test --packages-select chassis
colcon test-result --verbose
```

测试使用 FakeTransport，不接触实车。覆盖 ISWV 单位换算、模式与使能、四轴限位及取消、转向到位门控、正反转与方向映射、外部指令超时、错误清理。模拟通信不能证明实际力矩换算、机械限位位置或硬件急停有效性。
