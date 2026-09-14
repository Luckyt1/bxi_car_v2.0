# car2：ISWV 四舵轮底盘

底盘八个轴统一使用 ISWV CANopen 驱动。

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
2. CAN0 四个转向轴一起校准，各轴独立推进阶段。
3. 各轴检测到负限位后立即下发反向速度，不等待其他轴；随后搜索正限位，校验两端间距并计算中位。
4. 各轴检测到正限位后独立切换位置模式回中，先完成的轴保持中位等待其余轴。
5. 使能 CAN1/2 的四个 ISWV 行进轴，默认保持停车并等待遥控速度指令。

普通 CAN 帧依次发送，四轴并行搜索，并非硬件同步触发。估算输出力矩达到接触阈值后立即判定限位，不再等待位置停滞；第二端仍要求最小限位间距，若过早达到接触力矩会明确报告硬限位异常。搜索超时或驱动器报告过载时，日志会提示检查硬限位、机械卡滞、电流反馈与阈值。软件力矩上限始终优先中止，不能替代驱动器限矩或硬件急停。

故障、通信异常、超速、有限超时或取消会停止所有轴；主控制器退出时关闭共享电机电源。控制器校准与行驶使用同一进程、同一组 CAN master，期间不关闭再开启电源。每次启动都会重新做机械限位校准，测得的中位、限位和校准状态只保存在本次进程内存中，不写回 YAML，也不加载历史校准结果。

## 构建

需要 ROS 2 Humble、C++17、ament/colcon、yaml-cpp 和 BXI 静态库。ISWV 源码已放入 `car2/ISWV`，随整个 car2 目录一起复制即可；也可通过 `-DISWV_SOURCE_DIR=/绝对路径/ISWV` 指定其他位置。`ISWV/COLCON_IGNORE` 避免将其作为独立 ROS 包构建，底盘通过 CMake 直接编译该内部依赖。无需 libmodbus 或 pygame。

```bash
source /opt/ros/humble/setup.bash
# 在 car2 根目录执行
colcon build
source install/setup.bash
```

若已有构建缓存仍指向迁移前的外部 ISWV，首次执行 `colcon build --cmake-clean-cache` 更新路径，之后直接 `colcon build`。

## 启动底盘

以下命令会实际校准并转动车轮。BXI 驱动需要访问硬件的权限；在已准备好机械限位和急停的设备上运行。

遥控启动会同时运行底盘和 Linux joystick 节点。底盘读取 `CHASSIS_CONFIG` 指定的 YAML（默认 `src/chassis/config/steering.yaml`）；手柄只负责发布前后、横移和旋转速度。电机调参中的 `null` 项会在启动时读取实际值并回填此 YAML，配置文件需要可写。完成四轴并行校准并回到中位后，行进轮保持停止。底盘固定使用全向舵轮解算：轴 3 控制前后速度，轴 0 控制左右横移，方向键轴 6 控制旋转，支持组合运动。每轮按实际转角选择机械限位内最近的等效舵向，必要时将舵向翻转 180° 并反转行进轮；`swerve_branch_hysteresis_rad: 0.0` 不保留较远的旧分支。任一进程退出时脚本会停止两个进程，底盘节点随后停机、失能并断电。

```bash
sudo -E bash start_remote_control.sh
```

下面轴号与极性沿用现有手柄映射。全向模式独立发布前后 `linear.x`、横移 `linear.y` 和旋转 `angular.z`，零前进速度时也可以横移或原地旋转。

默认手柄为 `/dev/input/js0`：实测右摇杆上下为轴 3（前推值 `-1`），方向键左右为轴 6（左转值 `-1`），转弯/横向输入默认使用轴 0，按左推为负值配置为反转，左推发布正的 `linear.y`（向左），右推发布负的 `linear.y`（向右）。设备轴定义显示轴 4/5 分别是 ABS_GAS/ABS_BRAKE，当前初始值均为 -32767，不能当作回中为零的摇杆；横向轴 0 按指定序号配置，实际左右方向仍待拨杆确认。三个遥控输入轴默认都反转；急停为按钮 11。实车四个 CAN0 转向电机的机械方向也全部与控制坐标相反，`steering.yaml` 中的 `inverted` 独立设置为 `[true, true, true, true]`；这不会改变 CAN1/2 行进轮的 `drive_inverted`。前后、横向速度各自上限 `0.6 m/s`，均应用死区、曲线和加减速平滑；合成轮速仍受底盘限幅。角速度上限 `0.4 rad/s`。满速前进同时旋转时最外侧轮约需 101 输出轴 RPM，因此当前 `drive_max_output_rpm` 设置为 105 RPM。可用 `JOYSTICK_DEVICE`、`JOYSTICK_LINEAR_AXIS`、`JOYSTICK_LATERAL_AXIS`、`JOYSTICK_ANGULAR_AXIS`、`JOYSTICK_INVERT_LINEAR`、`JOYSTICK_INVERT_LATERAL`、`JOYSTICK_INVERT_ANGULAR`、`JOYSTICK_LINEAR_SCALE`、`JOYSTICK_LATERAL_SCALE`、`JOYSTICK_ANGULAR_SCALE`、`JOYSTICK_DEADZONE` 和 `JOYSTICK_ESTOP_BUTTON` 调整。

遥控已移除 **0.05 m/s 速度死区**。新增一阶摇杆低通滤波，时间常数默认 **0.1 秒**，可用 `JOYSTICK_LOW_PASS_TIME_CONSTANT_S` 调整（0 旁路）。原归一化回中死区 `JOYSTICK_DEADZONE` 和响应曲线保留；前后默认 axis 3，转弯/横向默认 axis 0。当前 `drive_alignment_policy: none` 关闭底盘余弦降速与等待，转向和驱动仍各自限速。

如果手柄轴编号或方向不确定，先运行 `sudo ./inspect_joystick.py`。该脚本只读取 `/dev/input/js0`，不会启动 ROS、上电或控制电机；依次操作右摇杆前后和左右、方向键左右和急停按钮，把输出中的 `axis`、正负 `value` 与 `button` 编号记录下来即可。其他设备可传路径，例如 `sudo ./inspect_joystick.py /dev/input/js1`；摇杆抖动较大时可用 `--threshold 3000` 提高输出阈值。

如需绕过遥控脚本单独调试底盘节点，可直接运行 ROS 入口：

```bash
ros2 run chassis chassis --ros-args \
  -p steering_config_file:=/绝对路径/steering.yaml \
  -p post_calibration_rpm:=0.0
```

`steering_config_file` 必须显式指定现场 YAML 路径。调参空项读取成功后会回填原文件，下次启动按已保存数值应用；机械限位校准结果只在本次进程中使用。

主节点接收 `/cmd_vel_car`（`geometry_msgs/msg/Twist`），使用 `linear.x`、`linear.y`（m/s）和 `angular.z`（rad/s）。收到首个指令后退出“正转等待”，持续执行最后一条外部速度指令，直到收到新指令、显式停车、故障或退出；停止发布指令不会自动停车。发送零速度可停车，Ctrl-C 停机退出。已移除 `command_timeout_s` 配置，旧 YAML 中的同名字段不再读取。电机 PDO 反馈超时、故障及停车监测仍然有效。

手柄节点也可单独运行：`ros2 run chassis key_control`。固定使用普通舵轮的三轴速度映射，不再读取底盘 YAML 或选择转向模式。前后输入轴 3、转弯/横向输入轴 0 与方向键左右轴 6 默认均反转，使前推对应正向前进、左推对应向左平移、左方向键对应左转。若手柄轴定义不同，可通过 ROS 参数 `axis_linear`、`axis_lateral`、`axis_angular`、`invert_linear_axis`、`invert_lateral_axis`、`invert_angular_axis` 修改。按钮 0 为普通停车键，仅在按住时发送零速；按钮 11 为紧急杀停键，按下后立即发送零速并退出遥控节点。通过 `start_remote_control.sh` 运行时，遥控节点退出会触发底盘进程停机、全部轴失能并断电。拔出手柄会发布零速。启动或重连后，要求前后、横向和旋转三个轴都有读数且全部回中，才接受运动输入；未收到的轴不能当成已回中。事件超时默认关闭，避免摇杆保持不动时因 Linux joystick 不重复上报事件而误停车。

转向校准中位对应的行进轮正方向实测为车体右侧，配置中的 `steering_forward_offset_rad: 1.5707963267948966` 会把车体前进方向换算为转向轴 `+90°`。若首次架空测试发现轮组从中位转向的方向与预期相反，应将该值改为 `-1.5707963267948966` 后再测试。

仅保持共享电机电源开启：`sudo -E bash start_motor_power_hold.sh`。该入口不会使能或转动任何电机，进程会持续运行以保持上电；按下 Ctrl+C、收到 SIGTERM/SIGHUP 或终端关闭时会先断电再退出。不要与底盘、标定或其它 BXI 程序同时运行。

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

API 必须由同一线程调用，或由主程序加锁串行化。`forward()` 为显式持续正转模式；`set_velocity()` 持续保持最后一条速度指令，无需定时刷新指令，但仍须持续调用 `update()` 执行控制与健康检查。异常后控制器进入 `faulted`，不会自动重新使能。直接调用示例需包含 `<thread>` 和 `<chrono>`，并提供取消回调 `keep_running`。

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
- `src/iswv_motor.cpp` / `include/chassis/motor/iswv_motor.hpp`：ISWV 转向轴、四轴校准及行进电机驱动。
- `src/eyou_motor.cpp` / `include/chassis/motor/eyou_motor.hpp`：意优电机启停、RPM 控制和反馈；底盘默认不创建意优电机。
- `src/bxi_motor.cpp` / `include/chassis/motor/bxi_motor.hpp`：BXI MIT 电机模式控制、五参数命令和位置/速度反馈解析。
- `src/chassis.cpp`：ROS 参数、消息与定时循环。
- `src/control.cpp` 和 `start_remote_control.sh`：保留手柄遥控、速度平滑、急停和断连停车，通过 `/cmd_vel_car` 控制普通舵轮。
- `src/bxi_pci_transport.cpp` / `include/chassis/bxi_pci_transport.hpp`：BXI PCI CAN 通信和共享电源控制。该文件仅负责板卡传输，与 `bxi_motor.cpp` 的电机协议分开。
- `include/chassis/swerve_kinematics.hpp`：唯一的普通舵轮解算，支持前后、横移、原地旋转和组合运动；保留机械限位、最近等效舵角和轮速比例限幅。

主文件包含对应头文件，通过 CMake 链接所需实现，不直接 `#include` `.cpp`：

| 电机驱动 / 通信实现 | 对外接口 | 安装后的 CMake 目标 |
| --- | --- | --- |
| `iswv_motor.cpp` | `IswvDrive`、`SteeringAxes`、`calibrate_steering_with_limits()` | `chassis::chassis_iswv_motor` |
| `eyou_motor.cpp` | `EyouMotor` | `chassis::chassis_eyou_motor` |
| `bxi_motor.cpp` | `bxi::Motor`、`bxi::pack_command()`、`bxi::decode_feedback()` | `chassis::chassis_bxi_motor` |
| `bxi_pci_transport.cpp` | `BxiPciTransport` | `chassis::chassis_bxi_transport` |

普通底盘主文件直接使用 `ChassisController` 即可，它统一调用 ISWV 校准、舵轮解算和驱动。单独使用电机类时，由调用方持有 CAN transport/master，确保其生命周期长于电机对象；意优 RPM 接口使用前先调用 `configure_rpm_units()` 读取换算比例。共享电源由调用方统一管理。

### BXI 电机调用

实现来自本地 `tools/Can/bxi_motor`，保留其 `bxi` 命名空间和 Apache-2.0 来源注释。当前底盘主流程使用 ISWV；单独调用 BXI 时包含 `chassis/motor/bxi_motor.hpp`，链接 `chassis::chassis_bxi_motor`。使用 PCI 板卡时另链接 `chassis::chassis_bxi_transport`，把已有 `BxiPciTransport` 作为构造参数传入。

```cpp
#include "chassis/motor/bxi_motor.hpp"
#include "iswv/fake_transport.hpp"

int main()
{
  iswv::FakeTransport transport;  // 离线示例，仅记录发送帧。
  bxi::Motor motor(transport, 1);
  if (!motor.enter_motor_mode()) {return 1;}
  const auto sent = motor.command({0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
  const auto exited = motor.exit_motor_mode();
  return sent && exited ? 0 : 1;
}
```

命令字段依次为位置、速度、`kp`、`kd`、前馈力矩，默认编码范围依次为 `[-12.5,12.5]`、`[-45,45]`、`[0,500]`、`[0,5]`、`[-40,40]`；可通过 `Command::ranges` 按电机型号修改。源协议未明确物理单位，不能直接套用 ISWV/意优的输出轴 RPM。越界、非有限值、无效范围或与模式命令重合的编码会返回错误且不发送。

构造参数是标准 CAN ID，默认 CAN FD+BRS；`bxi::FrameOptions{false, false}` 可选经典 CAN。调用方按实际 CAN 通道和反馈 ID 路由，再用 `bxi::decode_feedback(frame, ranges)` 解码位置、速度和原始前缀；此函数不安装接收回调。编码范围、帧类型及路由须与具体电机一致。

`Motor` 不控制共享电源，构造和析构均不发帧；应用负责显式退出模式及电源清理，保证 transport 存活时间长于 motor。发送成功表示传输层接受帧，不代表电机已确认或物理停车完成。

已移除未接入的 Modbus 升降柱、旧 CAN 头文件、Pygame 重复遥控、自动前进再后退演示和混合电机菜单。厂商库与原始驱动样例保留。

## 验证

底盘运行时，CAN1/2 行进轮的状态、实际转速与错误寄存器通过 TPDO1/2/3 更新，配置周期为 10 ms。底盘定时器及行进轮健康检查周期为 20 ms；读取反馈不再发送 SDO。每组 PDO 独立检查时间戳，任一组缺失或超过 250 ms 未更新就停机，日志包含轮名、CAN 通道、Node-ID 和 PDO 编号。启动时等待首组完整反馈，最长 250 ms，期间行进目标保持零速。

该周期是调度目标，不是硬实时保证。运行中的转向位置/状态与行进反馈使用 PDO 缓存，行进速度和转向位置指令使用 RPDO；初始化、校准及独立测试的 SDO 诊断接口保留。PDO 更新不代表物理链路丢帧已修复，也不修改板卡波特率。

```bash
cd diffbot_chassis
colcon test --packages-select chassis
colcon test-result --verbose
```

当前 CTest 保留 CLI 帮助检查及代码格式、静态检查，不依赖已删除的 `tests/` 目录。驱动整理时另用临时 FakeTransport 用例检查校准、ISWV/意优驱动和控制器故障路径；BXI 接入另验证两组原始协议用例及安装后的离线调用；这些临时用例不属于已安装软件。模拟通信不能证明实际力矩换算、机械限位位置或硬件急停有效性。
