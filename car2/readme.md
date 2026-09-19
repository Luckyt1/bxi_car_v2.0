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

遥控启动会同时运行底盘和 Linux joystick 节点。底盘读取 `CHASSIS_CONFIG` 指定的 YAML（默认 `src/chassis/config/steering.yaml`）；手柄发布底盘速度，并通过参数服务调整 CAN3 目标角度。电机调参中的 `null` 项会在启动时读取实际值并回填此 YAML，配置文件需要可写。完成四轴并行校准并回到中位后，行进轮保持停止。底盘固定使用全向舵轮解算：轴 3 控制前后速度，轴 0 控制左右横移；方向键轴 6 控制底盘旋转，button3/1 分别选择上一个/下一个 CAN3 电机，轴 7 专用于逐次调角。每轮按实际转角选择机械限位内最近的等效舵向，必要时将舵向翻转 180° 并反转行进轮；`swerve_branch_hysteresis_rad: 0.0` 不保留较远的旧分支。任一进程退出时脚本会停止两个进程，底盘节点随后停机、失能并断电。

```bash
sudo -E bash start_remote_control.sh
```

每次通过此脚本启动都会自动建立 `log/runtime/remote-control-时间-随机后缀/`，将终端完整输出保存到 `console.log`，包括 CAN3 启动信息、厂商 PCI/CAN 错误、实际执行文件路径和 SHA256、退出时间与状态码；启动配置另存为 `steering-at-start.yaml`。终端仍同步显示输出，`log/runtime/latest/console.log` 指向最近一次日志。可以通过 `CHASSIS_LOG_DIR` 指定其他日志目录。日志中的 CAN3 发送成功不代表电机已经应答；脚本会检查程序是否包含 CAN3 启动标记，帮助识别旧版本。此脚本修改无需重新编译，下次启动自动生效。

下面轴号与极性沿用现有手柄映射。全向模式独立发布前后 `linear.x`、横移 `linear.y` 和旋转 `angular.z`，零前进速度时也可以横移或原地旋转。

默认手柄为 `/dev/input/js0`：实测右摇杆上下为轴 3（前推值 `-1`），方向键左右为轴 6（向左值 `-1`，控制底盘旋转），转弯/横向输入默认使用轴 0，按左推为负值配置为反转，左推发布正的 `linear.y`（向左），右推发布负的 `linear.y`（向右）。设备轴定义显示轴 4/5 分别是 ABS_GAS/ABS_BRAKE，当前初始值均为 -32767，不能当作回中为零的摇杆；横向轴 0 按指定序号配置，实际左右方向仍待拨杆确认。底盘前后、横向、旋转输入默认反转；CAN3 方向另用 `JOYSTICK_CAN3_INVERT_DIRECTION` 配置；急停为按钮 11。实车四个 CAN0 转向电机的机械方向也全部与控制坐标相反，`steering.yaml` 中的 `inverted` 独立设置为 `[true, true, true, true]`；这不会改变 CAN1/2 行进轮的 `drive_inverted`。前后、横向速度各自上限 `0.6 m/s`，均应用死区、曲线和加减速平滑；合成轮速仍受底盘限幅。底盘旋转默认使用轴 6，角速度上限为 `0.4 rad/s`。满速前进同时旋转时最外侧轮约需 101 输出轴 RPM，因此当前 `drive_max_output_rpm` 设置为 105 RPM。可用 `JOYSTICK_DEVICE`、`JOYSTICK_LINEAR_AXIS`、`JOYSTICK_LATERAL_AXIS`、`JOYSTICK_ANGULAR_AXIS`、`JOYSTICK_INVERT_LINEAR`、`JOYSTICK_INVERT_LATERAL`、`JOYSTICK_INVERT_ANGULAR`、`JOYSTICK_LINEAR_SCALE`、`JOYSTICK_LATERAL_SCALE`、`JOYSTICK_ANGULAR_SCALE`、`JOYSTICK_DEADZONE` 和 `JOYSTICK_ESTOP_BUTTON` 调整。

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

手柄节点也可单独运行：`ros2 run chassis key_control`。默认底盘前后轴 3、横向轴 0、旋转轴 6；axis7 和 button3/1 由 CAN3 专用。底盘运动轴不能配置为 7，使能/停止/急停按钮不能配置为 1 或 3，配置冲突会拒绝启动。按钮 0 按住时发送底盘零速并阻止新的 CAN3 调角请求；按钮 11 按下后发送零速并退出遥控节点，通过主启动脚本联动底盘停机、失能和断电。拔出手柄会发布零速并停止提交调角；已经提交的单步请求可能已经生效，日志会说明。启动或重连后，底盘前后/横向/旋转轴和 CAN3 的 axis7 必须都报告回中，button3/1 必须都报告释放，才接受输入；未收到的轴或按钮状态不能当成已回中或已释放。事件超时默认关闭。

转向校准中位对应的行进轮正方向实测为车体右侧，配置中的 `steering_forward_offset_rad: 1.5707963267948966` 会把车体前进方向换算为转向轴 `+90°`。若首次架空测试发现轮组从中位转向的方向与预期相反，应将该值改为 `-1.5707963267948966` 后再测试。

仅保持共享电机电源开启：`sudo -E bash start_motor_power_hold.sh`。该入口不会使能或转动任何电机，进程会持续运行以保持上电；按下 Ctrl+C、收到 SIGTERM/SIGHUP 或终端关闭时会先断电再退出。不要与底盘、标定或其它 BXI 程序同时运行。

## CAN3 三个 BXI 电机

`BxiCan3` 管理 CAN3 上 ID **1、2、3** 的三个 **BXI8515-19**。ROS 主程序和 `chassis_main` 启动时退出电机模式，各发送一次保存当前零点指令，然后按 **Kp=200、Kd=4** 保持位置。初始目标为 0 rad、速度和前馈力矩均为 0；遥控器 button3/1 选择电机、axis7 逐次调整目标角度，可选第二终端也可微调。每次重新启动仍以启动时的位置为零点，不保存上次调整的目标角度。

CAN3 启动默认提交以下 12 帧，均使用标准 CAN ID、CAN FD+BRS、8 字节数据：

| 顺序 | CAN ID | 数据（十六进制） | 含义 |
| --- | --- | --- | --- |
| 1、2、3 | 1、2、3 | `FF FF FF FF FF FF FF FD` | 退出电机模式 |
| 4、5、6 | 1、2、3 | `FF FF FF FF FF FF FF FE` | 保存当前位置为零点 |
| 7、9、11 | 1、2、3 | `FF FF FF FF FF FF FF FC` | 使能对应电机 |
| 8、10、12 | 1、2、3 | `7F FF 7F F6 66 33 37 FF` | 零位保持，Kp=200、Kd=4 |

`FE` 的含义见 [BXI 特殊控制帧说明](https://wiki.bxirobotics.cn/actuators/can_communication/#4-8)。提交成功不代表设备已经确认使能或保存零点。之后 `hold_zero()` 每 20 ms 刷新三个电机的当前目标；方法名保留兼容旧调用，调整角度后保持的是新目标，不再强制回零。校准期间也调用刷新，但控制调度不是硬实时保证。每帧携带 Kp、Kd，普通 `bxi::Command{}` 默认值不变。

### 速度反馈和整车断电

普通 MIT 反馈按[官方协议](https://wiki.bxirobotics.cn/actuators/mit_polling_reply/)严格匹配：电机 1/2/3 的默认回复 ID 为 `0x11/0x12/0x13`，标准数据帧、8 字节、`data[0]` 匹配电机 ID。每秒的 `CAN3_FEEDBACK` 日志分别输出位置、`velocity_rad_s`、**`velocity_rpm`**、目标角度、累计回复数及最近回复年龄；首次有效回复立即在下次主线程诊断时输出。`REPLIED` 只确认通信，不证明实际使能力矩正常。未知 ID 原始帧单独计为 unmatched，不计作这三个电机的反馈。

三个电机统一按输出轴速度绝对值 **达到 60 rpm（2π rad/s）** 触发保护，正反转均检查。接收回调逐帧判断，发现超速后立即提交共享电源断电命令，不等待每秒日志或主线程校准完成。触发会锁存故障、拒绝后续使能与位置指令，主程序退出；必须排查后手动重新启动，降低速度或重新调用 `initialize()` 不会解除锁存。这里切断的是**整车共享电源**，包括底盘和 CAN3。

从首次保持指令开始监视反馈：主循环发现任一电机超过 1 秒没有合法反馈，也提交共享电源断电并锁存，防止在速度未知时继续保持。缺失反馈检查受主循环调度影响；超速检查直接在接收回调执行。日志中的断电提交成功不等于测量确认电源电压已消失，实际断电时延尚需现场验证。

通过主脚本启动后，设置说明、逐电机速度、角度调整与保护原因均保存到 `log/runtime/latest/console.log`。

### 遥控器调角

主程序校准完成后，遥控器默认选中 CAN3 电机 **1**：

- **button3 / button1**：button3 选择上一个（1→3→2→1），button1 选择下一个（1→2→3→1）；每按一次只切换一次，长按不重复，须松开后再按。日志 `CAN3_SELECTED motor_id=...` 显示当前选择。
- **axis7 两个方向**：负值增加目标 **1°**，正值减少目标 **1°**；长按不连转，必须回中后才接受下一次。主程序确认后日志显示 `CAN3_REMOTE motor_id=... target_deg=... accepted`。
- 松开 axis7 后保持新目标；改变的是相对本次启动零点的角度，速度/前馈力矩仍为 0，Kp=200、Kd=4。

物理顺/逆时针取决于安装观察方向和电机正向，未做硬件方向测试。若希望颠倒 axis7 两个方向，启动时设置 `JOYSTICK_CAN3_INVERT_DIRECTION=true`。axis6 恢复底盘旋转；axis3/0 控制底盘前后/横移。

遥控调角始终先读取所选电机当前目标，再提交 ±1°。每次只允许一个在途请求；忙时按键不会排队，超时丢弃且不自动重试。初始化、停用或重新连接后，axis7 需要回中且 button3/1 均须释放；切换电机时不会把旧电机的未提交调角转给新电机。60 rpm 超速整车断电与反馈丢失保护仍由主进程执行。

### 可选第二终端调角

先正常启动主程序，等主终端出现 `CAN3 angle control ready`，再在另一个连接同一台机器的终端执行：

```bash
cd /home/bxi/test/car2
bash start_can3_control.sh
```

- 按 `1`、`2`、`3` 选择电机。
- 按 `↑` 增加目标 **1°**，按 `↓` 减少 **1°**，相对本次启动零点。
- 按 `q` 只退出调角界面，主程序继续保持最后目标；停机仍使用主终端 Ctrl+C 或手柄急停。

终端使用现有 ROS 参数服务，不单独打开 PCI 板卡。每次调整先读取当前目标，再提交一个电机的新目标；只允许一个在途操作，超时显示结果未知，下次重新读取。服务器拒绝无新鲜反馈、故障锁存、非有限值、每次超过 1° 或超出协议位置范围的请求。协议范围为 ±12.5 rad（约 ±716.20°），**不是机械安全行程**，本功能没有自动标定 CAN3 机械限位，操作时须遵守机构实际行程。

参数 `can3_motor_1_angle_deg`、`can3_motor_2_angle_deg`、`can3_motor_3_angle_deg` 保存本次运行的目标；启动时要求为 0。`can3_speed_limit_rpm` 为只读 60 rpm。第二终端应使用与主程序相同的 `ROS_DOMAIN_ID` 等 ROS 环境；`start_can3_control.sh --help` 可查看选项。真实电机反馈速度在主终端和日志查看。

头文件为 `chassis/motor/bxi_can3.hpp`，链接目标为 `chassis::chassis_bxi_can3`。主程序串行调用 `initialize()`、`save_zero_positions()`、`hold_zero()`，正常退出调用 `stop()`；接收线程只共享受锁保护的反馈与故障状态。测试注入 `ICanTransport` 和断电回调，不接触真实硬件。

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
- `src/bxi_can3.cpp` / `include/chassis/motor/bxi_can3.hpp`：CAN3 上 ID 1、2、3 的 BXI 电机分组控制及退出清理。
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
| `bxi_can3.cpp` | `BxiCan3` | `chassis::chassis_bxi_can3` |
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
  bxi::Command command{0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
  command.ranges = bxi::encoding_ranges(bxi::Model::BXI5014_19);
  const auto sent = motor.command(command);
  const auto exited = motor.exit_motor_mode();
  return sent && exited ? 0 : 1;
}
```

四种型号已提供 `bxi::Model` 配置，调用 `bxi::encoding_ranges(model)` 后赋给 `Command::ranges`。下面列的是 [官网 MIT 通信协议](https://wiki.bxirobotics.cn/actuators/can_communication/) 的默认编码范围（核对日期：2026-09-15），不是允许电机持续工作的力矩限制。

| 型号 / 枚举 | 协议类型 | 力矩编码范围（Nm） | Kd 编码范围 | 额定 / 峰值力矩（Nm） |
| --- | --- | --- | --- | --- |
| [BXI5014-19](https://wiki.bxirobotics.cn/actuators/Introduction/BXI5014-19/) / `BXI5014_19` | MOTOR_50 | -40～40 | 0～5 | 7 / 25 |
| [BXI5018-19](https://wiki.bxirobotics.cn/actuators/Introduction/BXI5018-19/) / `BXI5018_19` | MOTOR_50_L | -40～40 | 0～5 | 11 / 35 |
| [BXI7010-19](https://wiki.bxirobotics.cn/actuators/Introduction/BXI7010-19/) / `BXI7010_19` | MOTOR_70 | -80～80 | 0～5 | 15 / 48 |
| [BXI8515-19](https://wiki.bxirobotics.cn/actuators/Introduction/BXI8515-19/) / `BXI8515_19` | MOTOR_85 | -160～160 | 0～20 | 40 / 150 |

四者的位置编码均为 `[-12.5,12.5] rad`、速度为 `[-45,45] rad/s`、Kp 为 `[0,500]`。产品页列出的减速比均为 19.5，输出端额定转速 100 RPM、空载转速 200 RPM；这些机械规格不参与 MIT 编码范围的缩放，也不能代替运行时运动限位。BXI 参数不是 ISWV/意优的 RPM 接口。

发送和反馈解析必须使用相同范围：`bxi::decode_feedback(frame, command.ranges)`。官网说明设备的 `max_pos/max_vel/max_tor/kp_max/kd_max` 可修改；如果实际寄存器参数已变，应覆盖 `command.ranges` 中对应项。未指定型号的旧代码仍使用50系列范围。型号工厂不读写设备寄存器，不设置运行速度或控制增益。

越界、非有限值、无效范围和普通编码与 `0xFA..0xFE` 特殊控制帧重合时，会返回错误且不发送，避免误触发零点保存、模式切换或终端输出。CAN ID 是设备的 `can_id`；帧默认 CAN FD+BRS，可用 `bxi::FrameOptions{false, false}` 选择经典 CAN。官网支持的最高速率为 CANFD `1M+5M`、不变速时 `1M`；当前接口不修改板卡波特率。

[官网反馈说明](https://wiki.bxirobotics.cn/actuators/mit_polling_reply/) 指定默认回复 ID 为 `master_id = can_id | 0x010`，且可单独修改。调用方按实际 `master_id` 路由反馈，再解码位置、速度和 `data[0]` 中的电机 ID；当前基础解码接口仍只取前5字节中的这些字段，兼容已有短反馈用例，不解析力矩、温度或 AUX，也不安装或覆盖接收回调。

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

当前 CTest 包含 CAN3 三电机离线控制测试、CLI 帮助检查及代码格式、静态检查。驱动整理时另用临时 FakeTransport 用例检查校准、ISWV/意优驱动和控制器故障路径；BXI 接入另验证两组原始协议用例及安装后的离线调用；这些临时用例不属于已安装软件。模拟通信不能证明实际力矩换算、机械限位位置或硬件急停有效性。
