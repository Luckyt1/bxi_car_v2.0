# 意优 PHU/RHU 电机 can2 试转

依据根目录 `yy.pdf` V1.08：4.7.2（PV 模式，印刷页 71–74）、6.2 对象字典、7.2.5。使用现有 BXI PCI 通信层及 iSWV 库中的通用 CANopen/CiA402 层，不使用 iSWV 厂商参数或它的速度换算。

## 接线与运行条件

- 电机接 BXI 的 CAN2，代码使用 `canfd_packet.bus = 2`（零起始通道编号）；发送经典 CAN 标准帧，不是 CAN FD，也不是 SocketCAN `can2` 网卡接口。
- 手册默认 Node-ID 为 1、CAN 波特率 1 Mbps。确认板卡 CAN2 和电机波特率一致、终端电阻及 CAN_H/CAN_L 接线正确。本项目的 BXI API 没有波特率设置接口，程序不会自动设置板卡波特率。
- 程序会打开共享电机电源，等待 4 秒，退出时由现有 BXI transport 关闭电源。先停止 chassis、steering_calibration 和其他 BXI 控制进程，避免同时访问和供电冲突。
- 首次使用固定好电机、移除危险负载，准备急停；确认 STO 接线和抱闸工作模式符合实物要求。程序不会解除 STO、强制释放抱闸或自动清故障。

## 编译与试转

在 car2 根目录运行（沿用项目现有依赖：ROS 2 Humble、相邻 `../ISWV` 源码和 BXI 静态库）：

```bash
source /opt/ros/humble/setup.bash
cd diffbot_chassis
colcon build --packages-select chassis --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run chassis eyou_motor_test --help
# 节点 1，输出轴目标 +1.5 RPM，持续 5 秒，然后减速并失能
ros2 run chassis eyou_motor_test 1 1.5 5
# 输出轴反向 5 RPM，显式指定加减速度为 10 RPM/s
ros2 run chassis eyou_motor_test 1 -5 5 10 10
# 输出轴 1 RPM 持续转动，Ctrl+C 减速停机并失能
ros2 run chassis eyou_motor_test 1 1 --continuous
```

参数：`NODE_ID RPM [SECONDS=5|--continuous] [ACCEL_RPM_S=10] [DECEL_RPM_S=10]`。
RPM 指**减速器输出轴**每分钟转数，支持小数，负数反转。菜单默认 1 RPM；
试转工具允许 ±30 RPM、运行时间 1–60 秒，这只是工具范围，不是电机额定限制。
显式指定 `--continuous` 时取消时间上限，持续读取实际转速与状态，直到 Ctrl+C、SIGTERM
或异常；退出仍执行原有停机、失能和断电流程。保持终端运行，设备端断线看门狗限制见下文。
v4 起命令行速度参数由原来的 pulse/s 改为 RPM，加减速度改为 RPM/s，旧的数值不能直接沿用。

程序读取 `0x2025` 编码器单圈计数（手册第 172 页）、`0x26A2/0x26A3` 减速比分子/分母
（第 229–230 页），计算：

```text
每输出轴转一圈的计数 = 编码器单圈计数 × 减速比分子 / 减速比分母
0x60FF（pulse/s）= round(输出轴 RPM × 每输出轴转一圈的计数 / 60)
反馈输出轴 RPM = 0x606C（pulse/s）× 60 / 每输出轴转一圈的计数
```

底层仍按手册 `0x60FF/0x606C` 的 pulse/s 单位通信；这两个对象是速度指令/反馈，不是脉冲位置指令。
加减速度也按相同比例换算，避免速度调高后仍被原来 20000 pulse/s² 的缓慢斜坡限制。
例如编码器 131072 count/rev、减速比 101:1 时，输出轴 20 RPM 对应 4412757 pulse/s；
程序使用实际读到的参数，不写死此示例。读取失败、参数为零或超出协议范围会在使能前报错。
采用手册定义的电机侧计数单位，实际输出轴转速仍应实机核对；不能据此修改编码器、电子齿轮或单位参数。

## 驱动行为与复用

### 独立信息读取

编译后在项目根目录运行 `./start_motor_info.sh`（交互节点号，默认 1），或
`./start_motor_info.sh 1`，菜单选择 `1`（默认）读取信息；底层命令为
`ros2 run chassis eyou_motor_info 1`。菜单选择 `2` 则进入持续转动测试，输入输出轴 RPM
（默认 1，范围 ±30，负数反转），按 Ctrl+C 停机。两种功能均保存日志。
程序独占 CAN2，上电等待 4 秒，打印一次信息后退出并关闭共享电机电源。
电机对象操作均为 SDO 读取，不发送 NMT、使能、速度设置、模式切换或清故障。
不要与其他底盘、校准或 BXI 程序同时运行。

输出包括状态字/故障码/模式/控制来源、编码器分辨率、减速比分子与分母、
目标和实际速度 pulse/s，以及比例有效时换算的输出轴 RPM。未使能状态也尝试读取。
某项不支持或超时会独立报告失败，其余项目继续；比例缺失或无效时不伪造 RPM。
这些信息不代表额定或最高转速。全部成功返回 0，信息不完整返回 1，处理中收到中断返回 130。
启动脚本保存日志到 `logs/motor_info_*.log`，并更新 `logs/latest_motor_info.log`。

`include/chassis/eyou_motor.hpp` 提供 `EyouMotor(master, node_id)`：

上层推荐先调用 `configure_rpm_units()`，再使用 `enable_rpm(accel, decel)`、
`set_velocity_rpm(rpm)` 和 `actual_velocity_rpm()`。换算参数只能在停止状态读取，
旧的原始单位接口保留用于协议调试：

1. `enable(acceleration, deceleration)`：先读 `0x2100` 控制权。手册印刷页 183 定义 `0=UART`、`1=EtherCAT`（默认）、`2=CANopen`；若不是 CANopen，仅在确认电机为未使能的已知状态后写入 `2` 并读回核对。控制权未确认、故障、状态未知或其他控制源已使能时拒绝继续，不发送后续运动控制写入。然后进入 NMT pre-op，失能并清空旧速度，写 `0x6060=3` 并核对 `0x6061`，配置 `0x6083/0x6084`，NMT start，按 `0x06 → 0x07 → 0x0F` 使能，每步轮询 `0x6041`。控制权设置不发送参数保存指令。
2. `set_velocity(int32_t)` 写 `0x60FF`，`actual_velocity()` 检查状态并读 `0x606C`。正负速度分别控制两个方向。
3. `stop()` 写零速度，最多等待 3 秒，再写 `0x6040=0` 并确认失能；即便零速操作失败仍尝试失能。异常包含 SDO abort 码；析构做尽力停机，调用者应显式调用 `stop()` 以接收错误。

模式切换使用轮询确认：写 `0x6060=3` 得到 SDO ACK 后，每 10 ms 读一次 `0x6061`，
等待窗口为 1 秒（单次 SDO 自身仍有超时）。固件短暂返回旧模式 `8` 时继续等待；
只有读到 `3` 才允许后续使能。超时输出最后读到的模式，并执行停机清理。

同一驱动实例只能由一个线程调用，master 和 transport 必须比电机实例存活更久。集成到现有 chassis 时复用其总线生命周期，不要新建多个 BXI transport（底层接口为进程级共享资源）。此任务提供独立试转入口，没有自动接入底盘运动回调。

程序每 100 ms 轮询速度与状态；持续通信失败、故障、运行结束或 SIGINT/SIGTERM 都进入停机清理。通信断线、SIGKILL 或主机断电时无法保证 SDO 停机送达。手册 2.7 提及 `0x1016` 消费者心跳，但字典未给出完整配置细节，本版没有配置设备侧看门狗；实机接入持续运行系统前须验证设备心跳丢失动作与硬件急停。不会写保存参数指令。

## 验证

### 错误诊断日志

运行根目录 `bash start_steering_calibration.sh` 会自动把两种测试的标准输出和错误输出保存到
`logs/motor_test_日期_时间_随机后缀.log`，`logs/latest.log` 指向最近一次启动的日志。
日志记录启动时间、测试选择和参数；Ctrl+C 后仍记录停机输出。

意优试转正常输出包含实际转速、实际位置和扩展遥测（以下为示例）：

```text
转速: 1.000 RPM  位置: -0.058595 rad  电压: 48.000 V  电流: -1.250 A  电机力矩: -1.000 Nm  内部温度: 35 C  功率温度: 40 C  力控力矩: 0 Nm
```

转速仍由 `0x606C` 换算为输出轴 RPM；位置读取手册 `0x6064:00` 有符号 32 位反馈，
按 `位置 rad = 计数 × 2π / (编码器单圈计数 × 减速比)` 换算输出轴弧度，显示 6 位小数。
例如编码器 524288、减速比 101 时，52953088 个计数对应 2π rad。
保留驱动器零点，不归一化到 ±π 或 0–2π、不做底层计数回绕展开。
每次速度和位置均读取成功后才输出完整一行。
速度或位置读取失败仍执行停机清理并报告错误，控制权、模式和状态检查保留。
扩展遥测约每秒刷新一次，其间复用最近一次结果。每个扩展 SDO 请求超时 50 ms，
不支持或读取失败的字段显示“不可用”及原因，不把缺失数据伪装为零，也不单独触发停机。

| 字段 | 读取对象 | 换算 |
| --- | --- | --- |
| 母线电压 | `0x6079` uint32 | 原始 mV / 1000 = V |
| 电机电流 | `0x6078` int16、`0x6075` uint32 | 千分比 × 额定 mA / 1000000 = A |
| 电机力矩 | `0x6077` int16、`0x6076` uint32 | 千分比 × 额定 mNm / 1000000 = Nm |
| 内部温度 | `0x2779` int32 | ℃；手册中英文对传感器位置描述不一致 |
| 功率模块温度 | `0x277A` int32 | ℃ |
| 力控力矩 | `0x27BD` int32 | Nm，仅适用于支持的力控关节 |

额定电流/力矩为零时对应换算不可用。电机电流不等于直流输入电流，电机力矩不等于
减速器输出端实测力矩。信息读取入口也尝试这些对象，扩展项缺失时返回非零码但保留已读结果。
试转关闭原始 CAN 帧追踪，去掉初始化参数与诊断快照；完整信息可使用菜单 `1` 单独查询。
`start_motor_info.sh` 的菜单和启动提示只显示在终端，运行日志正常情况下只保存反馈行，
异常情况下追加错误。最近一次为 `logs/latest_motor_info.log`。

```bash
cd /home/bxi/car2
cat logs/latest.log
# 只看最近 100 行
tail -n 100 logs/latest.log
```

直接运行 `ros2 run chassis eyou_motor_test ...` 也使用精简输出，自动保存文件由启动脚本负责。

`test_eyou_motor` 使用模拟 SDO 电机检查使能序列、数据宽度、正反向速度、模式不匹配、SDO abort、状态故障和退出清理；不需要实物。实际旋转、波特率、STO、抱闸与断线停机需上机确认。

```bash
colcon test --packages-select chassis --ctest-args -R test_eyou_motor --output-on-failure
colcon test-result --verbose
```
