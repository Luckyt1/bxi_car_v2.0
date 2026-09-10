# iSWV CANopen C++ Driver

面向 Kinco iSWV-AA 立式舵轮的可移植 C++17 驱动库。核心库不依赖 ROS 2、SocketCAN 或具体 CAN 适配器；实际总线通过回调式 `ICanTransport` 接入。

协议依据为同目录的《iSWV 立式舵轮使用手册 V1.0》。协议整理和手册歧义见 [iSWV_CANopen_通讯协议总结.md](iSWV_CANopen_通讯协议总结.md)。

## 架构

```text
应用 / ROS 2 控制节点
  └─ iswv::SteeringLayout（CAN0）
       ├─ iswv::Axis（前左，Node-ID 1）
       ├─ iswv::Axis（后左，Node-ID 2）
       ├─ iswv::Axis（后右，Node-ID 3）
       └─ iswv::Axis（前右，Node-ID 4）
            └─ iswv::cia402::Drive
                 └─ iswv::canopen::CanopenNode
                      └─ iswv::canopen::CanopenMaster
                           └─ iswv::ICanTransport
                                ├─ FakeTransport
                                ├─ ZqwlDeviceHub::channel()
                                └─ RosTopicHub::bus()
```

当前底盘的 4 个转向电机全部接在 CAN0，顺序固定为：前左 `1`、后左 `2`、后右 `3`、前右 `4`。`SteeringLayout` 按该映射创建 4 个 `AxisKind::steering` 轴。

## 已实现范围

- 独立的 C++17/CMake 核心；
- 通用回调式 CAN/CAN FD Transport 接口；
- 线程安全 `FakeTransport` 和确定性测试注入；
- NMT、Boot-up、Heartbeat、周期 Node Guard、EMCY、SYNC；
- 手动 SYNC 和普通 Linux 线程的周期 SYNC，含迟到/丢周期统计；
- expedited SDO 1/2/4 字节上传与下载；
- 每节点独立 SDO 队列、跨节点并发、超时和 Abort Code；
- 四组 RPDO/TPDO、任意字节对齐映射、标准映射配置流程；
- 默认 COB-ID 和自定义标准/扩展 PDO COB-ID；
- CiA 402 状态解析及 `0x06 → 0x07 → 0x0F` 状态迁移；
- iSWV 速度、位置、力矩、回零、插补和停止模式；
- IO、限位、增益、报警、参数保存等手册对象的类型化定义；
- rpm、rps/s、编码器 inc、转向角度、行走线速度换算；
- CAN0 四转向电机 `SteeringLayout` 映射；
- 保留行走/转向双轴 `Module` 便利接口；
- 状态快照、Heartbeat、超时、EMCY、PDO 和原始帧订阅；
- 可选通讯失联 SafetyPolicy；恢复通讯后不会自动恢复运动；
- ZqwlCan 多通道 Hub 适配器；
- ROS 2 `communication/msg/CANFDPacket` 多 bus Hub 适配器。

第一版按需求不包含 EDS 解析、SDO segmented/block transfer。所有 iSWV 对象都不超过 4 字节；以后可在 SDO 事务层扩展分段传输。

## 核心构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

禁用测试或示例：

```bash
cmake -S . -B build \
  -DISWV_BUILD_TESTS=OFF \
  -DISWV_BUILD_EXAMPLES=OFF
```

安装和下游使用：

```bash
cmake --install build --prefix /your/prefix
```

```cmake
find_package(iswv_canopen CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE iswv::canopen)
```

## Transport 接口

扩展新的 CAN 适配器只需实现：

```cpp
class MyTransport final : public iswv::ICanTransport {
public:
    iswv::Result<void> send(const iswv::CanFrame&) override;
    void set_receive_handler(iswv::ReceiveHandler) override;
    void set_error_handler(iswv::TransportErrorHandler) override;
    bool is_open() const noexcept override;
    std::string name() const override;
};
```

约束：

- 一个 Transport 实例表示一条逻辑 CAN 总线；
- Transport 可以从任意线程调用接收回调；
- 回调必须允许被替换和清空；
- `send()` 返回只表示帧交给了适配器，不代表总线 ACK；
- CANopen Master 会快速复制接收帧，解析不占用适配器回调线程；
- CANopen 只处理经典 CAN，通用 `CanFrame` 仍保留 CAN FD 字段供其他协议复用。

## FakeTransport

```cpp
auto transport = std::make_shared<iswv::FakeTransport>();
iswv::canopen::CanopenMaster master(transport);
auto node = master.node(1);

transport->set_send_hook([transport](const iswv::CanFrame& request) {
    // 单元测试可在这里构造并 inject() SDO/PDO 应答。
});
```

`FakeTransport` 支持发送记录、超时等待、同步注入、错误注入和发送 Hook。

## ZqwlCan 适配器

如果 `zqwl_can` 已安装：

```bash
cmake -S . -B build-zqwl \
  -DISWV_BUILD_ZQWL_TRANSPORT=ON
cmake --build build-zqwl -j
```

也可以直接引用源码：

```bash
cmake -S . -B build-zqwl \
  -DISWV_BUILD_ZQWL_TRANSPORT=ON \
  -DISWV_ZQWL_SOURCE_DIR=/home/kkkk/ZqwlCan/zqwl_can_linux
```

```cpp
auto hub = iswv::transports::ZqwlDeviceHub::open("/dev/zqwl-can-...");

zqwl::ChannelConfig config;
config.channel = 0;
config.arbitration_bitrate = 500000;
hub->configure_channel(config);
hub->set_channel_enabled(0, true);

iswv::canopen::CanopenMaster can0(hub->channel(0));
iswv::SteeringLayout steering(can0);
```

Hub 在一个 `zqwl::Device` 上只注册一次底层回调，再按 channel 分发。`attach()` 进来的 Device 也必须由 Hub 独占回调注册权。

仓库还提供了一个会真实使能电机的慢速旋转示例。它固定使用通道 0、500 kbit/s、Node-ID 1，以电机 30 rpm 运行最多 5 秒，然后下发零速度、Quick Stop 和失能：

```bash
./build-zqwl/iswv_zqwl_slow_rotate_example /dev/zqwl-can-... --run
```

运行前必须架空舵轮、确认转向不会夹碰、准备硬件急停，并确认总线上只有目标 Node-ID 1。没有 `--run` 时示例只打印警告，不会打开设备或使能电机。Ctrl+C 会提前进入停止流程，但软件停止不能替代硬件急停。

## ROS 2 话题适配器

适配器与 `/home/kkkk/bxi_revo2_example` 使用相同约定：

- 消息：`communication/msg/CANFDPacket`；
- 默认发送：`canfd_packet/tx`；
- 默认接收：`canfd_packet/rx`；
- 默认 QoS depth 100；
- `frame.can_id` 使用 Linux `can.h` 的 EFF/RTR/ERR 位；
- `frame.flags & CANFD_FDF` 表示 CAN FD，否则表示经典 CAN。

构建：

```bash
source /opt/ros/humble/setup.bash
source /home/kkkk/bxi_ros2_pkg/setup.bash

cmake -S . -B build-ros2 \
  -DISWV_BUILD_ROS2_TRANSPORT=ON
cmake --build build-ros2 -j
```

或用启用该选项的 colcon 工作空间构建。

```cpp
class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control")
    {
        hub = iswv::transports::RosTopicHub::create(*this);
        can0 = std::make_unique<iswv::canopen::CanopenMaster>(hub->bus(0));
        steering = std::make_unique<iswv::SteeringLayout>(*can0);
    }

    std::shared_ptr<iswv::transports::RosTopicHub> hub;
    std::unique_ptr<iswv::canopen::CanopenMaster> can0;
    std::unique_ptr<iswv::SteeringLayout> steering;
};
```

ROS Hub 不创建节点、不创建 Executor、也不调用 `spin()`。外部 `rclcpp::Node` 必须比 Hub 活得更久。话题名、QoS 和 bus 均可配置。

## 基本使用

默认参数模板位于 [`config/steering.ini`](config/steering.ini)。当前模板包含：编码器分辨率、转向减速比、轮径、位置模式速度/加减速度、PDO 周期参数，以及四个轮子的 Node-ID 和方向反转标志。

程序中读取配置并创建四个轴：

```cpp
auto loaded = iswv::SteeringLayoutConfig::from_file("config/steering.ini");
if (!loaded) {
    // 处理 loaded.error()
}
const auto& config = loaded.value();
iswv::SteeringLayout steering(can0, config);

// 这一步才向驱动器写 PDO、位置模式和运动参数；构造对象本身不会发运动命令。
auto configured = steering.configure(config);
```

配置文件是简单 INI 格式，支持 `#`/`;` 注释。四个轮子段名固定为 `front_left`、`rear_left`、`rear_right`、`front_right`。`node_id` 必须唯一且在 1~127；`inverted` 用 `true/false`。

```cpp
#include <iswv/iswv.hpp>

iswv::canopen::CanopenMaster master(transport);
iswv::SteeringLayout steering(master);

auto front_left = steering.at(iswv::WheelPosition::front_left);   // Node-ID 1
auto rear_left = steering.at(iswv::WheelPosition::rear_left);     // Node-ID 2
auto rear_right = steering.at(iswv::WheelPosition::rear_right);   // Node-ID 3
auto front_right = steering.at(iswv::WheelPosition::front_right); // Node-ID 4

// 配置阶段通常在 Pre-operational 完成。
for (auto axis : {front_left, rear_left, rear_right, front_right}) {
    axis->node()->send_nmt(iswv::canopen::NmtCommand::enter_pre_operational);
    axis->configure_default_pdos();
    axis->configure_position_mode(200.0, 100.0, 100.0);
}
```

库构造、析构和通讯恢复都不会自动使能或恢复运动。

## SDO API

异步 Future：

```cpp
auto future = node->upload({0x6041, 0x00});
auto result = future.get();
```

类型安全阻塞访问：

```cpp
auto status = node->read(iswv::objects::status_word);
auto written = node->write(iswv::objects::target_position, 100000);
```

类型安全异步回调：

```cpp
node->read_async(iswv::objects::status_word, {}, [](iswv::Result<std::uint16_t> result) {
    // 回调由库的事件线程执行。
});
```

同一节点的 SDO 串行，不同节点可并行。阻塞 SDO 如果从库自己的协议/事件线程调用，会立即返回 `ErrorCode::would_deadlock`。

## PDO

`CanopenNode::configure_pdo()` 执行标准配置顺序：禁用 PDO、清零映射数量、写映射、恢复数量、设置通讯参数、重新启用。

`Axis::configure_default_pdos()` 提供 iSWV 预设：

- RPDO1：控制字 + 模式 + 目标速度；
- RPDO2：控制字 + 目标位置；
- RPDO3：控制字 + 目标力矩；
- RPDO4：插补目标位置；
- TPDO1：状态字 + 实际位置；
- TPDO2：实际速度 + 实际电流；
- TPDO3：输入状态 + 两组错误状态。

这是库预设，不宣称是设备出厂映射。应用应先在 Pre-operational 配置，再进入 Operational。

## SYNC 和插补

```cpp
axis.configure_interpolation({std::chrono::milliseconds{4}, true});
master.start_periodic_sync(std::chrono::milliseconds{4});
axis.start_interpolation();
axis.command_interpolated_position_pdo(target_inc);

auto stats = master.sync_statistics();
master.stop_periodic_sync();
```

周期线程使用普通 Linux 调度，不承诺硬实时。`SyncStatistics` 提供发送错误、deadline miss 和最大迟到时间；设备侧同时监控 `3011:04`。

## Heartbeat、Node Guard 和安全策略

Heartbeat 与 Node Guard 二选一：

```cpp
node->configure_heartbeat(100ms, 350ms);
// 或：node->configure_node_guard(100ms, 3);
```

可选 SafetyPolicy：

```cpp
iswv::SafetyPolicy safety;
safety.enabled = true;
safety.configure_heartbeat_producer = true;
safety.heartbeat_producer_time = 100ms;
safety.local_heartbeat_timeout = 350ms;
safety.timeout_action = iswv::TimeoutAction::quick_stop;
safety.configure_device_interruption_fault = true;
axis.apply_safety_policy(safety);
```

默认 SafetyPolicy 不发送任何自动动作。即使启用策略，通讯恢复后也不会自动重新使能或恢复目标。

软件安全逻辑不能替代硬件急停、安全继电器和机械限位。

## 线程和生命周期

- CANopen Master 有一个协议线程和一个用户事件线程；
- 适配器接收回调只复制帧到有界队列；
- 接收队列满时丢弃最旧帧；事件队列优先丢弃非关键事件，并累计统计；
- SDO 完成事件为关键事件，不因普通事件队列满而丢失；
- 所有公开收发和状态 API 可由多个应用线程调用；
- 用户回调中不要做长时间阻塞工作；
- `Subscription` 析构会撤销订阅；
- Master 必须比由它创建的 Node、Axis、Module 和 SteeringLayout 活得更久；
- Transport 被 Master 独占接收回调。

## 手册数据换算

默认使用手册示例的编码器分辨率 65536：

```text
raw_speed = rpm × 512 × encoder_resolution / 1875
raw_acc   = rps/s × 65536 × encoder_resolution / 4000000
```

手册整机参数导出的默认机械配置：

- 行走减速比：`2800 / 280 = 10`，轮径 130 mm；
- 转向名义减速比：`2500 / 277`；
- 两项均可通过 `AxisConfiguration` 覆盖。

转向减速比来自额定转速比，不是手册明确给出的齿轮齿数。需要高精度角度时应以实际机械参数和整圈计数校准。

`command_torque_percent()` 当前把 `-100..100` 直接写成 `6071:00` 的整数百分比。手册对该对象同时给出了“额定力矩百分比”和电流换算两种相互矛盾的描述；上线前必须用 EDS、KS3 和小电流实测确认。如果设备采用不同缩放，可直接通过 `CanopenNode::write()` 写原始值。

Node-ID 或 CAN 波特率写入并保存后需要重启驱动器才生效；原有 `CanopenNode` 仍代表旧 Node-ID，重启后应按新地址重新创建节点对象。

## 错误处理

正常通讯失败使用 `Result<T>`，不抛异常：

- Transport 关闭/错误；
- 超时；
- SDO Abort；
- 非法设备状态；
- 参数错误；
- 从内部回调调用阻塞事务造成的潜在死锁。

构造函数中的明显编程错误使用 `std::invalid_argument`。ZqwlCan 异常在适配器边界转换为 `Result`。

`decode_fault_status()` 解析 `2601/2602`，`emergency_code_description()` 解释手册 EMCY 错误码。

## 许可证

当前仓库暂未声明开源许可证。
