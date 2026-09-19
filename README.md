# BXI 四舵轮底盘

这是基于 ROS 2 Humble、BXI PCI 板卡和 CANopen 电机驱动的四舵轮底盘工程。主要代码在 `car2/diffbot_chassis/src/chassis/`，包含三类电机接口、底盘控制、CAN3 机械臂控制、遥控器和统一主入口。

详细接线、构建、运行、代码分工和迁移后的依赖布局见 [car2/readme.md](car2/readme.md)。依赖收拢来源与保留理由见 [dependencies.md](car2/diffbot_chassis/src/chassis/docs/dependencies.md)。

## 快速构建

```bash
source /opt/ros/humble/setup.bash
cd car2/diffbot_chassis
colcon build --symlink-install --packages-select chassis
source install/setup.bash
colcon test --packages-select chassis
colcon test-result --verbose
```

## 主要入口

以下命令在 `car2/` 下执行。会访问真实硬件的入口需要先确认机械限位、架空/固定状态和硬件急停。

| 命令 | 用途 |
| --- | --- |
| `sudo -E bash start_remote_control.sh` | 旧双进程方式：底盘节点 + 纯遥控节点 |
| `ros2 run chassis key_control --ros-args -p steering_config_file:=/path.yaml` | 推荐自启动入口：等待 button14 后同进程启动底盘、CAN3 和遥控 |
| `ros2 run chassis robot_control --ros-args -p steering_config_file:=/path.yaml` | 同上，保留统一入口名称 |
| `ros2 run chassis key_control --ros-args -p start_chassis:=false` | 纯遥控发布模式，用于已有底盘节点 |
| `ros2 run chassis chassis ...` | 只启动底盘与 CAN3 机械臂 |
| `ros2 run chassis chassis_main --help` | 非 ROS 消息循环的 C++ 示例入口帮助 |
| `ros2 run chassis motor_power_hold --help` | 共享电源保持工具帮助，不使能电机 |

## 当前源码布局

`chassis` 包内已经收拢必要依赖，不再使用独立的 `car2/bxi_motor`、`car2/bxi_pci_drv`、`car2/ISWV` 或 `car2/libcanfd` 目录：

```text
car2/diffbot_chassis/src/chassis/
├── include/
│   ├── chassis/motor/      # BXI、ISWV、意优和 BXI PCI transport 公开接口
│   ├── chassis/control/    # 底盘、CAN3 机械臂、遥控器公开接口
│   └── iswv/               # 原 ISWV CANopen 公共头
├── src/
│   ├── motor/              # 三类电机与 BXI PCI transport
│   ├── motor/canopen/      # ISWV CANopen 核心实现
│   └── control/            # chassis/arm/remote/main
├── tests/                  # 离线回归测试
├── tests/canopen/          # 原 ISWV CANopen 测试
├── vendor/bxi_pci/         # 厂商头文件与静态库
└── docs/                   # 依赖来源、协议手册与许可证说明
```

离线测试只能证明协议编码、控制状态机和模拟通信路径，不证明真实接线、机械限位、力矩参数、电源断开时延或急停功能有效。
