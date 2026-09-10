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

## 详细说明

- [接线、轮位、控制流程与参数](car2/readme.md)
- [底盘控制实现](car2/diffbot_chassis/README.md)
- [ISWV 驱动与示例](ISWV/README.md)
- [ISWV CANopen 协议](ISWV/iSWV_CANopen_通讯协议总结.md)
- [BXI PCI 驱动样例](car2/bxi_pci_drv/README.md)
- [意优上装电机工具](car2/diffbot_chassis/src/chassis/EYOU.md)

离线测试使用模拟通信或参数检查，不证明实车接线、机械限位、力矩参数和急停功能有效。
