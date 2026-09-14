# 提取范围与验证计划

## 官方 bxi_car 补充（2026-09-13）

来源：`bxirobotics/bxi_car` 提交 `a02e41f59373b36e38686ed8be83f3421f1959a4` 的 `diffbot_chassis/src/chassis/src/chassis.cpp`。

1. 保留原始实现和 Apache-2.0 许可证，先增加命令字节、量化边界、反馈解析回归测试。
2. 提取 MIT 五参数打包、进入/退出模式、位置和速度反馈解析，复用现有 `ICanTransport`。
3. 对合法参数保留原量化截断方式；拒绝越界及非有限值，避免原样例 `kd=6` 超出编码范围 `[0,5]`。
4. 所有 CAN 帧显式初始化，帧类型可配置，保留既有接口默认 CAN FD+BRS；不复制原析构只初始化一帧却发送两帧的问题。
5. 拒绝普通参数编码成已知 FC/FD 模式指令；解析前检查帧类型与长度。
6. 离线构建与 FakeTransport 测试，不控制真实设备。源代码未说明的物理单位、反馈字段不作推断。

## 初次收录

1. 保留 `new_car/bxi_car_v2.0/car2/bxi_pci_drv/src/motor_test.c` 原始样例作为协议来源。
2. 先建立离线测试，固定样例中的标准 ID、CAN FD/BRS 标志及 `FF FF FF FF FF FF FF FD` 退出命令；检查构造不发帧、非法 ID 和传输失败。
3. 将已知退出命令封装到不依赖厂商 SDK 的接口，通过现有 `ICanTransport` 接入。
4. 初次收录时只找到退出样例；后续以上方官方仓库证据补充控制范围。

共享电源及 PCI 生命周期由相邻 `bxi_hardware` 模块管理。本模块不隐式开关电源或启动电机。
