# 意优前三轴：仿真零位与 ROS 末端目标

`chassis`、`key_control`、`robot_control` 的机械臂控制器已接入与 MuJoCo
仿真相同的平面 2+1 正逆解。`chassis_only` 系列不创建机械臂接口。
前三个 EtherCAT 站位依次为肩、肘、腕，第四轴 CST 电流控制独立保留。

## 零位与坐标

- 肩轴心为原点，X 向前、Z 向上，长度单位为米。
- pitch 为平面仰角，单位为弧度，正方向从 +X 转向 +Z（绕 -Y）。
- 连杆默认 0.30 m / 0.40 m，工具偏移 0 m。
- 关节角 q 相对标定零位；几何相对角为 q 加 `[150°,-150°,0°]`。
- q 全为 0 时，上臂朝后上方 150°，前臂和工具朝前水平。
  此时末端约为 `[0.1401923789,0.15,0.0]`。
- 肩、肘、腕标定角限位分别为 `[-150°,30°]`、`[-30°,330°]`、`[-180°,180°]`。

电机映射使用
`q = direction * (encoder_counts - zero_counts) * 2π / pulses_per_output_revolution`。
每个电机的编码器分辨率和减速比由驱动器反馈读取。几何 150° 偏置只参与解算，
不会额外写入电机目标。现场 YAML 配置的三轴方向为 `[1,-1,-1]`：第二、三关节反向，
零点计数仍均为 0；正解反馈和逆解下发使用同一方向映射。未加载 YAML 时，
程序方向参数默认仍为 `[1,1,1]`，应通过参数覆盖实机安装方向。

主程序启动先保持实测位置，再默认以 PP 模式回到已保存的三轴零位
（`yiyou_arm_zero_counts`，即 q=[0,0,0]），不重新设零或写 EEPROM。
`key_control` / `robot_control` 仍须先按 button14；回零完成后才校准底盘并开放遥控。
回零按各关节限速执行，不保证末端直线轨迹；最多等待 300 s，期间仍处理 button11 急停。
三轴必须完成本次 PP 握手、报告到位、位置误差不超过 100 脉冲且报告静止，才算回零完成。
日志依次为 `ARM_HOME_START`、`ARM_HOME_DONE`，底盘校准后显示 `EtherCAT arm IK ready`。
当前测量角或零位超限、单轴回零超过 180°、超时或通信故障会中止启动并请求共享电源断电。
设置 `yiyou_arm_home_on_start:=false` 可恢复启动原位保持；此时仍拒绝超限的末端请求。

## ROS 接口

主程序完成校准、日志显示 `EtherCAT arm IK ready` 后，设置同一数组参数：

```bash
# 请求运动：肩坐标系 x=0.15 m、z=0.15 m、pitch=0 rad。
ros2 param set /chassis yiyou_arm_target '[0.15, 0.15, 0.0]'
ros2 param get /chassis yiyou_arm_target
ros2 topic echo /arm_pose --once
```

三个元素必须都是浮点数。启动时该参数为空数组；拒绝非空启动覆盖值，
避免启动后意外追逐一个预设目标。参数回调只验证，已提交的新值在控制定时器中排队，
随后逐周期下发。包含其他非法参数的原子请求不会向电机下发目标。
重复设置相同目标不会重复触发运动。参数是请求值，不是运动完成反馈；
实际位置以 `/arm_pose` 和 `ETHERCAT_YIYOU_POSE` 日志为准。

`/arm_pose` 类型为 `geometry_msgs/msg/PoseStamped`，frame 为 `arm_shoulder`，
position.y 固定为 0；四元数为 `(x=0,y=-sin(pitch/2),z=0,w=cos(pitch/2))`。
它使用 ROS 四元数约定，所以 ROS RPY pitch 的符号与输入的平面仰角相反。

旧 `can3_motor_*_angle_deg` 保持只读；遥控器已移除选关节、写单关节角度的逻辑。

## 手柄末端点动

`key_control` / `robot_control` 完成启动校准后，沿用原机械臂按键：

| 操作 | 行为 |
| --- | --- |
| button1 | 选择下一项：x → z → pitch → x |
| button3 | 选择上一项：x → pitch → z → x |
| axis7 负方向 | 所选坐标增加一步 |
| axis7 正方向 | 所选坐标减少一步 |

默认选 x，x/z 每步 0.01 m（1 cm），pitch 每步 π/180 rad（1°）。
x 增加表示向前，z 增加表示向上，pitch 增加表示抬头；实际电机方向仍由标定映射决定。
短拨立即走一步；保持 axis7 同一方向时，每 0.1 s 触发一步，回中停止追加。
100 ms 重复使用单调时钟，由控制周期调度，繁忙或延迟时跳过、不补发积压步进。
切换坐标或不经过回中的直接反向会暂停连发，必须先回中再拨动；选项按钮每次松开再按才切换。
轴 0/3/6 继续控制底盘，button14 启动、button11 急停行为不变。
日志 `ARM_SELECTED coordinate=...` 显示当前选项，`ARM_REMOTE` 显示请求结果。

每一步先读取 `yiyou_arm_target`；首次为空时从 `/arm_pose` 的实际位姿起步，
之后基于主节点当前保存的目标做增量，并提交完整数组，走同一套 IK 和限位检查。
必须收到 `arm_shoulder` 坐标系的有效平面位姿，且最近一次接收距当前小于 500 ms。
回读参数后、提交前还会再次检查反馈新鲜度；没有反馈时不会拿默认零坐标运动。
请求异步执行，最多一个在途请求。请求忙或三轴握手忙时跳过本次长按步进，不排队；
服务超时 2 s、反馈无效、服务不可用或目标非法时暂停长按，回中后才能再次触发。
启动/校准期间不接受点动。重连需要回中和释放选择键；取消或切换坐标不会重放未提交的请求。
回中会取消尚未提交的长按重复请求，短拨的首步仍可完成提交。
button0/使能释放会取消未提交步进，但已提交的点到点目标可能继续运动；button11 才是急停。
其他 ROS 发布端也能设置目标，应避免同时操作手柄和外部目标写入端。

以下为 `remote_ctrl` 的启动参数，不改变主程序电机速度：

| 参数 | 默认值 |
| --- | --- |
| `arm_node` | `/chassis` |
| `arm_pose_topic` | `/arm_pose` |
| `arm_translation_step_m` | `0.01` |
| `arm_pitch_step_rad` | `0.017453292519943295` |
| `invert_arm_direction` | `false`（设为 true 反转 axis7 步进方向） |

旧遥控节点参数 `can3_node` / `invert_can3_direction` 已替换为上述参数。
完整机器人入口为 `src/example/main.cpp`，部署命令见根目录 `README.md`。
点动参数通过 ROS 启动参数传入，例如 `--ros-args -p arm_translation_step_m:=0.01`。
电机、离线机械臂解算和底盘调试统一使用 `python3 test/debug.py`。

## 配置

下面参数都只允许在启动时设置，示例见 `src/config/yiyou_ethercat.yaml`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `yiyou_arm_motor_directions` | `[1,1,1]` | 各轴电机正转到仿真正角的符号，只允许 ±1 |
| `yiyou_arm_zero_counts` | `[0,0,0]` | 仿真 q=0 对应的各轴绝对编码器计数 |
| `yiyou_arm_speed_rpm` | `0.6` | PP 输出轴速度，约 3.6°/s；回零和末端控制共用 |
| `yiyou_arm_acceleration_rpm_s` | `1.0` | PP 输出轴加减速度 |
| `yiyou_arm_home_on_start` | `true` | 主程序初始化后返回已保存的三轴零位；false 为原位保持 |
| `yiyou_arm_kinematics_file` | 空字符串 | 默认内置仿真几何，可指定 `arm_kinematics.json` 或仿真 `arm.json` |

JSON 接受与仿真相同的 `upper_arm`、`forearm`、`tool`、`zero_offsets` 和
`joint_limits` 字段。长度为米，偏置和限位为弧度；旧格式缺省 `zero_offsets`
时按 `[0,0,0]` 处理。更改仿真配置后应同步给控制器；本包中的配置是集成时的副本。

## 运动与故障边界

逆解用上一已接受的关节目标作种子，保留启动肘分支，按几何肘角判断正负，
不自动翻肘。不可达、非有限值、关节超限、编码器目标溢出、奇异目标以及
任一关节需跨越超过 180° 的请求都会被拒绝，保留原目标。

三轴目标先全部校验，再逐轴写入并读回，全部成功后才触发运动。运行期握手
逐周期推进，无等待应答的 sleep 循环；每周期最多一次目标 SDO 读回，显式超时 5 ms，
整组握手期限 1 s。握手期间新目标会被拒绝。握手/I/O/反馈故障会锁存，
请求关闭共享电源并停止所有轴；不会自动重试或恢复。顺序触发期间仍可能部分轴已接受目标，
因此通信失败采用整组断电处理。

三轴分别按驱动器 PP 速度和加减速度运动，这是关节点到点控制；不保证
三轴严格同步、末端直线轨迹或避碰。应答完成只表示目标已接受，不代表运动到位。
普通 Linux 调度、PDO 等待和错误清理也不提供硬实时保证。
这些软件行为有离线测试覆盖，实机标定、运动方向和机械响应尚未验证。

## 第四轴遥控电流

配置四个从站后，button0 / button4 分别控制第四轴正/负电流，默认额定电流的 5%。启动或重连后先松开两个按钮；按住输出，松开清零。两键同时按下清零并要求全部松开，button11 急停仍然有效。带机械臂入口不再把 button0 作为普通停车键；x/z/pitch 选择和 axis7 点动保持独立。

遥控节点参数 `arm_current_permille` 设置正幅值（1..1000，默认 50），启动脚本可用 `JOYSTICK_ARM_CURRENT_PERMILLE` 覆盖。遥控通过 `/arm_motor4_current` 以 50 ms 周期刷新 `rcl_interfaces/msg/Parameter`（参数名 `yiyou_motor_4_current_permille`，整数值）；`arm_current_topic` 可在发送和接收节点同时覆盖。断连或反馈失效清零，接收端 300 ms 未收到刷新也清零。手动 `ros2 param set` 电流接口仍是持续指令，没有刷新超时。第四轴零电流不锁位，且不使用前三轴 0.6 rpm 限速。
