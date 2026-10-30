# Gemini 335 双二维码／ArUco 中点识别

本模块独立打开 Gemini 335 相机，识别普通 QR 二维码或 ArUco 标记，标出两个码的中心、连线及中点。
它只输出识别结果，不连接 CAN、电机、机械臂参数服务或运动话题。
QR 与 ArUco 使用不同检测器，启动时须选择对应类型；不需要 Gemini AI API 或云端上传。

## 安装与启动

Python 3.10 环境已验证。依赖单独放在 `requirements-vision.txt`，不影响底盘 C++ 构建。
在项目根目录准备独立环境：

```bash
python3 -m venv .venv-vision
.venv-vision/bin/python -m pip install --no-deps -r requirements-vision.txt
```

文件已列出本模块所需的 NumPy、OpenCV 和相机 SDK。`--no-deps` 避免额外安装
SDK 示例使用的 Open3D、视频播放器和网页组件；本模块不调用那些示例。

通过 USB 连接相机，关闭占用同一相机的 Orbbec Viewer 或其他相机程序，再启动：

```bash
bash start_qr_midpoint.sh
# 无桌面、只输出 JSON；按 Ctrl-C 退出。
bash start_qr_midpoint.sh --headless --json-output /tmp/qr-midpoint.jsonl
# 先只看彩色识别，不启用深度。
bash start_qr_midpoint.sh --color-only
# 画面里有更多二维码时，明确指定两个码的完整文字内容。
bash start_qr_midpoint.sh --qr-texts 'left-marker' 'right-marker'
# 现场黑白方格标记为 ArUco 4×4，选择 ID 0 和 ID 1。
bash start_qr_midpoint.sh --marker-type aruco --marker-ids 0 1
# 使用其他已知字典时显式指定。
bash start_qr_midpoint.sh --marker-type aruco --aruco-dictionary DICT_5X5_100
# 多台相机时指定序列号；分辨率和帧率必须是设备支持的组合。
bash start_qr_midpoint.sh --serial YOUR_SERIAL --width 640 --height 480 --fps 30
```

脚本优先使用项目的 `.venv-vision/bin/python`，否则使用 `python3`；
`CHASSIS_VISION_PYTHON` 可指定已有解释器。窗口中 `q` 或 Esc 退出；
没有桌面环境时自动以无窗口方式运行。无需启动 `start.sh` 或整车服务。
Linux 设备权限应按 [Orbbec 官方安装说明](https://github.com/orbbec/pyorbbecsdk/tree/v2-main)
配置 udev 规则；脚本不自动提权。

ArUco 模式默认使用 `DICT_4X4_50`，预览显示实际 ID。标记的完整黑色边框及四角必须可见，
避免被金属支架遮挡或与暗色背景连成一片。现场采样中的右侧 ID 1 上边框受到支架遮挡，
即使内部格子可见，也不能据此猜测可靠角点或输出双码中点。
每格透视采样采用 8 像素，改善该现场图中的 ID 0 解码；保留默认纠错限制。

可打印模板：[ArUco 4×4 ID 0／1，A4 PDF](print/aruco_4x4_50_id0_id1_A4.pdf)。
两页分别提供 40 mm、60 mm 黑色码框，四周各留 10 mm 白边。选择一页按“实际大小／100%”
打印，关闭适应页面；用纸上的 50 mm 标尺检查缩放。沿虚线裁下后保留白边，画面中只放一对
ID 0／1，避免同时使用两页产生重复 ID。

也可直接运行源码，或构建安装后通过 ROS 的程序查找入口运行：

```bash
.venv-vision/bin/python -B src/example/qr_midpoint.py --headless
# 使用已安装视觉依赖的 Python 环境：
source .venv-vision/bin/activate
source install/setup.bash
ros2 run chassis qr_midpoint.py --headless
```

`ros2 run` 仅用于定位程序；本阶段没有新增 ROS 发布节点或机械臂控制接口。

## 离线识别

```bash
bash start_qr_midpoint.sh --image /path/to/two_codes.png --headless \
  --output /tmp/qr-annotated.png --json-output /tmp/qr-result.jsonl
```

该模式不导入相机 SDK，也不打开设备。普通图片没有深度，只提供像素中点。
`--output` 保存带标记的图片；`--json-output` 覆盖指定文件，逐帧写入 JSON。
终端默认最多每秒输出五次，识别有效性变化时立即输出；文件记录每一帧结果。
SDK 还可能向终端输出初始化信息，其他程序读取结果时使用 `--json-output` 的 JSONL 文件。

## 目标选择与坐标

- 单个二维码的中心取四角对角线交点，避免透视下直接平均角点产生偏差。
- 默认只有恰好两个有效二维码时输出目标；零个、一个或多于两个均返回无效。
- 普通 QR 码框可定位但内容无法解码时，默认模式仍可使用；指定 `--qr-texts` 时必须成功解码，
  且两个内容分别唯一匹配。两个二维码可以有相同内容，但这时不能按文字唯一选择。
- ArUco 模式要求字典解码成功，`--marker-ids` 按两个唯一 ID 选码；目标缺失或同一目标 ID
  出现多次时无效。JSON 中每个 ArUco 标记另有 `id`、`type`、`dictionary`，`text` 为 ID 字符串。
- `midpoint_px` 是两个码中心像素坐标的算术平均，原点在图像左上角，u 向右、v 向下。
- 深度通过 SDK 显式对齐到彩色图，使用每帧深度比例换算为米。
  分别在两个二维码中心附近取有效深度，结合当前彩色内参及畸变参数反投影，
  再对两个三维点求平均，得到 `position_3d.midpoint_m`。
- 三维点在 `gemini_color_optical_frame`：X 向右、Y 向下、Z 向前，单位米。
  这是程序声明的相机坐标名称，不代表已发布 ROS TF。
- **三维连线中点与像素中点处的表面深度不是一回事**。两码之间可能是远处背景；
  本模块不把那个背景点作为目标。两码距离相机不同时，三维中点的投影也可能不在
  `midpoint_px` 上。
- 深度缺失、未正确对齐或标定无效时，二维识别可以有效，但三维结果明确无效。
  每帧重新检测，不复用丢失前的目标；相机超时、错误及会话结束输出无效记录。

JSON 主要字段：

| 字段 | 含义 |
| --- | --- |
| `valid` / `status` | 当前帧是否选中有效二维码对及原因 |
| `codes` | 所有码的内容、四角 `corners_px` 与中心 `center_px` |
| `pair_indices` | 选中的两个码在 `codes` 中的下标 |
| `midpoint_px` | 二维中点；无有效码对时为 `null` |
| `position_3d.valid` / `status` | 三维结果是否可用及原因 |
| `position_3d.centers_m` / `midpoint_m` | 两码中心及中点的相机坐标；无效时为 `null` |
| `timestamp_unix_s` | 本机处理该帧的时间；不是硬件曝光时间 |
| `device_timestamp_ms` | 相机帧设备时间；离线图片为 `null` |

机械臂高度跟踪使用单独的 `start_arm_tracking.sh` 入口，默认也只预览，
详见 [末端视觉高度跟踪](arm_height_tracking.md)。该入口先应用相机外参，
再计算机器人竖直向下 7.5 cm 的目标，并通过有时效的控制接口更新 Z。
当前机械臂前三轴仅支持平面 `x/z/pitch`，不能把相机三维坐标直接写入
`yiyou_arm_target`；本文件的识别入口仍不发送运动指令。

## 文件与验证

- `src/thirdparty/vision/qr_midpoint.py`：检测、选择码对、像素几何和标注。
- `src/thirdparty/vision/gemini_camera.py`：SDK 取帧、深度对齐及三维中点。
- `src/example/qr_midpoint.py`：命令行与预览入口。
- `test/unit/test_qr_midpoint.py`：真实生成 QR／ArUco 标记和几何异常回归。
- `test/unit/test_gemini_camera.py`：伪 SDK、深度单位及三维计算测试。
- `test/unit/test_qr_midpoint_cli.py`：离线程序、安装布局及相机退出测试。

```bash
.venv-vision/bin/python -B test/unit/test_qr_midpoint.py
.venv-vision/bin/python -B test/unit/test_gemini_camera.py
.venv-vision/bin/python -B test/unit/test_qr_midpoint_cli.py
```

CMake 配置时若所选 Python 能导入 OpenCV 和 NumPy，会将这三组测试加入 CTest。
SDK 和相机不是离线测试的前提；实机图像质量、USB 流配置和测距精度仍须现场验证。

使用的相机接口与模型依据：
[Orbbec Python SDK v2 应用指南](https://orbbec.github.io/pyorbbecsdk/source/5_Application_Guide/ApplicationGuide.html)、
[Gemini330 系列彩色标定模型](https://github.com/orbbec/OrbbecSDK_v2/blob/main/src/device/gemini330/G330AlgParamManager.cpp)。
标记检测接口：[OpenCV 4.10 ArucoDetector](https://docs.opencv.org/4.10.0/d2/d1a/classcv_1_1aruco_1_1ArucoDetector.html)。
