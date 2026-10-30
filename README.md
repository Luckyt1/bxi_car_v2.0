# BXI Car v2.0

ROS 2 Humble 底盘与机械臂控制项目。源码、配置模板、离线测试和设备启动工具位于
[`car2/`](car2/README.md)。实机标定、位置记忆、日志和发布凭据不随仓库分发。

## main 自动发布

推送到 `main` 后，GitHub Actions 自动执行：

1. 使用 Ubuntu 22.04 / ROS Humble 环境编译 x86_64 程序。
2. 运行控制器离线测试和发布器测试。
3. 生成 `car2_vYYMMDD-N.tar.gz` 及 SHA-256 校验文件，并保留 Actions 下载产物。
4. 将安装包上传并校验成功后，最后发布校验文件。

公开下载目录：<https://download.bxirobotics.cn/ELF3/bxi_arm_car2>。
工作流文件是 [`.github/workflows/ota.yml`](.github/workflows/ota.yml)，账号权限、
GitHub Secrets、版本号和失败处理见 [发布说明](docs/ota-publishing.md)。
Pull request 运行构建验证；发布步骤仅在 `main` 执行。

机器人需要使用支持 `car2` 的 App 网关，并将 OTA 目录配置为上述发布目录。
包仍安装到 `/opt/bxi/car2`，保留 `/var/lib/bxi/car2` 中的本机配置和标定，
升级不会自动启动电机。当前流水线不生成 ARM64 安装包。

## 本地构建与测试

```bash
source /opt/ros/humble/setup.bash
cmake -S car2 -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s scripts -p 'test_publish_ota.py' -v
```

设备接入见 [App 说明](car2/docs/app_integration.md)，制包及持久配置约定见
[OTA 说明](car2/docs/car2_ota.md)。依赖来源和许可范围见
[Dependencies And Provenance](car2/docs/dependencies.md)。
