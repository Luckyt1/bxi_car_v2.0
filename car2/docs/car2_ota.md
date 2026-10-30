# car2 的 App 启动与 OTA 包

保持现有 `chassis` 控制模块和硬件参数。网关选择 `BXI_CONTROL_BACKEND=car2` 后，Wi-Fi/BLE 的启动调用 `/opt/bxi/car2/scripts/run_app_control.sh`。程序不启动手柄节点，继承网关的 ROS domain 和 DDS 环境；App 控制与导航必须使用同一 domain。

## 目录与配置

| 内容 | 默认路径 |
| --- | --- |
| 可替换程序与包内默认配置 | `/opt/bxi/car2` |
| 实机配置、转向标定、机械臂网卡及零偏 | `/var/lib/bxi/car2/config` |
| 第四轴多圈位置记忆 | `/var/lib/bxi/car2/state/yiyou_motor4_position.yaml` |
| CAN/机械臂日志 | `/var/log/bxi/car2` |
| 控制运行与 OTA 互斥锁 | `/var/lib/bxi/car2/control.lock` |

`scripts/postinstall.sh` 检查 ROS Humble 和已安装二进制的动态库依赖，然后只复制不存在的配置文件。已有配置和状态保留原值；更新包中的默认参数不会自动覆盖实机参数。新增参数请按发布说明合入持久配置。包外目录可用 `deploy/car2-app.env` 中的环境变量修改，启动器与 OTA 管理器必须使用同一个锁文件。四轴模式需要现有可信位置记忆；文件缺失时启动器会在访问硬件前拒绝启动，安装钩子不会伪造零位置。三轴模式不要求第四轴文件。

## 首次部署

这些步骤供部署时执行，本轮离线验证不会自动停止服务或启动电机。

1. 构建非 `--symlink-install` 安装前缀，部署到 `/opt/bxi/car2`。
2. 停止并禁用旧 `ros_elf_launch.service` 等会自动拉起手柄主控的服务；退出开发目录中直接运行的 `chassis`/`robot_control` 等控制程序。App 和旧手柄主控不能同时占用 CAN/EtherCAT。
3. **首次初始化时**从这台机器正在使用的源码目录迁移配置和位置记忆。例如原部署确为 `/opt/bxi_car/car2` 时：

   ```bash
   sudo bash /opt/bxi/car2/scripts/postinstall.sh --migrate-from /opt/bxi_car/car2
   ```

   路径应使用实机实际目录。若之前已初始化持久配置，迁移也不会覆盖它；应自行检查现有文件再合入差异。不要用另一台机器的位置记忆或标定覆盖本机。

4. 首次部署时将 `deploy/car2-app.env` 安装到 `/etc/bxi/control_backend.env`。网关会在包内 `env.conf` 后读取这个持久配置，网关 OTA 不会覆盖它。若该文件已经存在，应合并条目并保留原 DDS 配置；设备密钥仍留在原管理位置。`BXI_CAR2_ENABLE_ARM=1` 启动底盘及机械臂；`0` 仅启动底盘。启动器不改网卡状态，请沿用已经配置好的 EtherCAT 网口。测试可用 `BXI_CONTROL_BACKEND_CONFIG` 替换配置文件路径。
5. 先用下列命令检查将调用的路径和参数，再在实机条件具备时从 App 启动：

   ```bash
   bash /opt/bxi/car2/scripts/run_app_control.sh --dry-run
   ```

`--dry-run` 不创建目录、不加锁、不加载 ROS、不访问硬件。实际启动按当前 `yiyou_ethercat.yaml` 的 `yiyou_arm_home_on_start` 决定是否返回已保存零位；这不是重新设置零点。

## 构建与制包

下面示例只构建、安装到临时目录并制包，不部署或启动控制器：

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B /tmp/car2-release-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/tmp/car2-release-prefix
cmake --build /tmp/car2-release-build --parallel 4
ctest --test-dir /tmp/car2-release-build --output-on-failure
cmake --install /tmp/car2-release-build
python3 scripts/package_ota.py \
  --install-prefix /tmp/car2-release-prefix \
  --output-dir /tmp/car2-releases --version v261030
```

输出 `car2_v261030.tar.gz` 和 `car2_v261030.tar.gz.sha256`，包内根目录为 `car2/`、`car2/.version` 为 `v261030`。版本格式为 `vYYMMDD` 或 `vYYMMDD-N`。制包器拒绝 symlink 安装、不完整安装和同名已存在发布产物，不携带运行日志或位置记忆；保留根许可证、SOEM 来源和本地补丁记录。生产包应在同架构、同 ROS Humble ABI 的环境构建。

使用现有 OTA 发布渠道时，把两个文件放在设备身份对应的 `ELF3/bxi_arm_car2` 目录。仓库 `main` 分支的发布工作流为 x86_64/ROS Humble 构建并上传此目录；安装仍由各台机器人上的 App 发起。网关需要支持并启用 `car2` 后端，才会用 `car2` 替换旧 RL Controller 升级项。arm64 的目录约定是 `ELF3/arm64/bxi_arm_car2`，但需要另行准备匹配架构的 BXI 厂商库并构建，不能使用本仓库当前的 x86_64 包。

## 升级运行约束

App 控制进程持有互斥锁直到退出；OTA 部署持同一把锁，因此先停止 car2 再执行升级。下载阶段可以正常下载，目录替换阶段拒绝运行中的 car2，包括未使用新启动器的旧直接启动进程。升级不会自动上电或启动程序。配置、标定和位置记忆不在包交换范围内；若安装钩子失败，网关使用已有机制回退程序目录。该回退不等于电机或重启健康检查已通过，仍需实机验证启动、停止和机械臂动作。
