# GitHub 自动发布 car2 OTA

`main` 的 push 自动执行 `.github/workflows/ota.yml`：在 Ubuntu 22.04 / ROS Humble
容器里编译 x86_64 程序、运行离线测试、安装到独立前缀、生成安装包，然后发布到
`https://download.bxirobotics.cn/ELF3/bxi_arm_car2/`。Pull request 也构建、测试和制包，但不会
读取发布凭据或上传到下载站。可在 Actions 页面选择 main 手动运行。

云端安装 `python3-matplotlib`，使无条件执行的力矩报告 PNG 测试有完整依赖。
标准 ROS 镜像运行 37 项主控离线回归；另外 7 项视觉测试仅在 Python 可导入兼容的
OpenCV 和 NumPy 时启用，云端默认跳过这些可选测试。

包文件为 `car2_vYYMMDD-N.tar.gz` 和同名 `.sha256`；包内根目录是 `car2/`。
这里的日期使用 UTC 构建日期，`N = GITHUB_RUN_NUMBER × 1000 + GITHUB_RUN_ATTEMPT`。
例如第 12 次工作流的首次执行为 `v261030-12001`，重跑为 `v261030-12002`。
重跑会生成新版本，避免重新编译产生不同二进制却覆盖旧发布；同版本同内容的手工重传仍可幂等。

发布在同一 main 工作流内串行执行。上传前再次检查 main HEAD，已经被新提交取代的
旧任务只保留 Actions artifact，不发布旧代码。仓库只需要 `contents: read` 权限，
安装包和测试失败日志保留 30 天。

## 一次性站点和仓库配置

1. 站点管理员创建 `/ELF3/bxi_arm_car2`，允许机器人匿名列目录和下载。
2. 使用具备该目录列目录和写入权限的 OpenList 账号，并允许用户名/密码自动登录。
   当前使用已有 `tang` 账号。后续可选择基本路径限定为 `/ELF3/bxi_arm_car2` 的专用账号，
   这是缩小凭据权限的建议，并非运行流水线的前提；发布器不要求站点管理或删除权限。
3. 在 GitHub 仓库 **Settings → Secrets and variables → Actions → Repository secrets**
   设置 `OTA_USERNAME` 和 `OTA_PASSWORD`。凭据不写进源码或工作流 YAML。

发布器每次调用 `/api/auth/login` 取得短期 token，不使用会过期的浏览器登录 token。
它通过 `/api/me` 读取账号的 `base_path`：公开路径 `/ELF3/bxi_arm_car2` 对受限账号会转换为
API 目录 `/`，避免将账号基本路径重复拼接。管理员基本路径为 `/` 时，仍使用完整目录。

## 发布完成与失败处理

发布器先校验本地 SHA-256，并使用与机器人完全相同的匿名目录请求确认目录可访问，
再同步上传安装包。上传后使用**无登录凭据的公开下载接口**读取整个文件，验证大小和
SHA-256，并确认匿名目录能列出该包；成功后才上传 `.sha256`。最后验证校验文件，并确认
同一次匿名目录列表同时包含安装包和校验文件。访客无法列目录或文件被隐藏时，发布失败；
匿名目录检查不携带需要额外权限的 `refresh` 参数。
现有机器人目录发现逻辑只显示同时存在安装包与校验文件的版本，因此安装包上传中不会
成为可安装版本。上传、鉴权或公开下载校验失败会使工作流失败。

同名文件已经存在时，发布器验证其内容；相同则跳过，不同则失败。上传请求设置
`Overwrite: false`，不会覆盖已发布包。安装包上传失败或未通过校验时，不会发布校验文件。
下载站 API 不跟随重定向；公开下载只允许 HTTPS，且不会向存储服务或 CDN 传递发布 token。

版本包更新 `/opt/bxi/car2`，实机配置、标定和位置记忆仍留在 `/var/lib/bxi/car2`。
使用 car2 后端的兼容机器人才能识别此发布目录；更新不会自动启动电机。
该流程的测试使用模拟通信，实际机械运动仍需在对应机型上验证。

手工发布已经构建的同一对文件时，先在环境中设置上述两个 Secrets 对应的变量：

```bash
python3 scripts/publish_ota.py \
  --archive dist/car2_v261030-12001.tar.gz \
  --checksum dist/car2_v261030-12001.tar.gz.sha256
```

参考：[OpenList 流式上传实现](https://github.com/OpenListTeam/OpenList/blob/main/server/handles/fsup.go)、
[OpenList 路径约束](https://github.com/OpenListTeam/OpenList/blob/main/pkg/utils/path.go)、
[GitHub Actions Secrets](https://docs.github.com/en/actions/security-for-github-actions/security-guides/using-secrets-in-github-actions)、
[官方 ROS 容器](https://hub.docker.com/_/ros)。
