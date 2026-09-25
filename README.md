# SP-Vision 自瞄与标定调试工程

本仓库维护本机修改后的 RoboMaster 视觉自瞄源码、相机适配、标定工具及中文交互式调试脚本。个人维护仓库为 `zzyhp1-code/sp-vision-autoaim`，设置为私密。上游项目为 [sp-vision-2026-integration-archive](https://github.com/kukuaxing/sp-vision-2026-integration-archive)，保留原有许可证和来源信息。

## 目录分类

| 分类 | 路径 | 内容 |
| --- | --- | --- |
| 程序入口 | `src/` | 正式入口 `standard` 及其他应用入口 |
| 视觉算法 | `tasks/` | 装甲板检测、跟踪、预测、瞄准及其他视觉任务 |
| 设备接口 | `io/` | 相机、串口、ROS 2 与控制板接口 |
| 标定源码 | `calibration/` | 图像采集、内参、手眼标定及验证 |
| 测试源码 | `tests/` | 相机、装甲板识别与模块测试 |
| 调试与构建脚本 | `scripts/` | 中文菜单、构建与资源检查；菜单模块位于 `scripts/debug_menu/` |
| 公共工具 | `tools/` | C++ 公共工具及现有现场启停脚本 |
| 正式配置 | `configs/` | `standard.yaml` 与 `calibration.yaml` |
| 文档 | `docs/` | 部署依赖、配置、调试说明与维护状态 |
| 固件 | `firmware/` | 步兵、哨兵硬件对应的源码 |
| 仿真 | `simulation/` | 仿真源码、配置与已有结果报告 |
| 外部资源 | `assets/`、`librm/` | 模型放置说明、固定版本的源码子模块 |

`build/`（构建）、`logs/`（日志）、`data/`（采集数据）仅在本地生成，不提交。目录沿用当前 CMake 和脚本约定，避免移动源码造成路径失效。

## 部署准备

目标环境为 Linux 小电脑；现有环境使用 Ubuntu 22.04、ROS 2 Humble、OpenCV、OpenVINO 2024.6、Eigen、Ceres、fmt、spdlog、yaml-cpp 和厂商相机 SDK。Windows 工作区用于维护源码，不能直接代替 Linux 硬件验证。

在具有本私密仓库访问权限的账号下克隆：

```bash
git clone --recurse-submodules https://github.com/zzyhp1-code/sp-vision-autoaim.git /home/rm/sp_vision
cd /home/rm/sp_vision
```

目标目录须不存在；已有工程应在原目录检查改动后更新。先按 [依赖说明](docs/dependencies.md) 安装依赖，再按 [资源清单](docs/artifacts.json) 准备模型和 SDK 库。模型权重、厂商动态库、固件镜像不会随源码仓库下载。

```bash
python3 scripts/check_runtime_assets.py
bash scripts/build.sh standard camera_test camera_detect_test camera_pnp_test capture calibrate_camera calibrate_handeye verify_handeye
```

菜单使用 Bash、Python 3 和 Linux 进程管理工具；正式 YAML 写入需要 Python 包 `ruamel.yaml`，建议在所用 Python 环境安装 `ruamel.yaml` 与 `PyYAML`。详细说明见 [调试菜单文档](docs/debug_menu.md)。

## 中文调试菜单

在小电脑图形会话终端、项目根目录运行：

```bash
bash scripts/sp_vision_debug.sh --help
bash scripts/sp_vision_debug.sh --dry-run
bash scripts/sp_vision_debug.sh --check
bash scripts/sp_vision_debug.sh
```

菜单提供内参标定、外参标定、装甲板识别、相机预览、配置、环境检查、编译、日志及进程管理。内参下包含采集、计算、检查和写入；外参下包含发射口坐标和手眼标定。已有内参保留在配置中，迁移计算机不意味着必须重测，但更换相机、镜头或分辨率后须重新确认适用性。

菜单通过 `scripts/debug_menu/registry.sh` 适配已有程序。后续修改程序参数时，应同步修改适配层和接口检查。菜单存在需在 Linux 上继续验证的进程退出及配置写入边界，详见调试文档；本次入库不代表完成新一轮硬件验收。

## 本机修改内容

- 保留海康 Bayer 解码红蓝通道修复；OpenCV 的 Bayer 枚举命名不能直接当作物理输出通道定义，说明见相机实现及维护记录。
- 纳入当前相机适配、构建依赖和现场启动脚本改动。
- 纳入中文二级调试菜单、标定结果检查与配置处理辅助脚本。
- 完善源码、配置、脚本、文档分类；排除日志、缓存、录像、采集数据及本地凭据。

## 维护与检查

```bash
python3 scripts/check_source_tree.py
git diff --check
```

提交使用能识别用途的说明。日志处理脚本可以入库，脚本生成的日志不能入库。不要使用 `git add -f` 绕过忽略规则上传运行文件或密钥。

参阅 [开发维护](docs/development.md)、[配置说明](docs/configuration.md)、[状态记录](docs/status.md)、[私密仓库说明](docs/repository.md) 和 [项目约定](AGENTS.md)。授权条款见 [LICENSE](LICENSE)。
