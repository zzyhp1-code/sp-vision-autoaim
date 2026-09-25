# SP-Vision 中文调试菜单

## 本次代码审查结论

本次已修复下列可静态复现的调用与配置安全问题，但尚未在现场小电脑做硬件验收。
下文功能说明是设计说明；相机、标定和识别仍需在目标机完成独立测试。

已用隔离文本和模拟函数复现的问题：

- **已修复：命令交接。** 所有动作将 `REGISTRY_COMMAND` 数组显式传给
  `process_run_managed`，不再依赖全局命令的隐式读取。
- **已修复：识别配置返回变量。** 临时配置路径使用独立局部变量，避免 Bash
  动态作用域遮蔽调用者变量。
- **已修复：菜单会话隔离。** 进程记录文件包含随机会话标识；退出只读取当前
  会话记录，不再使用共享 `current_process`。
- **已修复：YAML 严格解析。** 格式错误会直接失败；PyYAML 使用 `safe_load`，
  文本扫描不再参与配置读取、运行时配置或验证。
- **已修复：提示与返回值分离。** 交互提示写入 stderr，命令替换只获取路径或数值。
- **已修复：接口参数逐项校验。** `--help` 输出必须包含每个所需参数，且实际启动
  前会执行相应接口校验。
- **已修复：试运行写保护。** `--dry-run` 下正式 YAML 写入函数直接返回，不落盘。
- **已修复：双配置写入恢复。** 内参和外参写入在同一目录中预生成候选文件与临时
  回滚文件；任一步失败会恢复两份配置，成功后删除全部临时文件。

静态审查还发现以下边界问题，尚未做真实硬件或 Linux 进程验证：

- `setsid` 的进程组建立和组长退出后的后代进程状态仍需在 Ubuntu 目标机验证。
- 日志跟随已改为变量回传 PID，不再把 `tail -f` 输出混入命令替换；其交互行为仍需
  在真实终端验证。
- 日志读取已覆盖 `logs/` 子目录；日志清理在缺少 `lsof` 时仍不能保证跳过活动日志，
  且需验证日志根本身未越出工程。
- 原测试套件仍使用真实工程的构建目录；现场构建产物存在时，应在隔离夹具中运行测试，
  不应把整套测试当作无副作用检查。

补充的中文注释覆盖模块职责、命令数组交接、stdout 返回约定、进程所有权、
配置写入边界和测试隔离要求。语法检查或“菜单能打开”均不能代替目标机功能与安全验收。

## 入口

```bash
cd /home/rm/sp_vision
bash scripts/sp_vision_debug.sh
```

支持的启动选项：

```bash
bash scripts/sp_vision_debug.sh --help
bash scripts/sp_vision_debug.sh --dry-run
bash scripts/sp_vision_debug.sh --check
```

运行环境需要 Bash 4+、`flock`、`setsid`、`ps`、`realpath`、`mktemp` 和可用的
Python 3。Python 只需调用菜单自带的 `yaml_tool.py`，不会进入业务算法。

入口根据脚本自身位置解析项目根目录，因此也可以从任意当前工作目录执行：

```bash
bash /home/rm/sp_vision/scripts/sp_vision_debug.sh
```

普通启动会先汇总根目录、配置、`build/`、Python、ROS、相机、模型、图形会话和已声明入口
的可用性；这些预检只读取状态，不打开相机。具体动作执行前仍会重新检查对应接口。

## 分层

```text
scripts/sp_vision_debug.sh
        |
        +-- debug_menu/menu.sh          中文菜单和稳定的编号
        +-- debug_menu/registry.sh      真实程序、参数、目标、配置字段
        +-- debug_menu/*.{sh}           内参、外参、识别、相机、日志适配
        +-- debug_menu/process.sh       受管进程 PID/PGID/starttime 和相机锁
        +-- debug_menu/common.sh        根目录、确认、日志路径、YAML 调用
        +-- debug_menu/yaml_tool.py     结构化 YAML 和日志结果辅助
```

菜单编号只调用 `registry_invoke_*`，再由注册表调用语义化 `action_*` 函数。可执行文件
路径、参数、CMake 目标、模型字段只在 `registry.sh` 和对应适配文件中定义。

## 已接入的现有入口

| 功能 | 现有入口 |
| --- | --- |
| 相机预览 | `build/camera_test -c=<config> -d` |
| 实时识别测试 | `build/camera_pnp_test <config> [--tradition]` |
| 单张图片识别 | `build/camera_detect_test <config> <image> [--tradition]` |
| 离线视频识别 | `build/detector_video_test -c=<config> -s=<start> -e=<end> <video>` |
| 内参采集 | `build/capture -c=<calibration-config> -o=<data-directory>` |
| 内参计算 | `build/calibrate_camera -c=<config> --no-display <input>` |
| 手眼计算 | `build/calibrate_handeye -c=<config> --no-display <input>` |
| 外参/手眼验证 | `build/verify_handeye -c=<config> -e=<standard-config> -i=<input>` |
| 构建 | `bash scripts/build.sh <explicit-target...>` |

`camera_pnp_test` 和 `camera_detect_test` 是专用测试程序，不会调用 `standard`，
不会发送串口控制，也不会控制云台或发射。

## 安全运行配置

相机、识别和标定运行项不会把 `configs/standard.yaml` 直接交给程序。适配层先用
`yaml_tool.py make-runtime` 创建系统临时目录中的运行缓存，并强制写入：

```yaml
xuc_enable: false
xuc_allow_control: false
xuc_allow_shoot: false
```

临时 YAML 在菜单退出时删除，不生成 final、copy、backup 或日期配置副本。
正式配置只有在结构指标通过或用户明确二次确认“谨慎使用”后才会写入，并且必须
保留 YAML 注释、显示字段差异、采用同目录临时文件原子替换、替换后重新解析。
当前目标机没有 `ruamel.yaml` 时，写入会停止，不会退化为字符串替换。

## 判定规则

内参计算程序当前只输出有效图像数、分辨率、平均重投影误差、相机矩阵和畸变参数。
菜单不会只用 RMS 放行；缺少覆盖范围、远近/倾斜姿态变化、单张最大误差、异常样本、
收敛和主点合理性等指标时，统一显示“综合判定：无法判断”。

手眼计算程序当前不输出残差。菜单会继续调用 `verify_handeye` 汇总运动范围、板世界系
平移/旋转残差、AX=XB 残差和重投影误差。即使结构指标合格，也只能显示
“计算一致性：通过，实际安装精度：无法判断”，并要求写入前再次输入“谨慎写入”。

## 当前版本不可用

- YOLO/传统算法同帧对比：现有程序只有单一算法参数，没有稳定的对比结果接口。
- 手眼标定采集：安全策略强制 `xuc_enable=false`，现有 `capture` 在该模式下无法获得
  手眼所需姿态；这是安全策略与旧入口能力的冲突，菜单不会放宽开关绕过。
- 保存识别画面、原始帧、显示帧：现有测试程序没有稳定命令行保存参数或控制接口。
- `calibrate_robotworld_handeye`：命令已在注册表登记并在 `--check` 中检查，但它无条件打开
  `imshow` 窗口，当前菜单的离线手眼计算使用 `calibrate_handeye` 加 `verify_handeye` 的
  非显示路径，避免在无图形会话时误启动窗口。
- 发射口 Z 轴：现有配置只有
  `xuc_camera_forward_of_muzzle_m` 和 `xuc_camera_above_muzzle_m`，没有右侧 Z 字段。
- 云台控制、完整自瞄、发射、固件烧录、串口协议修改、系统依赖安装：按项目要求不接入。

这些项目在菜单中会显示“当前版本不可用”，并指示维护人员只更新命令适配层。

## 维护方式

当业务程序改名或参数变化时，优先修改 `registry.sh` 中对应的
`registry_binary`、`registry_command_*` 或 `registry_build_targets`。不要在其他脚本
复制可执行文件路径，不要在菜单里猜测相似命令。接口检查失败时必须显式停止。

## 检查

```bash
bash -n scripts/sp_vision_debug.sh scripts/debug_menu/*.sh scripts/debug_menu/tests/*.sh
shellcheck scripts/sp_vision_debug.sh scripts/debug_menu/*.sh scripts/debug_menu/tests/*.sh
python3 scripts/debug_menu/yaml_tool.py --help
python3 scripts/check_source_tree.py
bash scripts/debug_menu/tests/test_debug_menu.sh
```

测试脚本会自动查找可用的 Python 3；也可以在特殊环境中显式传入：

```bash
SP_VISION_TEST_PYTHON=/usr/bin/python3 bash scripts/debug_menu/tests/test_debug_menu.sh
```

完整硬件测试仍应在目标小电脑上执行，至少覆盖相机占用、Ctrl+C、连续启动/停止、
内外参写入确认和日志清理边界。
