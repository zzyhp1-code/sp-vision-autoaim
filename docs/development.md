# 构建与维护

小电脑工程：`/home/rm/sp_vision`；Windows 仓库：`Documents/GitHub/sp-vision-2026-integration-archive`。

## 构建

新克隆先按 [依赖说明](dependencies.md) 准备模型与SDK；现有小电脑资源未删除。

在工程根目录执行 `bash scripts/build.sh`，默认以2个任务构建 `standard`。
可用 `bash scripts/build.sh standard calibrate_camera calibrate_handeye verify_handeye` 选择目标。
依赖沿用现场 ROS 2 Humble、OpenVINO 2024.6、OpenCV、Eigen、Ceres、fmt、spdlog、yaml-cpp 与 MVS 环境。`librm/` 使用固定提交的 Git 子模块；新克隆使用 `git clone --recurse-submodules`，已有克隆使用 `git submodule update --init --recursive`。小电脑依赖完整保留，不依赖旧 worktree。

编译产物只放在 `build/`。不要为每轮修改新建构建目录、复制配置或源码文件。修改前后使用 `git status`、`git diff` 核对，不创建备份副本。

## 运行入口

现场沿用桌面“启动自瞄”和“停止自瞄”，分别指向 `tools/field_start_autoaim.sh` 和 `tools/field_stop_autoaim.sh`。脚本相对自身定位工程，不再写死旧目录。
不启用新的开机自启动。运行日志位于 `logs/xuc-bringup/日期/`，跨日期、跨颜色保留最近20份一键日志；这不是单文件大小限制。

## Git

只维护当前文件。本次按用户要求将仓库重建为单次提交，删除旧标签、旧提交及本地历史备份。以后继续在同一目录修改；提交和推送按用户要求进行，不另建归档分支、标签或副本。

## 提交前检查

运行 `python3 scripts/check_source_tree.py` 检查跟踪文件和忽略规则，再执行 `git diff --check`。`.gitignore` 不会自动移除已经跟踪的文件，产物必须从索引中取消跟踪。

## 维护待办与整改计划（2026-10-10）

本节记录代码审查后的安排与整改进度。仅涉及构建、诊断工具与配置文件写入；现场验证由用户执行。现有未提交改动须保留，修改前重新核对 Git 状态。

部署与验收以 Linux 为准。仅在 Windows GBK 输出环境复现的状态符号编码问题不纳入整改清单；当前未在 Linux 复现该问题，也不将这一点写成 Linux 实测通过。

### 暂缓：与电控协调

- [ ] **下位机 CRC 校验与失败处理。** 用户确认当前运行固件忽略部分校验失败，回传姿态在现有观察中正确；现阶段暂缓下位机 CRC 整改，后续与电控共同确认校验约定、失败处理和实际有效帧统计。不能仅凭角度合理推定每一帧均有效，也不能将本地修复当成已经烧录。已有 `imu_crc.patch` 保留供后续讨论，本计划不应用或撤销固件修改。
- [ ] **IMU 回传参数单位为度（deg）。** 用户已确认当前 yaw/pitch 回传单位；后续与电控统一协议说明、字段注释和版本记录。当前上位机接收配置保持 `xuc_imu_angle_unit: deg`，由接收层转换一次为弧度。回传单位的确认不代表目标指令单位也为度，不据此修改发送单位。

### 第一项：修复主机测试构建入口

- [ ] **状态：代码已整改，本机隔离 CMake/CTest 通过；Linux 验收待补；优先级：P1。**
- 问题：`firmware/infantry/tests/CMakeLists.txt` 中 `project(infantry_host_tests)` 只声明项目，并不创建 target；随后对同名 target 调用 `target_include_directories`，实际在 CMake 配置阶段报错。
- 影响：文档中的主机侧测试构建命令无法运行，无法据此认定测试已经通过。
- 整改范围：只改主机测试的 `CMakeLists.txt`。当前仅一个测试目标，直接对 `hipnuc_crc_test` 设置 include 目录即可；若以后确有多个目标共享依赖，再创建单独的 INTERFACE library。补齐 CTest 注册，使检查可以实际执行测试而非只编译。
- 验收：在系统临时目录完成独立 CMake 配置、编译和 `ctest --output-on-failure`；确认至少一个测试被发现并通过。验证不连接硬件、不生成 STM32 镜像；通过后清理本轮临时构建目录。

### 第二项：保留配置写入失败时的恢复依据

- [ ] **状态：代码已整改，本机隔离故障注入通过；Linux 验收待补；优先级：P2。**
- 问题：`scripts/debug_menu/yaml_tool.py` 的双文件写入函数在回滚再次失败时，仍会在 `finally` 删除未恢复的 `.rollback` 文件。此项来自代码检查，尚未对正式配置注入故障。
- 影响：磁盘、权限等错误发生时，配置可能未恢复，原始内容的恢复文件也被删除。
- 整改范围：只调整双文件写入的错误处理和临时文件清理。分别记录两个目标的替换及恢复结果；某个目标恢复失败时仍尝试恢复另一个；保留未成功恢复的原始内容，并在错误信息中列出目标和恢复文件路径。避免回滚异常遮盖最初的写入异常。
- 验收：用系统临时目录中的两个简单 YAML 文件隔离测试正常写入、第二次替换失败、任一回滚失败。正常完成时无临时文件；能回滚时两份原文恢复；不能回滚时恢复文件保留且错误清楚，不触碰正式配置。

### 第三项：修正诊断脚本的失败退出码

- [ ] **状态：代码已整改，本机隔离命令回归通过；Linux 验收待补；优先级：P2；无需等待电控。**
- 问题：`scripts/analysis/hipnuc_crc_check.py` 校验不一致时仅打印结果，`main()` 没有返回失败状态，命令最终仍以退出码 0 结束。该逻辑与操作系统无关。
- 影响：Linux 上的 Shell、自动检查或 CI 无法根据退出码判断诊断失败，可能将不一致的样本当作通过。
- 整改范围：只修改诊断脚本的结果返回和命令入口；一致返回 0，不一致返回非零，并对缺少参数、样本不存在、样本格式错误和帧长度不完整给出明确失败。保留 CRC 计算约定与现有输出内容，不改固件行为。
- 验收：在 Linux 用隔离样本验证正确帧退出码为 0，错误 CRC、缺参、错误格式和不完整帧退出码均非零；完整读取标准输出和错误输出，确认无运行异常。无需连接硬件。

### 执行顺序与完成口径

三项代码已整改；仅下位机 CRC 与回传单位事项等待电控协调。每项独立核对 diff 和验证结果，Linux 验收通过后再勾选完成，不以“代码已改”代替“验收通过”。共享交接日志未修改，未提交或推送。

本轮验证（2026-10-10）：新增 `scripts/debug_menu/tests/test_maintenance.py` 用临时样本覆盖 7 种诊断命令输入，以及正常写入、写入失败、单份/双份回滚失败共 5 种双配置写入情况，本机全部通过。修复后的 CMake 独立配置、编译和 CTest 1/1 通过；本机 MinGW 中文路径限制通过临时目录中的未改动源码副本绕开。源码卫生检查与 diff 格式检查通过。测试依赖、构建与配置样本均位于系统临时目录，验证后删除。

Linux 验收命令：`python3 scripts/debug_menu/tests/test_maintenance.py -v`（沿用菜单正式写入所需的 `ruamel.yaml` 依赖），另在临时构建目录配置 `firmware/infantry/tests` 并运行 CTest。本轮两次连接已记录的小电脑 `rm@192.168.161.183` 均在 SSH 密钥交换前被远端关闭，未改远端文件，因此尚未确认 Linux 实际运行结果。
