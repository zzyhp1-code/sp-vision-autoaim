# 当前状态

## 2026-09-19 装甲板识别测试检查点

- SSH 连接已验证：`rcsdart@10.27.194.180`，正式工程目录为 `/home/rcsdart/sp_vision`。
- 小电脑在线，Hikrobot `MV-CS060-10UC-PRO` 已通过 USB 枚举，工程、`standard` 和 `camera_detect_test` 均存在。
- 内参已回填到远端 `configs/calibration.yaml` 与 `configs/standard.yaml`：
  - `camera_matrix: [2573.5701960975634, 0, 1303.6216672714954, 0, 2581.2583643553789, 1028.079874685799, 0, 0, 1]`
  - `distort_coeffs: [-0.037261897288824064, 0.19093271319153729, 0.0033120796944931682, -0.015036623214563031, 0]`
- 已从 Windows 的 `Documents/GitHub/TJ-advanced` 复制并校验以下运行资源到远端：
  - `assets/tiny_resnet.onnx`
  - `assets/yolo26.xml`
  - `assets/yolo26.bin`
  - Hikrobot 与 MindVision 的 amd64/arm64 SDK 动态库
- 远端 `python3 scripts/check_runtime_assets.py` 已通过：`Verified 7 current runtime resources`。
- 远端当前有图形会话：`/tmp/.X11-unix/X0` 和 `/tmp/.X11-unix/X1003`，桌面为 GNOME/Xorg。
- 首次 6 秒相机取流测试因 SSH 内嵌 shell 的转义错误，在程序启动前失败；这不是相机或程序故障，尚未得到相机取流结论。

### 明日继续

1. 以正确转义运行 `build/camera_test -c configs/standard.yaml`，确认 Hikrobot 连续取流和帧率。
2. 在 `DISPLAY=:0` 或当前登录会话 `:1003` 上运行不含云台/发射控制的识别测试。
3. 不要用 `build/standard` 做装甲板识别测试：远端正式配置当前含 `xuc_allow_control: true` 和 `xuc_allow_shoot: true`。

视觉以原小电脑9月12日的现场源码和参数为基础。已包含红蓝识别增强、颜色持久化、整车跟踪、SO(2) Yaw ESKF、运动模型切换、装甲面关联、相位事件调度及反馈时延记录。不是完整 SO(3) 刚体 ESKF 或云台 MPC。

最近一次现场阶段已转入哨兵。哨兵固件完成协议适配和 P 轴首次反馈保持修复；P 轴方向、正式机械限位和整机联调仍未完成。9月16日已换回原小电脑，准备重新标定。

## 尚未闭环

- 新安装内外参/手眼标定尚未完成，整理不应用手工测量值。
- 19.36 m/s 是历史估算弹速，不是测速反馈。
- 时延自适应观测拨弹启动，不是实际出膛；不得把预测区间当作整体命中率保证。
- 最新横向修正未完成成组现场验收。
- 旧步兵协议遥测复用了弹速/弹数字段，不能与哨兵 SV 状态帧混解。
- 本次只做文件整理、构建及离线检查，现场性能仍以用户后续实测为准。
