# 配置与标定状态

只编辑 `configs/standard.yaml`。启动器读取它并生成 `logs/xuc-bringup/field_autoaim.runtime.yaml`，仅覆盖已保存的敌方颜色。不要修改这个生成文件；下次启动会重建。

颜色保存在 `~/.config/sp-vision/field_autoaim_enemy_color`。标定工具使用 `configs/calibration.yaml`，其历史外参与正式运行配置并不相同；二者用途不同，不能互相覆盖。

用户最新安装测量：机器人中心为原点，X前、Y左、Z上，右手系；光心(15,0,50) cm，机身(10,0,50) cm，炮口(10,0,45) cm。测量记录尚未应用。代码中的相机、云台、IMU坐标原点和轴定义仍需核对，再完成标定。

标定源文件在 `calibration/`；原始采集资料集中保留于小电脑 `data/`。旧实验配置和旧报告已清除，工作区不保留候选参数副本。

## XUC IMU 回传单位

当前连接的下位机由用户于 2026-10-09 确认以度回传 yaw/pitch。正式配置使用
`xuc_imu_angle_unit: deg`；驱动在接收层转换为弧度，再应用符号和弧度偏移。
所有姿态接口（包括 `raw_imu_yaw()`）输出弧度，手眼采集可直接用于 `Eigen::AngleAxisd`。
发送目标角的格式和单位保持原协议约定的弧度；回传单位的确认不代表发送单位已实车验证。

`xuc_imu_angle_unit` 只接受 `deg` 或 `rad`，未配置时默认 `rad`，兼容原弧度固件。
更换下位机固件时须同步核对该项。可只接收检查：

```bash
./build/xuc_board_probe /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 460800 5 --imu-unit=deg
```

探测程序同时输出弧度和度，默认不发送控制或射击指令。

## 手眼采集与计算

2026-10-10 用户现场确认 yaw、pitch 读数准确。正式运行与标定配置均使用
`xuc_imu_yaw_sign: -1`，不再额外乘旧的 1.06298816；对应的发送侧倒数补偿也移除，
`xuc_yaw_sign: 1`。pitch 系数保持现有方向，发送角度仍为弧度。

XUC 接收线程保留两秒的成对 yaw/pitch 历史，采集按图像时间插值，跨 ±180° 使用短弧。
没有前后姿态、包间隔超过 30 ms、未连续停稳 500 ms（两轴变化范围各 ≤0.05°）时拒绝保存。
图像保存时超过 100 ms 也拒绝保存；相机队列满时丢弃旧帧。
对应设置为 `capture_pose_stable_window_ms`、`capture_pose_stable_range_deg`、
`capture_max_frame_age_ms`。采集目录必须没有已有图片/姿态，避免覆盖；保存时间信息到该目录的
`samples.csv`。这些时间戳来自主机接收时刻，**仍不是相机曝光与 IMU 硬件同步时间戳**。

`calibrate_handeye` 默认先求 Tsai 初值，再联合优化一组相机外参及固定棋盘位姿，
使所有角点的重投影误差最小。内参和实测 IMU 姿态保持固定。使用 `--no-refine` 可回放原方法，
`--self-test` 验证合成数据。计算仅输出候选结果，须通过原有验证门槛后才能采用。
原始数据应覆盖 yaw、pitch 多个方向，棋盘和云台底座固定，每次等画面显示 READY TO SAVE 再保存。
