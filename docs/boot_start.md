# 视觉开机自动启动

适用现场：`rm` 用户已通过 GDM 配置 X11 自动登录。用户服务在开机自动登录后启动，
等待串口、USB 相机及物理桌面可用，不依赖遥控器切档，也不需要手动登录。
若以后关闭自动登录，本方案不会保证无人登录时启动。

安装与静态检查：

```bash
cd /home/rm/sp_vision
python3 tools/boot_start_vision.py --check
python3 scripts/boot_start/install.py
```

安装器仅启用下次开机启动，不启动真实程序、不重启电脑。
开机入口直接运行已有 `build/standard`，不修改算法源码、二进制、正式配置或原手动启动器。
启动时根据正式配置和既有颜色选择生成原路径下的运行缓存；只替换缓存中的颜色字段。
ROS/MVS 环境与原入口保持一致，启动前清理继承的动态库及 shell 注入环境变量。

现场系统 `mv` 在临时文件重命名测试中出现 SIGBUS，因此新入口用 Python 原子写入缓存，
不经过原启动器的 `mv` 路径；操作系统故障的根因尚未确认。

进程 PID 和日志沿用 `logs/xuc-bringup/field_autoaim.pid`、`field_autoaim.last_log`。
旧的 `tools/field_stop_autoaim.sh` 仍可按 PID 停止主程序。正常退出或人工停止后本次不自动拉起；
异常退出由服务每 5 秒尝试恢复。入口发现同路径的 standard 已运行时不再创建第二个实例。

查看、停用或恢复下次开机启动：

```bash
systemctl --user status sp-vision-boot.service
journalctl --user -u sp-vision-boot.service -n 80 --no-pager
systemctl --user disable --now sp-vision-boot.service
systemctl --user enable sp-vision-boot.service
```

现场启动验证由用户重启小电脑完成；安装和离线测试不启动视觉、相机采集或控制程序。

## MediaMTX 与相机窗口推流

在同一个自动登录桌面下安装两个独立用户服务：

```bash
python3 scripts/boot_start/install_fpv.py
```

默认沿用 `/home/rm/mediamtx_v1.21.3_linux_amd64/mediamtx.yml`，不复制或覆盖配置。
安装器只启用下次启动。MediaMTX 服务不依赖视觉；推流服务等待物理 X11 桌面、
MediaMTX 8554 端口，以及属于本工程 `build/standard` 的 `reprojection` 窗口。
找到窗口后以 H.264、30 FPS、目标 3 Mbps 发布至 `rtsp://127.0.0.1:8554/fpv`。
窗口关闭或改变时停止本服务创建的 FFmpeg，等待新的窗口；FFmpeg 失败后重试。
已有手动 FFmpeg 发布到相同地址时等待，避免重复推流。
不自动启动或重启视觉，不打开相机设备。推流端在显示画面现有 Gamma 2.0 的基础上增加 `eq=gamma=1.25`，进一步提亮回传暗部；仅影响图传观看，不改变相机参数或算法输入。
捕获仍依赖窗口保持显示，遮挡、最小化及锁屏可能影响图像。

```bash
systemctl --user status sp-mediamtx.service sp-fpv.service
journalctl --user -u sp-fpv.service -n 50 --no-pager
python3 tools/fpv_publisher.py --check
# 仅在退出旧的手动 MediaMTX / FFmpeg 后接管当前会话：
systemctl --user start sp-mediamtx.service sp-fpv.service
# 停用这两个服务，不影响视觉服务：
systemctl --user disable --now sp-fpv.service sp-mediamtx.service
```

操作端访问 `http://小电脑IP:8889/fpv/`。IP 变化时更新浏览器地址。
本方案沿用 GDM 自动登录；完整断电重启、现场网络和实际画面效果须现场验收。
