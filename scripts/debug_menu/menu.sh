#!/usr/bin/env bash

# Interactive Simplified-Chinese menu. Menu numbers dispatch only through the
# semantic registry adapters; no executable path or business argument appears
# in this file.

# 维护约定：菜单编号只分发到 registry_invoke_*，路径和参数由适配层维护。
# 返回码：0 为正常结束，64 为无效编号，100 为返回上一级。
# 增加菜单项时须同时维护显示项与注册项，不能只增加显示文本。

menu_show_header() {
  common_title "$1"
  shift
  local line
  for line in "$@"; do
    common_info "$line"
  done
}

menu_loop() {
  local title=$1
  local invoker=$2
  shift 2
  local -a lines=("$@")
  local choice rc

  while true; do
    common_title "$title"
    local line
    for line in "${lines[@]}"; do
      common_info "$line"
    done
    printf '\n'
    if ! common_read "请输入编号：" choice; then
      printf '\n'
      return 0
    fi
    if [[ -z "$choice" ]]; then
      common_warn "输入不能为空，请重新选择。"
      continue
    fi
    if [[ ! "$choice" =~ ^[0-9]+$ ]]; then
      common_warn "只接受菜单中存在的数字编号。"
      continue
    fi

    if [[ "$choice" == "0" ]]; then
      "$invoker" 0
      return 0
    fi

    "$invoker" "$choice"
    rc=$?
    case "$rc" in
      0)
        common_info "退出码：0"
        common_pause
        ;;
      100)
        return 0
        ;;
      64)
        common_warn "该编号不存在，请重新选择。"
        ;;
      *)
        common_error "功能退出码：$rc"
        common_pause
        ;;
    esac
  done
}

menu_intrinsics() {
  menu_loop \
    "---------------- 内参标定 ----------------" \
    registry_invoke_intrinsics \
    "[1] 采集标定图像" \
    "[2] 计算内参" \
    "[3] 检查已有内参" \
    "[4] 查看当前内参" \
    "[5] 写入通过检查的内参" \
    "[0] 返回主菜单"
}

menu_extrinsics() {
  menu_loop \
    "---------------- 外参标定 ----------------" \
    registry_invoke_extrinsics \
    "[1] 填写发射口相对相机的三维坐标" \
    "[2] 手眼标定数据采集" \
    "[3] 计算手眼标定结果" \
    "[4] 检查外参/手眼标定结果" \
    "[5] 查看当前外参" \
    "[6] 写入通过检查的外参" \
    "[0] 返回主菜单"
}

menu_detection() {
  menu_loop \
    "-------------- 装甲板识别测试 --------------" \
    registry_invoke_detection \
    "[1] 实时 YOLO 识别" \
    "[2] 实时传统算法识别" \
    "[3] YOLO/传统算法对比" \
    "[4] 单张图片识别" \
    "[5] 录像离线识别" \
    "[6] 选择敌方颜色" \
    "[7] 调整本次测试置信度" \
    "[8] 保存当前识别画面" \
    "[9] 查看识别性能统计" \
    "[0] 停止识别并返回"
}

menu_camera() {
  menu_loop \
    "------------- 相机预览与颜色检查 -------------" \
    registry_invoke_camera \
    "[1] 普通相机预览" \
    "[2] 查看相机参数" \
    "[3] 临时调整曝光" \
    "[4] 临时调整增益" \
    "[5] 临时调整白平衡" \
    "[6] Bayer/RGB/BGR 通道检查" \
    "[7] 保存当前原始帧" \
    "[8] 保存当前显示帧" \
    "[9] 恢复配置文件参数" \
    "[0] 关闭相机并返回"
}

menu_config() {
  menu_loop \
    "---------------- 配置管理 ----------------" \
    registry_invoke_config \
    "[1] 查看正式配置" \
    "[2] 查看内参" \
    "[3] 查看外参" \
    "[4] 设置本次测试敌方颜色" \
    "[5] 查看检测阈值" \
    "[6] 修改本次测试阈值" \
    "[7] 检查模型路径" \
    "[8] 检查控制与发射安全开关" \
    "[9] 验证 YAML 格式" \
    "[0] 返回主菜单"
}

menu_environment() {
  menu_loop \
    "------------- 环境与资源检查 -------------" \
    registry_invoke_environment \
    "[1] 一键快速检查" \
    "[2] 相机与 MVS 检查" \
    "[3] OpenCV 检查" \
    "[4] OpenVINO 检查" \
    "[5] ROS 2 检查" \
    "[6] 模型文件检查" \
    "[7] 动态库检查" \
    "[8] USB 权限检查" \
    "[9] 图形显示环境检查" \
    "[10] 生成完整诊断报告" \
    "[0] 返回主菜单"
}

menu_build() {
  menu_loop \
    "---------------- 项目编译 ----------------" \
    registry_invoke_build \
    "[1] 增量编译" \
    "[2] 清理后重新编译" \
    "[3] 只编译相机测试" \
    "[4] 只编译装甲板识别测试" \
    "[5] 只编译标定工具" \
    "[6] 编译全部目标" \
    "[7] 查看上次编译错误" \
    "[0] 返回主菜单"
}

menu_logs() {
  menu_loop \
    "------------- 日志与调试数据 -------------" \
    registry_invoke_logs \
    "[1] 查看最新日志" \
    "[2] 实时跟踪当前日志" \
    "[3] 查看相机错误" \
    "[4] 查看识别结果统计" \
    "[5] 导出诊断材料" \
    "[6] 清理过期日志" \
    "[7] 查看磁盘占用" \
    "[0] 返回主菜单"
}

menu_processes() {
  menu_loop \
    "-------------- 运行进程管理 ---------------" \
    registry_invoke_processes \
    "[1] 查看本菜单启动的功能" \
    "[2] 停止当前测试" \
    "[3] 停止本菜单启动的所有测试" \
    "[4] 检查相机占用" \
    "[5] 释放本菜单持有的相机资源" \
    "[6] 检查残留窗口" \
    "[0] 返回主菜单"
}

menu_main() {
  menu_loop \
    "================ SP-Vision 调试工具 ================" \
    registry_invoke_main \
    "[1] 内参标定" \
    "[2] 外参标定" \
    "[3] 装甲板识别测试" \
    "[4] 相机预览与颜色检查" \
    "[5] 配置管理" \
    "[6] 环境与资源检查" \
    "[7] 项目编译" \
    "[8] 日志与调试数据" \
    "[9] 运行进程管理" \
    "[0] 退出"
}
