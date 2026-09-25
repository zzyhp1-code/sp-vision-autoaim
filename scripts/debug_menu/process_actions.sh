#!/usr/bin/env bash

# User-facing process actions. Killing is delegated to process.sh and remains
# restricted to processes whose PID, PGID, and start time all match this menu.

# PID、PGID、启动时间与会话专属记录共同验证进程身份；“本菜单”只指本实例任务。

action_list_menu_processes() {
  # 数据来源：本会话 logs/runtime/process_<session>；不读取外部进程。
  process_print_current
}

action_stop_current_process() {
  # 目标：process.sh 记录的单一 PID/PGID/starttime。
  process_stop_current 5
}

action_stop_all_menu_processes() {
  # 目标：本菜单当前受管进程；不按进程名搜索。
  common_info "本菜单同时只跟踪一个前台功能进程。"
  process_stop_current 5
}

action_check_camera_occupancy() {
  # 数据来源：菜单锁和 /dev/video*；只报告，不结束外部进程。
  process_check_camera_occupancy
}

action_release_camera_resource() {
  # 目标：只释放本菜单持有的相机锁和受管进程。
  local rc=0
  process_stop_current 5 || rc=$?
  process_release_camera_lock
  if ((rc == 0)); then
    common_ok "本菜单的相机任务和相机锁已释放。"
  fi
  common_info "外部程序占用的相机资源不会被菜单结束。"
  return "$rc"
}

action_check_residual_windows() {
  # 数据来源：xwininfo 窗口树；只报告，不结束窗口。
  local titles
  if command -v xwininfo >/dev/null 2>&1 && common_display_available; then
    titles="$(xwininfo -root -tree 2>/dev/null |
      grep -E 'Camera PnP Test|Chessboard Capture|Detection Result|verify_handeye|"img"' || true)"
    if [[ -n "$titles" ]]; then
      common_warn "发现可能的菜单测试窗口："
      printf '%s\n' "$titles"
      common_info "请先停止本菜单任务；菜单不会结束外部窗口所属进程。"
      return 1
    fi
    common_ok "未发现已知测试窗口标题。"
    return 0
  fi
  common_unavailable "缺少 xwininfo 或图形会话，无法检查残留窗口。" "xwininfo"
}
