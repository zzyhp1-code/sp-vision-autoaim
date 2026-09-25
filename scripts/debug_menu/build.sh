#!/usr/bin/env bash

# Build adapters. Every invocation explicitly passes a CMake target so the
# fallback target `standard` in scripts/build.sh is never used accidentally.

# 构建目标由 registry_build_targets 提供；空目标必须拒绝，避免触发默认目标。

build_run_targets() {
  local label=$1
  shift
  local transcript
  transcript="$(common_log_path build)"
  registry_command_build "$@"
  process_run_managed "$label" 0 "$transcript" "${REGISTRY_COMMAND[@]}"
}

build_all_targets() {
  local -a targets=()
  mapfile -t targets < <(registry_build_targets all)
  build_run_targets "编译全部目标" "${targets[@]}"
}

build_group_targets() {
  local label=$1
  local group=$2
  local -a targets=()
  mapfile -t targets < <(registry_build_targets "$group")
  build_run_targets "$label" "${targets[@]}"
}

action_build_incremental() {
  # 目标：scripts/build.sh；显式目标组 all。
  build_group_targets "增量编译" all
}

action_build_clean() {
  # 目标：scripts/build.sh；先 clean，再显式目标组 all。
  local rc=0
  if ! common_confirm "将清理 build/ 中的 CMake 目标产物，然后重新编译。是否继续？" n; then
    common_info "已取消。"
    return 0
  fi
  local transcript
  transcript="$(common_log_path build_clean)"
  registry_command_build clean
  process_run_managed "清理构建目标" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  if ((rc != 0)); then
    common_error "清理构建目标失败，停止后续编译。"
    return "$rc"
  fi
  build_all_targets
}

action_build_camera() {
  # 目标：scripts/build.sh camera_test。
  build_group_targets "编译相机测试" camera
}

action_build_detection() {
  # 目标：scripts/build.sh camera_detect_test camera_pnp_test detector_video_test。
  build_group_targets "编译装甲板识别测试" detection
}

action_build_calibration() {
  # 目标：scripts/build.sh capture/calibrate_*/verify_handeye。
  build_group_targets "编译标定工具" calibration
}

action_build_all() {
  # 目标：scripts/build.sh all。
  build_all_targets
}

action_show_last_build_error() {
  # 数据来源：logs/debug_menu/build_*.log，不启动编译。
  local directory="$SP_VISION_LOG_DIR/debug_menu"
  local latest=""
  if [[ -d "$directory" ]]; then
    latest="$(find "$directory" -maxdepth 1 -type f -name 'build_*.log' -printf '%T@ %p\n' \
      2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  fi
  if [[ -z "$latest" || ! -f "$latest" ]]; then
    common_unavailable "没有找到菜单编译日志。" "$directory/build_*.log"
    return
  fi
  common_info "编译日志：$latest"
  if grep -nE '(^|[^A-Za-z])(error|Error|ERROR|FAILED|失败):' "$latest" >/dev/null 2>&1; then
    grep -nE '(^|[^A-Za-z])(error|Error|ERROR|FAILED|失败):' "$latest" | tail -n 80
  else
    common_info "日志中未发现明确错误行，显示末尾内容。"
    common_show_tail "$latest" 80
  fi
}
