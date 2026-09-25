#!/usr/bin/env bash

# Camera preview adapters. Temporary exposure, gain, and white-balance changes
# are passed through a temporary safe YAML cache; the formal config is untouched.

# 相机预览和颜色处理由原有程序实现；这里仅管理参数和调用。
# “查看相机参数”展示配置文件值，不是设备寄存器的实时回读值。

camera_prepare_runtime_config() {
  local output_config=$1
  shift
  local standard_config
  standard_config="$(registry_config standard)" || return 1
  common_make_runtime_config "$standard_config" "$output_config" "$@" || return 1
  local key
  for key in xuc_enable xuc_allow_control xuc_allow_shoot; do
    if [[ "$(common_yaml_get "$output_config" "$key")" != "false" ]]; then
      common_error "运行时配置安全开关校验失败：$key"
      return 1
    fi
  done
  common_print_runtime_safety
}

camera_require_binary() {
  local binary
  binary="$(registry_binary camera_test)" || return 1
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：将检查可执行文件：$binary"
    return 0
  fi
  common_require_executable "$binary" "相机测试程序" || {
    common_interface_incompatible "缺少可执行文件：$binary"
    return 1
  }
  registry_validate_interface camera_test
}

camera_run_preview_config() {
  local label=$1
  local runtime_config=$2
  local transcript rc=0
  camera_require_binary || return 1
  common_require_display || return 1
  transcript="$(common_log_path camera)"
  registry_command_camera_test "$runtime_config" 1
  process_run_managed "$label" 1 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  common_info "相机预览退出码：$rc"
  return "$rc"
}

action_camera_preview() {
  # 目标：camera_test -c=<runtime> -d；配置来源：standard.yaml 的安全缓存。
  local runtime_config
  runtime_config="$(common_runtime_config_path camera)"
  camera_prepare_runtime_config "$runtime_config" || return 1
  camera_run_preview_config "普通相机预览" "$runtime_config"
}

action_camera_parameters() {
  # 数据来源：configs/standard.yaml；不打开相机。
  local config
  config="$(registry_config standard)" || return 1
  local key value
  for key in \
    camera_name \
    camera_width \
    camera_height \
    camera_fps \
    exposure_ms \
    gain \
    white_balance_auto \
    hikrobot_pixel_format \
    trigger_enable; do
    value="$(common_yaml_get "$config" "$key" 2>/dev/null || printf '未配置')"
    common_info "$key：$value"
  done
}

camera_adjust_and_preview() {
  local label=$1
  local key=$2
  local prompt=$3
  local value_prefix=${4:-}
  local value runtime_config
  common_read "$prompt" value || return 1
  [[ -n "$value" ]] || {
    common_warn "输入为空，未生成临时参数。"
    return 1
  }
  runtime_config="$(common_runtime_config_path camera_override)"
  camera_prepare_runtime_config "$runtime_config" "$key=$value_prefix$value" || return 1
  common_info "临时运行配置：$runtime_config"
  common_info "$key=$value（仅本次运行）"
  camera_run_preview_config "$label" "$runtime_config"
}

action_adjust_exposure() {
  # 目标：camera_test；参数来源：standard.yaml + 本次临时 exposure_ms。
  camera_adjust_and_preview "临时调整曝光" "exposure_ms" "请输入本次预览曝光时间（毫秒）："
}

action_adjust_gain() {
  # 目标：camera_test；参数来源：standard.yaml + 本次临时 gain。
  camera_adjust_and_preview "临时调整增益" "gain" "请输入本次预览增益："
}

action_adjust_white_balance() {
  # 目标：camera_test；参数来源：standard.yaml + 本次临时 white_balance_auto。
  camera_adjust_and_preview \
    "临时调整白平衡" "white_balance_auto" "请输入本次预览白平衡 [off/auto]：" "str:"
}

action_channel_check() {
  # 数据来源：standard.yaml 和已有相机程序日志；不执行图像转换。
  local config log_file
  local pixel_format camera_name
  config="$(registry_config standard)" || return 1
  camera_name="$(common_yaml_get "$config" camera_name 2>/dev/null || printf '未配置')"
  pixel_format="$(common_yaml_get "$config" hikrobot_pixel_format 2>/dev/null || printf '未配置')"

  common_info "相机类型（配置）：$camera_name"
  common_info "相机像素格式（配置）：$pixel_format"
  if [[ "$pixel_format" == Bayer* ]]; then
    common_info "Bayer 物理排列：配置值 $pixel_format；最终排列仍需相机程序日志确认。"
  else
    common_info "Bayer 物理排列：无法从当前配置值判断。"
  fi
  common_info "推理输入：项目识别程序要求 RGB，菜单不执行颜色转换。"
  common_info "显示输入：项目相机/显示程序负责 BGR 显示转换，菜单不执行颜色转换。"
  common_info "颜色正确性无法仅凭配置或日志自动声明。"
  common_info "数据无法判断，请放置红色或蓝色参照物，或保存原始帧后使用 AI 协助分析。"

  log_file="$(common_latest_log camera)" || {
    common_warn "没有找到已有的相机程序日志。"
    common_info "当前使用的转换路径：无法判断"
    common_info "请先运行普通相机预览，再从相机程序日志读取转换路径。"
    return 1
  }
  common_info "相机程序日志：$log_file"
  local evidence
  evidence="$(grep -iE 'Bayer|RGB|BGR|pixel format|pixel_format|cvtColor|color' "$log_file" |
    tail -n 60 || true)"
  if [[ -n "$evidence" ]]; then
    printf '%s\n' "$evidence"
  else
    common_warn "现有相机日志没有报告 Bayer/RGB/BGR 转换路径。"
    common_info "当前使用的转换路径：无法判断"
    return 1
  fi

}

action_save_raw_frame() {
  common_unavailable \
    "camera_test 没有保存原始帧的命令行参数或运行中控制接口。" \
    "camera_test 的原始帧保存参数或独立专用保存工具"
}

action_save_display_frame() {
  common_unavailable \
    "camera_test 没有保存显示帧的命令行参数或运行中控制接口。" \
    "camera_test 的显示帧保存参数或独立专用保存工具"
}

action_restore_camera_parameters() {
  # 目标：camera_test；参数来源：standard.yaml 的正式参数安全缓存。
  local config values runtime_config
  config="$(registry_config standard)" || return 1
  common_info "正式配置中的本次运行参数："
  values="$(common_yaml_get "$config" exposure_ms)"
  common_info "曝光：$values ms"
  values="$(common_yaml_get "$config" gain)"
  common_info "增益：$values"
  values="$(common_yaml_get "$config" white_balance_auto)"
  common_info "白平衡：$values"
  common_info "相机参数由程序启动时读取配置，菜单不会永久修改相机寄存器。"
  if ! common_confirm "是否使用正式配置启动一次预览，以确认参数恢复？" n; then
    common_info "已取消启动预览。"
    return 0
  fi
  runtime_config="$(common_runtime_config_path camera_restore)"
  camera_prepare_runtime_config "$runtime_config" || return 1
  camera_run_preview_config "恢复配置文件参数预览" "$runtime_config"
}
