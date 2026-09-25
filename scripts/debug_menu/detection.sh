#!/usr/bin/env bash

# Detection-test adapters. They only launch dedicated test executables with a
# temporary runtime config; they never launch `standard` or any control path.

# 适配职责：准备本次配置、校验依赖、构造调用；参数覆盖只保存在当前会话。

SP_VISION_TEST_ENEMY_COLOR="${SP_VISION_TEST_ENEMY_COLOR:-}"
SP_VISION_TEST_MIN_CONFIDENCE="${SP_VISION_TEST_MIN_CONFIDENCE:-}"

detection_current_algorithm() {
  local algorithm=$1
  case "$algorithm" in
    yolo|traditional) printf '%s\n' "$algorithm" ;;
    *) common_error "未知识别算法：$algorithm"; return 1 ;;
  esac
}

detection_prepare_runtime_config() {
  local source_config=$1
  local output_config=$2
  local -a overrides=()

  if [[ -n "$SP_VISION_TEST_ENEMY_COLOR" ]]; then
    overrides+=("enemy_color=str:$SP_VISION_TEST_ENEMY_COLOR")
  fi
  if [[ -n "$SP_VISION_TEST_MIN_CONFIDENCE" ]]; then
    overrides+=("min_confidence=float:$SP_VISION_TEST_MIN_CONFIDENCE")
  fi

  common_make_runtime_config "$source_config" "$output_config" "${overrides[@]}" || return 1
  local key
  for key in xuc_enable xuc_allow_control xuc_allow_shoot; do
    if [[ "$(common_yaml_get "$output_config" "$key")" != "false" ]]; then
      common_error "运行时配置安全开关校验失败：$key"
      return 1
    fi
  done
  common_print_runtime_safety
}

detection_require_model() {
  local config=$1
  local info key value resolved
  if ! info="$(environment_model_info "$config")"; then
    common_interface_incompatible "正式配置没有当前推理后端所需的模型字段。"
    return 1
  fi
  IFS='|' read -r key value resolved <<<"$info"
  if [[ ! -f "$resolved" ]]; then
    if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
      common_warn "dry-run：模型当前不存在，正式运行将安全失败：$key=$value（$resolved）"
      return 0
    fi
    common_interface_incompatible "模型不存在：$key=$value（解析为 $resolved）"
    return 1
  fi
  if [[ "$resolved" == *.xml && ! -f "${resolved%.xml}.bin" ]]; then
    common_interface_incompatible "OpenVINO 模型缺少同名 .bin：${resolved%.xml}.bin"
    return 1
  fi
  common_ok "模型检查通过：$resolved"
}

detection_require_test_binary() {
  local key=$1
  local binary
  binary="$(registry_binary "$key")" || return 1
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：将检查可执行文件：$binary"
    return 0
  fi
  common_require_executable "$binary" "识别测试程序" || {
    common_interface_incompatible "缺少可执行文件：$binary"
    return 1
  }
  registry_validate_interface "$key"
}

detection_prepare_common() {
  # 返回变量名与本函数的临时路径变量保持不同，避免 Bash 动态作用域遮蔽调用者。
  local binary_key=$1
  local output_var=$2
  local standard_config runtime_path
  standard_config="$(registry_config standard)" || return 1
  runtime_path="$(common_runtime_config_path detection)"
  detection_require_test_binary "$binary_key" || return 1
  detection_require_model "$standard_config" || return 1
  detection_prepare_runtime_config "$standard_config" "$runtime_path" || return 1
  printf -v "$output_var" '%s' "$runtime_path"
}

detection_run_pnp() {
  local label=$1
  local algorithm=$2
  local runtime_config transcript rc=0
  detection_prepare_common camera_pnp_test runtime_config || return
  common_require_display || return 1
  transcript="$(common_log_path detection)"
  registry_command_camera_pnp "$runtime_config" "$algorithm"
  process_run_managed "$label" 1 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  common_info "识别测试退出码：$rc"
  common_info "本次测试敌方颜色：${SP_VISION_TEST_ENEMY_COLOR:-使用正式配置}"
  common_info "本次测试最低置信度：${SP_VISION_TEST_MIN_CONFIDENCE:-使用正式配置}"
  return "$rc"
}

action_live_armor_detection() {
  # 目标：camera_pnp_test，非 tradition；配置来源：标准配置的安全运行缓存。
  detection_run_pnp "实时 YOLO 识别" yolo
}

action_live_traditional_detection() {
  # 目标：camera_pnp_test --tradition；配置来源：标准配置的安全运行缓存。
  detection_run_pnp "实时传统算法识别" traditional
}

action_compare_detection() {
  common_unavailable \
    "现有命令行只有单一算法选择参数，没有稳定的同帧 YOLO/传统算法对比输出接口。" \
    "detector_video_test 或专用对比程序及其结构化结果参数"
}

action_single_image_detection() {
  # 目标：camera_detect_test；配置来源：标准配置的安全运行缓存。
  local image input runtime_config transcript rc=0
  common_read "请输入图片绝对路径或项目根目录相对路径：" input || return 1
  [[ -n "$input" ]] || {
    common_warn "未输入图片路径，已取消。"
    return 0
  }
  image="$input"
  [[ "$image" == /* ]] || image="$SP_VISION_ROOT/$image"
  image="$(realpath -m -- "$image")" || return 1
  common_require_file "$image" "输入图片" || return 1
  common_require_display || return 1
  detection_prepare_common camera_detect_test runtime_config || return 1
  transcript="$(common_log_path detection)"
  registry_command_camera_detect "$runtime_config" "$image" yolo
  process_run_managed "单张图片识别" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  common_info "识别测试退出码：$rc"
  return "$rc"
}

action_video_detection() {
  # 目标：detector_video_test；配置来源：标准配置的安全运行缓存。
  local video input start_index end_index runtime_config transcript rc=0
  common_read "请输入视频绝对路径或项目根目录相对路径：" input || return 1
  [[ -n "$input" ]] || {
    common_warn "未输入视频路径，已取消。"
    return 0
  }
  video="$input"
  [[ "$video" == /* ]] || video="$SP_VISION_ROOT/$video"
  video="$(realpath -m -- "$video")" || return 1
  common_require_file "$video" "输入视频" || return 1
  common_read "起始帧下标（默认 0）：" start_index || return 1
  common_read "结束帧下标（默认 0，表示到结尾）：" end_index || return 1
  [[ -n "$start_index" ]] || start_index=0
  [[ -n "$end_index" ]] || end_index=0
  [[ "$start_index" =~ ^[0-9]+$ ]] || {
    common_error "起始帧下标必须为非负整数。"
    return 1
  }
  [[ "$end_index" =~ ^[0-9]+$ ]] || {
    common_error "结束帧下标必须为非负整数。"
    return 1
  }
  common_require_display || return 1
  detection_prepare_common detector_video_test runtime_config || return 1
  transcript="$(common_log_path detection)"
  registry_command_video_detect "$runtime_config" "$start_index" "$end_index" "$video" yolo
  process_run_managed "录像离线识别" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  common_info "识别测试退出码：$rc"
  return "$rc"
}

detection_set_enemy_color_for_session() {
  # 数据来源：当前菜单会话；不写入正式配置。
  local answer
  while true; do
    common_read "请选择本次测试敌方颜色 [red/blue]：" answer || return 1
    case "${answer,,}" in
      red|blue)
        SP_VISION_TEST_ENEMY_COLOR="${answer,,}"
        common_ok "本次菜单会话将使用敌方颜色：$SP_VISION_TEST_ENEMY_COLOR"
        common_info "该选择只影响临时运行配置，不会写入正式配置。"
        return 0
        ;;
      "") common_warn "输入不能为空。"; return 1 ;;
      *) common_warn "只接受 red 或 blue。" ;;
    esac
  done
}

detection_adjust_confidence() {
  # 数据来源：当前菜单会话；不写入正式配置。
  local answer
  common_read_number "请输入本次测试最低置信度（0 到 1）：" answer || return 1
  if ! awk -v value="$answer" 'BEGIN { exit !(value >= 0 && value <= 1) }'; then
    common_error "最低置信度必须位于 0 到 1。"
    return 1
  fi
  SP_VISION_TEST_MIN_CONFIDENCE="$answer"
  common_ok "本次菜单会话最低置信度：$SP_VISION_TEST_MIN_CONFIDENCE"
  common_info "该值只影响临时运行配置，不会写入正式配置。"
}

action_select_enemy_color() {
  detection_set_enemy_color_for_session
}

action_adjust_confidence() {
  detection_adjust_confidence
}

action_save_detection_frame() {
  common_unavailable \
    "现有识别测试程序没有“保存当前帧”命令行参数或远程控制接口。" \
    "camera_pnp_test/camera_detect_test 的保存路径参数或稳定控制接口"
}

action_detection_statistics() {
  # 数据来源：logs/ 中已有识别日志，不启动相机或推理程序。
  local log_file
  log_file="$(common_latest_log detection)" || {
    common_unavailable "logs/ 中没有可识别的检测日志。" "logs/*.log"
    return
  }
  common_info "日志：$log_file"
  common_info "最近检测结果："
  grep -E 'FPS:|Detected [0-9]+ armors|检测耗时|Camera PnP test' "$log_file" | tail -n 80
}
