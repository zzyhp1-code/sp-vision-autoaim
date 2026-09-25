#!/usr/bin/env bash

# Calibration orchestration. All chessboard detection, PnP, calibration, and
# hand-eye math remain in the existing C++ executables.

# 候选结果、质量状态和输入数据应作为同一次计算的整体管理，失败后不得复用
# 旧的通过状态。菜单只解释已有程序输出，指标不足时不能承诺实际安装精度。

SP_VISION_INTRINSICS_CANDIDATE="${SP_VISION_INTRINSICS_CANDIDATE:-}"
SP_VISION_INTRINSICS_STATUS="${SP_VISION_INTRINSICS_STATUS:-}"
SP_VISION_INTRINSICS_LOG="${SP_VISION_INTRINSICS_LOG:-}"
SP_VISION_HANDEYE_CANDIDATE="${SP_VISION_HANDEYE_CANDIDATE:-}"
SP_VISION_HANDEYE_STATUS="${SP_VISION_HANDEYE_STATUS:-}"
SP_VISION_HANDEYE_LOG="${SP_VISION_HANDEYE_LOG:-}"
SP_VISION_HANDEYE_INPUT="${SP_VISION_HANDEYE_INPUT:-}"

calibration_prepare_safe_config() {
  local output=$1
  local source
  source="$(registry_config calibration)" || return 1
  common_make_runtime_config "$source" "$output" || return 1
  local key
  for key in xuc_enable xuc_allow_control xuc_allow_shoot; do
    if [[ "$(common_yaml_get "$output" "$key")" != "false" ]]; then
      common_error "标定运行配置安全开关校验失败：$key"
      return 1
    fi
  done
  common_print_runtime_safety
}

calibration_resolve_input_dir() {
  # common_read 的提示写入 stderr，命令替换只会取得已解析的目录路径。
  local prompt=$1
  local default_dir=$2
  local answer resolved
  common_read "$prompt [$default_dir]：" answer || return 1
  [[ -n "$answer" ]] || answer="$default_dir"
  [[ "$answer" == /* ]] || answer="$SP_VISION_ROOT/$answer"
  resolved="$(realpath -m -- "$answer")" || return 1
  common_require_under "$resolved" "$SP_VISION_ROOT" "标定输入目录" || return 1
  common_require_dir "$resolved" "标定输入目录" || return 1
  printf '%s\n' "$resolved"
}

calibration_latest_input_dir() {
  local parent="$SP_VISION_DATA_DIR/calibration"
  local latest=""
  if [[ -d "$parent" ]]; then
    latest="$(find "$parent" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
      2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  fi
  if [[ -n "$latest" ]]; then
    printf '%s\n' "$latest"
  else
    printf '%s\n' "$parent/intrinsics"
  fi
}

calibration_require_offline_binary() {
  local key=$1
  local binary
  binary="$(registry_binary "$key")" || return 1
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：将检查可执行文件：$binary"
    return 0
  fi
  common_require_executable "$binary" "标定程序" || {
    common_interface_incompatible "缺少可执行文件：$binary"
    return 1
  }
  registry_validate_interface "$key"
}

calibration_count_capture_files() {
  local directory=$1
  local images quaternions
  images="$(find "$directory" -maxdepth 1 -type f -name '*.jpg' 2>/dev/null | wc -l | tr -d '[:space:]')"
  quaternions="$(find "$directory" -maxdepth 1 -type f -name '*.txt' 2>/dev/null | wc -l | tr -d '[:space:]')"
  printf '%s|%s\n' "$images" "$quaternions"
}

calibration_report_capture_metrics() {
  local directory=$1
  local transcript=${2:-}
  local counts images quaternions saved_events last_corners expected_corners
  counts="$(calibration_count_capture_files "$directory")"
  IFS='|' read -r images quaternions <<<"$counts"
  common_info "输出目录：$directory"
  common_info "图像数量：$images"
  common_info "姿态文件数量：$quaternions"
  common_info "采集程序返回码：${PROCESS_LAST_EXIT:-未知}"
  saved_events=0
  last_corners="未报告"
  if [[ -n "$transcript" && -f "$transcript" ]]; then
    saved_events="$(grep -cE 'Saved to|已保存' "$transcript" 2>/dev/null || true)"
    last_corners="$(grep -oE 'corners: [0-9]+' "$transcript" 2>/dev/null | tail -n 1 | awk '{print $2}' || true)"
    last_corners="${last_corners:-未报告}"
  fi
  common_info "程序报告保存次数：$saved_events"
  common_info "程序报告最近角点数：$last_corners；菜单未重新实现角点检测。"
  expected_corners="$(
    printf '%s*%s\n' \
      "$(common_yaml_get "$(registry_config calibration)" pattern_cols 2>/dev/null || printf 0)" \
      "$(common_yaml_get "$(registry_config calibration)" pattern_rows 2>/dev/null || printf 0)" |
      awk -F'*' '{print $1 * $2}'
  )"
  if [[ "$last_corners" =~ ^[0-9]+$ && "$expected_corners" =~ ^[0-9]+$ &&
    "$last_corners" -eq "$expected_corners" ]]; then
    common_info "棋盘格完整性：最近一帧程序报告为完整。"
  else
    common_info "棋盘格完整性：无法从现有日志确认。"
  fi
  common_info "清晰度：现有采集程序未输出结构化清晰度指标。"
  common_info "曝光状态：现有采集程序未输出结构化曝光指标。"
  common_info "画面覆盖区域：现有采集程序未输出结构化覆盖指标。"
  common_info "姿态重复检查：现有采集程序未输出结构化姿态去重指标。"
  if ((images < 10 || quaternions < 10)); then
    common_warn "样本少于 10 组，不足以进行标定质量判断。"
  else
    common_info "综合判定：无法判断"
    common_info "原因：现有采集程序没有提供覆盖、清晰度、曝光和姿态重复度指标。"
    common_info "数据无法判断，请使用 AI 协助分析。"
  fi
}

calibration_capture() {
  local label=$1
  local output_dir=$2
  local config transcript rc=0
  config="$(common_runtime_config_path calibration_capture)"
  calibration_prepare_safe_config "$config" || return 1
  calibration_require_offline_binary capture || return 1
  common_require_display || return 1
  common_require_under "$output_dir" "$SP_VISION_DATA_DIR" "标定图像输出目录" || return 1
  if [[ "$SP_VISION_DEBUG_DRY_RUN" != "1" ]]; then
    mkdir -p -- "$output_dir" || return 1
  fi
  transcript="$(common_log_path capture)"
  registry_command_capture "$config" "$output_dir"
  process_run_managed "$label" 1 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  if [[ "$SP_VISION_DEBUG_DRY_RUN" != "1" ]]; then
    calibration_report_capture_metrics "$output_dir" "$transcript"
  fi
  return "$rc"
}

action_capture_intrinsics() {
  # 目标：capture；配置来源：calibration.yaml 的强制安全缓存。
  local output_dir="$SP_VISION_DATA_DIR/calibration/intrinsics_$(common_timestamp)"
  common_info "本次只采集内参图像；输出目录位于 data/。"
  common_info "安全运行配置会强制 xuc_enable=false，采集程序不会发送控制或射击命令。"
  calibration_capture "采集内参标定图像" "$output_dir"
}

action_calibrate_intrinsics() {
  # 目标：calibrate_camera；配置来源：calibration.yaml 的安全缓存。
  local input_dir config transcript
  input_dir="$(calibration_resolve_input_dir "请输入内参标定图像目录" "$(calibration_latest_input_dir)")" || return 1
  config="$(common_runtime_config_path calibration_intrinsics)"
  calibration_prepare_safe_config "$config" || return 1
  calibration_require_offline_binary calibrate_camera || return 1
  transcript="$(common_log_path calibrate_camera)"
  registry_command_calibrate_camera "$config" "$input_dir"
  process_run_managed "计算内参" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || {
    local rc=$?
    common_error "内参计算退出码：$rc"
    return "$rc"
  }

  SP_VISION_INTRINSICS_LOG="$transcript"
  SP_VISION_INTRINSICS_CANDIDATE="$(common_temp_file intrinsics_candidate .yaml)"
  SP_VISION_INTRINSICS_STATUS="$(common_temp_file intrinsics_status .txt)"
  common_yaml_check_calibration \
    "$SP_VISION_INTRINSICS_LOG" \
    "$SP_VISION_INTRINSICS_CANDIDATE" \
    "$SP_VISION_INTRINSICS_STATUS" || return
  common_info "计算结果文件：$SP_VISION_INTRINSICS_CANDIDATE"
  common_info "质量状态文件：$SP_VISION_INTRINSICS_STATUS"
}

action_check_intrinsics() {
  # 数据来源：本次计算结果或当前正式内参；不重新实现角点检测。
  if [[ -z "$SP_VISION_INTRINSICS_CANDIDATE" || ! -f "$SP_VISION_INTRINSICS_CANDIDATE" ]]; then
    common_info "本次会话尚无内参计算结果，显示当前正式内参供检查。"
    config_action_view_intrinsics
    common_info "综合判定：无法判断"
    common_info "原因：当前配置只提供相机矩阵和畸变参数，没有覆盖范围、姿态变化、单张最大误差和收敛指标。"
    common_info "数据无法判断，请使用 AI 协助分析。"
    return 0
  fi
  common_info "本次会话内参计算结果已存在：$SP_VISION_INTRINSICS_CANDIDATE"
  common_info "状态：$(<"$SP_VISION_INTRINSICS_STATUS")"
  common_info "日志：$SP_VISION_INTRINSICS_LOG"
  common_show_tail "$SP_VISION_INTRINSICS_LOG" 80
}

action_view_intrinsics() {
  # 数据来源：两份正式 YAML；只读。
  config_action_view_intrinsics
}

action_write_intrinsics() {
  # dry-run 在公共写入封装中被拦截；两份配置通过同目录事务写入，失败会恢复。
  # 目标：yaml_tool.py write-fields；仅在通过/谨慎使用且确认后写正式 YAML。
  local status calibration_config standard_config
  if [[ -z "$SP_VISION_INTRINSICS_CANDIDATE" || ! -f "$SP_VISION_INTRINSICS_CANDIDATE" ]]; then
    common_unavailable "本次会话没有可写入的内参候选。" "先执行“计算内参”并取得通过或谨慎使用判定"
    return
  fi
  status="$(<"$SP_VISION_INTRINSICS_STATUS")"
  if [[ "$status" != "pass" && "$status" != "cautious" ]]; then
    common_unavailable \
      "内参综合判定为无法判断或不通过，禁止写入配置。" \
      "覆盖范围、远近/倾斜姿态变化、单张最大误差、异常样本、收敛和主点合理性指标"
    return
  fi

  calibration_config="$(registry_config calibration)" || return 1
  standard_config="$(registry_config standard)" || return 1
  common_title "字段级差异：configs/calibration.yaml"
  common_yaml_compare_fields \
    "$calibration_config" "$SP_VISION_INTRINSICS_CANDIDATE" camera_matrix distort_coeffs || return 1
  common_title "字段级差异：configs/standard.yaml"
  common_yaml_compare_fields \
    "$standard_config" "$SP_VISION_INTRINSICS_CANDIDATE" camera_matrix distort_coeffs || return 1

  if [[ "$status" == "cautious" ]]; then
    common_warn "当前为“谨慎使用”，必须再次明确确认。"
    common_confirm_exact "当前为谨慎使用，确认写入内参" "谨慎写入" || {
      common_info "已取消写入。"
      return 0
    }
  else
    common_confirm_exact "将原子写入内参到两份正式配置" "写入" || {
      common_info "已取消写入。"
      return 0
    }
  fi
  common_yaml_write_fields_pair \
    "$calibration_config" "$standard_config" "$SP_VISION_INTRINSICS_CANDIDATE" \
    camera_matrix distort_coeffs || return 1
  common_ok "内参已写入并完成重新解析验证。"
}

action_input_muzzle_coordinates() {
  # 坐标输入仅展示，不写配置。轴正方向不能替代坐标变换定义；这里沿用的
  # “右手系”界面说明需独立核实，不能直接作为下游旋转/平移运算依据。
  # 数据来源：用户毫米输入；当前缺少 Z 字段，只显示不写入。
  local x_mm y_mm z_mm
  local x_m y_m z_m x_text y_text z_text
  common_title "发射口相对相机的三维坐标"
  common_info "固定右手坐标系：X 前方为正，Y 上方为正，Z 右方为正。"
  common_read_number "向前 X（毫米）：" x_mm || return 1
  common_read_number "向上 Y（毫米）：" y_mm || return 1
  common_read_number "向右 Z（毫米）：" z_mm || return 1
  common_require_finite_range "$x_mm" -500 500 "向前 X" || return 1
  common_require_finite_range "$y_mm" -300 300 "向上 Y" || return 1
  common_require_finite_range "$z_mm" -300 300 "向右 Z" || return 1

  x_m="$(awk -v value="$x_mm" 'BEGIN { printf "%.6f", value / 1000.0 }')"
  y_m="$(awk -v value="$y_mm" 'BEGIN { printf "%.6f", value / 1000.0 }')"
  z_m="$(awk -v value="$z_mm" 'BEGIN { printf "%.6f", value / 1000.0 }')"
  if awk -v value="$y_mm" 'BEGIN { exit !(value >= 0) }'; then
    y_text="向上 ${y_mm} mm"
  else
    y_text="向下 $(awk -v value="$y_mm" 'BEGIN { printf "%.3f", -value }') mm"
  fi
  if awk -v value="$z_mm" 'BEGIN { exit !(value >= 0) }'; then
    z_text="向右 ${z_mm} mm"
  else
    z_text="向左 $(awk -v value="$z_mm" 'BEGIN { printf "%.3f", -value }') mm"
  fi
  if awk -v value="$x_mm" 'BEGIN { exit !(value >= 0) }'; then
    x_text="向前 ${x_mm} mm"
  else
    x_text="向后 $(awk -v value="$x_mm" 'BEGIN { printf "%.3f", -value }') mm"
  fi

  common_info "原始毫米数值：X=$x_mm，Y=$y_mm，Z=$z_mm"
  common_info "方向解释：$x_text，$y_text，$z_text"
  common_info "内部坐标：[X, Y, Z] = [$x_m, $y_m, $z_m] m"
  common_info "现有正式配置字段：xuc_camera_forward_of_muzzle_m、xuc_camera_above_muzzle_m"
  common_info "当前版本没有右侧 Z 对应字段，也没有完整的三维发射口接口。"
  common_info "综合判定：无法判断"
  common_info "坐标格式正确，但无法验证实测值是否真实。"
  common_info "数据无法判断，请使用 AI 协助分析。"
  common_info "未写入任何正式配置。"
}

action_capture_handeye() {
  # 目标：不启动；安全开关与 capture 姿态接口冲突时显示不可用。
  common_unavailable \
    "安全策略要求测试运行配置强制 xuc_enable=false；现有 capture 在该模式下无法获得手眼标定所需云台姿态。" \
    "不启用控制/射击且能提供姿态输入的专用采集入口"
}

action_calibrate_handeye() {
  # 目标：calibrate_handeye；配置来源：calibration.yaml 的安全缓存。
  local input_dir config transcript
  input_dir="$(calibration_resolve_input_dir "请输入手眼标定数据目录" "$(calibration_latest_input_dir)")" || return 1
  config="$(common_runtime_config_path calibration_handeye)"
  calibration_prepare_safe_config "$config" || return 1
  calibration_require_offline_binary calibrate_handeye || return 1
  transcript="$(common_log_path calibrate_handeye)"
  registry_command_calibrate_handeye "$config" "$input_dir"
  process_run_managed "计算手眼标定结果" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || {
    local rc=$?
    common_error "手眼计算退出码：$rc"
    return "$rc"
  }

  SP_VISION_HANDEYE_INPUT="$input_dir"
  SP_VISION_HANDEYE_LOG="$transcript"
  SP_VISION_HANDEYE_CANDIDATE="$(common_temp_file handeye_candidate .yaml)"
  SP_VISION_HANDEYE_STATUS="$(common_temp_file handeye_status .txt)"
  common_yaml_check_handeye \
    "$SP_VISION_HANDEYE_LOG" \
    "$SP_VISION_HANDEYE_CANDIDATE" \
    "$SP_VISION_HANDEYE_STATUS" || return
  common_info "计算结果文件：$SP_VISION_HANDEYE_CANDIDATE"
  common_info "质量状态文件：$SP_VISION_HANDEYE_STATUS"
}

calibration_make_verification_standard() {
  local output=$1
  local standard_config
  standard_config="$(registry_config standard)" || return 1
  if [[ -n "$SP_VISION_HANDEYE_CANDIDATE" && -f "$SP_VISION_HANDEYE_CANDIDATE" ]]; then
    local rotation translation
    rotation="$(common_yaml_get "$SP_VISION_HANDEYE_CANDIDATE" R_camera2gimbal)" || return 1
    translation="$(common_yaml_get "$SP_VISION_HANDEYE_CANDIDATE" t_camera2gimbal)" || return 1
    common_make_runtime_config \
      "$standard_config" "$output" \
      "R_camera2gimbal=json:$rotation" \
      "t_camera2gimbal=json:$translation" || return 1
  else
    common_make_runtime_config "$standard_config" "$output" || return 1
  fi
}

action_check_handeye() {
  # 目标：verify_handeye；配置来源：两份配置的安全缓存。
  local input_dir calibration_runtime standard_runtime transcript status
  local default_dir="${SP_VISION_HANDEYE_INPUT:-$(calibration_latest_input_dir)}"
  input_dir="$(calibration_resolve_input_dir "请输入外参/手眼验证数据目录" "$default_dir")" || return 1
  calibration_runtime="$(common_runtime_config_path calibration_verify)"
  standard_runtime="$(common_runtime_config_path standard_verify)"
  calibration_prepare_safe_config "$calibration_runtime" || return 1
  calibration_make_verification_standard "$standard_runtime" || return 1
  calibration_require_offline_binary verify_handeye || return 1

  transcript="$(common_log_path verify_handeye)"
  registry_command_verify_handeye "$calibration_runtime" "$standard_runtime" "$input_dir" 1
  process_run_managed "检查外参/手眼标定结果" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || {
    local rc=$?
    common_error "外参验证退出码：$rc"
    return "$rc"
  }

  status="$(common_temp_file handeye_verification_status .txt)"
  common_yaml_check_handeye_verification "$transcript" "$status" || return
  if [[ -n "$SP_VISION_HANDEYE_CANDIDATE" && -f "$SP_VISION_HANDEYE_CANDIDATE" ]]; then
    SP_VISION_HANDEYE_STATUS="$status"
  fi
}

action_view_extrinsics() {
  # 数据来源：两份正式 YAML；只读。
  config_action_view_extrinsics
}

action_write_extrinsics() {
  # dry-run 在公共写入封装中被拦截；两份配置通过同目录事务写入，失败会恢复。
  # 目标：yaml_tool.py write-fields；仅离线验证谨慎通过并二次确认后写 YAML。
  local status candidate=${SP_VISION_HANDEYE_CANDIDATE:-}
  local calibration_config standard_config
  if [[ -z "$candidate" || ! -f "$candidate" ]]; then
    common_unavailable "本次会话没有可写入的外参候选。" "先执行手眼计算并完成离线验证"
    return
  fi
  status="$(<"${SP_VISION_HANDEYE_STATUS:-/dev/null}")"
  if [[ "$status" != "pass" && "$status" != "cautious" ]]; then
    common_unavailable \
      "外参尚未通过离线一致性检查，禁止写入正式配置。" \
      "verify_handeye 的运动范围、平移残差、旋转残差、AX=XB 和重投影误差"
    return
  fi

  calibration_config="$(registry_config calibration)" || return 1
  standard_config="$(registry_config standard)" || return 1
  common_title "字段级差异：configs/calibration.yaml"
  common_yaml_compare_fields \
    "$calibration_config" "$candidate" R_camera2gimbal t_camera2gimbal || return 1
  common_title "字段级差异：configs/standard.yaml"
  common_yaml_compare_fields \
    "$standard_config" "$candidate" R_camera2gimbal t_camera2gimbal || return 1

  common_warn "离线残差只能证明计算自洽；实际安装精度无法判断。"
  if [[ "$status" == "cautious" ]]; then
    common_confirm_exact "当前为谨慎使用，确认写入外参" "谨慎写入" || {
      common_info "已取消写入。"
      return 0
    }
  else
    common_confirm_exact "确认写入外参" "写入" || {
      common_info "已取消写入。"
      return 0
    }
  fi

  common_yaml_write_fields_pair \
    "$calibration_config" "$standard_config" "$candidate" \
    R_camera2gimbal t_camera2gimbal || return 1
  common_ok "外参已写入并完成重新解析验证。"
}
