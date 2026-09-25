#!/usr/bin/env bash

# Central command registry. All executable paths, build targets, configuration
# paths, and shared command-line arguments are defined only in this file.

# 命令契约：registry_command_* 只填充 REGISTRY_COMMAND 数组，不启动进程。
# process_run_managed 接收 REGISTRY_COMMAND 数组的展开结果；动作构造命令后须
# 显式传递 "${REGISTRY_COMMAND[@]}"，避免使用可能陈旧的全局命令。
# 维护参数时须核对目标源码和实际构建程序，特别是位置参数与命名选项的区别。

registry_binary() {
  local key=$1
  case "$key" in
    camera_test) printf '%s\n' "$SP_VISION_BUILD_DIR/camera_test" ;;
    camera_detect_test) printf '%s\n' "$SP_VISION_BUILD_DIR/camera_detect_test" ;;
    camera_pnp_test) printf '%s\n' "$SP_VISION_BUILD_DIR/camera_pnp_test" ;;
    detector_video_test) printf '%s\n' "$SP_VISION_BUILD_DIR/detector_video_test" ;;
    capture) printf '%s\n' "$SP_VISION_BUILD_DIR/capture" ;;
    calibrate_camera) printf '%s\n' "$SP_VISION_BUILD_DIR/calibrate_camera" ;;
    calibrate_handeye) printf '%s\n' "$SP_VISION_BUILD_DIR/calibrate_handeye" ;;
    calibrate_robotworld_handeye)
      printf '%s\n' "$SP_VISION_BUILD_DIR/calibrate_robotworld_handeye"
      ;;
    verify_handeye) printf '%s\n' "$SP_VISION_BUILD_DIR/verify_handeye" ;;
    *)
      common_error "未知命令注册项：$key"
      return 1
      ;;
  esac
}

registry_config() {
  local key=$1
  case "$key" in
    standard) printf '%s\n' "$SP_VISION_CONFIG_DIR/standard.yaml" ;;
    calibration) printf '%s\n' "$SP_VISION_CONFIG_DIR/calibration.yaml" ;;
    *)
      common_error "未知配置注册项：$key"
      return 1
      ;;
  esac
}

registry_dir() {
  local key=$1
  case "$key" in
    root) printf '%s\n' "$SP_VISION_ROOT" ;;
    build) printf '%s\n' "$SP_VISION_BUILD_DIR" ;;
    logs) printf '%s\n' "$SP_VISION_LOG_DIR" ;;
    runtime) printf '%s\n' "$SP_VISION_RUNTIME_DIR" ;;
    data) printf '%s\n' "$SP_VISION_DATA_DIR" ;;
    debug_menu) printf '%s\n' "$SP_VISION_SCRIPT_DIR" ;;
    *) common_error "未知目录注册项：$key"; return 1 ;;
  esac
}

registry_build_script() {
  printf '%s\n' "$SP_VISION_ROOT/scripts/build.sh"
}

registry_set_command() {
  REGISTRY_COMMAND=("$@")
}

registry_command_camera_test() {
  # 目标：camera_test；参数来自 camera_test.cpp 的 config-path/display。
  local config=$1
  local display=${2:-0}
  local binary
  binary="$(registry_binary camera_test)" || return 1
  local -a command=("$binary" "-c=$config")
  ((display)) && command+=("-d")
  registry_set_command "${command[@]}"
}

registry_command_camera_pnp() {
  # 目标：camera_pnp_test；参数来自 camera_pnp_test.cpp 的 config-path/tradition。
  local config=$1
  local algorithm=$2
  local binary
  binary="$(registry_binary camera_pnp_test)" || return 1
  local -a command=("$binary" "$config")
  [[ "$algorithm" == "traditional" ]] && command+=("--tradition")
  registry_set_command "${command[@]}"
}

registry_command_camera_detect() {
  # 目标：camera_detect_test；参数来自 camera_detect_test.cpp 的 config/image/tradition。
  local config=$1
  local image=$2
  local algorithm=$3
  local binary
  binary="$(registry_binary camera_detect_test)" || return 1
  local -a command=("$binary" "$config" "$image")
  [[ "$algorithm" == "traditional" ]] && command+=("--tradition")
  registry_set_command "${command[@]}"
}

registry_command_video_detect() {
  # 目标：detector_video_test；参数来自 detector_video_test.cpp 的 config/start/end/video。
  local config=$1
  local start_index=$2
  local end_index=$3
  local video=$4
  local algorithm=$5
  local binary
  binary="$(registry_binary detector_video_test)" || return 1
  local -a command=(
    "$binary"
    "-c=$config"
    "-s=$start_index"
    "-e=$end_index"
    "$video"
  )
  [[ "$algorithm" == "traditional" ]] && command+=("--tradition")
  registry_set_command "${command[@]}"
}

registry_command_capture() {
  # 目标：capture；参数来自 capture.cpp 的 config-path/output-folder。
  local config=$1
  local output_dir=$2
  local binary
  binary="$(registry_binary capture)" || return 1
  registry_set_command "$binary" "-c=$config" "-o=$output_dir"
}

registry_command_calibrate_camera() {
  # 目标：calibrate_camera；参数来自 calibrate_camera.cpp 的 config/no-display/input。
  local config=$1
  local input_dir=$2
  local binary
  binary="$(registry_binary calibrate_camera)" || return 1
  registry_set_command "$binary" "-c=$config" "--no-display" "$input_dir"
}

registry_command_calibrate_handeye() {
  # 目标：calibrate_handeye；参数来自 calibrate_handeye.cpp 的 config/no-display/input。
  local config=$1
  local input_dir=$2
  local binary
  binary="$(registry_binary calibrate_handeye)" || return 1
  registry_set_command "$binary" "-c=$config" "--no-display" "$input_dir"
}

registry_command_calibrate_robotworld_handeye() {
  # 目标：calibrate_robotworld_handeye；参数来自其 main 的 config/input。
  local config=$1
  local input_dir=$2
  local binary
  binary="$(registry_binary calibrate_robotworld_handeye)" || return 1
  registry_set_command "$binary" "-c=$config" "$input_dir"
}

registry_command_verify_handeye() {
  # 目标：verify_handeye；参数来自 verify_handeye.cpp 的 config/extrinsics/input。
  local calibration_config=$1
  local standard_config=$2
  local input_dir=$3
  local verbose=${4:-0}
  local output_csv=${5:-}
  local display=${6:-0}
  local binary
  binary="$(registry_binary verify_handeye)" || return 1
  local -a command=(
    "$binary"
    "-c=$calibration_config"
    "-e=$standard_config"
    "-i=$input_dir"
  )
  ((verbose)) && command+=("--verbose")
  [[ -n "$output_csv" ]] && command+=("--output-csv=$output_csv")
  ((display)) && command+=("--display")
  registry_set_command "${command[@]}"
}

registry_command_build() {
  # 目标：scripts/build.sh；显式目标由 registry_build_targets 提供。
  local build_script
  build_script="$(registry_build_script)" || return 1
  registry_set_command "bash" "$build_script" "$@"
}

registry_command_system() {
  registry_set_command "$@"
}

registry_command_tail() {
  local tail_binary
  tail_binary="$(command -v tail)" || {
    common_error "缺少系统命令：tail"
    return 1
  }
  registry_set_command "$tail_binary" "$@"
}

registry_build_targets() {
  local kind=$1
  case "$kind" in
    camera) printf '%s\n' camera_test ;;
    detection) printf '%s\n' camera_detect_test camera_pnp_test detector_video_test ;;
    calibration)
      printf '%s\n' capture calibrate_camera calibrate_handeye \
        calibrate_robotworld_handeye verify_handeye
      ;;
    all) printf '%s\n' all ;;
    *) common_error "未知构建目标组：$kind"; return 1 ;;
  esac
}

registry_model_path() {
  local config=$1
  local key=$2
  common_yaml_get "$config" "$key"
}

registry_model_keys_for_backend() {
  local config=$1
  local backend yolo_name
  backend="$(common_yaml_get "$config" inference_backend)" || return 1
  yolo_name="$(common_yaml_get "$config" yolo_name)" || return 1
  case "$backend" in
    openvino)
      printf '%s\n' "${yolo_name}_model_path"
      ;;
    tensorrt)
      printf '%s\n' "${yolo_name}_engine_path"
      ;;
    *)
      common_error "正式配置中的推理后端不受支持：$backend"
      return 1
      ;;
  esac
}

registry_print_command() {
  local item
  printf '将调用：'
  for item in "${REGISTRY_COMMAND[@]}"; do
    printf ' %q' "$item"
  done
  printf '\n'
}

registry_required_interface_keys() {
  printf '%s\n' \
    camera_test \
    camera_detect_test \
    camera_pnp_test \
    detector_video_test \
    capture \
    calibrate_camera \
    calibrate_handeye \
    calibrate_robotworld_handeye \
    verify_handeye
}

registry_invoke_main() {
  case "$1" in
    1) menu_intrinsics ;;
    2) menu_extrinsics ;;
    3) menu_detection ;;
    4) menu_camera ;;
    5) menu_config ;;
    6) menu_environment ;;
    7) menu_build ;;
    8) menu_logs ;;
    9) menu_processes ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_intrinsics() {
  case "$1" in
    1) action_capture_intrinsics ;;
    2) action_calibrate_intrinsics ;;
    3) action_check_intrinsics ;;
    4) action_view_intrinsics ;;
    5) action_write_intrinsics ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_extrinsics() {
  case "$1" in
    1) action_input_muzzle_coordinates ;;
    2) action_capture_handeye ;;
    3) action_calibrate_handeye ;;
    4) action_check_handeye ;;
    5) action_view_extrinsics ;;
    6) action_write_extrinsics ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_detection() {
  case "$1" in
    1) action_live_armor_detection ;;
    2) action_live_traditional_detection ;;
    3) action_compare_detection ;;
    4) action_single_image_detection ;;
    5) action_video_detection ;;
    6) action_select_enemy_color ;;
    7) action_adjust_confidence ;;
    8) action_save_detection_frame ;;
    9) action_detection_statistics ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_camera() {
  case "$1" in
    1) action_camera_preview ;;
    2) action_camera_parameters ;;
    3) action_adjust_exposure ;;
    4) action_adjust_gain ;;
    5) action_adjust_white_balance ;;
    6) action_channel_check ;;
    7) action_save_raw_frame ;;
    8) action_save_display_frame ;;
    9) action_restore_camera_parameters ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_config() {
  case "$1" in
    1) action_view_standard_config ;;
    2) config_action_view_intrinsics ;;
    3) config_action_view_extrinsics ;;
    4) config_action_select_enemy_color ;;
    5) config_action_view_detection_threshold ;;
    6) config_action_modify_detection_threshold ;;
    7) config_action_check_model_paths ;;
    8) config_action_check_control_switches ;;
    9) config_action_validate_yaml ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_environment() {
  case "$1" in
    1) action_environment_quick ;;
    2) action_environment_camera ;;
    3) action_environment_opencv ;;
    4) action_environment_openvino ;;
    5) action_environment_ros ;;
    6) action_environment_models ;;
    7) action_environment_libraries ;;
    8) action_environment_usb ;;
    9) action_environment_display ;;
    10) action_environment_report ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_build() {
  case "$1" in
    1) action_build_incremental ;;
    2) action_build_clean ;;
    3) action_build_camera ;;
    4) action_build_detection ;;
    5) action_build_calibration ;;
    6) action_build_all ;;
    7) action_show_last_build_error ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_logs() {
  case "$1" in
    1) action_show_latest_log ;;
    2) action_follow_latest_log ;;
    3) action_show_camera_errors ;;
    4) action_show_detection_statistics ;;
    5) action_export_diagnostics ;;
    6) action_cleanup_logs ;;
    7) action_show_disk_usage ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_invoke_processes() {
  case "$1" in
    1) action_list_menu_processes ;;
    2) action_stop_current_process ;;
    3) action_stop_all_menu_processes ;;
    4) action_check_camera_occupancy ;;
    5) action_release_camera_resource ;;
    6) action_check_residual_windows ;;
    0) return 100 ;;
    *) return 64 ;;
  esac
}

registry_validate_interface() {
  local key=$1
  local binary output
  local rc=0
  binary="$(registry_binary "$key")" || return 1

  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "将检查接口：$binary --help"
    return 0
  fi
  if [[ ! -x "$binary" ]]; then
    common_interface_incompatible "缺少可执行文件：$binary"
    return 1
  fi

  if command -v timeout >/dev/null 2>&1; then
    output="$(timeout 8 "$binary" --help 2>&1)" || rc=$?
  else
    output="$("$binary" --help 2>&1)" || rc=$?
  fi
  if ((rc != 0)); then
    common_interface_incompatible "无法读取 $key --help，退出码 $rc"
    return 1
  fi

  local required
  case "$key" in
    camera_test) required='config-path display' ;;
    camera_detect_test) required='config-path image-path tradition' ;;
    camera_pnp_test) required='config-path tradition' ;;
    detector_video_test) required='config-path start-index end-index video_path tradition' ;;
    capture) required='config-path output-folder' ;;
    calibrate_camera|calibrate_handeye) required='config-path no-display input-folder' ;;
    calibrate_robotworld_handeye) required='config-path input-folder' ;;
    verify_handeye) required='config-path extrinsics-path input-folder output-csv display' ;;
    *) required='' ;;
  esac
  for required in $required; do
    if ! grep -qF -- "$required" <<<"$output"; then
      common_interface_incompatible "$key 不再声明必需参数：$required"
      return 1
    fi
  done
}

registry_validate_all_interfaces() {
  local key failures=0
  while IFS= read -r key; do
    registry_validate_interface "$key" || failures=$((failures + 1))
  done < <(registry_required_interface_keys)
  if ((failures)); then
    common_error "接口检查失败项：$failures"
    return 1
  fi
  common_ok "已声明 CLI 接口检查完成。"
}
