#!/usr/bin/env bash

# Environment and resource checks. This file never opens a camera or starts a
# business program; it only inspects files, environment variables, and devices.

# 检查报告区分“发现文件/设备”与“可实际使用”：USB 枚举不证明没有占用，
# 显示环境变量不证明有会话权限，动态库存在不证明 ABI 兼容。
# 接口报告会调用目标 --help，安全性取决于目标程序先处理帮助再初始化硬件。

environment_check_root() {
  local failures=0
  common_title "项目根目录"
  if common_require_root; then
    common_ok "根目录：$SP_VISION_ROOT"
  else
    failures=1
  fi
  local required
  for required in \
    CMakeLists.txt \
    configs/standard.yaml \
    configs/calibration.yaml \
    scripts/build.sh \
    scripts/check_source_tree.py; do
    if [[ -f "$SP_VISION_ROOT/$required" ]]; then
      common_ok "存在：$required"
    else
      common_error "缺少：$required"
      failures=1
    fi
  done
  return "$failures"
}

environment_check_ros() {
  local setup_file="${SP_VISION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  common_title "ROS 2 环境"
  if [[ "${SP_VISION_ROS_LOADED:-0}" == "1" ]]; then
    common_ok "已加载：$setup_file"
  elif [[ -f "$setup_file" ]]; then
    common_warn "环境文件存在，但当前菜单进程尚未加载：$setup_file"
  else
    common_warn "未发现 $setup_file；非 ROS 工具仍可使用。"
  fi
  if command -v ros2 >/dev/null 2>&1; then
    common_ok "ros2：$(command -v ros2)"
  else
    common_info "ros2：当前 PATH 中不可用"
  fi
  [[ -n "${AMENT_PREFIX_PATH:-}" ]] && common_ok "AMENT_PREFIX_PATH 已设置"
  return 0
}

environment_check_opencv() {
  common_title "OpenCV"
  local version=""
  if command -v pkg-config >/dev/null 2>&1; then
    version="$(pkg-config --modversion opencv4 2>/dev/null || true)"
  fi
  if [[ -n "$version" ]]; then
    common_ok "OpenCV 版本：$version"
    return 0
  fi
  if command -v ldconfig >/dev/null 2>&1 &&
    ldconfig -p 2>/dev/null | grep -q 'libopencv_core'; then
    common_ok "已发现 OpenCV 动态库；pkg-config 未报告版本。"
    return 0
  fi
  common_error "未发现 OpenCV 4 运行库或 pkg-config 元数据。"
  return 1
}

environment_check_openvino() {
  common_title "OpenVINO"
  local root="/opt/intel/openvino_2024.6.0"
  local found=0
  [[ -d "$root" ]] && found=1
  if command -v ldconfig >/dev/null 2>&1 &&
    ldconfig -p 2>/dev/null | grep -Eq 'libopenvino|libopenvino_c'; then
    found=1
  fi
  if ((found)); then
    common_ok "已发现 OpenVINO 安装或运行库。"
    return 0
  fi
  common_warn "未发现 OpenVINO；仅影响使用 OpenVINO 后端的识别测试。"
  return 1
}

environment_check_camera() {
  common_title "相机与 MVS"
  local config
  config="$(registry_config standard)" || return 1
  local camera_name vid_pid
  camera_name="$(common_yaml_get "$config" camera_name 2>/dev/null || printf '未配置')"
  vid_pid="$(common_yaml_get "$config" vid_pid 2>/dev/null || true)"
  common_info "配置相机类型：$camera_name"
  [[ -n "$vid_pid" ]] && common_info "配置 VID:PID：$vid_pid"

  local status=0
  if command -v lsusb >/dev/null 2>&1; then
    if [[ -n "$vid_pid" ]] && lsusb -d "$vid_pid" 2>/dev/null | grep -q .; then
      common_ok "USB 设备已发现：$vid_pid"
    else
      common_warn "未通过 lsusb 发现配置的相机 VID:PID。"
      status=1
    fi
  else
    common_warn "缺少 lsusb，无法检查 USB 相机。"
    status=1
  fi

  if command -v ldconfig >/dev/null 2>&1 &&
    ldconfig -p 2>/dev/null | grep -qi 'MvCameraControl'; then
    common_ok "已发现海康 MVS 运行库。"
  else
    common_warn "未通过 ldconfig 发现海康 MVS 运行库。"
    status=1
  fi
  return "$status"
}

environment_model_info() {
  local config=$1
  local key value resolved
  key="$(registry_model_keys_for_backend "$config")" || return 1
  value="$(common_yaml_get "$config" "$key")" || return 1
  resolved="$value"
  if [[ "$resolved" != /* ]]; then
    resolved="$SP_VISION_ROOT/$resolved"
  fi
  printf '%s|%s|%s\n' "$key" "$value" "$resolved"
}

environment_check_models() {
  common_title "模型文件"
  local config status=0
  config="$(registry_config standard)" || return 1
  local info key value resolved classify classify_path
  if info="$(environment_model_info "$config")"; then
    IFS='|' read -r key value resolved <<<"$info"
    common_info "推理模型字段：$key=$value"
    if [[ -f "$resolved" ]]; then
      common_ok "模型存在：$resolved"
      if [[ "$resolved" == *.xml && ! -f "${resolved%.xml}.bin" ]]; then
        common_error "OpenVINO 模型缺少同名 .bin：${resolved%.xml}.bin"
        status=1
      fi
    else
      common_error "模型不存在：$resolved"
      status=1
    fi
  else
    common_error "无法从正式配置解析当前推理后端对应的模型字段。"
    status=1
  fi

  classify="$(common_yaml_get "$config" classify_model 2>/dev/null || true)"
  if [[ -n "$classify" ]]; then
    classify_path="$classify"
    [[ "$classify_path" == /* ]] || classify_path="$SP_VISION_ROOT/$classify_path"
    if [[ -f "$classify_path" ]]; then
      common_ok "分类模型存在：$classify_path"
    else
      common_warn "分类模型不存在：$classify_path"
    fi
  fi
  return "$status"
}

environment_check_libraries() {
  common_title "动态库"
  local key binary status=0
  for key in camera_test camera_detect_test camera_pnp_test capture verify_handeye; do
    binary="$(registry_binary "$key")" || continue
    if [[ ! -x "$binary" ]]; then
      common_warn "未构建，跳过 ldd：$binary"
      continue
    fi
    if command -v ldd >/dev/null 2>&1; then
      if ldd "$binary" 2>/dev/null | grep -q 'not found'; then
        common_error "存在未解析动态库：$binary"
        ldd "$binary" 2>/dev/null | grep 'not found' || true
        status=1
      else
        common_ok "动态库可解析：$key"
      fi
    else
      common_warn "缺少 ldd，无法检查：$binary"
    fi
  done
  return "$status"
}

environment_usb_device_path() {
  local vid_pid=$1
  local line bus device
  command -v lsusb >/dev/null 2>&1 || return 1
  line="$(lsusb -d "$vid_pid" 2>/dev/null | head -n 1)"
  [[ -n "$line" ]] || return 1
  bus="$(awk '{gsub(":", "", $2); print $2}' <<<"$line")"
  device="$(awk '{gsub(":", "", $4); print $4}' <<<"$line")"
  [[ "$bus" =~ ^[0-9]+$ && "$device" =~ ^[0-9]+$ ]] || return 1
  printf '/dev/bus/usb/%03d/%03d\n' "$((10#$bus))" "$((10#$device))"
}

environment_check_usb_permissions() {
  common_title "USB 权限"
  local config vid_pid device mode owner
  config="$(registry_config standard)" || return 1
  vid_pid="$(common_yaml_get "$config" vid_pid 2>/dev/null || true)"
  if [[ -z "$vid_pid" ]]; then
    common_warn "正式配置没有 vid_pid，无法检查。"
    return 1
  fi
  device="$(environment_usb_device_path "$vid_pid")" || {
    common_error "无法从 lsusb 解析设备节点：$vid_pid"
    return 1
  }
  if [[ ! -e "$device" ]]; then
    common_error "设备节点不存在：$device"
    return 1
  fi
  mode="$(stat -c '%a' "$device" 2>/dev/null || true)"
  owner="$(stat -c '%U:%G' "$device" 2>/dev/null || true)"
  common_info "设备：$device  权限：${mode:-未知}  归属：${owner:-未知}"
  if [[ -r "$device" && -w "$device" ]]; then
    common_ok "当前用户具有读写权限。"
    return 0
  fi
  common_error "当前用户没有设备读写权限。"
  return 1
}

environment_check_display() {
  common_title "图形显示环境"
  common_info "DISPLAY=${DISPLAY:-未设置}"
  common_info "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-未设置}"
  if common_display_available; then
    common_ok "检测到图形显示会话。"
    return 0
  fi
  common_error "未检测到图形显示会话；带 imshow 的测试不可用。"
  return 1
}

environment_check_build() {
  common_title "构建目录"
  local build
  build="$(registry_dir build)" || return 1
  if [[ -d "$build" ]]; then
    common_ok "构建目录存在：$build"
    return 0
  fi
  common_error "构建目录不存在：$build"
  return 1
}

environment_quick_check() {
  local failures=0
  environment_check_root || failures=$((failures + 1))
  environment_check_build || failures=$((failures + 1))
  environment_check_ros || true
  environment_check_opencv || failures=$((failures + 1))
  environment_check_openvino || true
  environment_check_camera || failures=$((failures + 1))
  environment_check_models || failures=$((failures + 1))
  environment_check_libraries || failures=$((failures + 1))
  environment_check_usb_permissions || true
  environment_check_display || failures=$((failures + 1))

  common_title "快速检查结果"
  if ((failures)); then
    common_error "关键检查失败项：$failures。菜单仍可执行不依赖这些资源的查看、构建和日志操作。"
    return 1
  fi
  common_ok "环境和资源快速检查通过。"
  return 0
}

environment_preflight() {
  local root_ok=1 build_ok=1 config_ok=1 python_ok=1 camera_ok=1 model_ok=1 display_ok=1
  local binary missing_binaries=0
  local model_info
  common_require_root >/dev/null 2>&1 || root_ok=0
  [[ -d "$SP_VISION_BUILD_DIR" ]] || build_ok=0
  [[ -f "$SP_VISION_CONFIG_DIR/standard.yaml" && -f "$SP_VISION_CONFIG_DIR/calibration.yaml" ]] ||
    config_ok=0
  [[ -n "${SP_VISION_PYTHON:-}" ]] || python_ok=0
  environment_check_camera >/dev/null 2>&1 || camera_ok=0
  common_display_available || display_ok=0
  if model_info="$(environment_model_info "$(registry_config standard)" 2>/dev/null)"; then
    IFS='|' read -r _ _ model_info <<<"$model_info"
    [[ -f "$model_info" ]] || model_ok=0
  else
    model_ok=0
  fi
  while IFS= read -r binary; do
    [[ -x "$binary" ]] || missing_binaries=$((missing_binaries + 1))
  done < <(registry_required_interface_keys | while IFS= read -r key; do registry_binary "$key"; done)

  common_info "启动预检：根目录=$([[ $root_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "配置=$([[ $config_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "build=$([[ $build_ok == 1 ]] && printf 存在 || printf 缺失) " \
    "Python=$([[ $python_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "相机=$([[ $camera_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "模型=$([[ $model_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "显示=$([[ $display_ok == 1 ]] && printf 正常 || printf 缺失) " \
    "入口缺失=$missing_binaries"
  if [[ "$build_ok" != "1" ]]; then
    common_warn "未发现 build/；相机、标定和识别运行项会安全失败，可先使用项目编译菜单。"
  fi
  if ((missing_binaries)); then
    common_warn "有 $missing_binaries 个已声明入口当前不存在；对应菜单项会安全失败并提示维护适配层。"
  fi
}

action_environment_quick() {
  # 数据来源：文件、环境变量、设备节点和已有二进制；不打开相机。
  environment_quick_check
}

action_environment_camera() {
  # 数据来源：lsusb/ldconfig 和 standard.yaml；不打开相机。
  environment_check_camera
}

action_environment_opencv() {
  # 数据来源：pkg-config/ldconfig；只读。
  environment_check_opencv
}

action_environment_openvino() {
  # 数据来源：安装目录/ldconfig；只读。
  environment_check_openvino
}

action_environment_ros() {
  # 数据来源：当前菜单进程环境；只读。
  environment_check_ros
}

action_environment_models() {
  # 数据来源：configs/standard.yaml 和模型文件；只读。
  environment_check_models
}

action_environment_libraries() {
  # 数据来源：ldd 已有可执行文件；不启动程序。
  environment_check_libraries
}

action_environment_usb() {
  # 数据来源：lsusb/stat；只读。
  environment_check_usb_permissions
}

action_environment_display() {
  # 数据来源：DISPLAY/WAYLAND_DISPLAY；只读。
  environment_check_display
}

action_environment_report() {
  # 数据来源：上述检查；报告只写入 logs/debug_menu/。
  local report rc=0
  report="$(common_log_path diagnostic)"
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "将生成完整诊断报告：$report"
    return 0
  fi
  {
    printf 'SP-Vision 诊断报告\n'
    printf '生成时间：%s\n' "$(date '+%Y-%m-%d %H:%M:%S %z')"
    printf '项目根目录：%s\n\n' "$SP_VISION_ROOT"
    environment_check_root || true
    environment_check_build || true
    environment_check_ros || true
    environment_check_camera || true
    environment_check_opencv || true
    environment_check_openvino || true
    environment_check_models || true
    environment_check_libraries || true
    environment_check_usb_permissions || true
    environment_check_display || true
    if declare -F registry_validate_all_interfaces >/dev/null 2>&1; then
      common_title "CLI 接口"
      registry_validate_all_interfaces || true
    fi
  } >"$report" 2>&1 || rc=$?
  common_info "完整诊断报告：$report"
  common_show_tail "$report" 80
  return "$rc"
}
