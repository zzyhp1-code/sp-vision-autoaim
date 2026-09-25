#!/usr/bin/env bash

# Configuration inspection adapters. Ordinary views never modify YAML.

# 这里只展示配置或修改本会话覆盖值；正式结果落盘由 calibration.sh 显式确认。
# common_yaml_get 的文本回退不能证明整份 YAML 合法，安全校验须依赖严格解析。

config_print_key() {
  local config=$1
  local key=$2
  local value
  value="$(common_yaml_get "$config" "$key")" || return 1
  common_info "$key：$value"
}

action_view_standard_config() {
  # 数据来源：configs/standard.yaml；只读。
  local config
  config="$(registry_config standard)" || return 1
  common_require_file "$config" "正式配置" || return 1
  common_yaml_validate "$config" || return 1
  common_info "文件：$config"
  sed -n '1,360p' -- "$config"
}

config_action_view_intrinsics() {
  # 数据来源：configs/calibration.yaml 和 configs/standard.yaml；只读。
  local calibration standard key
  calibration="$(registry_config calibration)" || return 1
  standard="$(registry_config standard)" || return 1
  common_title "标定配置中的内参"
  for key in camera_matrix distort_coeffs; do
    config_print_key "$calibration" "$key"
  done
  common_title "正式配置中的内参"
  for key in camera_matrix distort_coeffs; do
    config_print_key "$standard" "$key"
  done
}

config_action_view_extrinsics() {
  # 数据来源：configs/calibration.yaml 和 configs/standard.yaml；只读。
  local calibration standard key
  calibration="$(registry_config calibration)" || return 1
  standard="$(registry_config standard)" || return 1
  common_title "标定配置中的外参"
  for key in R_gimbal2imubody R_camera2gimbal t_camera2gimbal; do
    config_print_key "$calibration" "$key"
  done
  common_title "正式配置中的外参"
  for key in R_gimbal2imubody R_camera2gimbal t_camera2gimbal; do
    config_print_key "$standard" "$key"
  done
}

config_action_select_enemy_color() {
  # 数据来源：当前菜单会话；不写入正式配置。
  detection_set_enemy_color_for_session
}

config_action_view_detection_threshold() {
  # 数据来源：configs/standard.yaml 和当前会话覆盖；只读。
  local config key
  config="$(registry_config standard)" || return 1
  for key in min_confidence min_keypoint_confidence threshold \
    yolo26_candidate_confidence yolo26_color_confirmed_min_confidence; do
    if common_yaml_get "$config" "$key" >/dev/null 2>&1; then
      config_print_key "$config" "$key"
    fi
  done
  common_info "本次菜单会话临时最低置信度：${SP_VISION_TEST_MIN_CONFIDENCE:-未覆盖}"
}

config_action_modify_detection_threshold() {
  # 数据来源：当前菜单会话；不写入正式配置。
  detection_adjust_confidence
}

config_action_check_model_paths() {
  # 数据来源：configs/standard.yaml 的模型字段；只读。
  local config info key value resolved classify classify_path
  config="$(registry_config standard)" || return 1
  if info="$(environment_model_info "$config")"; then
    IFS='|' read -r key value resolved <<<"$info"
    common_info "推理模型字段：$key=$value"
    common_info "解析路径：$resolved"
    if [[ -f "$resolved" ]]; then
      common_ok "推理模型存在。"
      if [[ "$resolved" == *.xml && ! -f "${resolved%.xml}.bin" ]]; then
        common_error "模型缺少同名 .bin：${resolved%.xml}.bin"
        return 1
      fi
    else
      common_error "推理模型不存在。"
      return 1
    fi
  else
    common_error "无法解析当前推理后端的模型字段。"
    return 1
  fi

  classify="$(common_yaml_get "$config" classify_model 2>/dev/null || true)"
  if [[ -n "$classify" ]]; then
    classify_path="$classify"
    [[ "$classify_path" == /* ]] || classify_path="$SP_VISION_ROOT/$classify_path"
    common_info "分类模型字段：classify_model=$classify"
    common_info "解析路径：$classify_path"
    if [[ -f "$classify_path" ]]; then
      common_ok "分类模型存在。"
    else
      common_warn "分类模型不存在；是否可用取决于当前识别后端。"
    fi
  fi
}

config_action_check_control_switches() {
  # 数据来源：configs/standard.yaml；只读并报告菜单强制安全值。
  local config key value unsafe=0
  config="$(registry_config standard)" || return 1
  common_title "正式配置开关"
  for key in xuc_enable xuc_allow_control xuc_allow_shoot auto_fire; do
    value="$(common_yaml_get "$config" "$key" 2>/dev/null || printf '未配置')"
    common_info "$key：$value"
    [[ "$key" == "auto_fire" ]] && continue
    [[ "$value" == "false" ]] || unsafe=1
  done
  common_title "菜单强制运行配置"
  common_info "xuc_enable=false"
  common_info "xuc_allow_control=false"
  common_info "xuc_allow_shoot=false"
  if ((unsafe)); then
    common_warn "正式配置包含控制/发射开启项；菜单不会把正式配置直接交给测试程序。"
  else
    common_ok "正式配置中的三项控制开关当前为 false。"
  fi
}

config_action_validate_yaml() {
  # 数据来源：两份正式 YAML；只读解析验证。
  local calibration standard failures=0
  calibration="$(registry_config calibration)" || return 1
  standard="$(registry_config standard)" || return 1
  common_yaml_validate "$calibration" || failures=$((failures + 1))
  common_yaml_validate "$standard" || failures=$((failures + 1))
  ((failures == 0))
}
