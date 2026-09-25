#!/usr/bin/env bash

# Log and debug-data adapters. Cleanup is restricted to regular *.log files
# under logs/ and never touches source, models, calibration data, or configs.

# 维护约定：展示、导出、清理分别保持只读、追加输出和显式确认的边界。
# 审查注意：缺少 lsof 时无法排除正在写入的日志；允许清理前还需验证 logs/
# 自身没有指向工程外的符号链接，并在实际删除前重新检查文件占用和路径。

logs_latest_any() {
  local latest=""
  if [[ -d "$SP_VISION_LOG_DIR" ]]; then
    latest="$(find "$SP_VISION_LOG_DIR" -type f -name '*.log' -printf '%T@ %p\n' \
      2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  fi
  [[ -n "$latest" ]] || return 1
  printf '%s\n' "$latest"
}

action_show_latest_log() {
  # 数据来源：logs/ 下最新日志；只读。
  local log_file
  log_file="$(logs_latest_any)" || {
    common_unavailable "logs/ 中没有日志文件。" "$SP_VISION_LOG_DIR/*.log"
    return
  }
  common_info "日志：$log_file"
  common_show_tail "$log_file" 120
}

action_follow_latest_log() {
  # 目标：系统 tail -f；只读取已有日志。
  local log_file transcript rc=0
  log_file="$(logs_latest_any)" || {
    common_unavailable "logs/ 中没有可跟踪的日志文件。" "$SP_VISION_LOG_DIR/*.log"
    return
  }
  common_info "实时跟踪（Ctrl+C 停止）：$log_file"
  transcript="$(common_log_path follow_log)"
  registry_command_tail -n 120 -f -- "$log_file"
  process_run_managed "实时跟踪日志" 0 "$transcript" "${REGISTRY_COMMAND[@]}" || rc=$?
  common_info "日志跟踪退出码：$rc"
  return "$rc"
}

action_show_camera_errors() {
  # 数据来源：已有相机日志；只读。
  local log_file
  log_file="$(common_latest_log camera)" || {
    common_unavailable "没有找到可识别的相机日志。" "logs/*.log 中的相机输出"
    return
  }
  common_info "相机日志：$log_file"
  if grep -inE 'error|warn|fail|失败|错误|断开' "$log_file" | tail -n 100; then
    return 0
  fi
  common_info "日志中未发现明确错误或警告行。"
}

action_show_detection_statistics() {
  # 数据来源：已有识别日志；只读。
  action_detection_statistics
}

action_export_diagnostics() {
  # 目标：logs/debug_menu/export_*；只复制日志和配置字段摘要。
  local export_dir="$SP_VISION_LOG_DIR/debug_menu/export_$(common_timestamp)"
  local source_file target_file
  local copied=0
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "将导出诊断材料到：$export_dir"
    return 0
  fi
  common_require_under "$export_dir" "$SP_VISION_LOG_DIR" "导出目录" || return 1
  mkdir -p -- "$export_dir" || return 1

  while IFS= read -r -d '' source_file; do
    target_file="$export_dir/${source_file##*/}"
    if [[ -f "$target_file" ]]; then
      target_file="$export_dir/${source_file##*/}.$(printf '%s' "$source_file" | cksum | cut -d' ' -f1)"
    fi
    cp -- "$source_file" "$target_file" && copied=$((copied + 1))
  done < <(find "$SP_VISION_LOG_DIR" -maxdepth 2 -type f -name '*.log' -print0 2>/dev/null)

  {
    printf 'SP-Vision 诊断导出\n'
    printf '时间：%s\n' "$(date '+%Y-%m-%d %H:%M:%S %z')"
    printf '项目根目录：%s\n' "$SP_VISION_ROOT"
    printf '导出日志数量：%s\n' "$copied"
    printf '\n正式配置字段摘要：\n'
    local config
    config="$(registry_config standard)" || return 1
    for key in camera_name enemy_color inference_backend yolo_name min_confidence \
      xuc_enable xuc_allow_control xuc_allow_shoot; do
      common_yaml_get "$config" "$key" 2>/dev/null | sed "s/^/$key: /" || true
    done
  } >"$export_dir/summary.txt"

  common_ok "已导出 $copied 份日志。"
  common_info "导出目录：$export_dir"
}

logs_cleanup_candidates() {
  local age_days=${1:-30}
  find "$SP_VISION_LOG_DIR" -type f -name '*.log' -mtime "+$age_days" \
    ! -path "$SP_VISION_RUNTIME_DIR/*" -print0 2>/dev/null
}

action_cleanup_logs() {
  # 目标：logs/ 下指定天数的 *.log；绝不删除源码、模型或正式配置。
  local age_days answer
  local -a candidates=()
  local file size total=0 invalid=0
  common_read "请输入仅清理多少天以前的日志（默认 30）：" answer || return 1
  [[ -n "$answer" ]] || answer=30
  if [[ ! "$answer" =~ ^[0-9]+$ ]]; then
    common_error "天数必须为非负整数。"
    return 1
  fi
  age_days="$answer"
  while IFS= read -r -d '' file; do
    if ! common_require_under "$file" "$SP_VISION_LOG_DIR" "日志文件" >/dev/null 2>&1; then
      invalid=$((invalid + 1))
      continue
    fi
    if command -v lsof >/dev/null 2>&1 && lsof -t -- "$file" 2>/dev/null | grep -q .; then
      invalid=$((invalid + 1))
      continue
    fi
    candidates+=("$file")
    size="$(stat -c '%s' "$file" 2>/dev/null || printf 0)"
    total=$((total + size))
  done < <(logs_cleanup_candidates "$age_days")

  common_title "清理目标摘要"
  common_info "允许目录：$SP_VISION_LOG_DIR"
  common_info "筛选规则：*.log 且修改时间早于 $age_days 天"
  common_info "文件数量：${#candidates[@]}"
  common_info "总大小：$total 字节"
  if ((invalid)); then
    common_warn "有 $invalid 个候选路径未通过目录安全验证，已排除。"
  fi
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：不会删除上述日志。"
    return 0
  fi
  if ((${#candidates[@]} == 0)); then
    common_info "没有符合条件的日志。"
    return 0
  fi
  printf '%s\n' "${candidates[@]}" | sed -n '1,40p'
  if ((${#candidates[@]} > 40)); then
    common_info "... 其余 $(( ${#candidates[@]} - 40 )) 个文件未展开显示"
  fi
  common_confirm_exact "确认只删除上述过期日志" "CLEAN" || {
    common_info "已取消清理。"
    return 0
  }
  for file in "${candidates[@]}"; do
    common_require_under "$file" "$SP_VISION_LOG_DIR" "日志文件" || continue
    rm -f -- "$file" || true
  done
  common_ok "过期日志清理完成；源码、模型、标定数据和正式配置未触碰。"
}

action_show_disk_usage() {
  # 数据来源：du logs/data/build；只读。
  local path
  for path in "$SP_VISION_LOG_DIR" "$SP_VISION_DATA_DIR" "$SP_VISION_BUILD_DIR"; do
    if [[ -e "$path" ]]; then
      du -sh -- "$path" 2>/dev/null || true
    else
      common_info "不存在：$path"
    fi
  done
}
