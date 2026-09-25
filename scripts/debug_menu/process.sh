#!/usr/bin/env bash

# Managed process handling. Only processes started by this menu are stopped.

# 实际边界：当前记录由所有菜单实例共享，尚不能保证“仅本次会话”所有权。
# PID、PGID、启动时间用于识别同一进程，不能取代会话标识和实例互斥锁。
# 以下审查注释说明尚未满足的保证，不代表已完成行为修复。

process_active_file() {
  # 记录文件与当前菜单实例绑定，退出时只清理本实例创建的任务。
  [[ "${SP_VISION_MENU_SESSION_ID:-}" =~ ^[A-Za-z0-9_.-]+$ ]] || return 1
  printf '%s\n' "$SP_VISION_RUNTIME_DIR/process_${SP_VISION_MENU_SESSION_ID}"
}

process_starttime_for_pid() {
  local pid=$1
  local stat_line rest
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/$pid/stat" ]] || return 1
  stat_line="$(<"/proc/$pid/stat")" || return 1
  rest="${stat_line##*) }"
  # /proc/<pid>/stat after comm: field 3 is token 1, field 22 is token 20.
  set -- $rest
  printf '%s\n' "${20:-}"
}

process_pgid_for_pid() {
  local pid=$1
  local pgid
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')" || return 1
  [[ "$pgid" =~ ^[0-9]+$ ]] || return 1
  [[ "$pgid" != "1" ]] || return 1
  printf '%s\n' "$pgid"
}

process_read_record() {
  local file
  file="$(process_active_file)"
  [[ -f "$file" ]] || return 1
  IFS='|' read -r PROCESS_RECORD_PID PROCESS_RECORD_PGID PROCESS_RECORD_STARTTIME PROCESS_RECORD_LABEL <"$file"
  [[ "$PROCESS_RECORD_PID" =~ ^[0-9]+$ ]] || return 1
  [[ "$PROCESS_RECORD_PGID" =~ ^[0-9]+$ ]] || return 1
  [[ -n "$PROCESS_RECORD_STARTTIME" && -n "$PROCESS_RECORD_LABEL" ]]
}

process_record_is_running() {
  # 这里检查的是组长进程；组长退出不等于同组所有后代均已退出。
  local pid=$1
  local pgid=$2
  local expected_starttime=$3
  local current_starttime current_pgid state
  [[ "$pid" =~ ^[0-9]+$ && "$pgid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  current_starttime="$(process_starttime_for_pid "$pid")" || return 1
  [[ "$current_starttime" == "$expected_starttime" ]] || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
  [[ -n "$state" && "$state" != Z* ]] || return 1
  current_pgid="$(process_pgid_for_pid "$pid")" || return 1
  [[ "$current_pgid" == "$pgid" ]]
}

process_write_record() {
  local pid=$1
  local pgid=$2
  local starttime=$3
  local label=$4
  local file temporary
  file="$(process_active_file)"
  temporary="$file.$$"
  common_ensure_runtime_dir || return 1
  printf '%s|%s|%s|%s\n' "$pid" "$pgid" "$starttime" "$label" >"$temporary" || return 1
  mv -f -- "$temporary" "$file"
}

process_clear_record() {
  local file
  file="$(process_active_file)"
  rm -f -- "$file"
}

process_assert_no_active() {
  if ! process_read_record; then
    return 0
  fi
  if process_record_is_running \
    "$PROCESS_RECORD_PID" "$PROCESS_RECORD_PGID" "$PROCESS_RECORD_STARTTIME"; then
    common_error "本菜单已有任务在运行：PID $PROCESS_RECORD_PID，PGID $PROCESS_RECORD_PGID，$PROCESS_RECORD_LABEL"
    return 1
  fi
  common_warn "发现陈旧进程记录，已清理：PID $PROCESS_RECORD_PID，$PROCESS_RECORD_LABEL"
  process_clear_record
}

process_acquire_camera_lock() {
  local lock_file="$SP_VISION_RUNTIME_DIR/camera.lock"
  common_require_command flock || return 1
  common_ensure_runtime_dir || return 1
  exec 9>"$lock_file" || return 1
  if ! flock -n 9; then
    common_error "相机资源锁已被本菜单的另一个任务占用。"
    exec 9>&-
    return 1
  fi
  SP_VISION_CAMERA_LOCK_FD=9
}

process_release_camera_lock() {
  if [[ "${SP_VISION_CAMERA_LOCK_FD:-}" == "9" ]]; then
    flock -u 9 2>/dev/null || true
    exec 9>&-
    SP_VISION_CAMERA_LOCK_FD=""
  fi
}

process_camera_lock_available() {
  local lock_file="$SP_VISION_RUNTIME_DIR/camera.lock"
  common_ensure_runtime_dir || return 1
  if [[ "${SP_VISION_CAMERA_LOCK_FD:-}" == "9" ]]; then
    common_error "本菜单当前任务持有相机资源锁。"
    return 1
  fi
  if ! command -v flock >/dev/null 2>&1; then
    common_error "缺少 flock，无法判断菜单相机锁。"
    return 1
  fi
  exec 9>"$lock_file" || return 1
  if ! flock -n 9; then
    exec 9>&-
    common_error "相机资源锁已被本菜单的另一个任务占用。"
    return 1
  fi
  flock -u 9
  exec 9>&-
}

process_start_tail() {
  # 通过变量返回 tail PID，避免将实时日志混入命令替换的返回值。
  local transcript=$1
  local managed_pid=$2
  local output_var=$3
  local tail_pid=""
  if [[ -t 1 ]] && command -v tail >/dev/null 2>&1; then
    tail -n +1 -f --pid="$managed_pid" -- "$transcript" &
    tail_pid=$!
  fi
  printf -v "$output_var" '%s' "$tail_pid"
}

process_stop_tail() {
  local tail_pid=${1:-}
  if [[ "$tail_pid" =~ ^[0-9]+$ ]] && kill -0 "$tail_pid" 2>/dev/null; then
    kill -TERM "$tail_pid" 2>/dev/null || true
    wait "$tail_pid" 2>/dev/null || true
  fi
}

process_run_managed() {
  # 参数：任务标签、是否占用相机、日志路径、可执行文件及逐项参数。
  # 这里只使用第 4 项起的实参；先调用注册表构造命令并不会自动传递到这里。
  local label=$1
  local camera_required=$2
  local transcript=$3
  shift 3
  local -a command=("$@")
  local pid pgid starttime rc=0 tail_pid="" attempt
  local lock_acquired=0 cleanup_status=0

  if ((${#command[@]} == 0)); then
    common_error "内部错误：没有可执行命令。"
    return 1
  fi

  process_assert_no_active || return 1
  if ((camera_required)) && [[ "$SP_VISION_DEBUG_DRY_RUN" != "1" ]]; then
    process_check_external_camera_occupancy || {
      common_error "外部程序可能正在占用相机，拒绝启动本菜单相机功能。"
      return 1
    }
    process_acquire_camera_lock || return 1
    lock_acquired=1
  fi

  registry_print_command
  common_info "工作目录：$SP_VISION_ROOT"
  if [[ -n "$transcript" ]]; then
    common_info "运行记录：$transcript"
  fi

  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    ((lock_acquired)) && process_release_camera_lock
    PROCESS_LAST_EXIT=0
    return 0
  fi

  if [[ ! -x "${command[0]}" && "${command[0]}" != "bash" ]]; then
    common_error "可执行文件不存在或不可执行：${command[0]}"
    ((lock_acquired)) && process_release_camera_lock
    return 127
  fi

  common_require_command setsid || {
    ((lock_acquired)) && process_release_camera_lock
    return 1
  }

  if [[ -n "$transcript" ]]; then
    common_require_under "$transcript" "$SP_VISION_LOG_DIR" "运行记录" || {
      ((lock_acquired)) && process_release_camera_lock
      return 1
    }
    mkdir -p -- "$(dirname -- "$transcript")" || {
      ((lock_acquired)) && process_release_camera_lock
      return 1
    }
    : >"$transcript" || {
      ((lock_acquired)) && process_release_camera_lock
      return 1
    }
  fi

  if [[ -n "$transcript" ]]; then
    (
      cd -- "$SP_VISION_ROOT" || exit 1
      exec setsid --wait "${command[@]}"
    ) >"$transcript" 2>&1 &
  else
    (
      cd -- "$SP_VISION_ROOT" || exit 1
      exec setsid --wait "${command[@]}"
    ) &
  fi
  pid=$!

  for attempt in 1 2 3 4 5; do
    # 记录 PID、启动时间和进程组；目标机验收还需确认 setsid 的组隔离行为。
    starttime="$(process_starttime_for_pid "$pid")" || starttime=""
    pgid="$(process_pgid_for_pid "$pid")" || pgid=""
    if [[ -n "$starttime" && -n "$pgid" ]]; then
      break
    fi
    sleep 0.02
  done

  if [[ -z "$pgid" || -z "$starttime" ]]; then
    common_error "无法记录子进程 PID/进程组，拒绝继续管理该任务。"
    kill -TERM -- "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    ((lock_acquired)) && process_release_camera_lock
    return 1
  fi

  SP_VISION_MANAGED_PID=$pid
  SP_VISION_MANAGED_PGID=$pgid
  SP_VISION_MANAGED_STARTTIME=$starttime
  process_write_record "$pid" "$pgid" "$starttime" "$label" || {
    common_error "无法写入受管进程记录。"
    kill -TERM -- "-$pgid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    ((lock_acquired)) && process_release_camera_lock
    return 1
  }

  if [[ -n "$transcript" ]]; then
    process_start_tail "$transcript" "$pid" tail_pid
  fi

  wait "$pid" || rc=$?
  PROCESS_LAST_EXIT=$rc
  process_stop_tail "$tail_pid"

  if process_record_is_running "$pid" "$pgid" "$starttime"; then
    common_warn "任务主进程已返回，但同一进程组仍有子进程存活；将只停止本菜单启动的进程组。"
    process_stop_group "$pid" "$pgid" "$starttime" 5 || cleanup_status=$?
  fi

  SP_VISION_MANAGED_PID=""
  SP_VISION_MANAGED_PGID=""
  SP_VISION_MANAGED_STARTTIME=""
  if ((cleanup_status == 0)); then
    process_clear_record
  else
    common_warn "进程组清理未确认完成，保留 PID 记录供后续重试。"
  fi
  ((lock_acquired)) && process_release_camera_lock
  if ((cleanup_status != 0)); then
    return "$cleanup_status"
  fi
  return "$rc"
}

process_stop_group() {
  local pid=$1
  local pgid=$2
  local expected_starttime=$3
  local timeout_seconds=${4:-5}
  local elapsed=0
  local attempt label=${PROCESS_RECORD_LABEL:-"受管任务"}

  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：将停止 PID=$pid、PGID=$pgid、$label 对应的精确进程组。"
    return 0
  fi

  if ! process_record_is_running "$pid" "$pgid" "$expected_starttime"; then
    common_info "目标进程已退出，无需停止。"
    return 0
  fi
  if [[ ! "$pgid" =~ ^[0-9]+$ || "$pgid" == "1" ]]; then
    common_error "拒绝向不安全的进程组发送信号：$pgid"
    return 1
  fi

  common_info "向本菜单启动的进程组 $pgid 发送 SIGTERM。"
  kill -TERM -- "-$pgid" 2>/dev/null || true

  while ((elapsed < timeout_seconds * 10)); do
    if ! process_record_is_running "$pid" "$pgid" "$expected_starttime"; then
      common_ok "进程已正常退出。"
      return 0
    fi
    sleep 0.1
    elapsed=$((elapsed + 1))
  done

  if process_record_is_running "$pid" "$pgid" "$expected_starttime"; then
    common_warn "超时未退出，向同一精确进程组发送 SIGKILL。"
    kill -KILL -- "-$pgid" 2>/dev/null || true
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
      if ! process_record_is_running "$pid" "$pgid" "$expected_starttime"; then
        common_ok "进程已被强制结束。"
        return 0
      fi
      sleep 0.1
    done
    common_error "进程组 $pgid 在 SIGKILL 后仍未确认退出，请人工检查。"
    return 1
  fi
}

process_stop_current() {
  # process_active_file 按会话隔离；这里只会读取本实例的记录。
  local timeout_seconds=${1:-5}
  local stop_status=0
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：不会停止或清理任何已有进程记录。"
    return 0
  fi
  if ! process_read_record; then
    if [[ -n "${SP_VISION_MANAGED_PID:-}" ]]; then
      process_stop_group \
        "$SP_VISION_MANAGED_PID" "$SP_VISION_MANAGED_PGID" \
        "$SP_VISION_MANAGED_STARTTIME" "$timeout_seconds"
      return
    fi
    common_info "当前没有本菜单启动的任务。"
    return 0
  fi

  if ! process_record_is_running \
    "$PROCESS_RECORD_PID" "$PROCESS_RECORD_PGID" "$PROCESS_RECORD_STARTTIME"; then
    common_info "记录中的任务已退出，正在清理陈旧记录。"
    process_clear_record
    return 0
  fi

  process_stop_group \
    "$PROCESS_RECORD_PID" \
    "$PROCESS_RECORD_PGID" \
    "$PROCESS_RECORD_STARTTIME" \
    "$timeout_seconds" || stop_status=$?
  if ((stop_status == 0)); then
    process_clear_record
  else
    common_warn "无法确认受管进程已停止，保留 PID 记录供后续重试。"
  fi
  return "$stop_status"
}

process_shutdown_all() {
  process_stop_current 5 >/dev/null 2>&1 || true
}

process_print_current() {
  if ! process_read_record; then
    common_info "没有本菜单启动且仍在记录中的任务。"
    return 1
  fi
  if process_record_is_running \
    "$PROCESS_RECORD_PID" "$PROCESS_RECORD_PGID" "$PROCESS_RECORD_STARTTIME"; then
    common_info "PID=$PROCESS_RECORD_PID PGID=$PROCESS_RECORD_PGID 名称=$PROCESS_RECORD_LABEL"
    return 0
  fi
  common_info "记录已陈旧：PID=$PROCESS_RECORD_PID 名称=$PROCESS_RECORD_LABEL"
  process_clear_record
  return 1
}

process_check_external_camera_occupancy() {
  local found=0
  local device
  local output

  for device in /dev/video*; do
    [[ -e "$device" ]] || continue
    common_info "检查设备：$device"
    if command -v fuser >/dev/null 2>&1; then
      output="$(fuser -v "$device" 2>&1)" || true
      if [[ -n "$output" ]]; then
        printf '%s\n' "$output"
        found=1
      fi
    elif command -v lsof >/dev/null 2>&1; then
      output="$(lsof -- "$device" 2>/dev/null)" || true
      if [[ -n "$output" ]]; then
        printf '%s\n' "$output"
        found=1
      fi
    else
      common_warn "缺少 fuser/lsof，无法检查 $device 的用户。"
    fi
  done

  common_info "海康 MVS 等 SDK 相机不一定暴露为 /dev/video*，系统无法仅凭文件名可靠确认外部占用。"
  if ((found)); then
    common_warn "检测到相机可能被占用。菜单不会结束外部进程。"
    return 1
  fi
  common_ok "未检测到 /dev/video* 占用；MVS SDK 占用仍需人工确认。"
  return 0
}

process_check_camera_occupancy() {
  local status=0
  process_camera_lock_available || status=1
  process_check_external_camera_occupancy || status=1
  return "$status"
}
