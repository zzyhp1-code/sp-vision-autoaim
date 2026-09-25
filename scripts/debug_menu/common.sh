#!/usr/bin/env bash

# Shared launcher helpers. This file must not implement camera, calibration,
# detection, control, or serial business logic.

# 公共层负责路径、交互、环境和配置辅助调用；不要把业务算法加入此文件。
# stdout 同时用于返回路径和显示信息时容易互相污染，供命令替换读取的函数
# 必须只输出返回值，交互提示应走独立输出通道。

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  C_RESET=$'\033[0m'
  C_BOLD=$'\033[1m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
  C_RED=$'\033[31m'
  C_CYAN=$'\033[36m'
else
  C_RESET=""
  C_BOLD=""
  C_GREEN=""
  C_YELLOW=""
  C_RED=""
  C_CYAN=""
fi

SP_VISION_DEBUG_DRY_RUN="${SP_VISION_DEBUG_DRY_RUN:-0}"
SP_VISION_DEBUG_CHECK_ONLY="${SP_VISION_DEBUG_CHECK_ONLY:-0}"
SP_VISION_MANAGED_PID="${SP_VISION_MANAGED_PID:-}"
SP_VISION_MANAGED_PGID="${SP_VISION_MANAGED_PGID:-}"
SP_VISION_MANAGED_STARTTIME="${SP_VISION_MANAGED_STARTTIME:-}"
SP_VISION_CAMERA_LOCK_FD=""
SP_VISION_CLEANED_UP=0

common_bootstrap() {
  if [[ -z "${SP_VISION_ROOT:-}" ]]; then
    local script_dir
    script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[1]}")" && pwd -P)"
    SP_VISION_ROOT="$(cd -- "$script_dir/../.." && pwd -P)"
  fi

  common_require_root || return 1
  SP_VISION_BUILD_DIR="$SP_VISION_ROOT/build"
  SP_VISION_LOG_DIR="$SP_VISION_ROOT/logs"
  SP_VISION_RUNTIME_DIR="$SP_VISION_LOG_DIR/runtime"
  SP_VISION_DATA_DIR="$SP_VISION_ROOT/data"
  SP_VISION_CONFIG_DIR="$SP_VISION_ROOT/configs"
  SP_VISION_SCRIPT_DIR="$SP_VISION_ROOT/scripts/debug_menu"
  SP_VISION_YAML_TOOL="$SP_VISION_SCRIPT_DIR/yaml_tool.py"
  # 每个菜单实例使用独立记录；退出时只清理本实例创建的任务。
  SP_VISION_MENU_SESSION_ID="${SP_VISION_MENU_SESSION_ID:-$$_${RANDOM}_${RANDOM}}"
  [[ "$SP_VISION_MENU_SESSION_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || {
    common_error "菜单会话标识不安全。"
    return 1
  }
  SP_VISION_TMP_PARENT="${TMPDIR:-/tmp}"
  if [[ -n "${SP_VISION_TMP_PARENT##/}" ]]; then
    SP_VISION_TMP_PARENT="$(cd -- "$SP_VISION_TMP_PARENT" && pwd -P)" || return 1
  fi
  SP_VISION_TMP_ROOT="$(mktemp -d "${SP_VISION_TMP_PARENT%/}/sp_vision_debug.XXXXXX")" || {
    common_error "无法创建菜单临时目录。"
    return 1
  }

  if command -v python3 >/dev/null 2>&1 &&
    python3 -c 'import sys' >/dev/null 2>&1; then
    SP_VISION_PYTHON=python3
  elif command -v python >/dev/null 2>&1 &&
    python -c 'import sys' >/dev/null 2>&1; then
    SP_VISION_PYTHON=python
  else
    SP_VISION_PYTHON=""
    common_warn "未找到可用的 Python 解释器；YAML 查看和写入功能将不可用。"
  fi

  trap 'common_handle_signal 130' INT
  trap 'common_handle_signal 143' TERM
  trap 'common_cleanup' EXIT
}

common_cleanup() {
  # process_shutdown_all 只读取本次会话的记录；退出不会处理其他菜单实例的任务。
  local status=$?
  if ((SP_VISION_CLEANED_UP)); then
    return
  fi
  SP_VISION_CLEANED_UP=1

  if declare -F process_shutdown_all >/dev/null 2>&1; then
    process_shutdown_all >/dev/null 2>&1 || true
  fi
  if declare -F process_release_camera_lock >/dev/null 2>&1; then
    process_release_camera_lock >/dev/null 2>&1 || true
  fi
  if common_temp_path_is_safe "${SP_VISION_TMP_ROOT:-}"; then
    rm -rf -- "$SP_VISION_TMP_ROOT"
  fi
  return "$status"
}

common_handle_signal() {
  local exit_code=$1
  printf '\n%s收到中断信号，正在停止本菜单启动的任务。%s\n' "$C_YELLOW" "$C_RESET"
  if declare -F process_stop_current >/dev/null 2>&1; then
    process_stop_current 5 || true
  fi
  exit "$exit_code"
}

common_log() {
  printf '%s%s%s\n' "$C_CYAN" "$*" "$C_RESET"
}

common_info() {
  printf '%s\n' "$*"
}

common_warn() {
  printf '%s警告：%s%s\n' "$C_YELLOW" "$*" "$C_RESET" >&2
}

common_error() {
  printf '%s错误：%s%s\n' "$C_RED" "$*" "$C_RESET" >&2
}

common_ok() {
  printf '%s通过：%s%s\n' "$C_GREEN" "$*" "$C_RESET"
}

common_title() {
  printf '\n%s%s%s\n' "$C_BOLD" "$*" "$C_RESET"
}

common_unavailable() {
  local reason=$1
  local missing=${2:-}
  common_title "当前版本不可用"
  common_info "原因：$reason"
  if [[ -n "$missing" ]]; then
    common_info "缺失的文件、目标或参数：$missing"
  fi
  common_info "维护人员只需更新 scripts/debug_menu/registry.sh 中对应的命令适配。"
  return 2
}

common_interface_incompatible() {
  common_unavailable "当前版本接口不兼容。" "$*"
}

common_read() {
  # 提示走 stderr，使通过 $(...) 返回路径或数值的辅助函数只返回数据本身。
  local prompt=$1
  local __var_name=$2
  printf '%s' "$prompt" >&2
  IFS= read -r "$__var_name"
}

common_pause() {
  local message=${1:-"按回车键继续"}
  if [[ -t 0 ]]; then
    local ignored
    common_read "$message..." ignored || true
  fi
}

common_confirm() {
  local prompt=$1
  local default_answer=${2:-n}
  local answer
  local suffix="[y/N]"
  [[ "$default_answer" == "y" ]] && suffix="[Y/n]"

  while true; do
    common_read "$prompt $suffix " answer || return 1
    answer="${answer,,}"
    if [[ -z "$answer" ]]; then
      [[ "$default_answer" == "y" ]]
      return
    fi
    case "$answer" in
      y|yes|是) return 0 ;;
      n|no|否) return 1 ;;
      *) common_warn "请输入 y 或 n。" ;;
    esac
  done
}

common_confirm_exact() {
  local prompt=$1
  local expected=$2
  local answer
  common_read "$prompt 请输入“$expected”确认：" answer || return 1
  [[ "$answer" == "$expected" ]]
}

common_require_root() {
  local root
  root="$(cd -- "$SP_VISION_ROOT" 2>/dev/null && pwd -P)" || {
    common_error "项目根目录不存在：$SP_VISION_ROOT"
    return 1
  }
  if [[ ! -f "$root/CMakeLists.txt" || ! -d "$root/scripts" || ! -f "$root/configs/standard.yaml" ]]; then
    common_error "项目根目录不完整：$root"
    return 1
  fi
  SP_VISION_ROOT="$root"
}

common_temp_path_is_safe() {
  local path=${1:-}
  local base=${SP_VISION_TMP_PARENT:-${TMPDIR:-/tmp}}
  local name
  [[ -n "$path" && -d "$path" ]] || return 1
  name="${path##*/}"
  [[ "$name" == sp_vision_debug.* ]] || return 1
  common_path_is_under "$path" "$base"
}

common_remove_temp_path() {
  local path=${1:-}
  if [[ -z "$path" ]]; then
    return 0
  fi
  if ! common_temp_path_is_safe "$path"; then
    common_error "拒绝清理不属于菜单临时目录的路径：$path"
    return 1
  fi
  rm -rf -- "$path"
}

common_require_file() {
  local path=$1
  local label=${2:-文件}
  if [[ ! -f "$path" ]]; then
    common_error "$label 不存在：$path"
    return 1
  fi
}

common_require_dir() {
  local path=$1
  local label=${2:-目录}
  if [[ ! -d "$path" ]]; then
    common_error "$label 不存在：$path"
    return 1
  fi
}

common_require_executable() {
  local path=$1
  local label=${2:-程序}
  if [[ ! -x "$path" ]]; then
    common_error "$label 不存在或不可执行：$path"
    return 1
  fi
}

common_require_command() {
  local command_name=$1
  if ! command -v "$command_name" >/dev/null 2>&1; then
    common_error "缺少系统命令：$command_name"
    return 1
  fi
}

common_require_python() {
  if [[ -z "${SP_VISION_PYTHON:-}" ]]; then
    common_error "没有可用的 Python 解释器，无法解析或写入 YAML。"
    return 1
  fi
}

common_print_runtime_safety() {
  common_info "安全检查：xuc_enable=false，xuc_allow_control=false，xuc_allow_shoot=false。"
}

common_path_is_under() {
  # 只验证相对给定 parent 的包含关系；parent 自身若被符号链接指向工程外，
  # 仍会通过。写入和清理前还需验证 logs/data 根本身处于允许的工程目录内。
  local path=$1
  local parent=$2
  local resolved_path resolved_parent
  resolved_path="$(realpath -m -- "$path")" || return 1
  resolved_parent="$(realpath -m -- "$parent")" || return 1
  [[ "$resolved_path" == "$resolved_parent" || "$resolved_path" == "$resolved_parent/"* ]]
}

common_require_under() {
  local path=$1
  local parent=$2
  local label=${3:-路径}
  if ! common_path_is_under "$path" "$parent"; then
    common_error "$label 必须位于 $parent 内：$path"
    return 1
  fi
}

common_ensure_runtime_dir() {
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    return 0
  fi
  mkdir -p -- "$SP_VISION_RUNTIME_DIR"
}

common_timestamp() {
  date '+%Y-%m-%d_%H-%M-%S'
}

common_today() {
  date '+%Y-%m-%d'
}

common_latest_log() {
  local kind=${1:-any}
  local -a candidates=()
  case "$kind" in
    camera)
      mapfile -t candidates < <(
        find "$SP_VISION_LOG_DIR" -type f -name '*.log' -print 2>/dev/null |
          while IFS= read -r file; do
            if grep -qE '\[HikRobot\]|\[MindVision\]|Camera constructor' "$file" 2>/dev/null; then
              printf '%s\n' "$file"
            fi
          done
      )
      ;;
    detection)
      mapfile -t candidates < <(
        find "$SP_VISION_LOG_DIR" -type f -name '*.log' -print 2>/dev/null |
          while IFS= read -r file; do
            if grep -qE 'Detected [0-9]+ armors|Camera PnP test' "$file" 2>/dev/null; then
              printf '%s\n' "$file"
            fi
          done
      )
      ;;
    *)
      mapfile -t candidates < <(find "$SP_VISION_LOG_DIR" -type f -name '*.log' -print 2>/dev/null)
      ;;
  esac

  if ((${#candidates[@]} == 0)); then
    return 1
  fi
  ls -1t -- "${candidates[@]}" 2>/dev/null | head -n 1
}

common_show_path_safety() {
  local path=$1
  local files=0
  local size="0"
  if [[ -d "$path" ]]; then
    files=$(find "$path" -type f -print 2>/dev/null | wc -l | tr -d ' ')
    size=$(du -sh -- "$path" 2>/dev/null | awk '{print $1}')
  elif [[ -f "$path" ]]; then
    files=1
    size=$(du -h -- "$path" 2>/dev/null | awk '{print $1}')
  fi
  common_info "目标路径：$path"
  common_info "文件数量：$files"
  common_info "总大小：${size:-0}"
}

common_read_number() {
  local prompt=$1
  local __var_name=$2
  local value
  while true; do
    common_read "$prompt" value || return 1
    if [[ "$value" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
      printf -v "$__var_name" '%s' "$value"
      return 0
    fi
    common_warn "请输入有限数字，不接受 NaN 或 Inf。"
  done
}

common_abs_path() {
  realpath -m -- "$1"
}

common_show_tail() {
  local path=$1
  local lines=${2:-20}
  if [[ -f "$path" ]]; then
    common_info "最近 $lines 行输出："
    tail -n "$lines" -- "$path"
  fi
}

common_display_available() {
  # 这是存在性提示，不证明当前用户有连接权限；X11 目录存在也不代表 DISPLAY 可用。
  [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" || -d /tmp/.X11-unix ]]
}

common_require_display() {
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：将检查图形显示会话。"
    return 0
  fi
  if ! common_display_available; then
    common_error "当前没有可用的图形显示会话，拒绝启动带 imshow 窗口的测试程序。"
    return 1
  fi
}

common_yaml_get() {
  common_require_python || return 1
  local config=$1
  local key=$2
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" get --config "$config" --key "$key"
}

common_yaml_validate() {
  common_require_python || return 1
  local config=$1
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" validate --config "$config"
}

common_make_runtime_config() {
  common_require_python || return 1
  local source_config=$1
  local output_config=$2
  shift 2
  local override key
  local -a arguments=(
    "$SP_VISION_PYTHON"
    "$SP_VISION_YAML_TOOL"
    make-runtime
    --source "$source_config"
    --output "$output_config"
  )

  for override in "$@"; do
    key="${override%%=*}"
    case "$key" in
      xuc_enable|xuc_allow_control|xuc_allow_shoot)
        common_error "调用方不得覆盖菜单强制安全开关：$key"
        return 1
        ;;
    esac
    arguments+=(--set "$override")
  done

  # These values are always written last and are additionally verified by the
  # Python helper before and after the runtime cache is produced.
  arguments+=(
    --set xuc_enable=false
    --set xuc_allow_control=false
    --set xuc_allow_shoot=false
  )
  "${arguments[@]}"
}

common_yaml_compare_fields() {
  common_require_python || return 1
  local config=$1
  local candidate=$2
  shift 2
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" compare \
    --config "$config" --candidate "$candidate" --keys "$@"
}

common_yaml_write_fields() {
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：不会写入正式配置。"
    return 0
  fi
  common_require_python || return 1
  local config=$1
  local candidate=$2
  shift 2
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" write-fields \
    --config "$config" --candidate "$candidate" --keys "$@"
}

common_yaml_write_fields_pair() {
  if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
    common_info "dry-run：不会写入正式配置。"
    return 0
  fi
  common_require_python || return 1
  local first_config=$1
  local second_config=$2
  local candidate=$3
  shift 3
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" write-fields-pair \
    --config-a "$first_config" --config-b "$second_config" \
    --candidate "$candidate" --keys "$@"
}

common_yaml_check_calibration() {
  common_require_python || return 1
  local log_file=$1
  local candidate_output=$2
  local status_output=$3
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" check-calibration \
    --log "$log_file" \
    --candidate-output "$candidate_output" \
    --status-output "$status_output"
}

common_yaml_check_handeye() {
  common_require_python || return 1
  local log_file=$1
  local candidate_output=$2
  local status_output=$3
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" check-handeye \
    --log "$log_file" \
    --candidate-output "$candidate_output" \
    --status-output "$status_output"
}

common_yaml_check_handeye_verification() {
  common_require_python || return 1
  local log_file=$1
  local status_output=$2
  "$SP_VISION_PYTHON" "$SP_VISION_YAML_TOOL" check-handeye-verification \
    --log "$log_file" \
    --status-output "$status_output"
}

common_log_path() {
  local name=$1
  local directory="$SP_VISION_LOG_DIR/debug_menu"
  if [[ "$SP_VISION_DEBUG_DRY_RUN" != "1" ]]; then
    mkdir -p -- "$directory"
  fi
  printf '%s\n' "$directory/${name}_$(common_timestamp)_$$.log"
}

common_runtime_config_path() {
  local label=$1
  local path
  path="$(mktemp "${SP_VISION_TMP_ROOT%/}/${label}.XXXXXX.yaml")" || {
    common_error "无法创建运行时配置临时文件。"
    return 1
  }
  printf '%s\n' "$path"
}

common_temp_file() {
  local label=$1
  local suffix=${2:-}
  local path
  path="$(mktemp "${SP_VISION_TMP_ROOT%/}/${label}.XXXXXX${suffix}")" || {
    common_error "无法创建菜单临时文件。"
    return 1
  }
  printf '%s\n' "$path"
}

common_require_finite_range() {
  local value=$1
  local minimum=$2
  local maximum=$3
  local label=$4
  if ! awk -v v="$value" -v lo="$minimum" -v hi="$maximum" \
    'BEGIN { exit !(v == v && v < 1e308 && v > -1e308 && v >= lo && v <= hi) }'; then
    common_error "$label 超出允许范围 [$minimum, $maximum]：$value"
    return 1
  fi
}

common_environment_load_ros() {
  local setup_file="${SP_VISION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  if [[ -f "$setup_file" ]]; then
    set +u
    # shellcheck disable=SC1090
    . "$setup_file"
    set -u
    SP_VISION_ROS_LOADED=1
  else
    SP_VISION_ROS_LOADED=0
  fi
}

common_trim() {
  local value=$1
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s\n' "$value"
}
