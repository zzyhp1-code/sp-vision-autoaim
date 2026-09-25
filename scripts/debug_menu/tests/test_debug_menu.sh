#!/usr/bin/env bash

# Safe repository-side checks. Hardware behavior still requires the target
# Ubuntu machine; this test never starts a camera, inference model, or motor.

# 审查警告：当前测试直接使用真实 ROOT/build 和 logs/runtime，且会运行菜单。
# 只有缺少构建产物时，“缺少程序”用例才安全；现场存在产物时不能承诺无硬件操作。
# 运行前应改为独立测试夹具与模拟程序，禁止在正在调试的工程目录直接执行整套测试。

set -u

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
ENTRY="$ROOT/scripts/sp_vision_debug.sh"
MENU_DIR="$ROOT/scripts/debug_menu"
BASH_BIN="${BASH:-/usr/bin/bash}"
PYTHON_BIN="${SP_VISION_TEST_PYTHON:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1 &&
    python3 -c 'import sys' >/dev/null 2>&1; then
    PYTHON_BIN=python3
  elif command -v python >/dev/null 2>&1 &&
    python -c 'import sys' >/dev/null 2>&1; then
    PYTHON_BIN=python
  fi
fi
TMP="$(mktemp -d "${TMPDIR:-/tmp}/sp_vision_menu_test.XXXXXX")"
cleanup() {
  rm -rf -- "$TMP"
}
trap cleanup EXIT

PASS=0
FAIL=0

ok() {
  PASS=$((PASS + 1))
  printf 'PASS: %s\n' "$*"
}

bad() {
  FAIL=$((FAIL + 1))
  printf 'FAIL: %s\n' "$*" >&2
}

run_menu() {
  local input=$1
  shift
  printf '%b' "$input" |
    "$BASH_BIN" "$ENTRY" "$@" >"$TMP/stdout" 2>"$TMP/stderr"
}

require_status() {
  local expected=$1
  local label=$2
  local actual=$3
  if [[ "$actual" == "$expected" ]]; then
    ok "$label"
  else
    bad "$label（期望退出码 $expected，实际 $actual）"
  fi
}

test_syntax() {
  local failed=0 file
  for file in "$ENTRY" "$MENU_DIR"/*.sh; do
    "$BASH_BIN" -n "$file" || failed=1
  done
  if ((failed == 0)); then
    ok "bash -n 全部菜单脚本"
  else
    bad "bash -n 失败"
  fi
}

test_help() {
  "$BASH_BIN" "$ENTRY" --help >"$TMP/help" 2>&1
  require_status 0 "--help 正常退出" "$?"
  grep -q 'sp_vision_debug.sh' "$TMP/help" ||
    bad "--help 缺少入口名称" || true
}

test_unknown_argument() {
  "$BASH_BIN" "$ENTRY" --not-a-real-option >"$TMP/unknown" 2>&1
  require_status 2 "未知启动参数返回 2" "$?"
}

test_check_mode_does_not_start_business_programs() {
  "$BASH_BIN" "$ENTRY" --check >"$TMP/check_out" 2>&1
  local status=$?
  if grep -q '环境与接口检查\|项目根目录' "$TMP/check_out" &&
    ! grep -q '将调用：' "$TMP/check_out"; then
    ok "--check 只执行检查，未启动业务程序"
  else
    bad "--check 未保持只检查模式"
  fi
  if find "$ROOT/logs/runtime" -maxdepth 1 -type f -name 'process_*' -print -quit 2>/dev/null | grep -q .; then
    bad "--check 留下了受管进程记录"
  fi
  return "$status"
}

test_dry_run_invalid_menu_input() {
  run_menu '99\n0\n' --dry-run
  require_status 0 "dry-run 非法编号后正常退出" "$?"
  grep -q '该编号不存在' "$TMP/stdout" "$TMP/stderr" ||
    bad "dry-run 未报告非法编号" || true
  if find "$ROOT/logs/runtime" -maxdepth 1 -type f -name 'process_*' -print -quit 2>/dev/null | grep -q .; then
    bad "dry-run 留下了受管进程记录"
  fi
}

test_empty_menu_input_does_not_repeat() {
  run_menu '\n0\n' --dry-run
  require_status 0 "空输入后正常退出" "$?"
  if grep -q '将调用：' "$TMP/stdout" "$TMP/stderr"; then
    bad "空输入触发了上一条命令或任何命令"
  else
    ok "空输入不触发旧命令"
  fi
}

test_missing_executable_fails_safely() {
  run_menu '3\n1\n0\n0\n'
  local status=$?
  if grep -q '当前版本不可用\|当前版本接口不兼容\|缺少可执行文件' "$TMP/stdout" "$TMP/stderr"; then
    ok "缺少识别测试程序时安全失败"
  else
    bad "缺少识别测试程序时未给出明确接口错误"
  fi
  if find "$ROOT/logs/runtime" -maxdepth 1 -type f -name 'process_*' -print -quit 2>/dev/null | grep -q .; then
    bad "安全失败后留下受管进程记录"
  fi
  [[ "$status" == "0" ]]
}

test_missing_model_fails_safely() {
  local fake_binary="$ROOT/build/camera_pnp_test"
  if [[ -e "$fake_binary" ]]; then
    printf 'SKIP: build/camera_pnp_test 已存在，避免覆盖现场构建产物\n'
    return
  fi
  if ! mkdir -p -- "$ROOT/build" 2>/dev/null; then
    printf 'SKIP: 无法创建临时 build/ 目录\n'
    return
  fi
  if ! printf '#!/usr/bin/env bash\nexit 0\n' >"$fake_binary"; then
    printf 'SKIP: 无法创建临时假可执行文件\n'
    rmdir -- "$ROOT/build" 2>/dev/null || true
    return
  fi
  chmod +x "$fake_binary" || {
    printf 'SKIP: 无法设置临时假可执行文件权限\n'
    rm -f -- "$fake_binary"
    rmdir -- "$ROOT/build" 2>/dev/null || true
    return
  }
  run_menu '3\n1\n0\n0\n'
  local status=$?
  if grep -q '模型不存在\|当前版本接口不兼容' "$TMP/stdout" "$TMP/stderr"; then
    ok "缺少模型时安全失败"
  else
    bad "缺少模型时未给出明确接口错误"
  fi
  rm -f -- "$fake_binary"
  rmdir -- "$ROOT/build" 2>/dev/null || true
  [[ "$status" == "0" ]]
}

test_external_camera_lock_is_not_killed() {
  if ! command -v flock >/dev/null 2>&1; then
    printf 'SKIP: flock 不可用\n'
    return
  fi
  local runtime="$ROOT/logs/runtime"
  local lock_file="$runtime/camera.lock"
  mkdir -p -- "$runtime" || {
    printf 'SKIP: 无法创建临时 logs/runtime\n'
    return
  }
  (
    exec 8>"$lock_file"
    flock -n 8 || exit 1
    printf '9\n4\n0\n0\n' |
      "$BASH_BIN" "$ENTRY" >"$TMP/camera_lock_out" 2>&1
  )
  local status=$?
  if grep -q '相机资源锁已被本菜单的另一个任务占用' "$TMP/camera_lock_out"; then
    ok "外部相机锁只报告占用，不结束外部持有者"
  else
    bad "外部相机锁未被安全报告"
  fi
  rm -f -- "$lock_file"
  rmdir -- "$runtime" 2>/dev/null || true
  rmdir -- "$ROOT/logs" 2>/dev/null || true
  [[ "$status" == "0" ]]
}

test_process_start_stop_twice() {
  # 使用显式命令数组和本次会话记录，验证两次启动/停止后都不遗留记录。
  if ! command -v setsid >/dev/null 2>&1 || ! command -v flock >/dev/null 2>&1; then
    printf 'SKIP: setsid 或 flock 不可用\n'
    return
  fi
  local sleep_bin
  sleep_bin="$(command -v sleep)" || {
    printf 'SKIP: sleep 不可用\n'
    return
  }
  (
    set -u
    SP_VISION_ROOT="$ROOT"
    SP_VISION_BUILD_DIR="$ROOT/build"
    SP_VISION_LOG_DIR="$ROOT/logs"
    SP_VISION_RUNTIME_DIR="$ROOT/logs/runtime"
    SP_VISION_DATA_DIR="$ROOT/data"
    SP_VISION_CONFIG_DIR="$ROOT/configs"
    SP_VISION_SCRIPT_DIR="$MENU_DIR"
    SP_VISION_YAML_TOOL="$MENU_DIR/yaml_tool.py"
    SP_VISION_TMP_PARENT="${TMPDIR:-/tmp}"
    SP_VISION_TMP_ROOT="$TMP/process-tmp"
    SP_VISION_DEBUG_DRY_RUN=0
    SP_VISION_CAMERA_LOCK_FD=""
    SP_VISION_MENU_SESSION_ID="process-test-$$"
    SP_VISION_MANAGED_PID=""
    SP_VISION_MANAGED_PGID=""
    SP_VISION_MANAGED_STARTTIME=""
    mkdir -p -- "$SP_VISION_RUNTIME_DIR"
    . "$MENU_DIR/common.sh"
    . "$MENU_DIR/process.sh"
    . "$MENU_DIR/registry.sh"
    local iteration runner
    for iteration in 1 2; do
      registry_command_system "$sleep_bin" 30
      process_run_managed "进程管理测试 $iteration" 0 "$TMP/process_$iteration.log" "$sleep_bin" 30 &
      runner=$!
      sleep 0.2
      process_stop_current 2 >/dev/null || exit 1
      wait "$runner" 2>/dev/null || true
      [[ ! -e "$(process_active_file)" ]] || exit 1
    done
  )
  local status=$?
  rm -f -- "$ROOT/logs/runtime"/process_* "$ROOT/logs/runtime/camera.lock"
  rmdir -- "$ROOT/logs/runtime" "$ROOT/logs" 2>/dev/null || true
  if ((status == 0)); then
    ok "连续启动和停止两次无残留记录"
  else
    bad "连续启动和停止两次失败"
  fi
}

test_forbidden_process_patterns() {
  if grep -RInE '\beval\b|\bpkill\b|\bkillall\b' \
    "$ENTRY" "$MENU_DIR"/*.sh >/dev/null 2>&1; then
    bad "发现 eval/pkill/killall"
  else
    ok "未发现 eval/pkill/killall"
  fi
}

test_standard_not_used_as_test_entry() {
  if grep -RInE 'build/standard|registry_command_standard|standard[[:space:]]+configs/' \
    "$ENTRY" "$MENU_DIR"/*.sh >/dev/null 2>&1; then
    bad "发现把 standard 当作测试入口的适配"
  else
    ok "识别/相机适配未调用 standard"
  fi
}

test_no_generated_config_names() {
  if grep -RInE '_(final|copy|backup)(\.|_|$)|[0-9]{4}-[0-9]{2}-[0-9]{2}\.yaml' \
    "$ENTRY" "$MENU_DIR"/*.sh >/dev/null 2>&1; then
    bad "发现 final/copy/backup/日期配置命名"
  else
    ok "未生成正式配置副本命名"
  fi
}

test_runtime_config_safety() {
  local source_config="$ROOT/configs/standard.yaml"
  local output_config="$TMP/runtime.yaml"
  if [[ -z "$PYTHON_BIN" ]] ||
    ! "$PYTHON_BIN" -c 'import sys' >/dev/null 2>&1; then
    printf 'SKIP: Python 解释器不可用\n'
    return
  fi
  local before after
  before="$(cksum "$source_config")"
  "$PYTHON_BIN" "$MENU_DIR/yaml_tool.py" make-runtime \
    --source "$source_config" --output "$output_config" \
    --set xuc_enable=false \
    --set xuc_allow_control=false \
    --set xuc_allow_shoot=false >"$TMP/runtime.log" 2>&1
  local rc=$?
  if ((rc != 0)); then
    bad "运行时配置生成失败"
    return
  fi
  after="$(cksum "$source_config")"
  if [[ "$before" != "$after" ]]; then
    bad "运行时配置生成修改了正式配置"
    return
  fi
  for key in xuc_enable xuc_allow_control xuc_allow_shoot; do
    if [[ "$("$PYTHON_BIN" "$MENU_DIR/yaml_tool.py" get --config "$output_config" --key "$key")" != "false" ]]; then
      bad "运行时配置安全开关不是 false：$key"
      return
    fi
  done
  ok "运行时配置强制安全开关且不影响正式配置"
}

test_yaml_write_guard() {
  local target="$TMP/write_target.yaml"
  local candidate="$TMP/write_candidate.yaml"
  if [[ -z "$PYTHON_BIN" ]]; then
    printf 'SKIP: Python 解释器不可用\n'
    return
  fi
  cp -- "$ROOT/configs/standard.yaml" "$target"
  cp -- "$ROOT/configs/standard.yaml" "$candidate"
  local before after rc
  before="$(cksum "$target")"
  "$PYTHON_BIN" "$MENU_DIR/yaml_tool.py" write-fields \
    --config "$target" --candidate "$candidate" --keys enemy_color \
    >"$TMP/write_guard.log" 2>&1
  rc=$?
  after="$(cksum "$target")"
  if ((rc == 0)) || [[ "$before" == "$after" ]]; then
    ok "YAML 写入成功或安全失败且目标未被破坏"
  else
    bad "YAML 写入既未成功也修改了目标文件"
  fi
}

test_log_cleanup_boundary() {
  if ! find --version 2>/dev/null | grep -q 'GNU find'; then
    printf 'SKIP: 当前 find 不是 GNU find，跳过日志边界测试\n'
    return
  fi
  local root="$TMP/cleanup_root"
  local logs="$root/logs"
  local runtime="$logs/runtime"
  local source_file="$root/src/keep.txt"
  local old_log="$logs/old.log"
  local fresh_log="$logs/fresh.log"
  mkdir -p -- "$logs" "$runtime" "$root/src"
  printf 'keep\n' >"$source_file"
  printf 'old\n' >"$old_log"
  printf 'fresh\n' >"$fresh_log"
  touch -d '2 days ago' "$old_log"
  (
    set -u
    SP_VISION_ROOT="$root"
    SP_VISION_LOG_DIR="$logs"
    SP_VISION_RUNTIME_DIR="$runtime"
    SP_VISION_DATA_DIR="$root/data"
    SP_VISION_DEBUG_DRY_RUN=0
    SP_VISION_PYTHON=""
    . "$MENU_DIR/common.sh"
    . "$MENU_DIR/logs.sh"
    printf '0\nCLEAN\n' | action_cleanup_logs >"$TMP/cleanup.log" 2>&1
  )
  if [[ ! -e "$old_log" && -e "$fresh_log" && -e "$source_file" ]]; then
    ok "日志清理只删除过期日志，不触碰源码和新鲜日志"
  else
    bad "日志清理边界验证失败"
  fi
}

test_shellcheck_if_available() {
  if command -v shellcheck >/dev/null 2>&1; then
    shellcheck "$ENTRY" "$MENU_DIR"/*.sh
    require_status 0 "shellcheck" "$?"
  else
    printf 'SKIP: shellcheck 不可用\n'
  fi
}

test_source_tree_if_available() {
  if [[ -n "$PYTHON_BIN" ]]; then
    "$PYTHON_BIN" "$ROOT/scripts/check_source_tree.py" >"$TMP/source_tree" 2>&1
    require_status 0 "check_source_tree.py" "$?"
  else
    printf 'SKIP: python3 不可用\n'
  fi
}

test_syntax
test_help
test_unknown_argument
test_check_mode_does_not_start_business_programs
test_dry_run_invalid_menu_input
test_empty_menu_input_does_not_repeat
test_missing_executable_fails_safely
test_missing_model_fails_safely
test_external_camera_lock_is_not_killed
test_process_start_stop_twice
test_forbidden_process_patterns
test_standard_not_used_as_test_entry
test_no_generated_config_names
test_runtime_config_safety
test_yaml_write_guard
test_log_cleanup_boundary
test_shellcheck_if_available
test_source_tree_if_available

printf '\n测试完成：通过 %s，失败 %s\n' "$PASS" "$FAIL"
exit "$((FAIL > 0))"
