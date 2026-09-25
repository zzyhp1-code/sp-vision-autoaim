#!/usr/bin/env bash

set -u

# 入口职责：定位工程、加载适配模块、分流检查模式和交互模式。
# SP_VISION_ROOT 允许测试时指定工程；模块自身仍从本入口所在目录加载。
# set -u 会拒绝未赋值变量，但不会自动传播命令失败；各调用点须检查返回码。
# 审查注意：--check 最终也执行 EXIT trap；当前共享进程记录尚未隔离会话，
# 不应在另一菜单任务运行时执行本入口，详见 process.sh 和审查文档。

SP_VISION_DEBUG_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SP_VISION_ROOT="${SP_VISION_ROOT:-$(cd -- "$SP_VISION_DEBUG_SCRIPT_DIR/.." && pwd -P)}"
export SP_VISION_ROOT

sp_vision_debug_usage() {
  printf '%s\n' \
    "SP-Vision 中文交互式调试菜单" \
    "" \
    "用法：" \
    "  bash scripts/sp_vision_debug.sh [选项]" \
    "" \
    "选项：" \
    "  --help      显示本帮助" \
    "  --dry-run   进入菜单，只显示将调用的程序、参数、配置和日志路径" \
    "  --check     只执行环境和 CLI 接口检查，不启动相机、标定或识别" \
    "" \
    "普通启动后会显示中文菜单。菜单只编排项目已有程序，不实现算法。"
}

SP_VISION_DEBUG_DRY_RUN=0
SP_VISION_DEBUG_CHECK_ONLY=0

while (($#)); do
  case "$1" in
    --help|-h)
      sp_vision_debug_usage
      exit 0
      ;;
    --dry-run)
      SP_VISION_DEBUG_DRY_RUN=1
      ;;
    --check)
      SP_VISION_DEBUG_CHECK_ONLY=1
      ;;
    *)
      printf '错误：未知参数：%s\n' "$1" >&2
      sp_vision_debug_usage >&2
      exit 2
      ;;
  esac
  shift
done

export SP_VISION_DEBUG_DRY_RUN
export SP_VISION_DEBUG_CHECK_ONLY

debug_menu_source() {
  local helper=$1
  local path="$SP_VISION_DEBUG_SCRIPT_DIR/debug_menu/$helper"
  if [[ ! -f "$path" ]]; then
    printf '错误：菜单适配层文件不存在：%s\n' "$path" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  . "$path" || exit 1
}

debug_menu_source common.sh
debug_menu_source process.sh
debug_menu_source registry.sh
debug_menu_source environment.sh
debug_menu_source build.sh
debug_menu_source detection.sh
debug_menu_source camera.sh
debug_menu_source config.sh
debug_menu_source calibration.sh
debug_menu_source logs.sh
debug_menu_source process_actions.sh
debug_menu_source menu.sh

common_bootstrap || exit 1
common_environment_load_ros

if [[ "$SP_VISION_DEBUG_CHECK_ONLY" == "1" ]]; then
  common_title "SP-Vision 环境与接口检查"
  rc=0
  environment_quick_check || rc=1
  registry_validate_all_interfaces || rc=1
  if ((rc)); then
    common_error "检查未全部通过。"
  else
    common_ok "全部检查通过。"
  fi
  exit "$rc"
fi

common_info "项目根目录：$SP_VISION_ROOT"
if [[ "$SP_VISION_DEBUG_DRY_RUN" == "1" ]]; then
  common_warn "当前为 dry-run：不会启动任何相机、标定、识别或构建程序。"
fi
environment_preflight
menu_main
