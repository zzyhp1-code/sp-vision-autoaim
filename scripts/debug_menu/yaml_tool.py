#!/usr/bin/env python3
"""YAML and log inspection helpers used only by the interactive menu.

This helper does not implement camera, calibration, detection, or control
algorithms. Runtime configuration is always written to a temporary file.
Formal YAML writes require ruamel.yaml so comments and structure are retained.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
from typing import Any


class MenuYamlError(RuntimeError):
    pass


def import_yaml(require_roundtrip: bool = False):
    try:
        from ruamel.yaml import YAML

        yaml = YAML(typ="rt")
        yaml.preserve_quotes = True
        return "ruamel", yaml
    except ImportError:
        if require_roundtrip:
            raise MenuYamlError(
                "未安装 ruamel.yaml，无法在保留 YAML 结构与注释的前提下安全写入正式配置。"
            )
    try:
        import yaml as pyyaml

        return "pyyaml", pyyaml
    except ImportError as exc:
        raise MenuYamlError(
            "未安装 PyYAML/ruamel.yaml，无法完成结构化 YAML 解析。"
        ) from exc


def load_yaml(path: Path):
    """严格解析完整 YAML；文本扫描不能作为安全配置的替代解析器。"""
    backend, yaml = import_yaml()
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle) if backend == "ruamel" else yaml.safe_load(handle)
    except Exception as exc:
        raise MenuYamlError(f"YAML 解析失败：{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise MenuYamlError(f"YAML 顶层必须是映射：{path}")
    return backend, yaml, data


def scalar_from_text(raw: str) -> Any:
    value = raw.strip()
    if not value:
        return ""
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    if lowered in {"null", "~"}:
        return None
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        try:
            return ast.literal_eval(value)
        except (SyntaxError, ValueError):
            return value[1:-1]
    try:
        return int(value, 0)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def strip_yaml_comment(line: str) -> str:
    """移除非引号内注释；受限解析器不接受复杂 YAML 语法。"""
    quote = ""
    escaped = False
    for index, char in enumerate(line):
        if quote:
            if quote == '"' and char == "\\" and not escaped:
                escaped = True
                continue
            if char == quote and not escaped:
                quote = ""
            escaped = False
        elif char in {"'", '"'}:
            quote = char
        elif char == "#":
            return line[:index]
    if quote:
        raise MenuYamlError("YAML 引号未闭合。")
    return line


def restricted_top_level_values(path: Path) -> dict[str, Any]:
    """无 YAML 库时的受限解析：只接受本项目现有的简单顶层配置格式。

    它允许顶层标量、JSON 风格行内列表和简单缩进映射；遇到无法完整验证的
    YAML 特性立即拒绝。正式配置写入仍只允许 ruamel.yaml round-trip 路径。
    """
    values: dict[str, Any] = {}
    key_pattern = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*):(?:\s*(.*))?$")
    nested_pattern = re.compile(r"^\s+([A-Za-z_][A-Za-z0-9_]*):(?:\s*(.*))?$")
    have_parent = False
    for number, original in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if "\t" in original:
            raise MenuYamlError(f"第 {number} 行包含制表符，受限解析器拒绝处理。")
        line = strip_yaml_comment(original).rstrip()
        if not line.strip():
            continue
        if original[0].isspace():
            if not have_parent or not nested_pattern.match(line):
                raise MenuYamlError(f"第 {number} 行不是可验证的简单缩进映射。")
            continue
        match = key_pattern.match(line)
        if not match:
            raise MenuYamlError(f"第 {number} 行不是可验证的顶层键值。")
        have_parent = True
        key = match.group(1)
        raw = (match.group(2) or "").strip()
        if key in values:
            raise MenuYamlError(f"YAML 顶层键重复，无法安全处理：{key}")
        if not raw:
            values[key] = None
            continue
        if raw.startswith("[") or raw.startswith("{"):
            try:
                raw_value = json.loads(raw)
            except json.JSONDecodeError:
                raise MenuYamlError(f"第 {number} 行的行内集合不是有效 JSON 风格 YAML。")
        elif raw.startswith(("'", '"')):
            try:
                raw_value = ast.literal_eval(raw)
            except (SyntaxError, ValueError) as exc:
                raise MenuYamlError(f"第 {number} 行的引号字符串无效。") from exc
        else:
            if any(char in raw for char in "[]{}&*!|>"):
                raise MenuYamlError(f"第 {number} 行包含受限解析器不支持的 YAML 语法。")
            raw_value = scalar_from_text(raw)
        values[key] = raw_value
    if not values:
        raise MenuYamlError("YAML 未包含可验证的顶层键。")
    return values


def get_top_level_value(path: Path, key: str) -> tuple[Any, str]:
    try:
        backend, _, data = load_yaml(path)
    except MenuYamlError as error:
        if "未安装 PyYAML/ruamel.yaml" not in str(error):
            raise
        data = restricted_top_level_values(path)
        backend = "restricted-top-level"
    if key not in data:
        raise MenuYamlError(f"YAML 缺少字段：{key}")
    return data[key], backend


def parse_set_argument(argument: str) -> tuple[str, Any]:
    if "=" not in argument:
        raise MenuYamlError(f"字段覆盖必须是 key=value：{argument}")
    key, raw = argument.split("=", 1)
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
        raise MenuYamlError(f"字段名不安全：{key}")

    if raw.startswith("str:"):
        value = raw[4:]
    elif raw.startswith("bool:"):
        bool_text = raw[5:].lower()
        if bool_text not in {"true", "false"}:
            raise MenuYamlError(f"布尔值必须是 true/false：{argument}")
        value = bool_text == "true"
    elif raw.startswith("int:"):
        value = int(raw[4:])
    elif raw.startswith("float:"):
        value = float(raw[5:])
    elif raw.startswith("json:"):
        value = json.loads(raw[5:])
    else:
        value = scalar_from_text(raw)

    if isinstance(value, float) and not math.isfinite(value):
        raise MenuYamlError(f"字段值必须是有限数字：{argument}")
    return key, value


def validate_safety(values: dict[str, Any]) -> None:
    required_false = ("xuc_enable", "xuc_allow_control", "xuc_allow_shoot")
    for key in required_false:
        if values.get(key) is not False:
            raise MenuYamlError(f"运行配置安全开关必须为 false：{key}")


def yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False)
    text = str(value)
    if not text or re.search(r"[:#\[\]{}]|^\s|\s$", text):
        return json.dumps(text, ensure_ascii=False)
    return text


def fallback_make_runtime(source: Path, output: Path, overrides: dict[str, Any]) -> None:
    validate_safety(overrides)
    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    seen: set[str] = set()
    key_pattern = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*):")
    result: list[str] = []
    for line in lines:
        match = key_pattern.match(line)
        if match and match.group(1) in overrides:
            key = match.group(1)
            if key in seen:
                raise MenuYamlError(f"YAML 顶层键重复，无法安全覆盖：{key}")
            newline = "\n" if line.endswith("\n") else ""
            result.append(f"{key}: {yaml_scalar(overrides[key])}{newline}")
            seen.add(key)
        else:
            result.append(line)
    missing = sorted(set(overrides) - seen)
    if missing:
        raise MenuYamlError("顶层字段不存在，拒绝新增字段：" + ", ".join(missing))
    output.write_text("".join(result), encoding="utf-8")

    values = restricted_top_level_values(output)
    validate_safety(values)
    for key, expected in overrides.items():
        if values.get(key) != expected:
            raise MenuYamlError(f"运行配置覆盖后校验失败：{key}")


def structured_make_runtime(source: Path, output: Path, overrides: dict[str, Any]) -> str:
    validate_safety(overrides)
    backend, yaml, data = load_yaml(source)
    for key, value in overrides.items():
        if key not in data:
            raise MenuYamlError(f"顶层字段不存在，拒绝新增字段：{key}")
        data[key] = value
    validate_safety(dict(data))
    with output.open("w", encoding="utf-8") as handle:
        yaml.dump(data, handle)
    return backend


def command_get(args: argparse.Namespace) -> int:
    value, backend = get_top_level_value(Path(args.config), args.key)
    if isinstance(value, (list, dict)):
        print(json.dumps(value, ensure_ascii=False))
    elif value is None:
        print("null")
    elif isinstance(value, bool):
        print("true" if value else "false")
    else:
        print(value)
    if args.show_backend:
        print(f"解析方式：{backend}", file=sys.stderr)
    return 0


def command_validate(args: argparse.Namespace) -> int:
    path = Path(args.config)
    try:
        backend, _, _ = load_yaml(path)
    except MenuYamlError as error:
        if "未安装 PyYAML/ruamel.yaml" not in str(error):
            raise
        restricted_top_level_values(path)
        backend = "restricted-top-level"
    print(f"通过：YAML 可解析，解析方式={backend}，文件={path}")
    return 0


def command_make_runtime(args: argparse.Namespace) -> int:
    source = Path(args.source).resolve()
    output = Path(args.output).resolve()
    overrides = dict(parse_set_argument(item) for item in args.set)
    try:
        backend = structured_make_runtime(source, output, overrides)
        _, _, values = load_yaml(output)
    except MenuYamlError as error:
        if "未安装 PyYAML/ruamel.yaml" not in str(error):
            raise
        fallback_make_runtime(source, output, overrides)
        backend = "restricted-top-level"
        values = restricted_top_level_values(output)
    validate_safety(values)
    print(f"通过：已生成运行时配置缓存 {output}（{backend}），安全开关均为 false。")
    return 0


def command_show_fields(args: argparse.Namespace) -> int:
    path = Path(args.config)
    for key in args.keys:
        value, _ = get_top_level_value(path, key)
        print(f"{key}: {json.dumps(value, ensure_ascii=False)}")
    return 0


def roundtrip_load(path: Path):
    backend, yaml = import_yaml(require_roundtrip=True)
    if backend != "ruamel":
        raise MenuYamlError("未安装 ruamel.yaml，无法保留 YAML 注释和结构。")
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle)
    except Exception as exc:
        raise MenuYamlError(f"无法解析正式配置：{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise MenuYamlError(f"正式配置顶层不是映射：{path}")
    return yaml, data


def command_compare(args: argparse.Namespace) -> int:
    target = Path(args.config)
    candidate = Path(args.candidate)
    _, target_data = roundtrip_load(target)
    _, candidate_data = roundtrip_load(candidate)
    for key in args.keys:
        if key not in target_data or key not in candidate_data:
            raise MenuYamlError(f"字段级比较缺少键：{key}")
        before = target_data[key]
        after = candidate_data[key]
        print(f"字段：{key}")
        print(f"  当前值：{json.dumps(before, ensure_ascii=False)}")
        print(f"  候选值：{json.dumps(after, ensure_ascii=False)}")
        print(f"  是否变化：{'是' if before != after else '否'}")
    return 0


def command_write_fields(args: argparse.Namespace) -> int:
    # 原子性仅覆盖单个目标文件；不保证两份配置同步更新，也不处理并发修改。
    # 此底层函数没有用户确认或 dry-run 参数，调用方必须承担确认和试运行拦截。
    target = Path(args.config).resolve()
    candidate_path = Path(args.candidate).resolve()
    yaml, target_data = roundtrip_load(target)
    _, candidate_data = roundtrip_load(candidate_path)

    for key in args.keys:
        if key not in target_data or key not in candidate_data:
            raise MenuYamlError(f"无法写入缺失字段：{key}")
        value = candidate_data[key]
        if isinstance(value, float) and not math.isfinite(value):
            raise MenuYamlError(f"候选字段不是有限数字：{key}")
        target_data[key] = value

    original_mode = stat.S_IMODE(target.stat().st_mode)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            yaml.dump(target_data, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, original_mode)

        _, _, verified = load_yaml(temporary)
        for key in args.keys:
            if verified.get(key) != candidate_data[key]:
                raise MenuYamlError(f"原子替换前重新解析校验失败：{key}")

        os.replace(temporary, target)
        _, _, final_data = load_yaml(target)
        for key in args.keys:
            if final_data.get(key) != candidate_data[key]:
                raise MenuYamlError(f"原子替换后重新解析校验失败：{key}")
    finally:
        if temporary.exists():
            temporary.unlink()
    print(f"通过：已原子写入并重新验证 {target}")
    return 0


def command_write_fields_pair(args: argparse.Namespace) -> int:
    """将同一候选字段写入两份正式配置，并在任一步失败时恢复两者。"""
    target_a = Path(args.config_a).resolve()
    target_b = Path(args.config_b).resolve()
    candidate_path = Path(args.candidate).resolve()
    if target_a == target_b:
        raise MenuYamlError("双配置写入要求两个不同的目标文件。")
    if target_a.parent != target_b.parent:
        raise MenuYamlError("双配置写入要求目标文件位于同一目录。")
    if not target_a.is_file() or not target_b.is_file():
        raise MenuYamlError("双配置写入目标必须是已有的普通文件。")

    yaml_a, data_a = roundtrip_load(target_a)
    yaml_b, data_b = roundtrip_load(target_b)
    _, candidate_data = roundtrip_load(candidate_path)
    for key in args.keys:
        if key not in data_a or key not in data_b or key not in candidate_data:
            raise MenuYamlError(f"双配置写入缺少字段：{key}")
        value = candidate_data[key]
        if isinstance(value, float) and not math.isfinite(value):
            raise MenuYamlError(f"候选字段不是有限数字：{key}")
        data_a[key] = value
        data_b[key] = value

    def render_temporary(target: Path, yaml, data: dict[str, Any]) -> Path:
        descriptor, name = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
        )
        temporary = Path(name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                yaml.dump(data, handle)
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary, stat.S_IMODE(target.stat().st_mode))
            _, _, verified = load_yaml(temporary)
            for key in args.keys:
                if verified.get(key) != candidate_data[key]:
                    raise MenuYamlError(f"替换前重新解析校验失败：{target.name}:{key}")
            return temporary
        except Exception:
            temporary.unlink(missing_ok=True)
            raise

    def preserve_original(target: Path) -> Path:
        descriptor, name = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".rollback", dir=str(target.parent)
        )
        backup = Path(name)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(target.read_bytes())
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(backup, stat.S_IMODE(target.stat().st_mode))
        return backup

    temporary_a = temporary_b = backup_a = backup_b = None
    replaced = False
    try:
        temporary_a = render_temporary(target_a, yaml_a, data_a)
        temporary_b = render_temporary(target_b, yaml_b, data_b)
        backup_a = preserve_original(target_a)
        backup_b = preserve_original(target_b)
        os.replace(temporary_a, target_a)
        temporary_a = None
        os.replace(temporary_b, target_b)
        temporary_b = None
        replaced = True
        for target in (target_a, target_b):
            _, _, verified = load_yaml(target)
            for key in args.keys:
                if verified.get(key) != candidate_data[key]:
                    raise MenuYamlError(f"替换后重新解析校验失败：{target.name}:{key}")
    except Exception:
        # 回滚文件仅存在于当前写入事务中，成功或失败后都会删除。
        if backup_a is not None and backup_a.exists():
            os.replace(backup_a, target_a)
            backup_a = None
        if backup_b is not None and backup_b.exists():
            os.replace(backup_b, target_b)
            backup_b = None
        raise
    finally:
        for path in (temporary_a, temporary_b, backup_a, backup_b):
            if path is not None and path.exists():
                path.unlink()

    if not replaced:
        raise MenuYamlError("双配置写入未完成。")
    print(f"通过：已一致写入并重新验证 {target_a} 与 {target_b}")
    return 0


def extract_numbers(text: str, pattern: str) -> list[float]:
    match = re.search(pattern, text, flags=re.MULTILINE | re.DOTALL)
    if not match:
        return []
    values: list[float] = []
    for item in match.group(1).split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    return values


def finite(values: list[float]) -> bool:
    return bool(values) and all(math.isfinite(value) for value in values)


def write_simple_yaml(path: Path, fields: dict[str, list[float]]) -> None:
    lines = []
    for key, values in fields.items():
        lines.append(f"{key}: {json.dumps(values, ensure_ascii=False)}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def command_check_calibration(args: argparse.Namespace) -> int:
    # 日志字段缺失意味着证据不足，不能据此断言采集数据本身质量差。
    # 当前输出属于保守摘要，不等于完整的标定验收；日志格式变更需同步解析契约。
    log_path = Path(args.log)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    valid_match = re.search(r"有效标定图像:\s*(\d+)", text)
    size_match = re.search(r"尺寸:\s*(\d+)x(\d+)", text)
    error_match = re.search(r"重投影误差:\s*([0-9.eE+-]+)", text)
    matrix = extract_numbers(text, r"^camera_matrix:\s*\[([^\]]+)\]")
    distortion = extract_numbers(text, r"^distort_coeffs:\s*\[([^\]]+)\]")

    if not valid_match or len(matrix) != 9 or not finite(matrix) or not finite(distortion):
        print("综合判定：不通过")
        print("原因：计算输出缺少有效图像数量、完整相机矩阵或有限畸变参数。")
        return 2

    valid_count = int(valid_match.group(1))
    if valid_count < 10:
        print("综合判定：不通过")
        print(f"原因：有效标定图像不足，当前为 {valid_count} 张，至少需要 10 张。")
        return 2

    resolution = (
        f"{size_match.group(1)}x{size_match.group(2)}" if size_match else "未报告"
    )
    average_error = error_match.group(1) if error_match else "未报告"
    candidate = Path(args.candidate_output)
    write_simple_yaml(
        candidate, {"camera_matrix": matrix, "distort_coeffs": distortion}
    )
    Path(args.status_output).write_text("unknown\n", encoding="utf-8")

    print("综合判定：无法判断")
    print("原因：现有程序没有提供足够的质量指标。")
    print("数据无法判断，请使用 AI 协助分析。")
    print("")
    print(f"输入图像数量：{valid_count}")
    print(f"有效图像数量：{valid_count}")
    print(f"相机分辨率：{resolution}")
    print(f"相机矩阵：{json.dumps(matrix, ensure_ascii=False)}")
    print(f"畸变参数：{json.dumps(distortion, ensure_ascii=False)}")
    print(f"平均重投影误差：{average_error}")
    print(
        "缺失的判定指标：标定板覆盖范围、远近变化、倾斜姿态变化、"
        "单张最大误差、异常样本数量、收敛判定及结构化质量报告。"
    )
    return 0


def command_check_handeye(args: argparse.Namespace) -> int:
    log_path = Path(args.log)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    valid_match = re.search(r"有效手眼标定图像:\s*(\d+)", text)
    rotation = extract_numbers(text, r"^R_camera2gimbal:\s*\[([^\]]+)\]")
    translation = extract_numbers(text, r"^t_camera2gimbal:\s*\[([^\]]+)\]")

    if not valid_match or len(rotation) != 9 or len(translation) != 3:
        print("综合判定：不通过")
        print("原因：计算输出缺少有效图像数量、完整旋转矩阵或平移向量。")
        return 2
    valid_count = int(valid_match.group(1))
    if valid_count < 5:
        print("综合判定：不通过")
        print(f"原因：有效手眼标定图像不足，当前为 {valid_count} 张，至少需要 5 张。")
        return 2
    if not finite(rotation) or not finite(translation):
        print("综合判定：不通过")
        print("原因：旋转矩阵或平移向量包含 NaN/Inf。")
        return 2

    write_simple_yaml(
        Path(args.candidate_output),
        {"R_camera2gimbal": rotation, "t_camera2gimbal": translation},
    )
    Path(args.status_output).write_text("unknown\n", encoding="utf-8")
    print("综合判定：无法判断")
    print("原因：现有手眼计算程序未输出旋转残差、平移残差、姿态覆盖范围和收敛信息。")
    print("计算一致性：无法判断")
    print("实际安装精度：无法判断")
    print("数据无法判断，请使用 AI 协助分析或进行实际对准验证。")
    print(f"有效样本数量：{valid_count}")
    print(f"R_camera2gimbal：{json.dumps(rotation, ensure_ascii=False)}")
    print(f"t_camera2gimbal：{json.dumps(translation, ensure_ascii=False)}")
    return 0


def parse_stat_triplet(text: str, section: int) -> tuple[float, float, float] | None:
    match = re.search(
        rf"^\[{section}\][^\n]*mean=([0-9.eE+-]+)\s+std=([0-9.eE+-]+)\s+max=([0-9.eE+-]+)",
        text,
        flags=re.MULTILINE,
    )
    if not match:
        return None
    values = tuple(float(item) for item in match.groups())
    if not all(math.isfinite(item) for item in values):
        return None
    return values  # type: ignore[return-value]


def parse_stat_pair(text: str, section: int) -> tuple[float, float] | None:
    match = re.search(
        rf"^\[{section}\][^\n]*(?<!\w)mean=([0-9.eE+-]+)\s+max=([0-9.eE+-]+)",
        text,
        flags=re.MULTILINE,
    )
    if not match:
        return None
    values = (float(match.group(1)), float(match.group(2)))
    if not all(math.isfinite(item) for item in values):
        return None
    return values


def parse_range(text: str, axis: str) -> tuple[float, float] | None:
    match = re.search(
        rf"{axis}\[([-+0-9.eE]+),([-+0-9.eE]+)\]",
        text,
    )
    if not match:
        return None
    low, high = (float(match.group(1)), float(match.group(2)))
    if not math.isfinite(low) or not math.isfinite(high):
        return None
    return low, high


def command_check_handeye_verification(args: argparse.Namespace) -> int:
    log_path = Path(args.log)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    sample_match = re.search(r"有效样本:\s*(\d+)\s*帧", text)
    position = parse_stat_triplet(text, 1)
    orientation = parse_stat_triplet(text, 2)
    axxb = parse_stat_pair(text, 4)
    reprojection = parse_stat_triplet(text, 5)
    yaw = parse_range(text, "yaw")
    pitch = parse_range(text, "pitch")
    roll = parse_range(text, "roll")

    missing: list[str] = []
    if not sample_match:
        missing.append("有效样本数")
    if position is None:
        missing.append("板位姿平移残差")
    if orientation is None:
        missing.append("板位姿旋转残差")
    if axxb is None:
        missing.append("AX=XB 残差")
    if reprojection is None:
        missing.append("重投影误差")
    if yaw is None or pitch is None or roll is None:
        missing.append("云台运动范围")

    if missing:
        Path(args.status_output).write_text("unknown\n", encoding="utf-8")
        print("综合判定：无法判断")
        print("原因：verify_handeye 输出缺少必要指标。")
        print("缺失指标：" + "、".join(missing))
        print("数据无法判断，请使用 AI 协助分析。")
        return 0

    sample_count = int(sample_match.group(1))
    yaw_range = yaw[1] - yaw[0]
    pitch_range = pitch[1] - pitch[0]
    roll_range = roll[1] - roll[0]

    print(f"有效样本数量：{sample_count}")
    print(
        "云台运动范围："
        f"yaw {yaw_range:.2f}°，pitch {pitch_range:.2f}°，roll {roll_range:.2f}°"
    )
    print(
        "板世界系平移残差："
        f"均值 {position[0]:.2f} mm，标准差 {position[1]:.2f} mm，最大 {position[2]:.2f} mm"
    )
    print(
        "板世界系旋转残差："
        f"均值 {orientation[0]:.2f}°，标准差 {orientation[1]:.2f}°，最大 {orientation[2]:.2f}°"
    )
    print(
        "手眼方程 AX=XB 残差："
        f"均值 {axxb[0]:.2f}°，最大值 {axxb[1]:.2f}°"
    )
    print(
        "重投影误差："
        f"均值 {reprojection[0]:.2f} px，标准差 {reprojection[1]:.2f} px，"
        f"最大 {reprojection[2]:.2f} px"
    )

    failures: list[str] = []
    if sample_count < 5:
        failures.append(f"有效样本不足：{sample_count}，至少需要 5 帧")
    if max(yaw_range, pitch_range, roll_range) < 5.0:
        failures.append("云台运动范围退化，三个轴的最大行程均小于 5°")
    if position[0] > 10.0 or position[2] > 20.0:
        failures.append("板世界系平移残差过大")
    if orientation[0] > 1.0 or orientation[2] > 3.0:
        failures.append("板世界系旋转残差过大")
    if axxb[0] > 1.0 or axxb[1] > 3.0:
        failures.append("AX=XB 残差过大")
    if reprojection[0] > 1.0 or reprojection[2] > 3.0:
        failures.append("重投影误差过大")

    if failures:
        Path(args.status_output).write_text("fail\n", encoding="utf-8")
        print("综合判定：不通过")
        for reason in failures:
            print(f"原因：{reason}")
        return 2

    Path(args.status_output).write_text("cautious\n", encoding="utf-8")
    print("计算一致性：通过")
    print("实际安装精度：无法判断")
    print("综合判定：谨慎使用")
    print("原因：没有外部测量或实际对准验证，数学残差只能证明计算结果自洽。")
    print("数据无法判断，请使用 AI 协助分析或进行实际对准验证。")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SP-Vision 调试菜单 YAML/日志辅助")
    subparsers = parser.add_subparsers(dest="command", required=True)

    get_parser = subparsers.add_parser("get")
    get_parser.add_argument("--config", required=True)
    get_parser.add_argument("--key", required=True)
    get_parser.add_argument("--show-backend", action="store_true")
    get_parser.set_defaults(func=command_get)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("--config", required=True)
    validate_parser.set_defaults(func=command_validate)

    runtime_parser = subparsers.add_parser("make-runtime")
    runtime_parser.add_argument("--source", required=True)
    runtime_parser.add_argument("--output", required=True)
    runtime_parser.add_argument("--set", action="append", default=[])
    runtime_parser.set_defaults(func=command_make_runtime)

    fields_parser = subparsers.add_parser("show-fields")
    fields_parser.add_argument("--config", required=True)
    fields_parser.add_argument("--keys", nargs="+", required=True)
    fields_parser.set_defaults(func=command_show_fields)

    compare_parser = subparsers.add_parser("compare")
    compare_parser.add_argument("--config", required=True)
    compare_parser.add_argument("--candidate", required=True)
    compare_parser.add_argument("--keys", nargs="+", required=True)
    compare_parser.set_defaults(func=command_compare)

    write_parser = subparsers.add_parser("write-fields")
    write_parser.add_argument("--config", required=True)
    write_parser.add_argument("--candidate", required=True)
    write_parser.add_argument("--keys", nargs="+", required=True)
    write_parser.set_defaults(func=command_write_fields)

    pair_parser = subparsers.add_parser("write-fields-pair")
    pair_parser.add_argument("--config-a", required=True)
    pair_parser.add_argument("--config-b", required=True)
    pair_parser.add_argument("--candidate", required=True)
    pair_parser.add_argument("--keys", nargs="+", required=True)
    pair_parser.set_defaults(func=command_write_fields_pair)

    calibration_parser = subparsers.add_parser("check-calibration")
    calibration_parser.add_argument("--log", required=True)
    calibration_parser.add_argument("--candidate-output", required=True)
    calibration_parser.add_argument("--status-output", required=True)
    calibration_parser.set_defaults(func=command_check_calibration)

    handeye_parser = subparsers.add_parser("check-handeye")
    handeye_parser.add_argument("--log", required=True)
    handeye_parser.add_argument("--candidate-output", required=True)
    handeye_parser.add_argument("--status-output", required=True)
    handeye_parser.set_defaults(func=command_check_handeye)

    verification_parser = subparsers.add_parser("check-handeye-verification")
    verification_parser.add_argument("--log", required=True)
    verification_parser.add_argument("--status-output", required=True)
    verification_parser.set_defaults(func=command_check_handeye_verification)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except MenuYamlError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
