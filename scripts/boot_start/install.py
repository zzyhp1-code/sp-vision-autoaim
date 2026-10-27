#!/usr/bin/env python3
"""Install and enable next-boot startup. Does not start the vision program now."""

import os
from pathlib import Path
import subprocess
import tempfile


def main():
    project = Path(__file__).resolve().parents[2]
    unit = Path.home() / ".config/systemd/user/sp-vision-boot.service"
    if unit.exists() and "Managed by sp-vision boot installer" not in unit.read_text():
        raise RuntimeError(f"拒绝覆盖非本工具创建的服务：{unit}")
    if any(c in str(project) for c in ('\n', '\r', '"', '\\')):
        raise ValueError("工程路径包含不支持的字符")
    text = (Path(__file__).parent / "sp-vision-boot.service").read_text().replace("@PROJECT@", str(project).replace("%", "%%"))
    unit.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=unit.parent, suffix=".tmp", delete=False) as out:
            temporary = Path(out.name)
            out.write(text)
        os.replace(temporary, unit)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    subprocess.run(["systemd-analyze", "--user", "verify", str(unit)], check=True)
    subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "--user", "enable", unit.name], check=True)
    print(f"已启用下次开机启动：{unit}\n本次未启动视觉程序。")


if __name__ == "__main__":
    main()
