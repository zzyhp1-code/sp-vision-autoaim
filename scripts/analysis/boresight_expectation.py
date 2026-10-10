# -*- coding: utf-8 -*-
"""从冻结的外参读出“现场零点修正”该按什么数量级期待的结果（离线，不启动任何程序）。

用途
----
运行时的偏航闭环用像素横向误差 atan2(dx, fx) 生成目标，俯仰闭环用 atan2(dy, fy)，
两者的基准都是“图像中心”。因此：

- 光轴与云台基准的失配、主点相对图像中心的偏置、贴装误差，
  最终都表现为一个固定的像素偏置 ⇒ 现场用固定靶把残差拉到 0 即可，不需要拆解各项。
- 命令里 yaw 方向是 `boresight + impact` 两个修正之和（standard.cpp:126-129），
  而像素误差又减掉了 impact（standard.cpp:1415-1416）：
    残差(收敛后) = 固定失配 - impact
  所以这两项在当前代码里只有“和”是可观测的；正确用法是先把 impact 置 0
  收敛 boresight，再用实弹落点单独定 impact。
"""
import re
import sys

import numpy as np

CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'configs/standard.yaml'
text = open(CONFIG, encoding='utf-8').read()


def numbers(key):
    m = re.search(r'^\s*%s\s*:\s*\[([^\]]*)\]' % key, text, re.M)
    if not m:
        raise SystemExit('配置里找不到 %s' % key)
    return [float(v) for v in m.group(1).replace(',', ' ').split()]


R = np.array(numbers('R_camera2gimbal')).reshape(3, 3)
t = np.array(numbers('t_camera2gimbal'))
K = np.array(numbers('camera_matrix')).reshape(3, 3)
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

print('冻结外参：R_camera2gimbal（行优先）=' )
for row in R:
    print('  [%+.6f %+.6f %+.6f]' % tuple(row))
print('  t_camera2gimbal = [%.3f, %.3f, %.3f] m' % tuple(t))
print('  正交性检查 |R·Rᵀ − I|max = %.2e，det = %.6f（应为 1）'
      % (np.abs(R @ R.T - np.eye(3)).max(), np.linalg.det(R)))
print('')
print('光轴在“相机与云台之间那个刚体系”里的方向 = R 第三列 = [%+.4f, %+.4f, %+.4f]'
      % tuple(R[:, 2]))
print('  该方向相对第 3 轴的最大偏离 = %.2f°（= 相机的固定装配偏角量级）'
      % np.degrees(np.arccos(min(1.0, abs(float(R[2, 2]))))))
print('')
print('像素与角度的换算（闭环两个方向都用它）：')
print('  偏航 1° = %.1f px，1 px = %.4f°（fx = %.1f）'
      % (fx * np.tan(np.radians(1.0)), np.degrees(np.arctan2(1.0, fx)), fx))
print('  俯仰 1° = %.1f px，1 px = %.4f°（fy = %.1f）'
      % (fy * np.tan(np.radians(1.0)), np.degrees(np.arctan2(1.0, fy)), fy))
print('')
print('主点 (%.1f, %.1f) 与图像中心 (720, 540) 的差：%+.1f px / %+.1f px'
      % (cx, cy, cx - 720.0, cy - 540.0))
print('  闭环以“图像中心”为基准，这段差会整体变成一个固定瞄准偏置。')
print('  按上面的换算，它相当于偏航 %+.2f°、俯仰 %+.2f°。'
      % (np.degrees(np.arctan2(cx - 720.0, fx)), np.degrees(np.arctan2(cy - 540.0, fy))))
print('  ⚠ 该等效角只是“主点相对图像中心”的量，实际光轴失配由外参决定，')
print('    两者不应相加，最终只以现场闭环残差为准（见下）。')
print('  ⚠ 修正量受 standard.cpp:112-129 的钳位限制：boresight ±5°、两项之和 ±10°。')
print('    现场闭环残差若超过该范围，说明失配不在可修范围内，必须先查装配/内参，')
print('    不要靠加大修正量硬凑。')
print('')
print('现场做法（参数在 configs/standard.yaml，改完重启生效）：')
print('  1. 把 xuc_direct_yaw_impact_correction_deg 置 0，固定靶停在画面里；')
print('  2. 读稳定后的横向残差 px ⇒ Δboresight = −atan2(残差_px, %.0f)，' % fx)
print('     （残差在中心右侧为正 ⇒ 修正取负号；若方向反了说明符号约定要看日志再确认）')
print('     迭代 2–3 次，直到残差在 ±10 px（约 ±0.22°）内；')
print('  3. 再按实弹落点单独定 xuc_direct_yaw_impact_correction_deg；')
print('  4. 俯仰侧同理用纵向残差检查 xuc_ballistic_pitch_trim_deg（当前 -1.42°），')
print('     2 px ≈ 0.044°，最终只有实弹落点能判定。')
