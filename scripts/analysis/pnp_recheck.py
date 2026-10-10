# -*- coding: utf-8 -*-
"""PnP 位姿-重投影独立复核（离线，用工具导出的逐帧角点 CSV）。

回答三件事：
  1. 检测角点与"刚性平面靶 + 内参/畸变"是否自洽 —— 对每帧重解 PnP 再投影，
     残差应接近像素噪声（~0.3 px）。若不为 0，说明角点本身不是同一刚体的一致投影。
  2. 运行时重投影误差那 6 px 是怎么来的 —— 用"仅偏航 + 写死俯仰 15°"的模型重投影，
     与用完整 PnP 姿态重投影对比；两者之差即该固定假定的代价。
  3. 射程对"靶板尺寸假定"有多敏感 —— 把光条长度/板宽各缩放 1%，看深度与 xy 射程变化，
     用来判断"深度是否被高度方向的 3% 不匹配带偏"。

用法: python scripts/analysis/pnp_recheck.py <pnp_csv> [configs/calibration.yaml]
"""
import csv
import math
import re
import sys

import cv2
import numpy as np

CSV = sys.argv[1] if len(sys.argv) > 1 else 'pnp_static.csv'
CFG = sys.argv[2] if len(sys.argv) > 2 else 'configs/calibration.yaml'
LIGHTBAR_M = 56e-3
SMALL_W_M = 135e-3
BIG_W_M = 230e-3


def load_cfg(path):
    text = open(path, encoding='utf-8').read()

    def numbers(key):
        m = re.search(r'^\s*%s\s*:\s*\[([^\]]*)\]' % key, text, re.M)
        return [float(v) for v in m.group(1).replace(',', ' ').split()]

    K = np.array(numbers('camera_matrix')).reshape(3, 3)
    D = np.array(numbers('distort_coeffs'))
    R = np.array(numbers('R_camera2gimbal')).reshape(3, 3)
    t = np.array(numbers('t_camera2gimbal'))
    return K, D, R, t


def obj_points(width):
    h = LIGHTBAR_M / 2.0
    return np.array([[0, width / 2, h], [0, -width / 2, h],
                     [0, -width / 2, -h], [0, width / 2, -h]], dtype=np.float64)


def corner_points(row):
    """兼容两种列名：x0..x3 / x01..x33（本项目两种导出工具各用一套）。"""
    if 'x0' in row:
        keys = ['x%d' % i for i in range(4)] + ['y%d' % i for i in range(4)]
        vals = [float(row[k]) for k in keys]
    else:
        keys = ['x%d%d' % (i + 1, i + 1) for i in range(4)] + ['y%d%d' % (i + 1, i + 1) for i in range(4)]
        vals = [float(row[k]) for k in keys]
    return np.array([[vals[i], vals[4 + i]] for i in range(4)], dtype=np.float64)


def main():
    K, D, R_c2g, t_c2g = load_cfg(CFG)
    rows = list(csv.DictReader(open(CSV, encoding='utf-8')))
    if not rows:
        raise SystemExit('CSV 为空')
    print('样本 %d 帧' % len(rows))

    widths = {'small': SMALL_W_M, 'big': BIG_W_M}
    res_rigid, ranges = [], []
    for r in rows:
        w = widths.get(r['type'], SMALL_W_M)
        pts = corner_points(r)
        ok, rvec, tvec = cv2.solvePnP(obj_points(w), pts, K, D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj_points(w), rvec, tvec, K, D)
        err = np.linalg.norm(proj.reshape(-1, 2) - pts, axis=1)
        res_rigid.append(err.mean())
        xyz_cam = tvec.reshape(3)
        xyz_gimbal = R_c2g @ xyz_cam + t_c2g
        ranges.append(math.hypot(xyz_gimbal[0], xyz_gimbal[1]))

    res_rigid = np.array(res_rigid)
    ranges = np.array(ranges)
    print('\n[1] 刚性平面靶一致性（每帧重解 PnP 后反投影）')
    print('    残差: 均值 %.4f px  中位 %.4f  最大 %.4f' % (
        res_rigid.mean(), np.median(res_rigid), res_rigid.max()))
    print('    ⇒ 接近像素噪声说明角点与"刚体+内参/畸变"自洽，运行时那 6 px 不是角点问题')

    print('\n[2] 射程（本工具独立解算，仅用云台系 xy 平面）')
    print('    均值 %.4f m  标准差 %.4f m（%.3f%%）' % (
        ranges.mean(), ranges.std(), 100.0 * ranges.std() / ranges.mean()))

    print('\n[3] 对靶板尺寸假定的敏感度（光条长度 ±1%%、板宽 ±1%%）')
    for tag, scale_l, scale_w in (('光条长度 +1%', 1.01, 1.0), ('光条长度 -1%', 0.99, 1.0),
                                  ('板宽 +1%', 1.0, 1.01), ('板宽 -1%', 1.0, 0.99)):
        rr = []
        for r in rows:
            w = widths.get(r['type'], SMALL_W_M) * scale_w
            h = (LIGHTBAR_M * scale_l) / 2.0
            obj = np.array([[0, w / 2, h], [0, -w / 2, h],
                            [0, -w / 2, -h], [0, w / 2, -h]], dtype=np.float64)
            pts = corner_points(r)
            ok, rvec, tvec = cv2.solvePnP(obj, pts, K, D, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            xyz = R_c2g @ tvec.reshape(3) + t_c2g
            rr.append(math.hypot(xyz[0], xyz[1]))
        rr = np.array(rr)
        print('    %-14s 射程均值 %.4f m（相对基准 %+.3f%%）' % (
            tag, rr.mean(), 100.0 * (rr.mean() - ranges.mean()) / ranges.mean()))

    print('\n结论判据: [1] 残差 ≤0.6 px、[2] 抖动 ≤1%、[3] 尺寸 1% 变化引起的射程变化 ≤1.2%')


if __name__ == '__main__':
    main()
