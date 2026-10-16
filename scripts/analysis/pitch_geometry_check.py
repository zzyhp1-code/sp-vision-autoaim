# -*- coding: utf-8 -*-
"""几何推理的自检：用 pitch 验证“相机转向与图像位移相反”这一前提。

已知（本仓库 standard.cpp 的注释与现场结论）：
  “目标位于画面中心下方时，需要更大的下位机 IMU pitch 目标”
  ⇒ pitch 增大 = 相机向下俯。
若本脚本从标定数据独立算出 “pitch 增大 ⇒ 棋盘在图像中向上移（v 减小）”，
则与“相机向下俯”一致，说明图像几何推理方向正确，可外推到 yaw 的左右判定。
"""
import csv
import math
import os
import sys

import cv2
import numpy as np

PATTERN = (9, 7)


def imread_u(path):
    try:
        with open(path, 'rb') as fh:
            buf = np.frombuffer(fh.read(), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    except OSError:
        return None


def analyse(name, folder):
    rows = list(csv.DictReader(open(os.path.join(folder, 'samples.csv'), encoding='utf-8')))
    us, vs, yaws, pitches = [], [], [], []
    for r in rows:
        n = int(r['sample'])
        img = imread_u(os.path.join(folder, '%d.jpg' % n))
        if img is None:
            continue
        ok, corners = cv2.findChessboardCorners(
            img, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            continue
        c = corners.reshape(-1, 2)
        us.append(float(c[:, 0].mean()))
        vs.append(float(c[:, 1].mean()))
        yaws.append(math.degrees(float(r['yaw_rad'])))
        pitches.append(math.degrees(float(r['pitch_rad'])))
    if len(us) < 5:
        return None

    u = np.array(us)
    v = np.array(vs)
    yaw = np.array(yaws)
    pitch = np.array(pitches)

    def resid(a, b):
        A = np.vstack([b, np.ones_like(b)]).T
        coef, *_ = np.linalg.lstsq(A, a, rcond=None)
        return a - A @ coef

    # 控制 yaw 后，v 与 pitch 的关系
    r_vp = float(np.corrcoef(resid(v, yaw), resid(pitch, yaw))[0, 1])
    A = np.vstack([pitch, yaw, np.ones_like(pitch)]).T
    coef, *_ = np.linalg.lstsq(A, v, rcond=None)
    dv_dpitch = float(coef[0])

    print('=== %s（%d 帧）===' % (name, len(us)))
    print('  dv/dpitch = %+.3f px/deg   偏相关 %+.4f' % (dv_dpitch, r_vp))
    if dv_dpitch < 0:
        print('  ⇒ pitch 增大时棋盘在图像中向上移（v 减小）')
        print('  ⇒ 相机向下俯 —— 与本仓库既有结论一致 ✅（几何推理前提成立）')
    else:
        print('  ⇒ pitch 增大时棋盘下移 ⇒ 相机向上抬（与既有结论相反，需复查）')
    print()
    return dv_dpitch


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else '.'
    dirs = []
    if os.path.exists(os.path.join(root, 'samples.csv')):
        dirs = [root]
    else:
        for d in sorted(os.listdir(root)):
            p = os.path.join(root, d)
            if os.path.isdir(p) and os.path.exists(os.path.join(p, 'samples.csv')):
                dirs.append(p)
    vals = [x for x in (analyse(os.path.basename(d), d) for d in dirs) if x is not None]
    if vals:
        print('=' * 60)
        print('全部为负: %s  （均值 %+.3f px/deg）' % (all(x < 0 for x in vals), float(np.mean(vals))))


if __name__ == '__main__':
    main()
