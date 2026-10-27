# -*- coding: utf-8 -*-
"""判定“原始 yaw（下位机上报值）增大”对应的物理转向。

链路（已从代码逐行确认）：
  下位机上报原始 yaw_raw(度)
    → io/xuc.cpp 接收层：* imu_angle_to_rad_（deg→rad），**不加符号**
    → imu_history_ 存的是 raw（弧度）
    → imu_at() 返回时再 * imu_yaw_sign_(-1) + offset
    → capture.cpp 把 imu_at() 的结果写进 samples.csv 的 yaw_rad

因此：  samples.csv 的 yaw  =  -1 × raw_yaw(rad)
  即 raw_yaw 与 samples.csv 的 yaw **符号相反**。

本脚本用标定数据先测出“samples.csv 的 yaw 增大时的物理转向”，
再按上面的符号关系换算成“原始 yaw 增大时的物理转向”。

物理依据：相机随云台转、棋盘固定在世界。
  云台向左转 ⇒ 世界物体在图像中向右移（u 增大）
  云台向右转 ⇒ 世界物体在图像中向左移（u 减小）
"""
import csv
import math
import os
import sys

import cv2
import numpy as np

PATTERN = (9, 7)
# 已确认：samples.csv 的 yaw = -1 * raw_yaw
CSV_YAW_SIGN = -1.0


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
        print('%s: 有效帧不足 (%d)' % (name, len(us)))
        return None

    u = np.array(us)
    yaw = np.array(yaws)
    pitch = np.array(pitches)

    def resid(a, b):
        A = np.vstack([b, np.ones_like(b)]).T
        coef, *_ = np.linalg.lstsq(A, a, rcond=None)
        return a - A @ coef

    r_partial = float(np.corrcoef(resid(u, pitch), resid(yaw, pitch))[0, 1])
    A = np.vstack([yaw, pitch, np.ones_like(yaw)]).T
    coef, *_ = np.linalg.lstsq(A, u, rcond=None)
    du_dyaw_csv = float(coef[0])

    # csv yaw 增大 ⇒ u 如何变 ⇒ 物理转向
    if du_dyaw_csv < 0:
        csv_dir = '向右转'
    else:
        csv_dir = '向左转'

    # raw_yaw = -csv_yaw ⇒ raw 增大等价于 csv 减小 ⇒ 转向相反
    raw_dir = '向左转' if csv_dir == '向右转' else '向右转'

    print('=== %s（%d 帧）===' % (name, len(us)))
    print('  csv yaw 跨度 %+.2f .. %+.2f°  |  du/dyaw(csv) = %+.2f px/deg  |  偏相关 %+.4f'
          % (yaw.min(), yaw.max(), du_dyaw_csv, r_partial))
    print('  ⇒ samples.csv 的 yaw 增大时：云台 %s' % csv_dir)
    print('  ⇒ 原始 yaw（= -csv yaw）增大时：云台 %s' % raw_dir)
    print()
    return raw_dir


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else '.'
    dirs = []
    if os.path.isdir(os.path.join(root, 'samples.csv')) or os.path.exists(
            os.path.join(root, 'samples.csv')):
        dirs = [root]
    else:
        for d in sorted(os.listdir(root)):
            p = os.path.join(root, d)
            if os.path.isdir(p) and os.path.exists(os.path.join(p, 'samples.csv')):
                dirs.append(p)
    results = []
    for d in dirs:
        r = analyse(os.path.basename(d), d)
        if r:
            results.append(r)
    print('=' * 60)
    if results:
        from collections import Counter
        c = Counter(results)
        print('汇总：原始 yaw 正方向 = %s' % c.most_common(1)[0][0])
        print('      （%d/%d 批数据一致）' % (c.most_common(1)[0][1], len(results)))
    else:
        print('没有可用数据')


if __name__ == '__main__':
    main()
