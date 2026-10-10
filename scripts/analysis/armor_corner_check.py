# -*- coding: utf-8 -*-
"""装甲板四角点检测核对工具（离线或整目录图片，不打开相机、不启动控制程序）。

用途
----
检测网络（yolo26 keypoint 头）给出装甲板四个角点后，需要回答两个问题：
  1. 四个点是否落在装甲板的物理角点上（左右灯条的上下端）？
  2. 它们的几何形状是否自洽（矩形度、上下边平行度、两侧灯条长度比）？
本工具用**已保存的图片**做只读复算，逐张输出几何量并导出 CSV，便于逐张比对与统计。

口径
----
角点顺序按项目约定：0=左上、1=右上、2=右下、3=左下（见 yolo26.cpp 的 sort_keypoints）。
几何量的定义与运行时**同构**（yolo26.cpp refine_corners / has_plausible_geometry）：
  left  = |P3-P0|   right = |P2-P1|   top = |P1-P0|   bottom = |P2-P3|
  ratio        = max(top,bottom) / max(left,right)      两灯条中点连线 / 长灯条
  side_ratio   = max(left,right) / min(left,right)      左右灯条长度比
  width_balance= max(top,bottom)  / min(top,bottom)     上下边长度比
  rectangular  = 两侧灯条与“中点连线”垂直方向的夹角最大值（弧度 → 度）
  parallel     = 上下边方向差（度）

两颗不同的门槛不要混用：
- **运行时放行门槛**（yolo26.cpp has_plausible_geometry + 配置）：宽高比 0.6–8.0、
  side_ratio ≤ 2.2、width_balance ≤ 2.5、面积 ≥ 0.18×宽×高；配置里还有
  min_armor_ratio 1 / max_armor_ratio 5 / max_side_ratio 1.5 / max_rectangular_error 25°。
  这些门槛很松，**通过不等于角点准**。
- **本工具默认判据**（更紧，用于发现问题）：用户可用参数调整。

重要限制
--------
角点是**网络输出**，本工具不接口模型、不做推理，只做几何与稳定性核算。
所以你需要提供每张图的四个角点（见 --corners 格式）。两种拿法：
  A) 运行时叠层图：yolo26 debug 叠层会画四点（LT/RT/RB/LB）与序号，可目视核对；
  B) 逐帧导出：需要给 `camera_detect_test`/`camera_pnp_test` 之类加角点日志（尚未加）。

静态目标的抖动（--jitter）是这套数据最有价值的指标：靶子不动时角点逐帧漂移多少，
直接决定 PnP 解算与后续跟踪的噪声底。
"""
import argparse
import csv
import math
import os
import re
import sys

import cv2
import numpy as np

IMAGE_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def imread_unicode(path, flags=cv2.IMREAD_COLOR):
    """Windows 上 cv2.imread 走 ANSI 路径，非 ASCII 目录要用 imdecode。"""
    try:
        with open(path, 'rb') as fh:
            buf = np.frombuffer(fh.read(), dtype=np.uint8)
        return cv2.imdecode(buf, flags)
    except OSError:
        return None


def geometry(points):
    p0, p1, p2, p3 = [np.array(p, dtype=float) for p in points]
    left = float(np.linalg.norm(p3 - p0))
    right = float(np.linalg.norm(p2 - p1))
    top = float(np.linalg.norm(p1 - p0))
    bottom = float(np.linalg.norm(p2 - p3))
    ratio = max(top, bottom) / max(1e-3, max(left, right))
    side_ratio = max(left, right) / max(1e-3, min(left, right))
    width_balance = max(top, bottom) / max(1e-3, min(top, bottom))
    left_center = (p0 + p3) * 0.5
    right_center = (p1 + p2) * 0.5
    roll = math.atan2(right_center[1] - left_center[1], right_center[0] - left_center[0])

    def rect_err(vector):
        ang = math.atan2(vector[1], vector[0]) - roll - math.pi / 2.0
        return abs(math.remainder(ang, math.pi))

    rectangular = max(rect_err(p3 - p0), rect_err(p2 - p1))
    center = (p0 + p1 + p2 + p3) * 0.25
    top_vec, bottom_vec = p1 - p0, p2 - p3
    parallel = abs(math.remainder(
        math.atan2(top_vec[1], top_vec[0]) - math.atan2(bottom_vec[1], bottom_vec[0]),
        math.pi))
    height = 0.5 * (left + right)
    width = 0.5 * (top + bottom)
    quad = np.array([p0, p1, p2, p3])
    area = 0.5 * abs(float(
        np.dot(quad[:, 0], np.roll(quad[:, 1], -1)) -
        np.dot(quad[:, 1], np.roll(quad[:, 0], -1))))
    return {
        'left_px': left, 'right_px': right, 'top_px': top, 'bottom_px': bottom,
        'width_px': width, 'height_px': height,
        'ratio': ratio, 'side_ratio': side_ratio, 'width_balance': width_balance,
        'rectangular_deg': math.degrees(rectangular),
        'parallel_deg': math.degrees(parallel),
        'center_x': float(center[0]), 'center_y': float(center[1]),
        'roll_deg': math.degrees(roll),
        'aspect': width / max(1e-3, height),
        'area_ratio': area / max(1e-3, width * height),
        'plausible': int(width >= 3.0 and height >= 3.0 and
                         0.6 <= width / max(1e-3, height) <= 8.0 and
                         side_ratio <= 2.2 and width_balance <= 2.5 and
                         area >= 0.18 * width * height),
    }


def read_corners(path):
    """读取 '<图片> x0 y0 ... x3 y3' 文本；返回 {文件名: [8 个数]}。"""
    table = {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = re.split(r'[,\s]+', line)
            name = parts[0]
            if len(parts) < 9:
                continue
            try:
                table[os.path.basename(name)] = [float(v) for v in parts[1:9]]
            except ValueError:
                continue
    return table


def overlay(img, points):
    out = img.copy()
    pts = np.array(points, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, (0, 255, 0), 2)
    names = ['LT', 'RT', 'RB', 'LB']
    for i, (x, y) in enumerate(points):
        cv2.circle(out, (int(round(x)), int(round(y))), 3, (0, 0, 255), -1)
        cv2.putText(out, names[i], (int(x) + 4, int(y) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return out


def judge(row, args):
    """返回 (是否通过, 原因)。判据是**本工具的收紧口径**，不是运行时放行门槛。"""
    reasons = []
    if not row['plausible']:
        reasons.append('不满足运行时几何门槛(plausible=0)')
    if not (args.min_ratio <= row['ratio'] <= args.max_ratio):
        reasons.append('ratio=%.3f 越界[%.2f,%.2f]' % (row['ratio'], args.min_ratio, args.max_ratio))
    if row['side_ratio'] > args.max_side_ratio:
        reasons.append('side_ratio=%.3f>%.2f' % (row['side_ratio'], args.max_side_ratio))
    if row['rectangular_deg'] > args.max_rectangular_deg:
        reasons.append('rectangular=%.2f°>%.1f°' % (row['rectangular_deg'], args.max_rectangular_deg))
    if row['parallel_deg'] > args.max_parallel_deg:
        reasons.append('parallel=%.2f°>%.1f°' % (row['parallel_deg'], args.max_parallel_deg))
    if row['top_px'] < args.min_edge_px or row['bottom_px'] < args.min_edge_px:
        reasons.append('上下边过短(%.1f/%.1f px)' % (row['top_px'], row['bottom_px']))
    return (not reasons), '; '.join(reasons)


def main():
    ap = argparse.ArgumentParser(description='装甲板四角点几何核对（只读）')
    ap.add_argument('input', help='单张图片，或包含图片的目录')
    ap.add_argument('--corners', help='角点文本：每行 "<图片> x0 y0 ... x3 y3"')
    ap.add_argument('--csv', help='导出逐张几何量 CSV')
    ap.add_argument('--overlay-dir', help='导出叠层图（含角点与顺序标注）')
    ap.add_argument('--min-ratio', type=float, default=0.80)
    ap.add_argument('--max-ratio', type=float, default=4.00)
    ap.add_argument('--max-side-ratio', type=float, default=1.30)
    ap.add_argument('--max-rectangular-deg', type=float, default=6.0)
    ap.add_argument('--max-parallel-deg', type=float, default=4.0)
    ap.add_argument('--min-edge-px', type=float, default=6.0)
    ap.add_argument('--jitter', action='store_true',
                    help='按输入顺序统计角点抖动（静态目标才有意义）')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    if os.path.isdir(args.input):
        files = sorted(f for f in os.listdir(args.input) if f.lower().endswith(IMAGE_EXT))
        paths = [os.path.join(args.input, f) for f in files]
    else:
        paths = [args.input]

    table = read_corners(args.corners) if args.corners else {}
    if args.corners and not table:
        raise SystemExit('角点文本里没有可解析的行：%s' % args.corners)

    rows = []
    passed = 0
    corner_series = []
    for path in paths:
        name = os.path.basename(path)
        img = imread_unicode(path)
        if img is None:
            print('[skip] 读不到图片: %s' % path, file=sys.stderr)
            continue
        h, w = img.shape[:2]
        nums = table.get(name)
        if nums is None:
            if not args.quiet:
                print('%-28s %4dx%-5d 无角点输入（跳过几何核对）' % (name, w, h))
            continue
        points = [(nums[0], nums[1]), (nums[2], nums[3]), (nums[4], nums[5]), (nums[6], nums[7])]
        g = geometry(points)
        ok, why = judge(g, args)
        passed += 1 if ok else 0
        row = {'image': name, 'width': w, 'height': h}
        row.update(g)
        row['pass'] = int(ok)
        row['reason'] = why
        rows.append(row)
        corner_series.append([(nums[0], nums[1]), (nums[2], nums[3]),
                              (nums[4], nums[5]), (nums[6], nums[7])])
        if not args.quiet:
            print('%-28s %4dx%-5d ratio=%.3f side=%.3f rect=%.2f° par=%.2f° 中心=(%.1f,%.1f) %s'
                  % (name, w, h, g['ratio'], g['side_ratio'], g['rectangular_deg'],
                     g['parallel_deg'], g['center_x'], g['center_y'],
                     'OK' if ok else 'NG: ' + why))
        if args.overlay_dir:
            os.makedirs(args.overlay_dir, exist_ok=True)
            cv2.imwrite(os.path.join(args.overlay_dir, name), overlay(img, points))

    if rows:
        print('')
        print('共 %d 张有角点，通过 %d 张（%.1f%%）' % (len(rows), passed, 100.0 * passed / len(rows)))
        for key in ('ratio', 'side_ratio', 'rectangular_deg', 'parallel_deg'):
            values = np.array([r[key] for r in rows], dtype=float)
            print('  %-16s 中位 %.4f  95分位 %.4f  最大 %.4f'
                  % (key, float(np.median(values)), float(np.percentile(values, 95)), float(values.max())))
        if args.csv:
            with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
                writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            print('已导出 %s' % args.csv)

        if args.jitter and len(corner_series) >= 3:
            pts = np.array(corner_series, dtype=float)   # (n, 4, 2)
            med = np.median(pts, axis=0)
            dev = np.linalg.norm(pts - med[None], axis=2)   # (n, 4)
            step = np.linalg.norm(np.diff(pts, axis=0), axis=2)  # (n-1, 4)
            names = ['LT', 'RT', 'RB', 'LB']
            print('')
            print('角点抖动（静态目标才有意义；n=%d 帧）：' % len(corner_series))
            for i, nm in enumerate(names):
                print('  %s 偏离中位: rms %.2f px  最大 %.2f px | 帧间位移: rms %.2f px 最大 %.2f px'
                      % (nm, float(np.sqrt((dev[:, i] ** 2).mean())), float(dev[:, i].max()),
                         float(np.sqrt((step[:, i] ** 2).mean())), float(step[:, i].max())))
            center = pts.mean(axis=1)
            cdev = np.linalg.norm(center - np.median(center, axis=0)[None], axis=1)
            print('  四角中心: rms %.2f px  最大 %.2f px（对比：1 px ≈ 0.022°）'
                  % (float(np.sqrt((cdev ** 2).mean())), float(cdev.max())))

    if not table:
        print('')
        print('提示：本工具核对的是“角度/边长几何”，需要你提供网络输出的四角点。')
        print('  取角点的两条路：')
        print('   1) 运行时日志：yolo26 的 debug 叠层图会画出四点（LT/RT/RB/LB）；')
        print('   2) camera_detect_test 目前只画包围框，若要逐帧导出四点需另行加日志。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
