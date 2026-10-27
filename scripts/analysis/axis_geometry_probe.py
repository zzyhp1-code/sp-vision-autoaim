# -*- coding: utf-8 -*-
"""视觉侧尺度误差归因探针（离线，只用标定数据，不启动硬件）。

用途
----
量化“视觉观测到的相对转动角 / IMU 相对转动角”这一尺度，并检验两种常见归因：

1. 靶面几何：平面靶的板面法向 n 与云台转轴 a 若不垂直，
   视觉角会被 cos(angle(n, a)) 压低。检验办法是把每帧法向变换到云台系看是否一致，
   并用 |n·a| 预测尺度。
2. 畸变/视场依赖：把逐对尺度按棋盘中心像素分左右、上下分组，看是否随视场变化。

口径
----
视觉角 = 两帧棋盘位姿的相对旋转角（PnP，IPPE）；IMU 角 = 由 samples.csv 的
yaw/pitch 按 capture.cpp 同构 Rz·Ry 重建后的相对旋转角。转轴按云台系分量分类，
只取该轴上占优的相对转动，避免把 yaw/pitch 混在一起。

实测结论（2026-10-10，011048 / short / wide4 三批）
--------------------------------------------------
- 尺度：pitch 0.966–0.982、yaw 0.990–1.007，批间散布约 ±1%，与逐对散度同量级。
- 靶面几何预测失效（预测 0.35–0.54，观测 ≈0.97），法向与转轴夹角也不支持该解释。
- pitch 尺度随像面位置变化（左 0.974–0.990 / 右 0.962–0.973；上 0.956–0.962 /
  下 0.996），说明残差带视场依赖，属相机模型/平面靶层面的模型级残差，
  不是可以用外参吸收的常量增益。

用法
----
    python scripts/analysis/axis_geometry_probe.py [数据目录]
数据目录需形如 <批>/{1.jpg,1.txt,...,samples.csv}（capture 的输出格式）。
内参与靶参数直接读 configs/calibration.yaml（可用环境变量 SPV_CALIBRATION_YAML 换路径）。
Windows 上 cv2.imread 走 ANSI 路径，脚本内部改用 imdecode 以支持非 ASCII 目录。
"""
import json
import math
import os
import re
import sys

import cv2
import numpy as np


def load_calibration_config(path):
    """从 configs/calibration.yaml 读内参与靶参数（避免脚本里复制一份内参）。"""
    text = open(path, encoding='utf-8').read()

    def numbers(key):
        m = re.search(r'^\s*%s\s*:\s*\[([^\]]*)\]' % key, text, re.M)
        if not m:
            raise SystemExit('配置里找不到 %s: %s' % (key, path))
        return [float(v) for v in m.group(1).replace(',', ' ').split()]

    def scalar(key):
        m = re.search(r'^\s*%s\s*:\s*([0-9.]+)' % key, text, re.M)
        if not m:
            raise SystemExit('配置里找不到 %s: %s' % (key, path))
        return float(m.group(1))

    K = np.array(numbers('camera_matrix'), dtype=float).reshape(3, 3)
    D = np.array(numbers('distort_coeffs'), dtype=float).reshape(-1)
    pattern = (int(scalar('pattern_cols')), int(scalar('pattern_rows')))
    square = scalar('square_size_mm')
    return K, D, pattern, square


CONFIG = os.environ.get('SPV_CALIBRATION_YAML', 'configs/calibration.yaml')
K, D, PATTERN, SQUARE = load_calibration_config(CONFIG)
DATA = sys.argv[1] if len(sys.argv) > 1 else 'data/calibration'
OBJ = np.array([[j * SQUARE, i * SQUARE, 0.0]
                for i in range(PATTERN[1]) for j in range(PATTERN[0])], dtype=np.float64)


def pose_from_yaw_pitch(yaw, pitch):
    """与 calibration/capture.cpp:123 同构：Rz(yaw) * Ry(pitch)。"""
    y, p = yaw, pitch
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y), math.cos(y), 0],
                   [0, 0, 1.0]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)],
                   [0, 1.0, 0],
                   [-math.sin(p), 0, math.cos(p)]])
    return Rz @ Ry


def axis_angle(R):
    """返回 (轴, 角) 角的单位、角度制。"""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if ang < 1e-12:
        return np.zeros(3), 0.0
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    return ax / np.linalg.norm(ax), math.degrees(ang)


def load_dataset(folder):
    """读取 samples.csv（若有）与每帧棋盘位姿。返回帧列表。"""
    csv_path = os.path.join(folder, 'samples.csv')
    imu = {}
    if os.path.exists(csv_path):
        with open(csv_path, encoding='utf-8', errors='replace') as f:
            head = f.readline().strip().split(',')
            idx = {name: i for i, name in enumerate(head)}
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < len(head):
                    continue
                try:
                    n = int(float(parts[idx['sample']]))
                except (ValueError, KeyError):
                    continue
                yaw = float(parts[idx['yaw_rad']])
                pitch = float(parts[idx['pitch_rad']])
                station = parts[idx['station']] if 'station' in idx and idx['station'] < len(parts) else ''
                imu[n] = (yaw, pitch, station)

    frames = []
    for n in range(1, 400):
        img_path = os.path.join(folder, '%d.jpg' % n)
        if not os.path.exists(img_path):
            break
        # Windows 上 cv2.imread 走 ANSI 路径，遇非 ASCII 目录会失败；改用 imdecode
        try:
            with open(img_path, 'rb') as fh:
                buf = np.frombuffer(fh.read(), dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        except OSError:
            img = None
        if img is None:
            continue
        ok, corners = cv2.findChessboardCorners(
            img, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print('  [skip] 未检测到棋盘: %d.jpg' % n)
            continue
        corners = cv2.cornerSubPix(
            img, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1))
        ok, rvec, tvec = cv2.solvePnP(OBJ, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        # 对称棋盘的角点编号可能整体反转（绕板面内轴 180°），用连续性修正
        if frames:
            prev = frames[-1]['R']
            if np.linalg.norm(cv2.Rodrigues(R @ prev.T)[0]) > math.pi / 2:
                corners = corners[::-1].copy()
                ok, rvec, tvec = cv2.solvePnP(OBJ, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
                if not ok:
                    continue
                R, _ = cv2.Rodrigues(rvec)
                print('  [fix] 角点编号反转: %d.jpg' % n)
        yaw, pitch, station = imu.get(n, (None, None, ''))
        proj, _ = cv2.projectPoints(OBJ, rvec, tvec, K, D)
        rep = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        frames.append({
            'n': n, 'R': R, 't': tvec.reshape(3), 'corners': corners,
            'yaw': yaw, 'pitch': pitch, 'station': station,
            'rep': float(rep.mean()), 'normal_cam': R[:, 2].copy(),
        })
    return frames


def analyse(name, folder):
    print('=' * 78)
    print('数据集 %s' % name)
    frames = load_dataset(folder)
    if len(frames) < 5:
        print('  有效帧不足: %d' % len(frames))
        return None
    have_imu = all(f['yaw'] is not None for f in frames)
    print('  有效帧 %d，重投影均值 %.3f px（PnP 自洽性）'
          % (len(frames), float(np.mean([f['rep'] for f in frames]))))
    if not have_imu:
        print('  缺少 samples.csv 中的 yaw/pitch，跳过尺度归因')
        return None

    # 板面法向在云台系内的一致性：n_g = R_g2c^T * n_c，R_g2c = Rz(yaw)*Ry(pitch)
    normals_g = []
    for f in frames:
        R_g2c = pose_from_yaw_pitch(f['yaw'], f['pitch'])
        normals_g.append(R_g2c.T @ f['normal_cam'])
    normals_g = np.array(normals_g)
    n_mean = normals_g.mean(axis=0)
    n_mean /= np.linalg.norm(n_mean)
    dev = [math.degrees(math.acos(max(-1.0, min(1.0, abs(float(np.dot(v, n_mean))))))) for v in normals_g]
    print('  板面法向(云台系)一致性: 相对均值的偏差 mean=%.2f° max=%.2f°'
          % (float(np.mean(dev)), float(np.max(dev))))
    print('  板面法向(云台系) = [%.4f, %.4f, %.4f]' % tuple(n_mean))

    # 逐对：视觉角 vs IMU 角，按视觉转轴分类
    s_pitch, s_yaw, rows = [], [], []
    for i in range(len(frames)):
        for j in range(i + 1, len(frames)):
            Rc_i, Rc_j = frames[i]['R'], frames[j]['R']
            Rg_i = pose_from_yaw_pitch(frames[i]['yaw'], frames[i]['pitch'])
            Rg_j = pose_from_yaw_pitch(frames[j]['yaw'], frames[j]['pitch'])
            dR_c = Rc_j @ Rc_i.T          # 板在相机系中的相对转动（与法向口径一致）
            dR_g = Rg_j @ Rg_i.T          # 云台相对转动（IMU）
            ax_c, ang_c = axis_angle(dR_c)
            ax_g, ang_g = axis_angle(dR_g)
            if ang_g < 3.0:              # 太小的动作没有信噪比
                continue
            # 云台系里该对的转轴，落在 Y(pitch) 还是 Z(yaw) 上
            comp = np.abs(ax_g)
            kind = 'pitch' if comp[1] > comp[2] else 'yaw'
            s = ang_c / ang_g
            # 该对的两帧板面法向与真实转轴的夹角
            n1 = normals_g[i]
            n2 = normals_g[j]
            cos_predict = abs(float(np.dot(ax_g, (n1 + n2) / 2.0)))
            rows.append({'i': i, 'j': j, 'kind': kind, 's': s, 'ang_g': ang_g,
                         'cos_predict': cos_predict, 'ax_c': ax_c, 'ax_g': ax_g})
            (s_pitch if kind == 'pitch' else s_yaw).append(s)

    def stat(v):
        if not v:
            return 'n=0'
        return 'n=%d mean=%.4f min=%.4f max=%.4f' % (len(v), float(np.mean(v)), float(np.min(v)), float(np.max(v)))

    print('  [逐对尺度] 视觉角/IMU角（仅取 %s 轴）' % 'pitch/yaw')
    print('    pitch: %s' % stat(s_pitch))
    print('    yaw  : %s' % stat(s_yaw))

    # 平面靶几何预测：s_pred = |n . a|；同时给出逐对尺度的稳健统计
    summary = {}
    for kind in ('pitch', 'yaw'):
        sub = [r for r in rows if r['kind'] == kind]
        if not sub:
            continue
        s_obs = np.array([r['s'] for r in sub])
        s_pred = np.array([r['cos_predict'] for r in sub])
        resid = s_obs - s_pred
        axg = np.array([np.abs(r['ax_g']) for r in sub]).mean(axis=0)
        axc = np.array([np.abs(r['ax_c']) for r in sub]).mean(axis=0)
        names = ['x', 'y', 'z']
        dom_g = names[int(np.argmax(axg))]
        dom_c = names[int(np.argmax(axc))]
        print('    %-5s 尺度: 均值 %.4f 中位 %.4f IQR[%.4f, %.4f] | 云台系主轴 %s，相机系主轴 %s'
              % (kind, s_obs.mean(), float(np.median(s_obs)),
                 float(np.percentile(s_obs, 25)), float(np.percentile(s_obs, 75)), dom_g, dom_c))
        print('          平面靶预测均值 %.4f（残差 rms %.4f）——预测失效 ⇒ 板面法向与转轴的关系不成立，'
              '不能把该 3%% 归因于靶面几何'
              % (s_pred.mean(), float(np.sqrt((resid ** 2).mean()))))
        summary['s_' + kind] = {'mean': float(s_obs.mean()), 'median': float(np.median(s_obs)),
                                'p25': float(np.percentile(s_obs, 25)),
                                'p75': float(np.percentile(s_obs, 75)), 'n': int(len(sub)),
                                'dominant': dom_g}

    # 场依赖：按图像位置分组，看尺度是否随视场变化（畸变形状的指纹）
    cx = np.array([np.mean(f['corners'].reshape(-1, 2)[:, 0]) for f in frames])
    cy = np.array([np.mean(f['corners'].reshape(-1, 2)[:, 1]) for f in frames])
    print('    场依赖检查（棋盘中心像素分左右/上下）：')
    for kind in ('pitch', 'yaw'):
        sub = [r for r in rows if r['kind'] == kind]
        line = ['      %-5s' % kind]
        for label, mask in (('左', cx < np.median(cx)), ('右', cx >= np.median(cx)),
                            ('上', cy < np.median(cy)), ('下', cy >= np.median(cy))):
            sel = [r['s'] for r in sub if mask[r['i']] and mask[r['j']]]
            line.append('%s=%.4f(n=%d)' % (label, float(np.mean(sel)) if sel else float('nan'), len(sel)))
        print('  '.join(line))

    # 站点重复性：同一站点内部（不动机器）的尺度理应≈1
    same = [r for r in rows if frames[r['i']]['station'] == frames[r['j']]['station']
            and frames[r['i']]['station'] not in ('', None)]
    if same:
        print('    同站点内部（机器不动，理想尺度 1.000）: n=%d 均值 %.4f 中位 %.4f'
              % (len(same), float(np.mean([r['s'] for r in same])),
                 float(np.median([r['s'] for r in same]))))

    return {'name': name, 'frames': len(frames), 'summary': summary,
            'n_gimbal': n_mean.tolist(), 'normal_dev_mean': float(np.mean(dev)),
            'normal_dev_max': float(np.max(dev))}


def main():
    out = []
    for name in sorted(os.listdir(DATA)):
        folder = os.path.join(DATA, name)
        if os.path.isdir(folder):
            r = analyse(name, folder)
            if r:
                out.append(r)
    with open(os.path.join(DATA, 'axis_geometry_report.json'), 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print('\n报告已写出: %s' % os.path.join(DATA, 'axis_geometry_report.json'))


if __name__ == '__main__':
    main()
