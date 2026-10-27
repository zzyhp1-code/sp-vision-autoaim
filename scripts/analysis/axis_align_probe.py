# -*- coding: utf-8 -*-
"""检验：视觉侧转轴与 IMU 侧转轴之间是否存在一个恒定刚体旋转。

若两侧只差一个坐标系，则存在 R_align 使两批转轴一一对上、残差极小；
残差大则说明“相对转动角之比”这个量把 yaw/pitch 混在了一起，
用它的 s_pitch/s_yaw 当验收判据本身带方法误差（2026-10-10 实测残差 33–47°）。

用法: python scripts/analysis/axis_align_probe.py [数据目录]
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from axis_geometry_probe import DATA, K, D, OBJ, PATTERN, load_dataset, pose_from_yaw_pitch, axis_angle  # noqa: E402


def wahba(a, b, w=None):
    """求 R 使 sum w * angle(R a_i, b_i) 尽量小（用向量对的 SVD）。"""
    H = np.zeros((3, 3))
    for i in range(len(a)):
        wi = 1.0 if w is None else w[i]
        H += wi * np.outer(b[i], a[i])
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    S = np.diag([1, 1, d])
    return U @ S @ Vt


def run(name):
    folder = os.path.join(DATA, name)
    frames = load_dataset(folder)
    if not all(f['yaw'] is not None for f in frames):
        return
    Rg = [pose_from_yaw_pitch(f['yaw'], f['pitch']) for f in frames]
    a_imu, a_vis, pair, pair_ang = [], [], [], []
    for i in range(len(frames)):
        for j in range(i + 1, len(frames)):
            dRg = Rg[j] @ Rg[i].T
            dRc = frames[j]['R'] @ frames[i]['R'].T
            axg, ang_g = axis_angle(dRg)
            axc, ang_c = axis_angle(dRc)
            if ang_g < 3.0:
                continue
            # 统一符号：让 IMU 轴与相机轴的朝向在“同一次转动”里可比
            a_imu.append(axg)
            a_vis.append(axc if np.dot(axc, axg) >= 0 else -axc)
            pair.append((i, j))
            pair_ang.append((ang_g, ang_c))
    a_imu = np.array(a_imu)
    a_vis = np.array(a_vis)
    R = wahba(a_imu, a_vis)
    err = [math.degrees(math.acos(max(-1.0, min(1.0, abs(float(np.dot(R @ a_imu[k], a_vis[k])))))))
           for k in range(len(a_imu))]
    print('=' * 78)
    print('数据集 %s（%d 帧，%d 对）' % (name, len(frames), len(a_imu)))
    print('  转轴对齐残差（R_align 之后）: mean=%.2f° median=%.2f° max=%.2f°'
          % (float(np.mean(err)), float(np.median(err)), float(np.max(err))))
    print('  R_align =')
    for row in R:
        print('    [%+.3f %+.3f %+.3f]' % tuple(row))
    # 把 IMU 轴映射到相机系后，与板面法向的夹角
    norms = np.array([f['normal_cam'] for f in frames])
    print('  映射后 转轴 vs 板面法向(相机系) 夹角:')
    for k in range(len(a_imu)):
        axc = R @ a_imu[k]
        nn = norms[pair[k][0]]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(np.dot(axc, nn)))))))
        if k < 6:
            print('    对 %d: %.1f°' % (k, ang))
    angs = []
    for k in range(len(a_imu)):
        axc = R @ a_imu[k]
        nn = norms[pair[k][0]]
        angs.append(math.degrees(math.acos(max(-1.0, min(1.0, abs(float(np.dot(axc, nn))))))))
    print('  全部对: 夹角中位 %.1f°（对应 cos=%.3f）；观测尺度 %.3f'
          % (float(np.median(angs)), math.cos(math.radians(float(np.median(angs)))),
             float(np.mean([c / g for g, c in pair_ang]))))
    # 板面法向在相机系里的一致性与“面内方向”分布
    n_mean = norms.mean(axis=0)
    n_mean /= np.linalg.norm(n_mean)
    dev = [math.degrees(math.acos(max(-1.0, min(1.0, abs(float(np.dot(v, n_mean))))))) for v in norms]
    print('  板面法向(相机系)一致性: mean=%.2f° max=%.2f°' % (float(np.mean(dev)), float(np.max(dev))))


if __name__ == '__main__':
    for name in sorted(os.listdir(DATA)):
        if os.path.isdir(os.path.join(DATA, name)):
            run(name)
