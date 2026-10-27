// 已废弃的占位文件，请勿在本文件里加内容。
//
// 原因：本项目**已有**云台跟随实现，不需要独立的跟随工具：
//   - configs/standard.yaml: xuc_direct_yaw_guard / xuc_direct_pitch_guard（均已启用）
//   - src/standard.cpp: pitch 为像素闭环（measured_pitch = xuc.imu_pitch() + gain*像素误差）；
//     yaw 采用**世界坐标系**（atan2(xyz_in_world)），按用户 2026-10-12 指示保持不变。
//
// 保留本文件是因为它在当前 Windows 工作区里被权限占用、无法删除；下次在 Linux 侧清理即可。
