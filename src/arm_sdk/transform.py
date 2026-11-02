# -*- coding: utf-8 -*-
# =====================================================================
# 【复原实现｜非原始文件】
# 文件名 : transform.py
# 背景   : DICE SDK 的 arm_test.py 只用到一处空间变换接口：
#              from transform import Transform
#              self.T_ws2arm = Transform.inverse(self.T_arm2ws)
#          transform.py 本身未随交付材料提供。
# 本文件 : 按上述调用点复原出一个 4×4 齐次变换小工具：
#              · Transform.inverse(T)   —— 齐次变换求逆（用 R^T / −R^T·t，不用通用矩阵求逆）
#              · Transform.trans / rot_x / rot_y / rot_z / from_xyz_rpy —— 构造常用变换
#          保证 arm_test.py 的 import 与 tf_ws2arm / tf_arm2ws 两条路径可读通。
# 风险   : 原实现的其它接口（若有）未复原；本文件只覆盖有调用证据的部分，
#          属「复原」而非「原始文件」，已在 代码包自检.md 与 README 第 7 节写明。
# =====================================================================
"""4×4 齐次变换工具（最小复原版）。

约定：T_arm2ws 表示「工作台坐标系 → 机械臂坐标系」的变换矩阵（列向量右乘），
     arm_vect = T_arm2ws · ws_vect ，与 arm_test.py 的写法一致。
"""

import numpy as np


class Transform(object):
    """齐次变换矩阵的构造与求逆。"""

    @staticmethod
    def identity():
        return np.eye(4, dtype=float)

    @staticmethod
    def trans(x, y, z):
        """纯平移变换矩阵。"""
        T = np.eye(4, dtype=float)
        T[0, 3], T[1, 3], T[2, 3] = float(x), float(y), float(z)
        return T

    @staticmethod
    def rot_x(theta):
        """绕 x 轴旋转 theta（弧度）。"""
        c, s = np.cos(theta), np.sin(theta)
        T = np.eye(4, dtype=float)
        T[1, 1], T[1, 2] = c, -s
        T[2, 1], T[2, 2] = s, c
        return T

    @staticmethod
    def rot_y(theta):
        """绕 y 轴旋转 theta（弧度）。"""
        c, s = np.cos(theta), np.sin(theta)
        T = np.eye(4, dtype=float)
        T[0, 0], T[0, 2] = c, s
        T[2, 0], T[2, 2] = -s, c
        return T

    @staticmethod
    def rot_z(theta):
        """绕 z 轴旋转 theta（弧度）。"""
        c, s = np.cos(theta), np.sin(theta)
        T = np.eye(4, dtype=float)
        T[0, 0], T[0, 1] = c, -s
        T[1, 0], T[1, 1] = s, c
        return T

    @staticmethod
    def from_xyz_rpy(xyz, rpy):
        """URDF 语义的 origin：先按固定轴 x→y→z 转 rpy，再平移 xyz。"""
        T = Transform.trans(*xyz)
        roll, pitch, yaw = rpy
        return T.dot(Transform.rot_z(yaw)).dot(Transform.rot_y(pitch)).dot(Transform.rot_x(roll))

    @staticmethod
    def inverse(T):
        """齐次变换求逆：[[R, t],[0,1]]⁻¹ = [[Rᵀ, −Rᵀt],[0,1]]。

        比 np.linalg.inv 快且数值更稳，是 arm_test.py 里唯一的调用点。
        """
        T = np.asarray(T, dtype=float)
        if T.shape != (4, 4):
            raise ValueError("Transform.inverse 需要 4x4 矩阵，收到 %s" % (T.shape,))
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        Ti = np.eye(4, dtype=float)
        Ti[0:3, 0:3] = R.T
        Ti[0:3, 3] = -R.T.dot(t)
        return Ti


if __name__ == "__main__":
    # 自检：构造一个位姿并验证 T · T⁻¹ = I（误差应到 1e-15 量级）
    T = Transform.from_xyz_rpy((0.1, -0.2, 0.05), (0.3, -1.5708, 0.2))
    err = np.abs(T.dot(Transform.inverse(T)) - np.eye(4)).max()
    print("T · T⁻¹ 与单位阵的最大偏差 = %.3e（应 < 1e-12）" % err)
