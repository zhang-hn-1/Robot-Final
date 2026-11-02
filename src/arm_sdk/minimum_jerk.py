# -*- coding: utf-8 -*-
# =====================================================================
# 【复原实现｜非原始文件，未经原机回归】
# 文件名 : minimum_jerk.py
# 背景   : DICE SDK 的 arm5dof_uservo.py 里有
#              from minimum_jerk import minimum_jerk_plan, minimum_jerk_seq
#          但 minimum_jerk.py 本身未随交付材料提供（4 个实验目录里的
#          arm5dof_uservo.py MD5 完全相同，均只 import 不附带实现）。
# 本文件 : 按课程报告给出的 Minimum Jerk（最小加加速度）公式重写的兼容实现，
#          用于补全依赖、便于阅读「六段式轨迹」到底怎么算出来。公式为
#              s(τ) = 10τ³ − 15τ⁴ + 6τ⁵,  τ = t/T ∈ [0,1]
#              q(t) = q0 + (q1 − q0)·s(τ)
#              q̇(t) = (q1 − q0)/T · (30τ² − 60τ³ + 30τ⁴)
#          （Flash & Hogan, 1985；报告第 4 章轨迹规划一节引用）
# 风险   : 原 SDK 的 minimum_jerk_plan / minimum_jerk_seq 真实签名未提供，
#          本文件按「起点、终点、时长、采样周期」这四个必需要素推断签名。
#          因此本文件只能保证公式正确，不能保证与原 SDK 逐位一致；
#          若函数签名不同，import 会在调用处报 TypeError——这是「复原」而非
#          「原始文件」的固有局限，已在 代码包自检.md 与 README 第 7 节写明。
# =====================================================================
"""最小加加速度（Minimum Jerk）轨迹规划的最小实现。

对外提供两个函数，函数名与 SDK 的 import 语句一致：

    minimum_jerk_plan(q0, q1, T, delta_t) -> (t_list, q_list, dq_list)
    minimum_jerk_seq(q0, q1, T, delta_t)  -> q_list

q0 / q1 可以是 float，也可以是等长 list / numpy 数组（多关节同时规划）。
"""

import numpy as np

# 采样周期：与 config.TRAJECTORY_DELTA_T = 0.012 s 保持一致
TRAJECTORY_DELTA_T = 0.012


def _s(tau):
    """归一化位置曲线 s(τ) = 10τ³ − 15τ⁴ + 6τ⁵"""
    return 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5


def _ds(tau):
    """归一化速度曲线 ds/dτ = 30τ² − 60τ³ + 30τ⁴"""
    return 30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4


def minimum_jerk_plan(q0, q1, T, delta_t=TRAJECTORY_DELTA_T):
    """把 q0→q1 在 T 秒内按最小加加速度规律插补。

    参数:
        q0, q1   : 起点 / 终点（float 或 1 维序列）
        T        : 运动时长（s），必须 > 0
        delta_t  : 采样周期（s），默认 0.012
    返回:
        (t_list, q_list, dq_list)
        t_list  : list[float]，时间戳，末点严格等于 T
        q_list  : list[ndarray|float]，每个采样点的位置
        dq_list : list[ndarray|float]，每个采样点的速度
    """
    if T <= 0:
        raise ValueError("运动时长 T 必须大于 0，当前 T=%r" % (T,))
    if delta_t <= 0:
        raise ValueError("采样周期 delta_t 必须大于 0，当前 delta_t=%r" % (delta_t,))

    scalar = np.isscalar(q0) and np.isscalar(q1)
    a = np.asarray(q0, dtype=float)
    b = np.asarray(q1, dtype=float)
    if a.shape != b.shape:
        raise ValueError("q0 与 q1 形状不一致: %s vs %s" % (a.shape, b.shape))
    diff = b - a

    n = max(int(np.ceil(T / delta_t)), 1)
    t_list, q_list, dq_list = [], [], []
    for i in range(n + 1):
        t = min(i * delta_t, T)
        tau = t / T
        pos = a + diff * _s(tau)
        vel = diff * _ds(tau) / T
        t_list.append(t)
        q_list.append(float(pos) if scalar else pos.copy())
        dq_list.append(float(vel) if scalar else vel.copy())
        if t >= T:
            break
    return t_list, q_list, dq_list


def minimum_jerk_seq(q0, q1, T, delta_t=TRAJECTORY_DELTA_T):
    """只要位置序列的便捷封装（SDK 里用来逐点下发关节角）。"""
    return minimum_jerk_plan(q0, q1, T, delta_t)[1]


if __name__ == "__main__":
    # 自检：单关节 0 → 1 rad，T=1.0 s，末端位置必须精确到 1，速度必须回到 0
    ts, qs, vs = minimum_jerk_plan(0.0, 1.0, 1.0)
    print("采样点数: %d（T=1.0s, Δt=%.3fs）" % (len(ts), TRAJECTORY_DELTA_T))
    print("末点位置 = %.12f（应为 1.0）" % qs[-1])
    print("末点速度 = %.12f（应为 0.0）" % vs[-1])
    print("峰值速度 = %.6f（理论 1.875 rad/s）" % max(abs(v) for v in vs))
    print("峰值加速度 ≈ %.6f（理论 5.7735 rad/s²）"
          % (max(abs(vs[i + 1] - vs[i]) / (ts[i + 1] - ts[i]) for i in range(len(vs) - 1))))
