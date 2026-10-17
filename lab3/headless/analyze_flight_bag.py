#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_flight_bag.py —— 官方 Unity(TESSE) 实跑 bag 的指标提取。

输入：`rosbag record /clock /current_state /desired_state /rotor_speed_cmds` 录下的 flight.bag
输出：metrics.json + flight_tracking.png

为什么单独写一份（不复用 lab4 的）
----------------------------------
lab3 的 `/desired_state` 是 `MultiDOFJointTrajectoryPoint` 且 **`time_from_start` 恒为 0**
（`traj_publisher.cpp` 没填），所以**不能**像 lab4 那样用 time_from_start 建轨迹时间轴。
这里直接用 bag 的到达时刻（`stamp_ns`，use_sim_time 下就是仿真时间）做时间轴，
与 `/current_state.header.stamp`（同样是仿真时间）天然同轴。

三个关键判据
------------
1. **位置跟踪**：稳态一圈内的 RMSE / 最大误差 —— 与离线仿真（3.87 mm）对账。
   真仿真里有 50 Hz 物理步长 + UDP 桥接 + 一阶电机滞后，数量级上会比离线差，属正常。
2. **偏航速率**：期望 ψ̇ = -1/timeScale = -0.5 rad/s。实测若明显偏离或发散，
   说明分配矩阵 F 的第 4 行（drag 反扭矩符号）与 TESSE 的桨叶旋向不一致 ——
   这是本 lab 唯一"离线自查查不出来"的风险点。
3. **转速量级**：悬停解析值 sqrt(m·g/(4·c_f)) = 49.52 rad/s。四桨稳态应都在它附近；
   若出现长时间负值或量级差一个 1/sqrt(2)，说明 F 的几何布局（45° 臂角）搞错了。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 与 controller_node.cpp 一致的物理常数
# ---------------------------------------------------------------------------
MASS, G, CF = 1.0, 9.81, 1e-3
W_HOVER = float(np.sqrt(MASS * G / (4 * CF)))     # 49.52 rad/s
TS = 2.0                                          # traj_publisher: timeScale
YAW_RATE_D = -1.0 / TS                            # -0.5 rad/s
PERIOD = 2 * np.pi * TS                           # 12.566 s

ACTUATORS_MSG = """std_msgs/Header header
float64[] angles
float64[] angular_velocities
float64[] normalized
"""


# ---------------------------------------------------------------------------
def read_bag(path):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    ts = get_typestore(Stores.ROS1_NOETIC)
    ts.register(get_types_from_msg(ACTUATORS_MSG, "mav_msgs/msg/Actuators"))

    def _sec(s):
        if hasattr(s, "to_sec"):
            return float(s.to_sec())
        return float(s.sec) + float(s.nanosec) * 1e-9

    des_t, des_p = [], []
    cur_t, cur_arr, cur_p, cur_q, cur_v = [], [], [], [], []
    rot_t, rot_w = [], []

    with AnyReader([Path(path)], default_typestore=ts) as reader:
        want = {"/desired_state", "/current_state", "/rotor_speed_cmds"}
        conns = [c for c in reader.connections if c.topic in want]
        topics = sorted({c.topic for c in reader.connections})
        print("[analyze] bag 内话题：%s" % ", ".join(topics))

        for conn, stamp_ns, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            now = stamp_ns * 1e-9
            if conn.topic == "/desired_state":
                if not m.transforms:
                    continue
                tr = m.transforms[0].translation
                des_t.append(now)
                des_p.append((tr.x, tr.y, tr.z))
            elif conn.topic == "/current_state":
                p = m.pose.pose.position
                q = m.pose.pose.orientation
                v = m.twist.twist.linear
                cur_t.append(_sec(m.header.stamp))
                cur_arr.append(now)
                cur_p.append((p.x, p.y, p.z))
                cur_q.append((q.x, q.y, q.z, q.w))
                cur_v.append((v.x, v.y, v.z))
            else:  # /rotor_speed_cmds
                if len(m.angular_velocities) < 4:
                    continue
                rot_t.append(now)
                rot_w.append(tuple(m.angular_velocities[:4]))

    out = dict(
        des_t=np.asarray(des_t, float),
        des_p=np.asarray(des_p, float).reshape(-1, 3),
        cur_t=np.asarray(cur_t, float),
        cur_arr=np.asarray(cur_arr, float),
        cur_p=np.asarray(cur_p, float).reshape(-1, 3),
        cur_q=np.asarray(cur_q, float).reshape(-1, 4),
        cur_v=np.asarray(cur_v, float).reshape(-1, 3),
        rot_t=np.asarray(rot_t, float),
        rot_w=np.asarray(rot_w, float).reshape(-1, 4),
    )
    for k in ("des_t", "cur_t", "rot_t"):
        o = np.argsort(out[k])
        base = k[:-2]
        out[k] = out[k][o]
        for kk in ("des_p", "cur_p", "cur_q", "cur_v", "rot_w"):
            if kk.startswith(base):
                out[kk] = out[kk][o]
    # 有些桥接版本不给 header.stamp 打时间（全 0）—— 那就退回 bag 到达时刻
    if len(out["cur_t"]) > 1 and float(np.ptp(out["cur_t"])) < 1e-3:
        print("[analyze] /current_state 的 header.stamp 没有区分度 —— 改用 bag 到达时刻")
        out["cur_t"] = out["cur_arr"]
    return out


# ---------------------------------------------------------------------------
def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(q):
    R = quat_to_R(q)
    return np.arctan2(R[1, 0], R[0, 0])


def tilt_of(q):
    """机体 z 轴与世界上方向夹角 [deg]。"""
    R = quat_to_R(q)
    c = float(np.clip(R[2, 2], -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default="metrics.json")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    d = read_bag(args.bag)
    if len(d["cur_t"]) < 10 or len(d["des_p"]) < 10:
        print("[analyze] 数据太少：current=%d desired=%d"
              % (len(d["cur_t"]), len(d["des_p"])), file=sys.stderr)
        return 2

    # ---- 统一的相对时间轴（第一帧 /current_state 为 0） --------------------
    t0 = d["cur_t"][0]
    tc = d["cur_t"] - t0
    td = d["des_t"] - t0

    # 期望位置插值到 current 的采样时刻
    ok = td >= -1e9
    des_at_cur = np.column_stack([
        np.interp(tc, td[ok], d["des_p"][ok, i]) for i in range(3)
    ])
    err = np.linalg.norm(d["cur_p"] - des_at_cur, axis=1)

    t_end = float(tc[-1])
    # 稳态窗口：最后完整一圈
    lap = tc >= max(0.0, t_end - PERIOD)
    # 收敛：最后一次误差 > 0.10 m 的时刻之后
    bad = np.where(err > 0.10)[0]
    settle = float(tc[bad[-1]]) if len(bad) else 0.0

    tilts = np.array([tilt_of(q) for q in d["cur_q"]])
    yaws = np.unwrap(np.array([yaw_of(q) for q in d["cur_q"]]))

    # 稳态偏航速率（最小二乘斜率）
    if lap.sum() > 5:
        A = np.vstack([tc[lap], np.ones(lap.sum())]).T
        slope = float(np.linalg.lstsq(A, yaws[lap], rcond=None)[0][0])
    else:
        slope = float("nan")

    # 转速
    if len(d["rot_t"]):
        rw = d["rot_w"]
        rl = (d["rot_t"] - t0) >= max(0.0, t_end - PERIOD)
        rw_lap = rw[rl] if rl.sum() > 3 else rw
    else:
        rw_lap = np.zeros((1, 4))
    w_mean = rw_lap.mean(axis=0)
    w_max = np.abs(rw_lap).max()
    n_neg = int((rw_lap < -1.0).sum())

    metrics = {
        "source": "official Unity/TESSE (lab3.x86_64) via tesse_ros_bridge",
        "n_current": int(len(d["cur_t"])),
        "n_desired": int(len(d["des_p"])),
        "n_rotor_cmd": int(len(d["rot_t"])),
        "duration_s": round(t_end, 2),
        "period_s": round(PERIOD, 2),
        "pos_rmse_full_mm": round(float(np.sqrt(np.mean(err ** 2)) * 1e3), 2),
        "pos_max_full_mm": round(float(err.max() * 1e3), 2),
        "pos_rmse_last_lap_mm": round(float(np.sqrt(np.mean(err[lap] ** 2)) * 1e3), 2) if lap.sum() else None,
        "pos_max_last_lap_mm": round(float(err[lap].max() * 1e3), 2) if lap.sum() else None,
        "settle_to_100mm_s": round(settle, 2),
        "tilt_max_deg": round(float(tilts.max()), 2),
        "tilt_mean_last_lap_deg": round(float(tilts[lap].mean()), 2) if lap.sum() else None,
        "yaw_rate_measured_rad_s": round(slope, 4),
        "yaw_rate_desired_rad_s": YAW_RATE_D,
        "yaw_rate_err_pct": round(abs(slope - YAW_RATE_D) / abs(YAW_RATE_D) * 100, 1) if slope == slope else None,
        "rotor_hover_theory_rad_s": round(W_HOVER, 2),
        "rotor_mean_last_lap_rad_s": [round(float(v), 2) for v in w_mean],
        "rotor_abs_max_rad_s": round(float(w_max), 2),
        "rotor_negative_samples": n_neg,
        "offline_reference": {
            "pos_rmse_steady_mm": 3.87,
            "note": "sim/out/metrics.json（等价离线模型，无桥接/无 50 Hz 物理步长）",
        },
    }

    # ---- 判决 ------------------------------------------------------------
    verdict = []
    if metrics["pos_rmse_last_lap_mm"] is not None:
        if metrics["pos_rmse_last_lap_mm"] < 200:
            verdict.append("PASS 稳态跟踪：最后一圈 RMSE %.1f mm（<200 mm）"
                           % metrics["pos_rmse_last_lap_mm"])
        else:
            verdict.append("WARN 稳态跟踪偏差偏大：%.1f mm"
                           % metrics["pos_rmse_last_lap_mm"])
    if slope == slope and abs(slope - YAW_RATE_D) < 0.15:
        verdict.append("PASS 偏航速率 %.3f rad/s ≈ 期望 %.2f（F 矩阵 drag 符号与 TESSE 一致）"
                       % (slope, YAW_RATE_D))
    else:
        verdict.append("WARN 偏航速率实测 %s rad/s，期望 %.2f —— 若远大于期望，"
                       "优先怀疑 F 第 4 行符号/桨叶旋向" % (slope, YAW_RATE_D))
    if n_neg == 0 and abs(w_mean.mean() - W_HOVER) < 15:
        verdict.append("PASS 四桨稳态转速均值 %.1f rad/s ≈ 悬停解析值 %.1f"
                       % (w_mean.mean(), W_HOVER))
    else:
        verdict.append("WARN 转速异常：均值 %.1f rad/s（解析 %.1f），负值样本 %d"
                       % (w_mean.mean(), W_HOVER, n_neg))
    if tilts.max() < 45:
        verdict.append("PASS 最大倾角 %.1f°（未翻覆）" % tilts.max())
    else:
        verdict.append("FAIL 出现大倾角 %.1f°（接近翻覆）" % tilts.max())
    metrics["verdict"] = verdict

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("\n--- 判决 ---")
    for v in verdict:
        print("  " + v)

    # ---- 图 --------------------------------------------------------------
    if args.plot:
        fig = plt.figure(figsize=(12.6, 7.2))
        gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.24)

        ax = fig.add_subplot(gs[0, 0])
        ax.plot(d["des_p"][:, 0], d["des_p"][:, 1], "--", color="#1f77b4", lw=1.6, label="desired")
        ax.plot(d["cur_p"][:, 0], d["cur_p"][:, 1], "-", color="#d62728", lw=1.4, label="actual")
        ax.scatter(d["cur_p"][0, 0], d["cur_p"][0, 1], marker="X", s=70, c="k", zorder=5, label="takeoff")
        ax.set_aspect("equal"); ax.grid(alpha=.3)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title("(a) xy 俯视：实际 vs 期望（官方 Unity 实跑）", fontsize=10)
        ax.legend(fontsize=8)

        ax = fig.add_subplot(gs[0, 1])
        ax.semilogy(tc, err * 1e3, color="#d62728", lw=1.2)
        ax.axhline(100, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("|e_p| [mm]")
        ax.grid(alpha=.3)
        ax.set_title("(b) 位置误差（对数）", fontsize=10)

        ax = fig.add_subplot(gs[1, 0])
        ax.plot(tc, tilts, color="#2ca02c", lw=1.1)
        ax.axhline(45, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("tilt [deg]")
        ax.grid(alpha=.3)
        ax.set_title("(c) 倾角 |angle(R e3, e3)|", fontsize=10)

        ax = fig.add_subplot(gs[1, 1])
        if len(d["rot_t"]):
            rt = d["rot_t"] - t0
            for i, c in enumerate(["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]):
                ax.plot(rt, d["rot_w"][:, i], color=c, lw=0.9, label=r"$\omega_%d$" % (i + 1))
            ax.axhline(W_HOVER, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("rotor [rad/s]")
        ax.grid(alpha=.3)
        ax.set_title("(d) 四桨转速（虚线=悬停解析值 %.1f）" % W_HOVER, fontsize=10)
        ax.legend(fontsize=7, ncol=4)

        fig.suptitle("LAB3 官方 Unity(TESSE) 实跑：圆周轨迹跟踪", fontsize=12.5, y=0.99)
        fig.savefig(args.plot, dpi=150, bbox_inches="tight")
        print("[analyze] 图：%s" % args.plot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
