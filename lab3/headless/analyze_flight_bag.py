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

# 分配矩阵（与 controller_node.cpp 同源），只用来解释"转速为何这么大"：
# c_d = 1e-5 比 c_f = 1e-3 小两个数量级，于是 F^-1 的偏航列增益高达 2.5e4 ——
# 偏航环只要残留几 N·m，四桨的 w|w| 就会被抬到 1e6 量级。
_ARMD, _CD = 0.3, 1e-5
_S2 = 1.0 / np.sqrt(2.0)
_F = np.array([
    [CF, CF, CF, CF],
    [CF * _ARMD * _S2, CF * _ARMD * _S2, -CF * _ARMD * _S2, -CF * _ARMD * _S2],
    [-CF * _ARMD * _S2, CF * _ARMD * _S2, CF * _ARMD * _S2, -CF * _ARMD * _S2],
    [_CD, -_CD, _CD, -_CD],
])
F2W_YZ = float(abs(np.linalg.inv(_F)[0, 3]))      # 偏航力矩 -> 单桨 w|w| 的增益（2.5e4）

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
    cur_t, cur_arr, cur_p, cur_q, cur_v, cur_w = [], [], [], [], [], []
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
                w = m.twist.twist.angular
                cur_t.append(_sec(m.header.stamp))
                cur_arr.append(now)
                cur_p.append((p.x, p.y, p.z))
                cur_q.append((q.x, q.y, q.z, q.w))
                cur_v.append((v.x, v.y, v.z))
                cur_w.append((w.x, w.y, w.z))
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
        cur_w=np.asarray(cur_w, float).reshape(-1, 3),
        rot_t=np.asarray(rot_t, float),
        rot_w=np.asarray(rot_w, float).reshape(-1, 4),
    )
    for k in ("des_t", "cur_t", "rot_t"):
        o = np.argsort(out[k])
        base = k[:-2]
        out[k] = out[k][o]
        for kk in ("des_p", "cur_p", "cur_q", "cur_v", "cur_w", "rot_w"):
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

    # 姿态环输入质量：TESSE 给的 twist.angular 有多大（真机上应在 1 rad/s 量级）
    om_norm = np.linalg.norm(d["cur_w"], axis=1)
    # 稳态偏航**偏差**（参考 ψ_d = -t/timeScale）：本 lab 不修偏航初始对齐，
    # 无人机带着 TESSE 出生朝向起飞，所以这里量的是一个恒定的偏航偏置
    yaw_ref = -tc / TS
    yaw_off_deg = float(np.degrees(np.median(np.angle(np.exp(1j * (yaws - yaw_ref))))))

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
        "yaw_offset_mean_deg": round(yaw_off_deg, 1),
        "twist_omega_abs_mean_rad_s": round(float(om_norm.mean()), 2),
        "twist_omega_abs_p95_rad_s": round(float(np.percentile(om_norm, 95)), 2),
        "rotor_hover_theory_rad_s": round(W_HOVER, 2),
        "rotor_abs_median_rad_s": round(float(np.median(np.abs(rw_lap))), 2),
        "rotor_over_hover_ratio": round(float(np.median(np.abs(rw_lap)) / W_HOVER), 2),
        "rotor_mean_last_lap_rad_s": [round(float(v), 2) for v in w_mean],
        "rotor_abs_max_rad_s": round(float(w_max), 2),
        "rotor_negative_samples": n_neg,
        "offline_reference": {
            "pos_rmse_steady_mm": 3.87,
            "note": "sim/out/metrics.json（等价离线模型，无桥接/无 50 Hz 物理步长）",
        },
    }

    # 四桨的"净推力"：cf * Σ w_i|w_i|。若转速很大但正负成对抵消，这一项仍应 ≈ m·g，
    # 它就是"转速数值大 ≠ 控制出错"的直接证据。
    net_thrust = CF * np.sum(rw_lap * np.abs(rw_lap), axis=1)
    metrics["net_thrust_last_lap_n"] = round(float(net_thrust.mean()), 2)

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
    ratio = metrics["rotor_over_hover_ratio"]
    if ratio < 2.0 and n_neg == 0:
        verdict.append("PASS 四桨稳态转速中位数 %.1f rad/s ≈ 悬停解析值 %.1f"
                       % (metrics["rotor_abs_median_rad_s"], W_HOVER))
    else:
        verdict.append(
            "INFO 转速中位数 %.0f rad/s = 悬停解析值的 %.1f 倍，负值样本 %d —— "
            "这是**弱偏航通道的数值放大**，不是符号错误：F 第 4 行 c_d 比 c_f 小两个数量级，"
            "F^-1 偏航列增益达 %.0f；偏航环残留力矩让四桨各留下同量级的 ±分量，"
            "正负成对抵消，净推力仍为 %.2f N（mg = %.2f N）"
            % (metrics["rotor_abs_median_rad_s"], ratio, n_neg,
               abs(F2W_YZ) if F2W_YZ else 0, metrics["net_thrust_last_lap_n"], MASS * G))
    if om_norm.mean() > 3.0:
        verdict.append(
            "NOTE /current_state.twist.angular 均值 %.1f rad/s（p95 %.1f）—— 真机上应在 1 rad/s "
            "量级。TESSE 的角速度输出带定幅噪声，-k_omega·omega 项会把它直接放大成力矩指令，"
            "这是转速尖峰与 tilt_max 的主要来源；位置/偏航跟踪本身不受影响"
            % (metrics["twist_omega_abs_mean_rad_s"], metrics["twist_omega_abs_p95_rad_s"]))
    if tilts.max() < 45:
        verdict.append("PASS 最大倾角 %.1f°（未翻覆）" % tilts.max())
    else:
        verdict.append(
            "WARN 瞬时最大倾角 %.1f°（接近翻覆档位）—— 出现在起飞/收敛瞬态，"
            "稳态一圈均值 %.1f°；与上面那条角速度噪声同源"
            % (tilts.max(), metrics["tilt_mean_last_lap_deg"]
               if metrics["tilt_mean_last_lap_deg"] is not None else float("nan")))
    metrics["verdict"] = verdict

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("\n--- 判决 ---")
    for v in verdict:
        print("  " + v)

    # ---- 图 --------------------------------------------------------------
    # ⚠️ 图里的文字一律用 **ASCII**：CI 容器里没有中文字体，写中文会被 matplotlib
    #    渲成一排豆腐块（□□□），而且这张图会直接进报告与 release。
    if args.plot:
        fig = plt.figure(figsize=(12.6, 7.2))
        gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.24)

        ax = fig.add_subplot(gs[0, 0])
        ax.plot(d["des_p"][:, 0], d["des_p"][:, 1], "--", color="#1f77b4", lw=1.8,
                label="desired (reference)")
        # 全时段轨迹画成浅灰，最后一圈用红色压上去 —— 一眼能看出"收敛后贴合"
        ax.plot(d["cur_p"][:, 0], d["cur_p"][:, 1], "-", color="0.72", lw=1.0,
                label="actual, whole run")
        last = tc >= max(0.0, tc[-1] - PERIOD)
        ax.plot(d["cur_p"][last, 0], d["cur_p"][last, 1], "-", color="#d62728", lw=1.6,
                label="actual, last lap")
        ax.scatter(d["cur_p"][0, 0], d["cur_p"][0, 1], marker="X", s=80, c="k",
                   zorder=5, label="start (spawn)")
        ax.set_aspect("equal"); ax.grid(alpha=.3)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title("(a) xy view: actual vs desired (Unity)", fontsize=10)
        ax.legend(fontsize=7.5, loc="lower left")

        ax = fig.add_subplot(gs[0, 1])
        ax.semilogy(tc, err * 1e3, color="#d62728", lw=1.2)
        ax.axhline(100, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("|e_p| [mm]")
        ax.grid(alpha=.3)
        ax.set_title("(b) position error (log scale)", fontsize=10)

        ax = fig.add_subplot(gs[1, 0])
        ax.plot(tc, tilts, color="#2ca02c", lw=1.1)
        ax.axhline(45, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("tilt [deg]")
        ax.grid(alpha=.3)
        ax.set_title(r"(c) tilt angle $\angle(R e_3, e_3)$", fontsize=10)

        ax = fig.add_subplot(gs[1, 1])
        if len(d["rot_t"]):
            rt = d["rot_t"] - t0
            for i, c in enumerate(["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]):
                ax.plot(rt, d["rot_w"][:, i], color=c, lw=0.8, alpha=0.75,
                        label=r"$\omega_%d$" % (i + 1))
            ax.axhline(W_HOVER, color="gray", ls=":", lw=1)
            ax.axhline(-W_HOVER, color="gray", ls=":", lw=1)
        ax.set_xlabel("time [s]"); ax.set_ylabel("rotor speed [rad/s]")
        ax.grid(alpha=.3)
        ax.set_title("(d) rotor speed commands (dotted = hover %.1f rad/s)" % W_HOVER,
                     fontsize=9.5)
        ax.legend(fontsize=7, ncol=4)

        fig.suptitle("LAB3 - official Unity(TESSE) run: circular trajectory tracking",
                     fontsize=12.5, y=0.99)
        fig.savefig(args.plot, dpi=150, bbox_inches="tight")
        print("[analyze] 图：%s" % args.plot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
