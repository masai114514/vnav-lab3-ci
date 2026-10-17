#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_rviz_cfg.py —— 由官方 `controller_pkg/rviz/lab3.rviz` 生成本次录屏用的配置。

为什么要改
----------
官方配置里挂了三个**本 lab 用不到**的显示项：
    rviz_plugin_tutorials/Imu  ×2   ← 这个插件包根本没装，rviz 起来就报
                                      "The class required for this display ... could not be loaded"
    rviz/LaserScan  ×2              ← 话题 /tesse/front_lidar/scan 在 lab3 场景里不存在
留着它们不影响运行，但会让 rviz 启动时刷一堆红字，也占重绘时间（llvmpipe 上很贵）。

另外官方配置只显示 **Odometry 位姿**，没有轨迹留痕 —— 录出来的视频看不出一圈圆。
所以这里补两条 `rviz/Path`：
    /desired_path  期望圆周（蓝）
    /drone_path    实际轨迹（红）
两个话题由 `headless/path_logger.py` 发布。

除上述增删外，**其余键一律沿用官方配置**（Fixed Frame=world、Grid、TF、Odometry 的
参数都不动），这样视频里的画面就是官方 rviz 的画面，只是多了两条线。
"""

from __future__ import annotations

import argparse
import sys

import yaml

KEEP_CLASSES = {"rviz/Grid", "rviz/TF", "rviz/Odometry", "rviz/Axes"}


def path_display(name: str, topic: str, color: str, width: float) -> dict:
    return {
        "Class": "rviz/Path",
        "Enabled": True,
        "Name": name,
        "Topic": topic,
        "Color": color,
        "Alpha": 1.0,
        "Line Style": "Lines",
        "Line Width": width,
        "Offset": {"X": 0, "Y": 0, "Z": 0},
        "Pose Style": "None",
        "Pose Color": "255; 255; 255",
        "Value": True,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="官方 lab3.rviz")
    ap.add_argument("--out", required=True, help="输出路径")
    ap.add_argument("--fps", type=int, default=20, help="rviz 重绘帧率")
    ap.add_argument("--size", default=None,
                    help="目标窗口尺寸 WxH；不传则沿用官方存档几何")
    args = ap.parse_args()

    with open(args.src, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    vm = cfg["Visualization Manager"]
    old = vm.get("Displays") or []

    kept, dropped = [], []
    for d in old:
        cls = d.get("Class", "")
        if cls in KEEP_CLASSES:
            kept.append(d)
        else:
            dropped.append(cls)

    # Odometry 的 Keep 放开一点：即便没有 Path 也能看出一点轨迹残留
    for d in kept:
        if d.get("Class") == "rviz/Odometry":
            d["Keep"] = 2000
            d["Shape"]["Value"] = "Axes"
            d["Shape"]["Axes Length"] = 0.8
            d["Shape"]["Axes Radius"] = 0.08

    # 期望轨迹画在下面、实际轨迹压在上面（红压蓝，和报告里"红=实际"一致）
    kept.append(path_display("期望轨迹 (desired)", "/desired_path", "31; 119; 180", 0.04))
    kept.append(path_display("实际轨迹 (drone)",    "/drone_path",    "214; 39; 40", 0.05))
    vm["Displays"] = kept

    go = vm.setdefault("Global Options", {})
    go["Fixed Frame"] = "world"
    go["Frame Rate"] = args.fps

    view = vm.setdefault("Views", {}).setdefault("Current", {})
    view["Class"] = "rviz/Orbit"
    view["Distance"] = 24.0                 # 半径 5 m 的圆，留边
    view["Focal Point"] = {"X": 0.0, "Y": 0.0, "Z": 2.0}
    view["Pitch"] = 0.52
    view["Yaw"] = 0.9
    view["Target Frame"] = "<Fixed Frame>"  # 官方里写的 base_link_gt 在 lab3 不一定存在
    view["Near Clip Distance"] = 0.01

    # ---- 窗口几何 --------------------------------------------------------
    # 官方存档是 1920x1052 @ (339,181)，放进 1024x640 的 Xvfb 里会被裁掉一大半
    # （第一趟就是因此只录到一个角落 + 一个弹窗）。所以把尺寸改成虚拟屏大小。
    #
    # ⚠️ 但 `QMainWindow State` **不能删**：那一串是 QMainWindow::saveState() 的
    #    QByteArray，里面记着各 dock 的位置/大小。删掉后 Qt 会用默认布局，实测
    #    Displays/Selection/Tool Properties/Views 四块把整个窗口铺满，**3D 视口
    #    一条缝都不剩**（第二趟就是这么录的）。保留它、只改外框尺寸，Qt 会把
    #    dock 宽度夹到新窗口里，视口就能正常露出来。
    wg = cfg.setdefault("Window Geometry", {})
    wg["Hide Left Dock"] = False
    wg["Hide Right Dock"] = False
    if args.size:
        w, h = args.size.lower().split("x")
        wg["Width"], wg["Height"] = int(w), int(h)
        wg["X"], wg["Y"] = 0, 0
        print("[rviz-cfg] 窗口几何 %sx%s @(0,0)（保留 QMainWindow State）" % (w, h))

    # 左侧 Displays 树别占太高的竖向空间（官方存的是 755 px，1024x640 下太挤）
    for panel in (cfg.get("Panels") or []):
        if panel.get("Class") == "rviz/Displays" and "Tree Height" in panel:
            panel["Tree Height"] = 360

    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    print("[rviz-cfg] 保留显示 %d 项：%s"
          % (len(kept), ", ".join(d.get("Name", "?") for d in kept)))
    print("[rviz-cfg] 去掉显示 %d 项：%s" % (len(dropped), ", ".join(dropped) or "(无)"))
    print("[rviz-cfg] Fixed Frame=world，帧率 %d Hz，距离 %.0f m" % (args.fps, view["Distance"]))
    print("[rviz-cfg] 写出 %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
