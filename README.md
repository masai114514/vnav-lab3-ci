# vnav-lab3-ci —— 官方 Unity(TESSE) + rviz 链路的 CI 载体

MIT 16.485 VNAV **Lab 3** 的交付物里有一件必须在**官方 Unity 仿真**里跑出来：
`rviz_drone_flight.txt` 要求的「rviz 里飞满一圈的录屏」（D3，50 分的一部分）。

本机（Apple Silicon Mac）做不了 —— 不是没桌面、也不是架构不对（Xvfb 与 x86_64 容器都实测 OK），
而是 **Mono 的 JIT trampoline 在 Rosetta 转译下越出 ±2 GB 约束而断言自杀**：
`* Assertion: should not be reached at tramp-amd64.c:596`。这条在本机结构性无解。

所以把链路搬到 **GitHub Actions 的原生 x86_64 runner**，用 `ros:noetic-ros-base` 容器
拿到原版 focal + ROS Noetic —— 无 Rosetta、无 Noetic deb 缺失问题，也不需要 GPU 云机。
（这套做法先在 lab4 上跑通过，本仓是同一骨架的 lab3 版。）

## 怎么用

```bash
# 1) 改完 lab3/controller_pkg 后，先同步进本仓镜像
bash sync.sh

# 2) 推上去（本机 git push 走代理会挂，所以走 gh 的 Git Data API）
python3 push_via_api.py --message "ci: xxx"

# 3) 触发运行
gh workflow run official-flight.yml --repo masai114514/vnav-lab3-ci

# 4) 等跑完取产物
bash fetch_artifacts.sh              # 最近一次
bash fetch_artifacts.sh <run_id>
```

## 目录

| 路径 | 是什么 |
|---|---|
| `.github/workflows/official-flight.yml` | 工作流：容器 `ros:noetic-ros-base`，参数化（分辨率/帧率/飞行时长/上电键） |
| `ci/official_flight.sh` | ★ 一条龙：装依赖 → catkin 建 `controller_pkg`+`tesse_ros_bridge` → 下 `lab3.zip` → 两块 Xvfb → 按 W 上电 → 起 rviz 与控制器 → 录 rviz + 录 bag → 分析 |
| `lab3/controller_pkg/` | **只读镜像**，由 `sync.sh` 从 `../controller_pkg` 生成，不要手改 |
| `lab3/dependencies/` | `catkin_simple` + `mav_msgs`（controller_pkg 的唯一编译依赖） |
| `lab3/headless/make_rviz_cfg.py` | 由官方 `rviz/lab3.rviz` 生成录屏配置（去掉本 lab 用不到的 Imu/LaserScan，补两条 Path） |
| `lab3/headless/path_logger.py` | 纯观察者节点：`/current_state`→`/drone_path`，`/desired_state`→`/desired_path` |
| `lab3/headless/analyze_flight_bag.py` | bag → `metrics.json`（RMSE/最大误差/转速/偏航速率）+ 图 |
| `sync.sh` / `push_via_api.py` / `fetch_artifacts.sh` | 同步 / 推送 / 取产物 |

## 为什么是「两块虚拟屏」

| 屏 | 分辨率 | 用途 |
|---|---|---|
| `:99` | 320x200 | 跑 Unity。不上镜，**分辨率越低控制环越稳**（lab4 实测：640x400 时控制器 45% 时间拿不到新状态，320x200 降到 1.3%） |
| `:98` | 1024x640 | 跑 rviz，**只录这一块**。交付视频里不会出现 Unity 窗口 |

## lab3 特有的三个坑

1. **`/desired_state` 没有 `time_from_start`**（`traj_publisher.cpp` 没填），所以分析脚本
   用 bag 到达时刻当时间轴，不能用 lab4 那套 `time_from_start` 对齐。
2. **`use_sim_time` 取决于 `/clock` 是否真的在发**。开了 sim time 而 `/clock` 不发，
   `ros::Time::now()` 永远停在 0，控制环一次都不触发（无人机纹丝不动、bag 里没有
   `/rotor_speed_cmds`）。脚本里验一次，收不到就自动关掉并重启桥接。
3. **官方 `rviz/lab3.rviz` 挂了 `rviz_plugin_tutorials/Imu`**（这包没装）和两条 lab4 的
   LaserScan 话题。rviz 能起来，但会刷红字、拖慢 llvmpipe 重绘 —— 生成配置时剔掉。

## 判据（`analyze_flight_bag.py` 的 verdict）

| 判据 | 通过条件 | 为什么重要 |
|---|---|---|
| 稳态位置 RMSE | < 200 mm | 与离线仿真 3.87 mm 对账，数量级差是桥接/50 Hz 物理步长的代价 |
| 偏航速率 | ≈ −0.5 rad/s | **本 lab 唯一离线查不出的风险**：F 矩阵第 4 行 drag 反扭矩符号必须与 TESSE 桨叶旋向一致，否则机体自旋 |
| 四桨稳态转速 | ≈ 49.52 rad/s（悬停解析值） | 差 1/√2 说明 45° 臂角的几何布局搞错了 |
| 最大倾角 | < 45° | 翻覆 = 控制器发散 |

## 产物

artifact 名 `official-flight`：

```
rviz_flight.mp4       ★ rviz 录屏（交付物）
rviz_flight_full.mp4  上采样版
flight.bag            /clock /current_state /desired_state /rotor_speed_cmds
metrics.json          指标 + verdict
flight_tracking.png   实际 vs 期望 + 误差 + 倾角 + 转速
build_evidence.txt    catkin build 真实编译证据（报告里此前标为"未验证"）
bag_info.txt / console.log / unity.log / bridge.log / traj_tracking.log / rviz.log
fps_*.png             Unity 帧时浮层截图（核对真实帧率）
```
