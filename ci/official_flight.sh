#!/usr/bin/env bash
# =============================================================================
# official_flight.sh —— 在**原生 x86_64 Linux** 上跑通官方 Unity(TESSE) + rviz 的
#                       Lab 3「几何控制器 · 圆周轨迹跟踪」链路
#
# 产出（$OUT/）：
#   rviz_flight.mp4      ★ 交付物：rviz 里飞满一圈的录屏（:98 虚拟屏）
#   flight.bag           ★ 证据：/clock /current_state /desired_state /rotor_speed_cmds
#   metrics.json         位置 RMSE / 最大误差 / 转速 / 偏航速率（与离线仿真对账）
#   flight_tracking.png  实际 vs 期望（xy 俯视 + 误差曲线）
#   unity.log / bridge.log / traj_tracking.log / rviz.log / rosbag.log
#   fps_*.png            Unity 帧时浮层截图（核对真实帧率）
#
# 用法（容器内，root）：
#   OUT=$PWD/out BUNDLE=$PWD/lab3 bash ci/official_flight.sh
#
# -----------------------------------------------------------------------------
# 从 lab4 那条链路继承下来的四个坑（改之前先读）
# -----------------------------------------------------------------------------
# 1) 就绪判据读 **Unity 自己日志里的监听器标记**，不是「睡 N 秒」，也**不是** TCP 探端口。
#    TESSE 的 metadata(9001)/position(9000) 都是 **UDP**，没有 listen 状态，
#    /dev/tcp 永远连不上 → 会把已就绪的 Unity 判成没起来（lab4 Run 3 一整趟白跑）。
#
# 2) 桥接必须用 `MIT-SPARK/VNAV-labs` 的 **tag VNAV2021**（`lab3/tesse-ros-bridge/ROS`）。
#    `MIT-TESSE/tesse-ros-bridge` 的 master 已是 ROS 2 分支，launch 语法都不兼容。
#
# 3) 桥接脚本 shebang 是 `#!/usr/bin/env python`，focal 默认只有 python3 →
#    必须 `python-is-python3`（否则 exit 127，桥接两个节点当场全灭）。
#
# 4) `bash x.sh | tee log` 的退出码来自 tee —— workflow 端必须 `set -o pipefail`。
#
# 5) 【lab3 新增】rviz 与 Unity 分屏：Unity 在 :99（低分辨率、不上镜），
#    rviz 在 :98（1024x640、专供录屏）。两个 Xvfb 各自独立，录 :98 就不会把
#    Unity 窗口录进去。
# =============================================================================

set -uo pipefail

OUT="${OUT:-$PWD/out}"
BUNDLE="${BUNDLE:-$PWD/lab3}"

# ---- 两块虚拟屏 -------------------------------------------------------------
DISP_UNITY="${DISP_UNITY:-:99}"
DISP_RVIZ="${DISP_RVIZ:-:98}"

# Unity 侧：只喂物理，不上镜 —— 用低分辨率把 CPU 让给控制环与 rviz。
# （lab4 实测：640x400 → 控制器 45% 时间拿不到新状态；320x200 后开环占比压到 1.3%）
RES="${RES:-320x200}"
# rviz 侧：这是要交付的画面，分辨率不能太寒酸；但 llvmpipe 是纯 CPU 光栅化，
# 1024x640 已是 4 vCPU 上能接受的折中（再大 rviz 自己会掉帧）。
RVFB_RES="${RVFB_RES:-1024x640}"
CAPFPS="${CAPFPS:-12}"          # 录屏帧率
RVIZ_FPS="${RVIZ_FPS:-20}"      # rviz 自身重绘帧率（与录屏帧率解耦）

UPSCALE="${UPSCALE:-}"
CTRL_NICE="${CTRL_NICE:-5}"     # 控制栈 nice（正数 = 降 5 档）
UNITY_NICE="${UNITY_NICE:-10}"  # Unity / rviz / 录屏 nice
LP_NUM_THREADS="${LP_NUM_THREADS:-2}"

UNITY_BOOT_WAIT="${UNITY_BOOT_WAIT:-240}"
UNITY_SETTLE="${UNITY_SETTLE:-8}"
BRIDGE_WAIT="${BRIDGE_WAIT:-120}"
# 飞行墙钟时长。圆周周期 = 2*pi*timeScale = 2*pi*2 = 12.57 s（traj_publisher.cpp），
# 55 s ≈ 4 圈多一点：足够把「从出生点飞入 + 稳定 + 完整一圈」全覆盖。
FLIGHT_SECS="${FLIGHT_SECS:-55}"

ARM_KEY="${ARM_KEY-w}"
ARM_DELAY="${ARM_DELAY:-5}"
UNITY_WIN_NAME="${UNITY_WIN_NAME:-tesse_vnav_scenes}"

PUBLISH_IMU="${PUBLISH_IMU:-false}"   # 控制器不读 IMU，200 Hz 发它是白烧 CPU
RECORD_UNITY="${RECORD_UNITY:-0}"     # 1 = 顺带把 Unity 窗口也录一份（费 CPU）

LAB3_ZIP_URL="${LAB3_ZIP_URL:-https://vnav.mit.edu/material/lab3.zip}"
JOBS="${JOBS:-$(nproc)}"
SKIP_APT="${SKIP_APT:-0}"
PROBE_ONLY="${PROBE_ONLY:-0}"

say()  { printf '\033[1;36m[flight]\033[0m %s\n' "$*"; }
note() { printf '\033[1;35m[flight]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[flight]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[flight] %s\033[0m\n' "$*" >&2; exit 1; }
indent() { sed 's/^/    /'; }

# ROS 的 setup.bash 会在 profile.d 里引用未定义变量（ROS_MASTER_URI: unbound variable），
# 在 set -u 下直接中止 → 凡 source ROS 一律临时关掉 nounset。
ros_source() {
  set +u
  # shellcheck disable=SC1090,SC1091
  . "$1"
  set -u
}

mkdir -p "$OUT"
[ -d "$BUNDLE/dependencies" ] || die "BUNDLE=$BUNDLE 下没有 dependencies/ —— 仓库布局不对（应为 <repo>/lab3/dependencies）"

PIDS=()
killall_bg() {
  local p
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null || true; done
  pkill -f "lab3.x86_64"        2>/dev/null || true
  pkill -f "Xvfb $DISP_UNITY"   2>/dev/null || true
  pkill -f "Xvfb $DISP_RVIZ"    2>/dev/null || true
  return 0
}
trap 'killall_bg' EXIT

# =============================================================================
# 1. 系统依赖
# =============================================================================
if [ "$SKIP_APT" != "1" ]; then
  export DEBIAN_FRONTEND=noninteractive
  say "apt：构建工具"
  apt-get update -qq || warn "apt-get update 有告警"
  apt-get install -y -qq --no-install-recommends \
    build-essential cmake pkg-config git wget curl patch unzip ca-certificates \
    autoconf automake libtool \
    libeigen3-dev libyaml-cpp-dev \
    python3-numpy python3-matplotlib python3-pip python3-yaml python3-dev \
    iproute2 net-tools procps \
    || die "基础构建包装不上"

  say "apt：ROS 组件"
  for p in ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
           ros-noetic-eigen-conversions ros-noetic-cmake-modules ros-noetic-roslib \
           ros-noetic-roslaunch ros-noetic-rosbag ros-noetic-rosbag-storage \
           ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf2-ros \
           ros-noetic-ackermann-msgs ros-noetic-message-generation \
           ros-noetic-message-runtime ros-noetic-rviz; do
    # 逐个装：apt 碰到一个不存在的包名会**整批原子失败**，会把同批的其余包一起拖下水
    # （lab4 Run 6 就是这么丢了 cv_bridge / tf2_ros，11 分钟后才在 roslaunch 处炸）
    apt-get install -y -qq --no-install-recommends "$p" || warn "  $p 装不上（继续）"
  done

  say "apt：桥接 python 侧与图形栈"
  apt-get install -y -qq --no-install-recommends \
    python3-scipy python3-opencv python3-empy python3-defusedxml \
    xvfb x11-utils xdotool imagemagick ffmpeg \
    libgl1-mesa-dri libglx-mesa0 libglu1-mesa mesa-utils \
    libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libasound2 \
    libnss3 libgbm1 libgconf-2-4 \
    || warn "部分包装不上，继续"

  printf 'pcm.!default { type null }\nctl.!default { type null }\n' > "$HOME/.asoundrc"

  apt-get install -y -qq --no-install-recommends python-is-python3 \
    || warn "python-is-python3 装不上 —— 下面手动建链接"
  if ! command -v python >/dev/null 2>&1; then
    ln -sf "$(command -v python3)" /usr/local/bin/python || die "建 python 链接失败"
  fi
  command -v python >/dev/null 2>&1 || die "没有 python 解释器，桥接必然起不来"
  note "python → $(command -v python)  [$(python -V 2>&1)]"

  command -v catkin >/dev/null 2>&1 || pip3 install -q catkin_tools || die "catkin_tools 装不上"
fi

# =============================================================================
# 2. catkin 工作区（controller_pkg + tesse_ros_bridge + 两个依赖）
# =============================================================================
ros_source /opt/ros/noetic/setup.bash

# 依赖自检：按「桥接节点真实 import 到的模块」验，不赌 apt 包名拼对。
MOD_DEB=(
  "numpy            python3-numpy"
  "scipy            python3-scipy"
  "defusedxml       python3-defusedxml"
  "rospy            ros-noetic-rospy"
  "tf               ros-noetic-tf"
  "tf2_ros          ros-noetic-tf2-ros"
  "std_msgs         ros-noetic-std-msgs"
  "sensor_msgs      ros-noetic-sensor-msgs"
  "nav_msgs         ros-noetic-nav-msgs"
  "geometry_msgs    ros-noetic-geometry-msgs"
  "cv_bridge        ros-noetic-cv-bridge"
  "ackermann_msgs   ros-noetic-ackermann-msgs"
  "rosgraph_msgs    ros-noetic-rosgraph-msgs"
)
say "依赖自检：桥接用到的 python 模块"
for pair in "${MOD_DEB[@]}"; do
  mod="${pair%% *}"; deb="${pair##* }"
  python3 -c "import $mod" >/dev/null 2>&1 || { warn "缺模块 $mod → 补装 $deb"; apt-get install -y -qq --no-install-recommends "$deb" || warn "  $deb 仍装不上"; }
done
still=""
for pair in "${MOD_DEB[@]}"; do
  mod="${pair%% *}"
  python3 -c "import $mod" >/dev/null 2>&1 || still="$still $mod"
done
[ -z "$still" ] || die "桥接 python 依赖不齐：$still"
note "python 依赖自检通过（${#MOD_DEB[@]} 个模块）"

WS="${WS:-$HOME/vnav_ws}"
say "重建工作区 $WS"
rm -rf "$WS"; mkdir -p "$WS/src"
cp -r "$BUNDLE/controller_pkg"        "$WS/src/"
cp -r "$BUNDLE/dependencies/catkin_simple" "$WS/src/"
cp -r "$BUNDLE/dependencies/mav_msgs"      "$WS/src/"

VNAV_TAG="${VNAV_TAG:-VNAV2021}"
VNAV_URL="${VNAV_URL:-https://github.com/MIT-SPARK/VNAV-labs.git}"
VNAV_DIR="${VNAV_DIR:-$HOME/vnav-labs}"
BRIDGE_SRC="$VNAV_DIR/lab3/tesse-ros-bridge"

if [ ! -d "$VNAV_DIR/.git" ]; then
  say "clone VNAV-labs @ $VNAV_TAG（约 2.5 MB）"
  git clone -q --depth 1 --branch "$VNAV_TAG" "$VNAV_URL" "$VNAV_DIR" \
    || die "clone VNAV-labs 失败（tag $VNAV_TAG 不存在？）"
fi
[ -f "$BRIDGE_SRC/ROS/package.xml" ] || die "在 $BRIDGE_SRC/ROS 下找不到 package.xml"

say "拷 catkin 包 tesse_ros_bridge"
cp -r "$BRIDGE_SRC/ROS" "$WS/src/tesse_ros_bridge"
find "$WS/src/tesse_ros_bridge" -name '*.pyc' -delete 2>/dev/null || true
# ROS/setup.py 声明 packages=['tesse_ros_bridge','key_teleop']，上游缺 __init__.py
KEYINIT="$WS/src/tesse_ros_bridge/src/key_teleop/__init__.py"
[ -f "$KEYINIT" ] || : > "$KEYINIT"

say "装 tesse-interface（跳过 py2-only 的 enum34）"
pip3 install -q numpy defusedxml Pillow rosbags || warn "pip 有告警"
pip3 install -q -e "$BRIDGE_SRC/tesse-interface" --no-deps || die "pip install tesse-interface 失败"
for mod in PIL defusedxml tesse; do
  python3 -c "import $mod" >/dev/null 2>&1 || die "pip 侧模块 $mod 导不进来 —— 桥接必然起不来"
done
note "pip 侧模块自检通过（PIL / defusedxml / tesse）"

say "src/ 内容："; ls -1 "$WS/src" | indent
cd "$WS" || die "cd $WS 失败"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || die "catkin config 失败"
say "catkin build -j$JOBS"
catkin build -j"$JOBS" --no-status > "$OUT/catkin_build.log" 2>&1 \
  || { tail -40 "$OUT/catkin_build.log" | indent; die "构建未通过（见 catkin_build.log）"; }
ros_source "$WS/devel/setup.bash"
rospack find controller_pkg    >/dev/null || die "找不到 controller_pkg"
rospack find tesse_ros_bridge  >/dev/null || die "找不到 tesse_ros_bridge"
python3 -c "import mav_msgs.msg" >/dev/null 2>&1 || die "mav_msgs 不可 import"
python3 -c "import tesse_ros_bridge.msg, tesse_ros_bridge.srv" >/dev/null 2>&1 \
  || die "tesse_ros_bridge 的 msg/srv 不可 import"
note "工作区就绪：$WS"

# 顺带把「真实 catkin 编译」这条证据落盘（报告里一直被标为未验证）
{
  echo "=== catkin build（$WS）==="
  grep -E "^\[build\] (Finished|Failed)|Summary" -A12 "$OUT/catkin_build.log" 2>/dev/null | head -30
  echo
  echo "=== 编译器 ==="
  g++ --version | head -1
  echo "=== controller_pkg 可执行文件 ==="
  ls -l "$WS/devel/lib/controller_pkg/" 2>/dev/null || true
} > "$OUT/build_evidence.txt" 2>&1 || true
cat "$OUT/build_evidence.txt" | indent || true

# =============================================================================
# 3. 官方 Unity 播放器
# =============================================================================
if [ -z "${UNITY_BIN:-}" ]; then
  if [ ! -s "$OUT/lab3.zip" ]; then
    say "下载官方 lab3.zip（$LAB3_ZIP_URL）"
    curl -fL --retry 3 --retry-delay 5 --max-time 900 -o "$OUT/lab3.zip" "$LAB3_ZIP_URL" \
      || die "下载 lab3.zip 失败"
  fi
  rm -rf "$OUT/unity"; mkdir -p "$OUT/unity"
  unzip -q -o "$OUT/lab3.zip" -d "$OUT/unity" || die "解包 lab3.zip 失败"
  UNITY_BIN="$(find "$OUT/unity" -maxdepth 3 -type f -name '*.x86_64' | head -1)"
fi
[ -n "$UNITY_BIN" ] || die "解包后找不到 *.x86_64"
chmod +x "$UNITY_BIN" 2>/dev/null || true
[ -x "$UNITY_BIN" ] || die "$UNITY_BIN 不可执行"
UNITY_DIR="$(dirname "$UNITY_BIN")"
UNITY_NAME="$(basename "$UNITY_BIN")"
say "Unity  : $UNITY_BIN"
find "$UNITY_DIR" -maxdepth 1 | indent

# =============================================================================
# 4. Xvfb（Unity :99 / rviz :98）
# =============================================================================
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS="$LP_NUM_THREADS"

start_xvfb() {                       # $1=display  $2=WxH  $3=log
  pkill -f "Xvfb $1" 2>/dev/null || true
  Xvfb "$1" -screen 0 "$2x24" -ac +extension GLX +render -noreset > "$3" 2>&1 &
  PIDS+=("$!")
  for _ in $(seq 1 30); do xdpyinfo -display "$1" >/dev/null 2>&1 && return 0; sleep 1; done
  return 1
}
say "启动 Xvfb $DISP_UNITY（${RES}x24，Unity）"
start_xvfb "$DISP_UNITY" "$RES" "$OUT/xvfb_unity.log" || die "Xvfb $DISP_UNITY 起不来"
say "启动 Xvfb $DISP_RVIZ（${RVFB_RES}x24，rviz 录屏）"
start_xvfb "$DISP_RVIZ" "$RVFB_RES" "$OUT/xvfb_rviz.log" || die "Xvfb $DISP_RVIZ 起不来"
note "两块虚拟屏就绪"
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -display "$DISP_RVIZ" -B 2>/dev/null | grep -iE "OpenGL (renderer|version)" | indent || true
fi

# CPU 优先级：控制栈提，渲染/录屏降（lab4 Run 8 的实测结论：方向反了会饿死控制器）
UNITY_NICE_CMD=(nice -n "$UNITY_NICE")
CNICE=(nice -n 0)
if nice -n "-$CTRL_NICE" true 2>/dev/null; then
  CNICE=(nice -n "-$CTRL_NICE")
  note "控制栈优先级：nice -n -$CTRL_NICE（生效）"
else
  warn "无法给控制栈提优先级（缺 CAP_SYS_NICE）—— 按默认 nice 跑"
fi

say "启动 Unity（${RES}，force-glcore，nice +$UNITY_NICE）"
( cd "$UNITY_DIR" && DISPLAY="$DISP_UNITY" exec "${UNITY_NICE_CMD[@]}" "./$UNITY_NAME" \
    -logFile "$OUT/unity.log" \
    -screen-fullscreen 0 -screen-width "${RES%x*}" -screen-height "${RES#*x}" \
    -force-glcore \
    < /dev/null > "$OUT/unity_stdout.log" 2>&1 ) &
PIDS+=("$!")

readiness_marker() {
  grep -aqE "Metadata request listener started|Image request listener started" \
       "$OUT/unity.log" 2>/dev/null
}
unity_ready() {
  # 两个 local 必须分开写：`local a=.. b=$a` 里 $a 在声明生效前就展开（set -u 崩）
  local wait_s="$1"
  local end=$((SECONDS + wait_s))
  while [ "$SECONDS" -lt "$end" ]; do
    pgrep -f "$UNITY_NAME" >/dev/null 2>&1 || return 2
    readiness_marker && return 0
    sleep 2
  done
  return 1
}

say "等 Unity 的 TESSE 监听器就绪（读 unity.log 标记，最长 ${UNITY_BOOT_WAIT}s）"
unity_ready "$UNITY_BOOT_WAIT"; UP_RC=$?
{ echo "--- ss -lntup"; ss -lntup 2>/dev/null || netstat -lntup 2>/dev/null || echo "(无 ss/netstat)"; } \
  > "$OUT/sockets.txt" 2>&1 || true
if [ "$UP_RC" = 2 ]; then
  tail -40 "$OUT/unity.log" | indent || true
  die "Unity 进程死了"
elif [ "$UP_RC" != 0 ]; then
  tail -40 "$OUT/unity.log" | indent || true
  die "Unity 在 ${UNITY_BOOT_WAIT}s 内没打出监听器就绪标记"
fi
note "Unity 已就绪："
grep -aoE "Display [0-9]+ '[^']*': [0-9]+x[0-9]+|Creating OpenGL [0-9.]+ graphics device" \
     "$OUT/unity.log" 2>/dev/null | head -4 | indent || true
sleep "$UNITY_SETTLE"

# =============================================================================
# 5. roscore + 桥接
# =============================================================================
say "启动 roscore"
roscore -p 11311 > "$OUT/roscore.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 40); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
rostopic list >/dev/null 2>&1 || die "roscore 起不来"

rosparam set /use_sim_time true
say "roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch（publish_imu=$PUBLISH_IMU）"
# use_broadcast 必须保持官方默认 true：改成 false 会把 UDP 监听绑到单播地址，
# 而 Unity 往广播地址发 metadata → /current_state 直接丢，整条链路废掉。
"${CNICE[@]}" roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch \
    publish_imu:="$PUBLISH_IMU" > "$OUT/bridge.log" 2>&1 &
PIDS+=("$!")

say "等第一条 /current_state（最长 ${BRIDGE_WAIT}s）"
ok=0
for _ in $(seq 1 $((BRIDGE_WAIT / 2))); do
  if timeout 3 rostopic echo -n1 /current_state >/dev/null 2>&1; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != 1 ]; then
  { echo "--- rosnode list";  rosnode list 2>&1 | indent
    echo "--- 话题";          rostopic list 2>&1 | grep -E "current|clock|rotor" | indent
    echo "--- bridge.log";    tail -30 "$OUT/bridge.log" 2>/dev/null | indent
    echo "--- unity.log";     tail -20 "$OUT/unity.log" 2>/dev/null | indent; } || true
  die "桥接未连通"
fi
note "/current_state 已到 —— 官方链路通了"
timeout 9 rostopic hz /current_state 2>&1 | tail -3 | indent || true

# ---------------------------------------------------------------------------
# /clock 是否真的在发？—— 决定 use_sim_time 能不能开
# ---------------------------------------------------------------------------
# 控制器与 traj_publisher 的定时器/ros::Rate 都挂在 ROS 时间上。若把 use_sim_time
# 置 true 而 /clock 根本不发，ros::Time::now() 永远停在 0，**控制环一次都不会触发**
# （表现为 rviz 里无人机纹丝不动、bag 里 /rotor_speed_cmds 一条没有）。
# 所以这里先验一次：没 /clock 就把 use_sim_time 关掉并重启桥接。
CLOCK_OK=0
if timeout 6 rostopic echo -n1 /clock >/dev/null 2>&1; then CLOCK_OK=1; fi
if [ "$CLOCK_OK" = 1 ]; then
  note "/clock 在发 —— 保持 use_sim_time=true"
  timeout 9 rostopic hz /clock 2>&1 | tail -3 | indent || true
else
  warn "收不到 /clock —— 改回 use_sim_time=false 并重启桥接（否则控制环不触发）"
  rosparam set /use_sim_time false
  kill "${PIDS[-1]}" 2>/dev/null || true
  pkill -f tesse_quadrotor_bridge 2>/dev/null || true
  sleep 3
  "${CNICE[@]}" roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch \
      publish_imu:="$PUBLISH_IMU" > "$OUT/bridge.log" 2>&1 &
  PIDS+=("$!")
  ok=0
  for _ in $(seq 1 $((BRIDGE_WAIT / 2))); do
    if timeout 3 rostopic echo -n1 /current_state >/dev/null 2>&1; then ok=1; break; fi
    sleep 2
  done
  [ "$ok" = 1 ] || die "关掉 use_sim_time 后桥接仍未连上"
  note "桥接已按 wall-clock 时间重启"
fi

# 出生位姿（判断无人机是不是从原点起飞的；离线 B 场景就是按这个设定的）
timeout 5 rostopic echo -n1 /current_state/pose/pose/position 2>/dev/null | indent || true

if [ "$PROBE_ONLY" = "1" ]; then
  say "PROBE_ONLY=1 —— 探针结束（Unity + 桥接已通）"
  exit 0
fi

# =============================================================================
# 6. 上电（README：「按 W 开桨」）
# =============================================================================
if [ -n "$ARM_KEY" ]; then
  sleep "$ARM_DELAY"
  say "向 Unity 发上电键 '$ARM_KEY'"
  export DISPLAY="$DISP_UNITY"
  {
    echo "--- xdotool 可见窗口清单"
    for w in $(xdotool search --onlyvisible --name '.*' 2>/dev/null); do
      printf '%s  |  %s  |  %s\n' "$w" \
        "$(xdotool getwindowname "$w" 2>/dev/null)" \
        "$(xdotool getwindowgeometry --shell "$w" 2>/dev/null | tr '\n' ' ')"
    done
  } > "$OUT/windows.txt" 2>&1 || true
  cat "$OUT/windows.txt" | indent || true
  WID="$(xdotool search --name "$UNITY_WIN_NAME" 2>/dev/null | head -1)"
  if [ -n "$WID" ]; then
    xdotool windowfocus "$WID" 2>/dev/null || true
    note "  窗口 id = $WID（匹配 '$UNITY_WIN_NAME'）"
  else
    warn "  没有名字含 '$UNITY_WIN_NAME' 的窗口 —— 退化为对焦点窗口发键"
  fi
  xdotool key --clearmodifiers "$ARM_KEY" 2>/dev/null || warn "  xdotool 发键失败"
  sleep 2
else
  note "ARM_KEY 为空 —— 不发上电键"
fi

# =============================================================================
# 7. rviz（:98，专供录屏）+ 轨迹记录节点
# =============================================================================
say "生成 rviz 配置（官方 lab3.rviz + 两条 Path）"
python3 "$BUNDLE/headless/make_rviz_cfg.py" \
    --src "$WS/src/controller_pkg/rviz/lab3.rviz" \
    --out "$OUT/lab3_flight.rviz" --fps "$RVIZ_FPS" \
    || die "生成 rviz 配置失败"
note "配置：$OUT/lab3_flight.rviz"

say "启动轨迹记录节点（/current_state → /drone_path，/desired_state → /desired_path）"
"${CNICE[@]}" python3 "$BUNDLE/headless/path_logger.py" \
    > "$OUT/path_logger.log" 2>&1 &
PIDS+=("$!")

say "启动 rviz（DISPLAY=$DISP_RVIZ）"
( export DISPLAY="$DISP_RVIZ"
  exec "${UNITY_NICE_CMD[@]}" rviz -d "$OUT/lab3_flight.rviz" ) \
    > "$OUT/rviz.log" 2>&1 &
PIDS+=("$!")

# rviz 窗口有自己的 WM 尺寸；Xvfb 没有窗口管理器，得手动铺满整屏，
# 否则录出来是一块灰底 + 一个小窗口。
for _ in $(seq 1 30); do
  RVID="$(DISPLAY="$DISP_RVIZ" xdotool search --name "RViz" 2>/dev/null | head -1)"
  [ -n "$RVID" ] && break
  sleep 1
done
if [ -n "${RVID:-}" ]; then
  DISPLAY="$DISP_RVIZ" xdotool windowmove "$RVID" 0 0 2>/dev/null || true
  DISPLAY="$DISP_RVIZ" xdotool windowsize "$RVID" "${RVFB_RES%x*}" "${RVFB_RES#*x}" 2>/dev/null || true
  DISPLAY="$DISP_RVIZ" xdotool windowraise "$RVID" 2>/dev/null || true
  note "rviz 窗口 id=$RVID，已铺满 ${RVFB_RES}"
  sleep 3
  DISPLAY="$DISP_RVIZ" import -window root "$OUT/rviz_preflight.png" 2>/dev/null || true
else
  warn "没找到 rviz 窗口 —— 见 $OUT/rviz.log；视频可能是空的"
  tail -20 "$OUT/rviz.log" | indent || true
fi

# =============================================================================
# 8. 起控制器 + 录 bag + 录屏
# =============================================================================
say "roslaunch controller_pkg traj_tracking.launch（traj_publisher + controller_node）"
# 注意：controller_node 的 1 kHz 定时器挂在 ROS 时间上，而 /clock 只有 ~50 Hz，
# 实际控制频率≈Unity 帧率；这与官方在真机上的行为一致，不需要改代码。
"${CNICE[@]}" roslaunch controller_pkg traj_tracking.launch \
    > "$OUT/traj_tracking.log" 2>&1 &
PIDS+=("$!")
sleep 4

say "开始录 rosbag → $OUT/flight.bag"
"${CNICE[@]}" rosbag record -O "$OUT/flight.bag" \
    /clock /current_state /desired_state /rotor_speed_cmds \
    > "$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")
sleep 2

say "开始录 rviz 屏 → $OUT/rviz_flight.mp4（${CAPFPS} fps @ ${RVFB_RES}）"
nice -n "$UNITY_NICE" ffmpeg -y -loglevel warning -f x11grab -framerate "$CAPFPS" \
    -video_size "$RVFB_RES" -i "$DISP_RVIZ" \
    -c:v libx264 -preset ultrafast -crf 22 -pix_fmt yuv420p -an -threads 1 \
    "$OUT/rviz_flight.mp4" > "$OUT/ffmpeg.log" 2>&1 &
FFMPEG_PID=$!
PIDS+=("$FFMPEG_PID")

if [ "$RECORD_UNITY" = "1" ]; then
  say "顺带录 Unity 窗口 → $OUT/unity_flight.mp4"
  nice -n "$UNITY_NICE" ffmpeg -y -loglevel warning -f x11grab -framerate "$CAPFPS" \
      -video_size "$RES" -i "$DISP_UNITY" \
      -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -an -threads 1 \
      "$OUT/unity_flight.mp4" > "$OUT/ffmpeg_unity.log" 2>&1 &
  PIDS+=("$!")
fi

# 帧时浮层定时截图（Unity 开发构建左上角自带，事后可核对真实帧率）
( i=0
  while [ "$i" -lt "${SHOT_COUNT:-12}" ]; do
    sleep "${SHOT_GAP:-10}"; i=$((i + 1))
    DISPLAY="$DISP_UNITY" import -window root "$OUT/shot_t$(printf '%03d' "$i").png" 2>/dev/null || continue
    convert "$OUT/shot_t$(printf '%03d' "$i").png" -crop 320x30+0+0 +repage \
            -filter point -resize 300% "$OUT/fps_$(printf '%03d' "$i").png" 2>/dev/null || true
  done ) &
PIDS+=("$!")

say "飞行 ${FLIGHT_SECS}s（圆周周期 12.57 s）"
for i in $(seq 1 "$FLIGHT_SECS"); do
  sleep 1
  if [ $((i % 10)) -eq 0 ]; then
    printf '    +%2ds  /current_state=%s  /rotor_speed_cmds=%s\n' "$i" \
      "$(timeout 4 bash -c 'rostopic hz /current_state 2>&1 | grep -o "average rate: [0-9.]*" | head -1' || echo '?')" \
      "$(timeout 4 bash -c 'rostopic hz /rotor_speed_cmds 2>&1 | grep -o "average rate: [0-9.]*" | head -1' || echo '?')" || true
  fi
done

# =============================================================================
# 9. 收尾
# =============================================================================
say "停止录 bag / 录屏"
[ -n "${BAG_PID:-}" ]    && kill -INT "$BAG_PID"    2>/dev/null || true
[ -n "${FFMPEG_PID:-}" ] && kill -INT "$FFMPEG_PID" 2>/dev/null || true
sleep 4
[ -n "${BAG_PID:-}" ]    && kill "$BAG_PID"    2>/dev/null || true
[ -n "${FFMPEG_PID:-}" ] && kill "$FFMPEG_PID" 2>/dev/null || true
sleep 1

BAGF="$OUT/flight.bag"
[ -f "$BAGF" ] || BAGF="$OUT/flight.bag.active"
if [ -f "$BAGF" ]; then
  say "bag: $BAGF ($(du -h "$BAGF" | cut -f1))"
  rosbag info "$BAGF" > "$OUT/bag_info.txt" 2>&1 || true
  cat "$OUT/bag_info.txt" | indent || true
else
  warn "没有生成 bag —— 看 $OUT/rosbag.log"
fi

# 控制器自己的日志落在 $HOME/.ros/log，launch 里没 output=screen 就收不进产物
if [ -d "$HOME/.ros/log" ]; then
  rm -rf "$OUT/roslog"; mkdir -p "$OUT/roslog"
  cp -r "$HOME/.ros/log/." "$OUT/roslog/" 2>/dev/null || true
  grep -ah "Gain values\|F (rotor" "$OUT/roslog"/*/*.log 2>/dev/null | head -12 | indent || true
fi

# =============================================================================
# 10. 分析（与离线仿真对账）
# =============================================================================
if [ -f "$BAGF" ]; then
  say "analyze_flight_bag.py（RMSE / 最大误差 / 转速 / 偏航速率）"
  if python3 "$BUNDLE/headless/analyze_flight_bag.py" --bag "$BAGF" \
        --out "$OUT/metrics.json" --plot "$OUT/flight_tracking.png" \
        > "$OUT/analyze.log" 2>&1; then
    cat "$OUT/analyze.log" | indent
  else
    warn "分析失败，见 $OUT/analyze.log（bag 本身是好的）"
  fi
fi

# =============================================================================
# 11. 交付：上采样 + 清单
# =============================================================================
if [ -n "$UPSCALE" ] && [ -f "$OUT/rviz_flight.mp4" ] && [ "$UPSCALE" != "$RVFB_RES" ]; then
  say "上采样视频 $RVFB_RES → $UPSCALE"
  ffmpeg -y -loglevel warning -i "$OUT/rviz_flight.mp4" \
    -vf "scale=${UPSCALE}:flags=lanczos" -c:v libx264 -preset fast -crf 20 \
    -pix_fmt yuv420p -an "$OUT/rviz_flight_full.mp4" || warn "上采样失败，保留原片"
fi

say "产物清单 $OUT"
ls -la "$OUT" | indent

MISSING=0
[ -f "$OUT/rviz_flight.mp4" ] || { warn "缺 rviz_flight.mp4"; MISSING=1; }
[ -f "$OUT/flight.bag" ]      || { warn "缺 flight.bag";      MISSING=1; }
[ -f "$OUT/metrics.json" ]    || { warn "缺 metrics.json";    MISSING=1; }
[ "$MISSING" = 0 ] || die "关键产物缺失 —— 这趟不算成功"
printf '\033[1;32m[flight] 全部完成。产物在 %s\033[0m\n' "$OUT"
