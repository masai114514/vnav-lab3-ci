#!/usr/bin/env bash
# =============================================================================
# sync.sh —— 把 lab3 的交付代码同步进 CI 仓镜像（lab3/controller_pkg）
#
# 为什么需要它：CI 仓里的 lab3/controller_pkg 是 `controller_pkg` 的**只读镜像**，
# 手改会在下次 sync 时被覆盖。改代码永远改 ../controller_pkg，然后跑本脚本。
#
# 不动的目录（CI 仓自己的东西）：
#   ci/、.github/、lab3/headless/、lab3/dependencies/
# =============================================================================
set -euo pipefail
cd "$(dirname "$0")"

SRC="../controller_pkg"
DST="lab3/controller_pkg"

[ -d "$SRC" ] || { echo "找不到 $SRC —— 本脚本应放在 <lab3>/github-ci/ 下" >&2; exit 1; }
mkdir -p "$DST"
# --delete 保证源码侧删掉的文件在镜像里也删掉
rsync -a --delete --exclude '.git' "$SRC/" "$DST/"

echo "[sync] $SRC → $DST"
echo "[sync] 文件："
find "$DST" -type f | sed 's/^/    /' | sort
echo "[sync] 完。接着跑："
echo "    python3 push_via_api.py --message 'ci: ...'"
echo "    gh workflow run official-flight.yml --repo masai114514/vnav-lab3-ci"
