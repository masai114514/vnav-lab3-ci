#!/usr/bin/env bash
# =============================================================================
# fetch_artifacts.sh —— 把某次 official-flight 运行的产物拉回本地
#
# 用法：
#   bash fetch_artifacts.sh                 # 取最近一次已完成运行
#   bash fetch_artifacts.sh <run_id>        # 取指定运行
#   bash fetch_artifacts.sh <run_id> <dest> # 指定落地目录
#   DEST=lab3/official_flight_r1 bash fetch_artifacts.sh
#
# 落地内容（默认 DEST=<lab3>/official_flight_rN）：
#   rviz_flight.mp4 / rviz_flight_full.mp4 / flight.bag / metrics.json
#   flight_tracking.png / bag_info.txt / build_evidence.txt / console.log / 各类日志
# =============================================================================
set -euo pipefail

REPO="${REPO:-masai114514/vnav-lab3-ci}"
WORKFLOW="${WORKFLOW:-official-flight.yml}"
ARTIFACT="${ARTIFACT:-official-flight}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB3="$(cd "$HERE/.." && pwd)"

RUN_ID="${1:-}"
if [ -n "${2:-}" ]; then
  DEST="$2"
elif [ -n "${DEST:-}" ]; then
  : # 用外部传入的
elif [ -n "$RUN_ID" ]; then
  DEST="$LAB3/official_flight_run${RUN_ID}"
else
  DEST="$LAB3/official_flight_r$(date +%m%d)"
fi

say()  { printf '\033[1;36m[fetch]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[fetch]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fetch] %s\033[0m\n' "$*" >&2; exit 1; }

say "仓        = $REPO"
say "工作流    = $WORKFLOW"
say "落地目录  = $DEST"

if [ -z "$RUN_ID" ]; then
  say "找最近一次已完成运行 ..."
  RUN_ID="$(gh run list --repo "$REPO" --workflow "$WORKFLOW" --limit 30 \
              --json databaseId,conclusion,status,createdAt \
              --jq '[.[] | select(.status == "completed")][0].databaseId' 2>/dev/null || true)"
  [ -n "$RUN_ID" ] && [ "$RUN_ID" != "null" ] || \
    die "找不到已完成的运行。用 gh run list --repo $REPO --workflow $WORKFLOW 看看"
fi

say "run_id    = $RUN_ID"
mkdir -p "$DEST"

{
  echo "repo        : $REPO"
  echo "workflow    : $WORKFLOW"
  echo "run_id      : $RUN_ID"
  echo "下载时间    : $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo
  echo "--- gh run view ---"
  gh run view "$RUN_ID" --repo "$REPO" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' || true
} > "$DEST/run_meta.txt"
say "元信息 → $DEST/run_meta.txt"

AID="$(gh api "repos/$REPO/actions/runs/$RUN_ID/artifacts" \
        --jq ".artifacts[] | select(.name==\"$ARTIFACT\") | .id" 2>/dev/null | head -1 || true)"
if [ -n "$AID" ]; then
  SZB="$(gh api "repos/$REPO/actions/artifacts/$AID" --jq '.size_in_bytes' 2>/dev/null || echo '?')"
  say "artifact id=$AID  size=$(( ${SZB:-0} / 1048576 )) MB"
else
  warn "没查到名为 '$ARTIFACT' 的 artifact（可能没传成功）"
fi

say "下载 artifact '$ARTIFACT' ..."
ok=0
if gh run download "$RUN_ID" --repo "$REPO" --name "$ARTIFACT" --dir "$DEST"; then ok=1; fi
if [ "$ok" != 1 ] && [ -n "$AID" ]; then
  warn "gh run download 失败 —— 退回 REST zip 端点"
  TMPZ="$(mktemp -d)/artifact.zip"
  if curl -fL --retry 3 --retry-delay 5 --max-time 1800 \
        -H "Authorization: Bearer $(gh auth token)" \
        -H "Accept: application/vnd.github+json" \
        -o "$TMPZ" \
        "https://api.github.com/repos/$REPO/actions/artifacts/$AID/zip"; then
    unzip -q -o "$TMPZ" -d "$DEST" && ok=1
  fi
fi
[ "$ok" = 1 ] || die "artifact 下载失败"

if [ -d "$DEST/out" ]; then
  say "artifact 里带了一层 out/，摊平"
  shopt -s dotglob nullglob
  mv "$DEST/out/"* "$DEST/" 2>/dev/null || true
  rmdir "$DEST/out" 2>/dev/null || true
  shopt -u dotglob nullglob
fi

say "落地内容："
ls -la "$DEST" | sed 's/^/    /'

echo
say "快速判读："
if [ -f "$DEST/metrics.json" ]; then
  echo "  metrics.json:"
  sed 's/^/      /' "$DEST/metrics.json" | head -45
else
  warn "  没有 metrics.json（probe 模式或飞行没跑完）"
fi
if [ -f "$DEST/rviz_flight.mp4" ]; then
  echo "  rviz 视频: 有 ($(du -h "$DEST/rviz_flight.mp4" | cut -f1))"
else
  warn "  rviz 视频: 无"
fi
[ -f "$DEST/flight.bag" ] && echo "  bag: 有" || warn "  bag: 无"

echo
say "下一步："
echo "    open $DEST/rviz_flight_full.mp4 2>/dev/null || open $DEST/rviz_flight.mp4"
