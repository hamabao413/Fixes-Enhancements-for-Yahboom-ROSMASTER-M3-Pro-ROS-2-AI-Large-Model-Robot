\
#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/yahboomcar_ws}"
TARGET="$WS/src/M3Pro_demo/M3Pro_demo/grasp_desktop.py"

if [ ! -f "$TARGET" ]; then
  echo "[ERR] 找不到：$TARGET"
  exit 2
fi

LATEST="$(ls -1t "$TARGET".bak_* 2>/dev/null | head -n 1 || true)"
if [ -z "$LATEST" ]; then
  echo "[ERR] 找不到任何備份：$TARGET.bak_*"
  exit 3
fi

cp -f "$LATEST" "$TARGET"
echo "[OK] 已還原：$LATEST -> $TARGET"
