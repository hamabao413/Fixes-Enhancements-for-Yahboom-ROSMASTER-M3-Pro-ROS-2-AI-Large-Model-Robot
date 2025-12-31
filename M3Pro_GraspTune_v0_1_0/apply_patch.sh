\
#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/yahboomcar_ws}"
TARGET="$WS/src/M3Pro_demo/M3Pro_demo/grasp_desktop.py"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAK="$TARGET.bak_${STAMP}"

if [ ! -f "$TARGET" ]; then
  echo "[ERR] 找不到：$TARGET"
  exit 2
fi

cp -f "$TARGET" "$BAK"
cp -f "$(dirname "$0")/grasp_desktop.py" "$TARGET"
chmod +x "$TARGET" || true

echo "[OK] 已套用 grasp_desktop.py"
echo "[OK] 備份：$BAK"
echo ""
echo "測試（不重編譯也可）："
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  ros2 run M3Pro_demo grasp_desktop"
echo ""
echo "確認OK後再編譯（標準流程）："
echo "  cd $WS"
echo "  colcon build --symlink-install --packages-select M3Pro_demo --allow-overriding M3Pro_demo"
echo "  source install/setup.bash"
