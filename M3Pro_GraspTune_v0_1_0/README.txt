M3Pro_GraspTune_v0_1_0

Purpose
- Fix common issues: “can’t grasp / grasp point is too far back” and “gripper not tight enough”
- Only modifies M3Pro_demo/M3Pro_demo/grasp_desktop.py (Python only)

Apply
1) After extracting the archive, enter the folder
2) Run:
   chmod +x apply_patch.sh restore_backup.sh
   ./apply_patch.sh   # Automatically backs up and overwrites into ~/yahboomcar_ws

Test (usually no rebuild required)
- Just restart the node:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  ros2 run M3Pro_demo grasp_desktop

Standard build (run once after confirming it works)
  cd ~/yahboomcar_ws
  colcon build --symlink-install --packages-select M3Pro_demo --allow-overriding M3Pro_demo
  source install/setup.bash

Tuning suggestions
- grasp_bias_x: positive = move forward (closer to the color block); each +0.005 is about 5 mm
- grasp_bias_z: negative = move downward (closer to the block); adjust by 0.002–0.005 each time
- gripper_close_angle: gripper closing angle; recommended 130–170. Too tight may jam or overload the servo

I consolidated the tuning points into:
~/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/grasp_desktop.py

New adjustable parameters (actively used; not placeholder fields)
- grasp_bias_x / y / z: grasp point offsets (unit: meters)
- gripper_open_angle / gripper_close_angle: gripper open/close angles (unit: degrees)

The original hard-coded values:
- request.tar_x = pose_T[0] - 0.015 (often causes the target to be “too far back”)
- multiple hard-coded gripper angles (originally 30/135 in several places)

have been changed to use the unified parameters above, so you can fine-tune by editing just a few numbers.


M3Pro_GraspTune_v0_1_0

目的
- 解決「夾不到/太後面」與「夾爪不夠緊」的常見問題
- 僅修改 M3Pro_demo/M3Pro_demo/grasp_desktop.py（純 Python）

套用
1) 解壓縮後進入資料夾
2) 執行：
   chmod +x apply_patch.sh restore_backup.sh
   ./apply_patch.sh   # 會自動備份並覆蓋到 ~/yahboomcar_ws

測試（通常不必重編譯）
- 只要重啟節點即可：
  source /opt/ros/$ROS_DISTRO/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  ros2 run M3Pro_demo grasp_desktop

標準編譯（確認OK後做一次即可）
  cd ~/yahboomcar_ws
  colcon build --symlink-install --packages-select M3Pro_demo --allow-overriding M3Pro_demo
  source install/setup.bash

調參建議
- grasp_bias_x：正值 = 往前（更靠近色塊），每次 +0.005 約 5mm
- grasp_bias_z：負值 = 往下（更貼近色塊），建議每次 0.002~0.005
- gripper_close_angle：夾爪關閉角度，建議 130~170，夾太緊可能卡住或讓舵機吃力

我把調整點集中在 ~/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/grasp_desktop.py


新增可直接調的參數（有用到，不是預留欄位）
grasp_bias_x / y / z：夾取點偏移（單位：公尺）
gripper_open_angle / gripper_close_angle：夾爪開/關角度（單位：度）
把原本固定寫死的：
request.tar_x = pose_T[0] - 0.015（很容易造成「太後面」）
以及夾爪開合角度（原本多處寫死 30/135）
改成統一走上述參數，之後只要改幾個數字就能微調。
