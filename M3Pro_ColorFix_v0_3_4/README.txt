M3Pro ColorFix v0.3.4
=====================

Bug fixes:
  Roi_hsv() treated the Hue upper limit as 255 (but OpenCV Hue is correctly 0..179)
  If the ROI includes the white body / glare / the table, S_min/V_min get pulled too low, causing the mask to include lots of white
  AttributeError: 'color_detect' object has no attribute 'Roi_hsv'

Changes in this release:
1) Put Roi_hsv back inside class color_detect (passed py_compile syntax check)
2) Added Hue wrap-around support for object_follow / object_follow_list (when H_min > H_max, use two inRange segments)
3) Calibration convergence: Hue uses main peak ±16, and S_min/V_min are floored at >= 60 (reduces full-white masks caused by white/glare)

Validation (recommended):
  python3 -m py_compile ~/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/color_common.py

Important:
  After applying this, you must recalibrate “Red” and “Yellow” using the C key.
  Place only one color block at a time. For the ROI, select only the top colored surface and avoid the white body / table / shadows.



M3Pro ColorFix v0.3.4
====================

修正錯誤：
  Roi_hsv() 把 Hue 的上限當成 255（但 OpenCV Hue 正確是 0..179）
  ROI 一旦框到白色本體/反光/桌面，S_min/V_min 會被拉到很低，導致遮罩把白色大量吃進來
  AttributeError: 'color_detect' object has no attribute 'Roi_hsv'

本版修正：
1) Roi_hsv 確實放回 class color_detect 內（已通過 py_compile 語法檢查）
2) object_follow / object_follow_list 支援 Hue wrap-around（H_min > H_max，用兩段 inRange）
3) 標定收斂：Hue 以主峰 ±16，S_min/V_min 保底 >= 60（減少白色/反光導致遮罩整片白）

驗證（建議）：
  python3 -m py_compile ~/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/color_common.py

重要：
  套用後一定要重新用 C 鍵標定「紅」與「黃」。
  一次只放一顆色塊，ROI 請只框上方彩色面，避免白色本體/桌面/陰影。
