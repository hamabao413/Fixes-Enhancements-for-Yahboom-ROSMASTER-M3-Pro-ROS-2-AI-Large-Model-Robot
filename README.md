Yahboom ROSMASTER M3 Pro (ROS 2) — Fixes & Enhancements Hub
==========================================================

An unofficial, community-driven repository that collects fixes, patches, and practical improvements for the Yahboom ROSMASTER M3 Pro ROS 2 robot stack (including AI/LLM related components and day-to-day usability tooling).

This repository is intended to be a single source of truth for all future fixes I publish for my M3 Pro setup. Each fix is shipped as a tagged release with clear notes, verification steps, and rollback guidance.

Disclaimer:
This project is NOT affiliated with or endorsed by Yahboom. Use at your own risk. Always back up your workspace before applying changes.


What You’ll Find Here
---------------------
- Bug fixes for vendor code and common integration issues
- Stability improvements (runtime errors, edge cases, parameter correctness)
- Behavior corrections based on real-world testing (e.g., calibration robustness)
- Validation commands and recommended verification procedures
- Release notes describing what changed, why, and how to test

This is not intended to be a full fork of Yahboom’s entire stack. It focuses on small, surgical, high-value fixes that can be applied safely.


Supported Environment (Reference)
---------------------------------
This repo targets common ROSMASTER M3 Pro setups. Exact compatibility depends on vendor package versions.

- ROS 2: Humble (commonly used on Ubuntu 22.04)
- Robot platform: Yahboom ROSMASTER M3 Pro (Jetson-based variants supported)
- Workspace style: colcon overlay workspace (e.g., ~/yahboomcar_ws)

If your setup differs, apply fixes carefully and verify.


Repository Structure (Recommended)
----------------------------------
.
├─ fixes/
│  ├─ color/                 (vision / color sorting related fixes)
│  ├─ navigation/            (SLAM / Nav2 integration fixes, if any)
│  ├─ web_console/           (PC / Web console fixes, if any)
│  └─ ...                    (more modules over time)
├─ docs/
│  ├─ release-notes/         (optional: extended release notes)
│  └─ troubleshooting.txt
├─ scripts/
│  ├─ verify/                (verification helpers)
│  └─ backup/                (backup helpers)
└─ README_EN.txt / README_ZH_TW.txt

Notes:
- Keep each fix scoped (one module / one problem domain).
- Each release should clearly state:
  - impacted files
  - how to apply
  - how to verify
  - any required recalibration steps
  - rollback steps


Release Policy (Important)
--------------------------
All fixes are published via GitHub Releases.

Versioning (recommended tag format):
- M3Pro-ColorFix-v0.3.4

Each release should include:
- Release notes (what/why/how to verify)
- The actual committed changes in this repo
- Optional: a ZIP asset for convenience

Rule of thumb:
If a fix changes runtime behavior (calibration, thresholds, mapping frames, etc.), the release must include:
- a test procedure
- a rollback procedure
- explicit recalibration requirements (if applicable)


Quick Start — Applying a Fix
----------------------------
Always back up your workspace first.

1) Backup (recommended)
If your workspace is at ~/yahboomcar_ws:

cd ~
cp -a yahboomcar_ws yahboomcar_ws.backup.YYYYMMDD_HHMMSS

2) Get this repo
Clone to any location:

cd ~
git clone <your-repo-url> m3pro-fixes

3) Apply a fix
Follow the release notes. Most fixes copy one or more files into your overlay source tree:

cp -a ~/m3pro-fixes/fixes/<module>/path/to/file.py \
      ~/yahboomcar_ws/src/<vendor_pkg>/<subdir>/file.py

4) Verify
Use basic compilation checks when applicable:

python3 -m py_compile <path-to-python-file>

5) Rebuild (if you changed ROS 2 package sources)
cd ~/yahboomcar_ws
colcon build --symlink-install
source install/setup.bash


Included Fix Areas (Examples)
-----------------------------
Color Detection / Color Sorting (fixes/color/)
Common improvements may include:
- Correct HSV/Hue handling for OpenCV (Hue 0..179)
- Hue wrap-around support (when H_min > H_max)
- More robust calibration (avoid glare/white surfaces)
- Fixing missing methods / AttributeError issues

Each release note will specify exactly which files were modified and what to re-test.


Verification Checklist (General)
--------------------------------
After applying a fix:
- run the provided verification commands (py_compile, smoke tests)
- validate robot behavior in a safe setup:
  - wheels off the ground (if relevant)
  - clear area, low speed
  - confirm emergency stop / kill procedure

For vision-related fixes:
- recalibrate if required
- test under both bright and dim lighting
- avoid reflective surfaces during calibration steps


Troubleshooting
---------------
If something breaks after applying a fix:
1) confirm you copied files to the correct paths
2) re-run py_compile on modified Python files
3) rebuild the workspace (colcon build) if you touched ROS packages
4) roll back using your workspace backup

See: docs/troubleshooting.txt (recommended to maintain over time)


Contributing
------------
Contributions are welcome if they are:
- minimal, focused, and tested
- clearly documented
- include verification steps
- do not introduce unused or placeholder options

Suggested workflow:
1) fork the repo
2) create a feature branch
3) submit a PR with:
   - problem description
   - fix summary
   - test steps
   - affected environments (ROS distro / hardware, if known)


License
-------
- MIT


Acknowledgements
----------------
- Yahboom ROSMASTER M3 Pro original SDK / packages (vendor)
- OpenCV / ROS 2 ecosystem contributors
- Community testers and issue reporters


Yahboom ROSMASTER M3 Pro（ROS 2）— 修復與改良集合
==============================================

這是一個非官方、以社群維護為導向的倉庫，用來彙整 Yahboom ROSMASTER M3 Pro 的 ROS 2 相關修復（fix）、補丁（patch）與實用改良（enhancement），範圍包含 AI/LLM 相關元件與日常使用的可靠度/可用性改善。

本倉庫的目標是成為我後續所有 M3 Pro 修復發佈的「單一入口」。每個修復都會以 GitHub Releases 的方式發佈，並提供清楚的說明、驗證步驟與回滾方式。

免責聲明：
本專案與 Yahboom 無任何隸屬或背書關係。請自行評估風險並在套用前務必備份工作空間。


你可以在這裡找到什麼
--------------------
- 原廠套件常見問題修正與整合型錯誤修復
- 穩定性提升（執行期錯誤、邊界情況、參數正確性）
- 依真實測試回饋的行為修正（例如：校正流程更穩定）
- 建議的驗證指令與測試流程
- 每次發佈的 Release Notes：改了什麼、為什麼改、怎麼驗證

本倉庫不是要完整 fork Yahboom 全套原始碼，而是以「小幅、精準、高價值」的修正為主，盡可能降低套用風險。


支援環境（參考）
--------------
本倉庫面向常見的 M3 Pro 使用情境；實際相容性仍取決於你所使用的原廠套件版本。

- ROS 2：Humble（常見於 Ubuntu 22.04）
- 平台：Yahboom ROSMASTER M3 Pro（含 Jetson Orin 系列常見配置）
- 工作空間型態：colcon overlay（例如：~/yahboomcar_ws）

若你的環境不同，請務必更謹慎套用並自行驗證。


倉庫結構（建議）
--------------
.
├─ fixes/
│  ├─ color/                 （視覺 / 色塊分揀相關修復）
│  ├─ navigation/            （SLAM / Nav2 整合修復，若有）
│  ├─ web_console/           （PC / Web 控制台修復，若有）
│  └─ ...                    （後續逐步擴充）
├─ docs/
│  ├─ release-notes/         （可選：更長版的發佈說明）
│  └─ troubleshooting.txt
├─ scripts/
│  ├─ verify/                （驗證小工具）
│  └─ backup/                （備份小工具）
└─ README_EN.txt / README_ZH_TW.txt

原則：
- 每個修復盡量聚焦單一模組/單一問題領域。
- 每次發佈需清楚列出：
  - 影響哪些檔案
  - 如何套用
  - 如何驗證
  - 是否需要重新校正
  - 如何回滾


發佈與版本規範（重要）
-------------------
所有修復以 GitHub Releases 發佈。

版本/標籤建議格式：
- M3Pro-ColorFix-v0.3.4

每次 Release 應包含：
- Release Notes（改動內容/原因/驗證）
- 倉庫內已提交的實際變更
- 可選：提供 ZIP 附件便於下載套用

經驗法則：
只要修復會改變執行行為（校正方式、閾值、座標/框架、導航流程等），Release Notes 必須提供：
- 測試流程
- 回滾方式
- 明確的重新校正要求（若需要）


快速開始：如何套用修復
--------------------
務必先備份你的工作空間。

1）備份（建議）
若工作空間是 ~/yahboomcar_ws：

cd ~
cp -a yahboomcar_ws yahboomcar_ws.backup.YYYYMMDD_HHMMSS

2）取得本倉庫
可放在任意位置（不一定要在 ROS workspace 內）：

cd ~
git clone <your-repo-url> m3pro-fixes

3）套用修復
依照該 Release 的說明操作。大多數修復是把特定檔案複製到 overlay source 內，例如：

cp -a ~/m3pro-fixes/fixes/<module>/path/to/file.py \
      ~/yahboomcar_ws/src/<vendor_pkg>/<subdir>/file.py

4）驗證
視情況做基本語法/編譯檢查：

python3 -m py_compile <path-to-python-file>

5）重建（若你改到 ROS 2 套件來源）
cd ~/yahboomcar_ws
colcon build --symlink-install
source install/setup.bash


已涵蓋的修復領域（範例）
----------------------
色彩辨識 / 色塊分揀（fixes/color/）
常見改良可能包含：
- OpenCV Hue 正確範圍（0..179）的處理
- Hue 跨界（wrap-around，H_min > H_max）支援
- 校正流程更穩健（降低白色/反光干擾）
- 修正缺失方法導致的 AttributeError 等問題

每次 Release 都會明確說明改了哪些檔案，以及需要重新測試/校正的項目。


驗證檢查清單（通用）
------------------
套用修復後，建議：
- 先跑驗證指令（py_compile、簡易 smoke test）
- 在安全環境測試機器人行為：
  - 必要時先離地測輪（避免暴衝）
  - 場地淨空、低速測試
  - 確認緊急停止/停止機制

視覺相關修復：
- 若 Release Notes 要求，務必重新校正
- 在不同光線下測（亮/暗）
- 校正時避免反光桌面/高反射物與陰影干擾


疑難排解
--------
若套用後出現異常：
1）確認檔案是否複製到正確路徑
2）對修改過的 Python 檔重跑 py_compile
3）若改動 ROS 套件來源，重新 colcon build
4）使用備份回滾到套用前版本

參考：docs/troubleshooting.txt（建議後續逐步補齊常見問題）


貢獻方式
--------
歡迎提交修復，條件建議如下：
- 變更小而精準，且有測試
- 文件清楚（問題描述、修正內容、驗證步驟）
- 不加入未啟用或沒有實際作用的選項/欄位

建議流程：
1）Fork 本倉庫
2）建立分支
3）提交 PR，內容至少包含：
   - 問題描述
   - 修正摘要
   - 測試步驟
   - 影響環境（ROS 版本/硬體，若已知）


授權
----
- MIT


致謝
----
- Yahboom ROSMASTER M3 Pro 原廠 SDK / 套件（vendor）
- OpenCV / ROS 2 生態系貢獻者
- 協助回報與測試的社群使用者

