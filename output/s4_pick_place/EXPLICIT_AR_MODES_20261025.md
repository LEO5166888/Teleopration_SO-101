# AR 明確切換平移／手腕（explicit_v2，本機測試版）

僅修改 Leo Mac 的 `output/teleop_app_sp6` 與 `output/s4_pick_place`。
2026-10-25 Leo 已在模擬器操作後回報「現在順好多了」，並要求保存此版至自己的
`LEO5166888/Teleopration_SO-101`；未修改 Thomas 的 GitHub。
這是操作手感的實測回饋，尚不代表完整夾取任務、停止後漂移與邊界情境均已驗收。

## 問題與行為

seq2760 在平移途中因 −1.03° pitch 進入舊自動手腕鎖定，後續回移被忽略。
這版取消運行時自動判斷：AR 畫面新增「平移／手腕」選項，預設平移。

- 平移：保留 `base = [x, y, -z]`，固定兩個手腕目標，忽略所有手機旋轉。
- 手腕：從第一筆中立姿態就固定其他關節目標，只有 wrist_flex 跟隨前後傾。
  保留已實測的方向、限速、35° 相對限幅及 soft joint limits。
- 模式切換先發送 stop，不自動重啟。必須放開再按，新按住重設 p0/q0。
- 手腕按住時不開合夾爪；放開後仍可用下方開／關。平移時可同時開合。
- 手腕模式沒有 XYZ 固定點補償，指尖自然畫弧；固定關節目標不保證物理關節完全不動。
- 工作範圍、watchdog、急停及其他安全限制保持原有處理；12 公分輸入仍可能碰工作範圍限制。

`pose_start` 新增可選 `control_mode: translation | wrist`，Server 端未帶欄位時預設 translation。
舊 Server 會拒收新欄位，所以 App 與 Server 要一起更新。`pose_update` 欄位沒有改變。
App 也做輸入分離，Server 再依明確模式執行；不依賴單靠前端清零。

## 備份

`../s4_before_explicit_modes_20261025_165925/controller_and_app.tar.gz`
包含修改前的 App 與 Server 檔案。不要解壓覆蓋其他新工作；需要退回時先停止模擬器。

## 檢查

2026-10-25 本機結果：53 項 Python 測試通過、4 項 JavaScript 模式測試通過，
隔離 Chrome 完整頁面測試通過（平移→停止→手腕→停止→平移），無 JavaScript 錯誤。
Python／JavaScript／部署腳本語法檢查通過；390×844 手機尺寸畫面已檢視。

Server 純 Python 邏輯：
```sh
python3 -m unittest -v test_explicit_pose_modes.py test_pose_wrist_only.py test_phone_translation_20261025.py test_s4_task_logic.py test_s4_jog_transport.py
```
網頁（需 Node）：`node --test test_ar_control_modes.cjs`。
完整頁面流程：`node test_ar_ui.cjs`（需 Playwright 與 Chromium；可指定 PLAYWRIGHT_MODULE / CHROME_EXECUTABLE）。
瀏覽器測試隔離所有 URL，只使用假 socket，不連機器人；不能代表 iPhone WebXR 或 Isaac 動作已驗收。

## 部署與驗收

1. 在執行模擬器的視窗 Ctrl+C。
2. Mac：
```sh
cd "/Users/lichengyan/Documents/ChatGPT/碩士論文/output/s4_pick_place"
zsh deploy_base_pose_controller.zsh
```
本次部署必須包含 `a3_transport.py`，腳本已更新。

3. Server 的 `leo@RTXWS:~$`：
```sh
docker exec -it isaac-sim bash -lc 'bash /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/start_s4_base_pose.sh'
```
容器內則直接 `bash /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/start_s4_base_pose.sh`。
確認 `control_modes=explicit_v2 default=translation`、SIM READY、WS READY。

4. Mac HTTPS 與 tunnel 沿用現在可連線的程序。serve_https.py 會直接讀取更新的靜態檔，無需重新啟動。
   手機結束 AR、重新整理頁面、重新連線再進入 AR；應看到「平移／手腕」。
   若頁面不是由 `output/teleop_app_sp6/serve_https.py` 提供，要先改用此目錄。
5. 選平移：向左約 2 公分，再向右回來；帶少許傾斜仍應平移，不應出現 WRIST-ONLY START。
6. 放開，選手腕，重新按住：前傾／回正／後傾，只改 wrist_flex 目標。
7. 放開，選平移，再按住：確認平移恢復，貼 POSE START / STOP 驗收。
