# S-4 固定方向 Pick-and-Place

更新日期：2026-09-08

## 任務與 DoD

Notion 票號 S-4：

1. 物體起始位置與朝向固定。
2. 物體進入固定目標區後自動判定成功。
3. 成功畫面保留 1.5 秒，接著自動 reset 機器人與物體。
4. DoD：場景可重複執行，成功判定與 reset 可用。

這一版沿用 A3 的 Thomas App → Mac relay → SSH tunnel → Isaac Sim 控制通路。A3 原始檔案不修改；S4 使用獨立目錄、啟動檔與 JSONL。

## 場景配置

- 棕色固定工作台：中心 `(0.035, -0.190, 0.045)` m，尺寸 `0.300 x 0.300` m，桌面高度 `0.090` m。
- 機械臂底座高度 `0.050` m，下方有固定支座。
- 紅色方塊：邊長 `0.025` m、質量 `0.015` kg，使用高摩擦物理材質。
- 方塊固定起點：`(0.005, -0.200, 0.1035)` m，固定 identity orientation。
- 綠色目標區中心：`(0.075, -0.200, 0.1035)` m。
- 成功範圍：目標中心的 X/Y ±25 mm、Z ±20 mm。
- 成功還要求：夾爪已 open、方塊線速度 ≤20 mm/s、角速度 ≤0.5 rad/s，連續 30 個 physics steps。

實際位置第一次啟動後仍需用 WebRTC 確認工作台、方塊與手臂沒有穿模；若 SO-101 夾爪中心和預估位置不同，只調整本檔常數，不改成功判定語意。

## 檔案

- `s4_sim_pick_place.py`：場景、A3 控制、成功判定、自動 reset。
- `a3_transport.py`：已驗收的 A3 WebSocket transport 副本。
- `s4_task_logic.py`：不依賴 Isaac Sim 的成功與穩定判定。
- `test_s4_task_logic.py`：成功判定單元測試。

## 本機已完成的檢查

```bash
python3 -m unittest -v test_s4_task_logic.py
python3 -m py_compile s4_sim_pick_place.py a3_transport.py s4_task_logic.py
```

這只能證明純邏輯與 Python 語法；尚未證明 Isaac Sim API、碰撞、抓取或 WebRTC 畫面。

## 一次性上傳［Mac Terminal］

```bash
cd "/Users/lichengyan/Documents/ChatGPT/碩士論文/output"
scp -r s4_pick_place leo@140.115.54.63:~/
```

## 複製進 Docker［VS Code 的 RTXWS host Terminal］

提示字元必須是 `leo@RTXWS`：

```bash
docker exec isaac-sim mkdir -p /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place
docker cp ~/s4_pick_place/. isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/
docker exec -it isaac-sim bash
```

## 第一輪：只驗證場景、成功判定與 reset［Docker Terminal］

先停止其他使用 8766 的 A3 程式。在 `root@RTXWS` 容器中執行：

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab

PYTHONPATH="/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_assets:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_tasks:/root/Documents/robotics/upstream/leisaac/source/leisaac${PYTHONPATH:+:$PYTHONPATH}" \
CUDA_VISIBLE_DEVICES=0 \
PUBLIC_IP=140.115.54.63 \
LIVESTREAM=1 \
ENABLE_CAMERAS=1 \
PYTHONUNBUFFERED=1 \
./isaaclab.sh -p \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_sim_pick_place.py \
--livestream 1 --enable_cameras --ws-host 127.0.0.1 --ws-port 8766 \
--task-self-test
```

必須依序看到：

```text
S4 PICK_PLACE SIM READY
S4 WS READY ws://127.0.0.1:8766
S4 SELF TEST: object placed in goal
S4 PICK_PLACE SUCCESS ...
S4 AUTO RESET episode=1 reason=success ...
S4 TASK SELF TEST PASS
```

WebRTC 中應看到紅色方塊被測試程式放進綠色區域，成功後回到固定起點。這輪只驗證成功判定與自動 reset，不能當作機器人抓取成功證據。

## 第二輪：Thomas App 實際 Pick-and-Place

第一輪通過後在 Docker Terminal 按 `Ctrl-C`，用相同指令重啟，但刪除最後一行的 `--task-self-test`。

Mac 端照 A3 冷啟動流程開啟 SSH tunnel 與 `mac_phone_relay.py`，Thomas App 仍連：

```text
ws://<Mac 本次 Wi-Fi IP>:8765
```

建議第一次從 fine 模式逐步對位，不執行自動連按腳本：

1. Open。
2. 用 X/Y 對準紅色方塊中心。
3. Z- 逐步下降。
4. Close。
5. Z+ 抬起。
6. X+ 移向綠色目標區。
7. Z- 放下。
8. Open 並等待方塊穩定。

Mac 鍵盤控制可輸入 `0` 送出 `reset_scene`，不需重啟 Isaac Sim 即可將機械臂、夾爪與紅色方塊回復初始狀態，並解除急停。

鍵盤的 `i` / `k` 會送出 `joint_delta` 調整 `wrist_flex`：fine 每次 `5` 度，coarse 每次 `15` 度，並由 Server 依照關節 soft limits 夾限。這項操作會改變末端位置，只能在空爪且遠離桌面時先用 fine 測試。

成功時 Server 必須輸出：

```text
S4 PICK_PLACE SUCCESS ...
S4 AUTO RESET episode=1 reason=success ...
```

## S-4 完成證據

- [ ] 固定起點與目標區在 WebRTC 中可見，沒有穿模。
- [ ] `--task-self-test` 輸出 `S4 TASK SELF TEST PASS`。
- [ ] Thomas App 或 Mac 指令實際抓取並放入目標區一次。
- [ ] 真實操作觸發 `S4 PICK_PLACE SUCCESS`。
- [ ] 自動 reset 後，手臂和方塊回到相同起始狀態。
- [ ] reset 後可再次操作，至少重複成功一次。
- [ ] 保存 `/root/Documents/robotics/data/network_logs/s4_pick_place.jsonl` 與 WebRTC 畫面。

## 安全規則

- 每個命令必須等最終 ack 才能送下一個。
- 出現 `rejected`、IK failure、Estop 或物體穿模時停止，不自動重送。
- App 在移動中斷線仍會沿用 A3 行為，自動進入 Estop。
- `object_fallen` 會自動 reset，但這只代表場景恢復，不代表該次 episode 成功。
