# ON-1｜Isaac Sim＋Mac 鍵盤操控快速驗收

更新日期：2026-10-17  
驗收邊界：依 Leo 2026-10-17 確認，本票只需以 Mac 鍵盤控制 Isaac Sim 手臂；不要求手機 App。

## 要完成什麼

```text
Mac keyboard
  → mac_hold_teleop.py
  → Mac 127.0.0.1:8766
  → SSH local forward
  → Server/Docker 127.0.0.1:8766
  → S4 Bridge
  → Isaac Sim SO-101
```

驗收項目：

1. 按住鍵盤時手臂移動，放開後停止。
2. 急停後，移動命令被拒絕且手臂不動。
3. 解除急停後，可以再次移動。
4. Reset 後，手臂、夾爪與方塊回到初始場景。
5. 錄製 1 分鐘內的操作影片，畫面包含鍵盤控制視窗與 WebRTC。

本票不需要：

- `WIDM/teleop-app`
- `mac_phone_relay.py`
- Mac port `8765`
- HTTPS/WSS server
- iPhone 憑證

---

## 一、需要的 repositories

### Server／Docker

```text
https://github.com/LEO5166888/Teleopration_SO-101
```

執行檔：

```text
output/s4_pick_place/s4_sim_pick_place.py
output/s4_pick_place/a3_transport.py
output/s4_pick_place/s4_task_logic.py
```

### Mac／鍵盤控制

```text
https://github.com/LEO5166888/Teleopration_laptop
```

執行檔：

```text
mac_hold_teleop.py
```

`mac_move_delta.py` 是它會載入的共用 helper，不要漏掉。

---

## 二、第一次設定 Mac 鍵盤程式［Mac Terminal］

```bash
cd "$HOME/Documents"
git clone https://github.com/LEO5166888/Teleopration_laptop.git
cd Teleopration_laptop
./scripts/setup_mac.sh
git rev-parse --short HEAD
```

把最後顯示的 commit SHA 記在 Notion worklog。

---

## 三、部署 Server 程式［只有第一次或版本更新時執行］

### 3.1 Mac 上傳

先 clone Server repository：

```bash
cd "$HOME/Documents"
git clone https://github.com/LEO5166888/Teleopration_SO-101.git
cd Teleopration_SO-101
git pull --ff-only origin main
git rev-parse --short HEAD
```

將 `<LAB_USER>` 與 `<LAB_HOST>` 換成 Thomas 提供的帳號與主機：

```bash
scp -r output/s4_pick_place \
  <LAB_USER>@<LAB_HOST>:~/
```

### 3.2 Server host 複製進 Docker

```bash
ssh <LAB_USER>@<LAB_HOST>
```

提示字元應是一般使用者，例如 `joanna@widm-rs720:~$`。

```bash
docker exec isaac-sim mkdir -p \
  /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place

docker cp ~/s4_pick_place/. \
  isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/
```

`docker cp` 必須在 Server host 執行；不要進入 Docker 後再打 `docker cp`。

語法檢查：

```bash
docker exec -e TERM=xterm isaac-sim \
/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/_isaac_sim/python.sh \
-m py_compile \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/a3_transport.py \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_task_logic.py \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_sim_pick_place.py
```

沒有輸出只代表語法通過，不等於遠端控制已經成功。

---

## 四、每次測試的啟動順序

```text
1. Server/Docker S4
2. Mac WebRTC
3. Mac SSH tunnel
4. Mac keyboard controller
```

### 4.1 檢查 Docker［Server host Terminal］

```bash
docker ps -a --filter name=isaac-sim \
  --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
```

若狀態是 `Exited`：

```bash
docker start isaac-sim
```

確認 GPU：

```bash
docker exec -e TERM=xterm isaac-sim nvidia-smi
```

確認沒有另一份 S4：

```bash
docker exec -e TERM=xterm isaac-sim \
  pgrep -af 's4_sim_pick_place.py'
```

沒有輸出才繼續。若有輸出，回原本 S4 Terminal 按 `Ctrl-C` 正常停止。

進入容器：

```bash
docker exec -it -e TERM=xterm isaac-sim bash
```

### 4.2 啟動 S4［Docker Terminal］

將 `<SERVER_PUBLIC_IP>` 換成 Thomas 提供的 WebRTC server IP：

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab

PYTHONPATH="/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_assets:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_tasks:/root/Documents/robotics/upstream/leisaac/source/leisaac${PYTHONPATH:+:$PYTHONPATH}" \
CUDA_VISIBLE_DEVICES=0 \
LEISAAC_ASSETS_ROOT=/root/Documents/robotics/upstream/leisaac/assets \
PUBLIC_IP=<SERVER_PUBLIC_IP> \
LIVESTREAM=1 \
ENABLE_CAMERAS=1 \
PYTHONUNBUFFERED=1 \
./isaaclab.sh -p \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_sim_pick_place.py \
--livestream 1 \
--enable_cameras \
--ws-host 127.0.0.1 \
--ws-port 8766 \
--operator-multiview
```

不要加 `--task-self-test` 或 `--record-demos`。

必須看到：

```text
S4 PICK_PLACE SIM READY
S4 WS READY ws://127.0.0.1:8766
```

### 4.3 開啟 WebRTC［Mac］

使用 Thomas 提供的 WebRTC 網址。確認：

- 手臂、工作台、紅方塊與綠色目標區都存在。
- 手臂沒有自行移動、抖動或穿模。
- 畫面與 Server log 都正常後才連鍵盤控制。

### 4.4 檢查 SSH tunnel［Mac Terminal 1］

```bash
lsof -nP -iTCP:8766 -sTCP:LISTEN
```

若已看到 `Code Helper` 或 `ssh` 監聽 `127.0.0.1:8766`，不要再開第二條。

完全沒有輸出時才執行：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8766:127.0.0.1:8766 \
  <LAB_USER>@<LAB_HOST>
```

登入後 Terminal 停在空白畫面是正常的，測試期間保持開啟。

### 4.5 啟動鍵盤控制［Mac Terminal 2］

```bash
cd "$HOME/Documents/Teleopration_laptop"
git pull --ff-only origin main
./scripts/start_keyboard.sh
```

成功時 Terminal 顯示：

```text
S4 HOLD CONTROL READY — use the focused control window
```

並開啟 `S4 SO-101 Hold Control` 視窗。先點一下視窗，再按鍵。

---

## 五、鍵盤功能

| 按鍵 | 功能 |
|---|---|
| `W / S` | 往桌內／往機器人底座 |
| `A / D` | 往綠色目標／反方向 |
| `R / F` | 上／下 |
| 按住 `I / K` | wrist flex 正／反向 |
| 按住 `O / C` | 張開／閉合夾爪 |
| `1 / 2` | fine／coarse |
| `M` | base／tool frame 切換 |
| `E / T` | 急停／解除急停 |
| `0` | 重置手臂、夾爪與方塊 |
| `Q` | 關閉控制器 |

控制語意：

- 按住移動鍵才會持續移動。
- 放開按鍵就停止。
- 視窗失去焦點也會停止。
- 桌面附近一律使用 `fine + base`。

---

## 六、最小驗收流程

1. 確認目前為 `fine + base`。
2. 短暫按住 `R`，WebRTC 應看到手臂向上。
3. 放開 `R`，手臂必須停止。
4. 按 `E` 急停。
5. 再短按一個移動鍵；命令必須被拒絕，手臂不得移動。
6. 按 `T` 解除急停。
7. 再短按 `R`，確認恢復控制。
8. 按 `0`，確認手臂、夾爪與方塊回到初始位置。
9. Reset 後再做一次安全短移動。

每個命令都要同時檢查：

- 控制器 Terminal 的 ACK／state。
- WebRTC 的實際動作。

只有 ACK、沒有畫面動作，不算通過。

---

## 七、1 分鐘影片內容

| 時間 | 內容 |
|---|---|
| 0–10 秒 | 鍵盤控制視窗已連線；WebRTC 顯示初始場景 |
| 10–25 秒 | 按住 `R` 移動，放開後停止 |
| 25–35 秒 | 按 `E`，再按移動鍵，證明急停有效 |
| 35–45 秒 | 按 `T`，再次短移動成功 |
| 45–55 秒 | 按 `0`，場景回到初始位置 |

本票不需要完成 pick-and-place。

---

## 八、停止規則

出現以下任一情況立即放開按鍵並停止：

- `rejected`
- IK failure 或 `position_tolerance_not_met`
- `estop=true`
- 實際方向與預期不同
- 碰撞、穿模、持續抖動
- 放開按鍵後仍移動
- WebSocket 斷線

不要自動重送失敗命令。等 final ACK 後才能送下一個動作。

---

## 九、正常關閉

1. 放開所有按鍵。
2. 若手臂仍移動，按 `E`。
3. 在鍵盤控制視窗按 `Q`。
4. 若 tunnel 是手動建立，在 tunnel Terminal 按 `Ctrl-C`。
5. 最後在 Docker S4 Terminal 按 `Ctrl-C`，等待正常退出。

---

## 十、Notion worklog 範本

```text
2026-10-__｜Joanna｜做了：完成 Isaac Sim 與 Mac 鍵盤控制，驗證按住移動／放開停止、急停、解除急停與 reset，影片已附｜下一步：依排程進入下一張票｜卡：無
```

若卡住超過 30 分鐘：

```text
2026-10-__｜Joanna｜做了：完成到第 4.__ 節｜下一步：______｜卡：錯誤訊息是 ______，最後成功畫面／輸出是 ______
```

