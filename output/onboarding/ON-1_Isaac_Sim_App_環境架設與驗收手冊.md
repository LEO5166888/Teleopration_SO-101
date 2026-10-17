# ON-1｜Isaac Sim＋手機 App 環境架設與驗收手冊

> **2026-10-17 更新：Leo 已確認目前 ON-1 只需用 Mac 鍵盤操控。** 目前請改用
> [ON-1｜Isaac Sim＋Mac 鍵盤操控快速驗收](./ON-1_Isaac_Sim_鍵盤操控_快速驗收.md)。
> 本文件保留作為之後手機 App 整合的完整流程，不是目前 ON-1 的必要步驟。

更新日期：2026-10-17  
適用票號：ON-1「環境架設（Joanna）：Isaac Sim 開起來、用 App 按鈕把手臂動起來」  
Owner：Joanna｜Sprint 6｜截止日：2026-10-23

## 這張票要完成什麼

本票不是要先理解 IK、WebSocket 或 Isaac Sim 內部原理，而是要親手把整條控制鏈跑通一次：

```text
iPhone Teleop App
  → Mac HTTPS/WSS server
  → Mac phone relay :8765
  → Mac SSH tunnel :8766
  → widm-rs720／Docker S4 server :8766
  → Isaac Sim SO-101
```

完成時必須證明以下四件事：

1. App 按鈕能讓 Isaac Sim 裡的手臂移動，放開後停止。
2. 急停後，移動命令不能繼續執行。
3. 解除急停後，可以再次控制。
4. 重置場景後，手臂、夾爪與方塊回到初始位置。

DoD 是一段 **1 分鐘內**的操作影片，需同時看得到手機頁面與 Isaac Sim 畫面；若本手冊或 App README 與實際狀況不同，需記在 Notion 票的 worklog。

---

## 一、Leo／Thomas 在開始前必須提供的資料

不要只丟一個 GitHub 網址。第一次交接前要把下表補齊：

| 項目 | 由誰提供 | 用途 |
|---|---|---|
| `https://github.com/LEO5166888/Teleopration_SO-101.git` 存取權 | Leo | Isaac Sim S4 server、Bridge、Mac relay |
| `github.com/WIDM/teleop-app` 存取權 | Thomas | 手機 Teleop 網頁與 HTTPS/WSS server |
| 本次指定的 branch 與 commit SHA | Leo、Thomas | 保證大家執行同一版本 |
| `widm-rs720` 的帳號與可連線位址 | Thomas | SSH 登入實驗室主機 |
| Isaac Sim WebRTC 網址 | Thomas | 在 Mac 觀看模擬畫面 |
| 是否需要校內網、VPN 或跳板機 | Thomas | 確保 Mac 能連到主機 |
| iPhone 與 Mac 使用的同一個 Wi-Fi | 操作者 | 手機必須能連 Mac 的 8443 port |
| iPhone HTTPS 憑證安裝方式 | Thomas | Safari 必須信任 Teleop 頁面 |

> `WIDM/teleop-app` 目前不是公開 repository。GitHub 顯示 404 時，先請 Thomas 加入權限，不要自行猜 repository 或改用不明副本。

---

## 二、本次會執行哪些檔案

### Leo 的 repository

| 檔案 | 執行位置 | 功能 |
|---|---|---|
| `output/s4_pick_place/s4_sim_pick_place.py` | Docker | 建立 SO-101、工作台、方塊與目標區；執行控制、急停與 reset |
| `output/s4_pick_place/a3_transport.py` | 由上列程式載入 | WebSocket protocol、命令驗證、queue、watchdog 與 ACK |
| `output/s4_pick_place/s4_task_logic.py` | 由上列程式載入 | 任務成功與座標轉換邏輯 |
| `output/s4_pick_place/s3_recorder.py` | 只有開錄製參數時載入 | HDF5 示範資料錄製；ON-1 不開啟 |
| `output/a3_move_delta/mac_phone_relay.py` | Mac | 把手機命令原封不動轉送到 SSH tunnel |

### Thomas 的 `WIDM/teleop-app`

| 檔案 | 執行位置 | 功能 |
|---|---|---|
| `serve_https.py` | Mac | 提供 iPhone HTTPS 頁面，並將 WSS 轉到 `ws://127.0.0.1:8765` |
| Teleop 前端檔案 | iPhone Safari | 移動、急停、解除急停與重置按鈕 |

不要另外啟動下列舊程式：

```text
server_v0.py
a3_sim_move_delta.py
獨立執行 a3_transport.py
```

`s4_sim_pick_place.py` 會自行建立 Bridge。重複啟動舊 server 會搶占 `8766`。

---

## 三、先準備環境變數

以下三個值必須由 Thomas 依實驗室當天環境提供。後續看到尖括號時不可原樣貼上：

```text
<LAB_USER>          例如 joanna
<LAB_HOST>          widm-rs720 可連線的 hostname 或 IP
<SERVER_PUBLIC_IP>  WebRTC 使用的 server IP
```

---

## 四、第一次設定：下載 Leo 的程式［Mac Terminal］

只需做一次：

```bash
cd "$HOME/Documents"
git clone https://github.com/LEO5166888/Teleopration_SO-101.git
cd Teleopration_SO-101
git switch main
git pull --ff-only origin main
git rev-parse --short HEAD
```

把最後一行輸出的 commit SHA 記在 Notion worklog。若 Leo 指定的 SHA 不相同，先停止並詢問，不要混用版本。

建立 Mac relay 的 Python 環境：

```bash
cd "$HOME/Documents/Teleopration_SO-101"
python3 -m venv .venv
.venv/bin/python -m pip install 'websockets==12.0'
```

---

## 五、第一次設定：取得 Thomas App［Mac Terminal］

Thomas 加入 GitHub 權限後，依 App README clone。若使用 SSH：

```bash
cd "$HOME/Documents"
git clone git@github.com:WIDM/teleop-app.git
cd teleop-app
git switch main
git pull --ff-only origin main
git rev-parse --short HEAD
```

先確認 current repository 真的包含啟動檔：

```bash
ls README* serve_https.py
```

接著依 `teleop-app` README 完成憑證與 Python 環境。Leo 在 2026-10-14 驗證過的啟動入口是：

```bash
.venv/bin/python serve_https.py
```

若目前 README 已改成其他指令，以 **Thomas 指定的 commit 對應 README** 為準，並把差異寫進 ON-1 worklog。

---

## 六、部署 Leo 程式到 Server／Docker

只有第一次部署或 Leo 更新版本時才需要做本節。

### 6.1 Mac 上傳到實驗室主機［Mac Terminal］

將 `<LAB_USER>`、`<LAB_HOST>` 換成 Thomas 提供的值：

```bash
cd "$HOME/Documents/Teleopration_SO-101"
scp -r output/s4_pick_place \
  <LAB_USER>@<LAB_HOST>:~/
```

### 6.2 複製進 Docker［Server host Terminal］

先登入 Server：

```bash
ssh <LAB_USER>@<LAB_HOST>
```

提示字元應是一般使用者，例如：

```text
joanna@widm-rs720:~$
```

執行：

```bash
docker ps -a --filter name=isaac-sim \
  --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'

docker exec isaac-sim mkdir -p \
  /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place

docker cp ~/s4_pick_place/. \
  isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/
```

> `docker cp` 要在 Server host 執行。看到 `root@...:/isaac-sim#` 時已經在容器內，裡面沒有 `docker` 指令。

語法檢查：

```bash
docker exec -e TERM=xterm isaac-sim \
/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/_isaac_sim/python.sh \
-m py_compile \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/a3_transport.py \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_task_logic.py \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_sim_pick_place.py
```

沒有輸出並回到 `$` 只代表 Python 語法通過；還不能當作 Isaac Sim 與手機連線成功。

---

## 七、每次測試的啟動順序

啟動順序固定為：

```text
1. Docker／Isaac Sim
2. WebRTC 畫面
3. SSH tunnel
4. Mac phone relay
5. Teleop HTTPS/WSS server
6. iPhone App
```

每一步成功後才做下一步。

### 7.1 啟動 Docker［Server host Terminal］

```bash
docker ps -a --filter name=isaac-sim \
  --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
```

若狀態是 `Exited`，才執行：

```bash
docker start isaac-sim
```

確認 GPU：

```bash
docker exec -e TERM=xterm isaac-sim nvidia-smi
```

確認沒有舊 S4：

```bash
docker exec -e TERM=xterm isaac-sim \
  pgrep -af 's4_sim_pick_place.py'
```

沒有輸出才可啟動。若有輸出，回原本 Terminal 以 `Ctrl-C` 正常停止；不要同時跑兩份。

進入容器：

```bash
docker exec -it -e TERM=xterm isaac-sim bash
```

### 7.2 啟動 Isaac Sim S4［Docker Terminal］

提示字元必須類似 `root@widm-rs720:/isaac-sim#`。先將下列 `<SERVER_PUBLIC_IP>` 換成 Thomas 提供的值：

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

ON-1 是人工操作測試，因此不要加 `--task-self-test` 或 `--record-demos`。

必須看到：

```text
S4 PICK_PLACE SIM READY
S4 WS READY ws://127.0.0.1:8766
```

若有啟用三視角，還會看到：

```text
S8 MULTIVIEW READY ...
```

出現 traceback、`ModuleNotFoundError`，或沒有 `S4 WS READY` 時先停止，不要繼續接手機。

### 7.3 開 WebRTC［Mac］

依 Heptabase「Isaac Sim 啟動 SOP（widm-rs720）」開啟 Thomas 提供的 WebRTC 網址。畫面中需確認：

- SO-101、工作台、紅方塊與綠色目標區都存在。
- 手臂沒有持續抖動、穿模或自行移動。
- 若使用 `--operator-multiview`，主視角、側視角與夾爪視角可正常顯示。

### 7.4 建立 SSH tunnel［Mac Terminal 1］

先檢查 8766：

```bash
lsof -nP -iTCP:8766 -sTCP:LISTEN
```

若已看到 `Code Helper` 或 `ssh` 監聽 `127.0.0.1:8766`，不要重複建立。

只有完全沒有輸出時才執行：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8766:127.0.0.1:8766 \
  <LAB_USER>@<LAB_HOST>
```

輸入密碼後停在空白畫面是正常的，Terminal 要保持開啟。

### 7.5 啟動 Mac phone relay［Mac Terminal 2］

```bash
cd "$HOME/Documents/Teleopration_SO-101"
.venv/bin/python output/a3_move_delta/mac_phone_relay.py
```

必須看到：

```text
MAC PHONE RELAY READY listen=ws://0.0.0.0:8765 upstream=ws://127.0.0.1:8766
Thomas App URL: ws://<Mac IP>:8765
```

### 7.6 啟動 Teleop HTTPS/WSS server［Mac Terminal 3］

```bash
cd "$HOME/Documents/teleop-app"
.venv/bin/python serve_https.py
```

Leo 驗證過的輸出格式為：

```text
App：      https://<Mac IP>:8443/
wss 轉接： wss://<Mac IP>:8443/relay → ws://127.0.0.1:8765
```

若 current App README 的命令或 port 不同，以本次鎖定 commit 的 README 為準，並把差異寫進 worklog。

### 7.7 iPhone 連線

查 Mac Wi-Fi IP：

```bash
ipconfig getifaddr en0
```

iPhone 與 Mac 必須在同一個 Wi-Fi。使用 iPhone Safari 開啟：

```text
https://<Mac IP>:8443/
```

按「連線」後，成功條件是：

- 手機頁面顯示「已連」與「可操作」。
- Mac relay 顯示 `[PHONE CONNECT]`。
- Mac relay 顯示 `[RTXWS CONNECT] ws://127.0.0.1:8766`。

---

## 八、ON-1 最小安全驗收流程

開始前把模式切到 fine／微調，一次只操作一個按鈕。

### 8.1 移動與放開停止

1. 選一個不會撞桌面的安全方向，建議先短按「上」。
2. 按住時，WebRTC 中手臂應持續移動。
3. 放開後，手臂應停止。
4. 等 App 收到 final ACK，再做下一個動作。

通過證據：手機顯示 ACK，且 WebRTC 真的看到手臂動作與停止。只有 ACK、沒有畫面動作，不算通過。

### 8.2 急停

1. 在手臂已停止時按「急停 STOP」。
2. 再短按一次安全移動方向。
3. 移動命令必須被拒絕，手臂不得移動。

### 8.3 解除急停

1. 按「解除急停」。
2. 再短按一次安全方向。
3. WebRTC 中應再次看到手臂移動並在放開後停止。

### 8.4 重置場景

1. 按「重置場景」。
2. 確認手臂、夾爪與紅方塊都回到初始位置。
3. Reset 後再做一次很短的安全移動，確認仍可操作。

---

## 九、1 分鐘驗收影片怎麼錄

建議用 Mac 螢幕錄影，同時放入 WebRTC 視窗與 iPhone 畫面鏡像；總長不要超過 1 分鐘。

| 時間 | 畫面內容 |
|---|---|
| 0–10 秒 | 手機顯示已連；WebRTC 顯示初始場景 |
| 10–25 秒 | 按住安全方向移動，放開後停止 |
| 25–35 秒 | 按急停，再按移動，證明手臂不動／命令被拒絕 |
| 35–45 秒 | 解除急停，再次短移動成功 |
| 45–55 秒 | 按重置，畫面回到初始狀態 |

影片不需要完成 pick-and-place；ON-1 的重點是環境、控制、安全與 reset 都能運作。

---

## 十、遇到錯誤時的停止規則

出現以下任一情況，立即停止操作，不要重送同一命令：

- App 收到 `rejected`。
- Server 顯示 IK failure 或 `position_tolerance_not_met`。
- `estop=true`。
- 手臂方向與預期相反。
- 手臂、夾爪或方塊穿模、碰撞或持續抖動。
- App 斷線後手臂仍移動。

按住控制的 safety watchdog 為 0.75 秒。App 必須在按住期間持續送 keepalive；斷線或 keepalive 超時時 Server 會停止並進入安全狀態。不要用自動 retry 掩蓋錯誤。

---

## 十一、快速故障排查

| 現象 | 先檢查 |
|---|---|
| GitHub 顯示 `WIDM/teleop-app` 404 | 尚未取得 private repository 權限 |
| `docker: command not found` | 目前在 Docker 裡；先 `exit` 回 Server host |
| `./isaaclab.sh: No such file or directory` | 忘了 `cd .../IsaacLab`，或貼成 `d` |
| 8766 `Address already in use` | `lsof`；通常 VS Code 已建立 tunnel，不要再開第二條 |
| Relay 顯示 `[RTXWS UNAVAILABLE]` | S4 是否有 `S4 WS READY`、8766 tunnel 是否存在 |
| 手機打不開 8443 | Mac IP、同一 Wi-Fi、`serve_https.py`、iPhone 憑證 |
| App 已連但手臂不動 | 同時看 App ACK、relay log、Server log 與 WebRTC；缺一不可 |
| WebRTC 全黑 | GPU、S4 是否仍在執行、livestream 與 Thomas 提供的 WebRTC SOP |
| `estop_active` | 先確認現場安全，再按解除急停；不要連按移動 |
| 重置後仍異常 | 停止 App，正常結束 S4，重新依第七節啟動 |

卡住超過 30 分鐘，不要繼續亂改環境；直接在 ON-1 worklog 記錄。

---

## 十二、正常關閉順序

1. 放開手機所有控制按鈕。
2. 若手臂仍移動，按急停。
3. 手機 App 按斷線。
4. Mac Terminal 3 的 `serve_https.py` 按 `Ctrl-C`。
5. Mac Terminal 2 的 relay 按 `Ctrl-C`。
6. 若 tunnel 是手動建立，Mac Terminal 1 按 `Ctrl-C`。
7. Docker 的 Isaac Sim Terminal 最後按 `Ctrl-C`，等待正常退出。

一般不需停止 container。確定當天不再使用時，才在 Server host 執行：

```bash
docker stop isaac-sim
```

---

## 十三、Notion worklog 範本

把最新一行放在最上面：

```text
2026-10-__｜Joanna｜做了：依 ON-1 SOP 啟動 S4、App，完成移動／急停／解除急停／重置並附 1 分鐘影片｜下一步：開始 IMU 任務｜卡：無
```

若有差異：

```text
2026-10-__｜Joanna｜做了：完成至第 7.5 節｜下一步：確認 App HTTPS 啟動方式｜卡：teleop-app README 寫 ______，實際執行需改成 ______
```

---

## 十四、交付前 checklist

- [ ] 已取得 `WIDM/teleop-app`、lab host 與 Notion 權限。
- [ ] Leo 與 Thomas 已提供 branch／commit SHA。
- [ ] Docker 中只有一份 `s4_sim_pick_place.py`。
- [ ] Server 顯示 `S4 PICK_PLACE SIM READY`。
- [ ] Server 顯示 `S4 WS READY ws://127.0.0.1:8766`。
- [ ] Mac 8766 tunnel、8765 relay 與 8443 HTTPS/WSS 都在執行。
- [ ] iPhone 顯示已連、可操作。
- [ ] 移動與放開停止通過。
- [ ] 急停與拒絕移動通過。
- [ ] 解除急停後可再次移動。
- [ ] 重置場景通過。
- [ ] 1 分鐘內影片已附到 ON-1。
- [ ] README／SOP 差異已寫入 worklog。

