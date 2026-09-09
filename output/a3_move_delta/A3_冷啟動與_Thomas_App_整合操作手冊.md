# A3 冷啟動與 Thomas App 整合操作手冊

這份手冊從 RTXWS、Mac、手機全部關閉的狀態開始。每一段確認成功後再進入下一段；遇到 `FAIL`、`rejected` 或連線錯誤時不要自動重送移動命令。

## 先辨認指令要貼在哪裡

| 執行位置 | 常見提示字元 | 在這裡執行的內容 |
|---|---|---|
| **Mac 本機 Terminal** | `lichengyan@MacBookPro ... %` | `ipconfig`、`ssh -N` tunnel、`scp`、所有 `mac_*.py`、Mac relay |
| **VS Code Server Terminal（RTXWS host）** | `leo@RTXWS ... $` | `docker ps/start/cp/exec/stop` |
| **VS Code Server Terminal 裡的 Docker** | `root@RTXWS ... #` | Isaac Lab／Isaac Sim 啟動指令、Server JSONL |
| **Mac 瀏覽器** | 沒有終端機提示字元 | WebRTC 畫面 |
| **Thomas 手機 App** | 沒有終端機提示字元 | 設定 `ws://<Mac IP>:8765`、按鈕與回傳訊息測試 |

不要在 `root@RTXWS` 裡執行 `/Users/lichengyan/...` 或 `ipconfig`；也不要在 Mac 本機執行 `/root/Documents/...` 或 `docker exec`。

## 系統通路

```text
Thomas App
  -> ws://<Mac Wi-Fi IP>:8765
  -> Mac mac_phone_relay.py
  -> ws://127.0.0.1:8766
  -> Mac SSH tunnel
  -> RTXWS Docker / Isaac Sim WebSocket server
  -> SO-101
```

Thomas App 不需要 RTXWS 的帳號或密碼。SSH 密碼只由使用者在 Mac 建立 tunnel 時輸入，且不會寫入 Python 程式。

## 一、開啟 RTXWS 與 Docker［VS Code Server］

1. 開啟實驗室 RTXWS，等待 Ubuntu 和網路就緒。
2. 在 VS Code 連上 RTXWS，開啟 Terminal。確認提示字元是：

   ```text
   leo@RTXWS:...$
   ```

   如果 VS Code Remote SSH 尚未連上，才在 **Mac 本機 Terminal** 執行：

   ```bash
   ssh leo@140.115.54.63
   ```

3. SSH 登入需要密碼；登入後應看到 `leo@RTXWS`。
4. 以下改在 **VS Code Server Terminal，提示字元 `leo@RTXWS`** 執行。檢查容器：

   ```bash
   docker ps -a --filter name=isaac-sim
   ```

5. 若狀態為 `Exited`，啟動容器：

   ```bash
   docker start isaac-sim
   ```

6. 進入容器：

   ```bash
   docker exec -it isaac-sim bash
   ```

   成功後應看到 `root@RTXWS:/isaac-sim#`。

### 下次啟動前的一次性控制更新［Mac 本機＋VS Code Server］

目前版本新增了「移動中斷線或執行失敗即自動 Estop」，並將 coarse/fine 改為距離比例的 90／23 trajectory steps、5 個穩定樣本與 90 steps hold limit。在下一次啟動模擬前做一次以下更新；成功複製後，往後一般冷啟動不必重做。

先在 **Mac 本機 Terminal，提示字元 `lichengyan@MacBookPro`** 執行：

```bash
scp "/Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta/a3_transport.py" \
    "/Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta/a3_sim_move_delta.py" \
    leo@140.115.54.63:~/
```

回到 **VS Code Server Terminal，提示字元 `leo@RTXWS`** 執行：

```bash
docker cp ~/a3_transport.py \
  isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/a3_transport.py

docker cp ~/a3_sim_move_delta.py \
  isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/a3_sim_move_delta.py
```

再用 `docker exec -it isaac-sim bash` 進入容器並繼續第二節。

## 二、啟動 Isaac Sim 與 A3 server［VS Code Server 的 Docker］

在 **VS Code Server Terminal 裡的 Docker**，確認提示字元為 `root@RTXWS`，再貼上完整指令：

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab

PYTHONPATH="/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_assets:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_tasks:/root/Documents/robotics/upstream/leisaac/source/leisaac${PYTHONPATH:+:$PYTHONPATH}" \
CUDA_VISIBLE_DEVICES=0 \
PUBLIC_IP=140.115.54.63 \
LIVESTREAM=1 \
ENABLE_CAMERAS=1 \
PYTHONUNBUFFERED=1 \
./isaaclab.sh -p \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/a3_sim_move_delta.py \
--livestream 1 --enable_cameras --ws-host 127.0.0.1 --ws-port 8766
```

不要關閉這個終端機。必須看到：

```text
A3 MOVE_DELTA SIM READY
A3 WS READY ws://127.0.0.1:8766
A3 initial EE base position [m]: [...]
Accepted: move_delta axis=x/y/z, dir=+/-1, step=coarse/fine; gripper action=open/close; estop/estop_reset
Motion profile: coarse=90 trajectory steps, fine=23 trajectory steps, stable=5, hold_limit=90
```

黃色的 `Performance warning` 不代表啟動失敗。若出現 Python traceback 或沒有 `A3 WS READY`，停在這一步處理。

## 三、開啟 WebRTC 畫面［Mac 瀏覽器］

使用原本連 RTXWS Isaac Sim 的 WebRTC 頁面。確認：

- 畫面能載入。
- 場景中看得到 SO-101。
- 手臂沒有自行持續移動或抖動。

後續測試時用這個畫面確認實際方向、夾爪與 Estop。

## 四、重新確認 Mac Wi-Fi IP［Mac 本機 Terminal］

在 **Mac 本機 Terminal，提示字元 `lichengyan@MacBookPro`** 執行。Mac 每次重新開機或換網路後都要重新查一次：

```bash
ipconfig getifaddr en0
```

目前曾取得：

```text
192.168.50.13
```

如果重開機後仍為 `192.168.50.13`，Thomas App 使用：

```text
ws://192.168.50.13:8765
```

如果數字改變，把新 IP 告訴 Thomas，App URL 也要跟著改。手機與 Mac 必須在允許裝置互相連線的同一個區域網路。

## 五、建立 SSH tunnel［Mac 本機 Terminal 1］

開啟一個新的 **Mac 本機 Terminal**。確認提示字元是 `lichengyan@MacBookPro`，再執行：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8766:127.0.0.1:8766 \
  leo@140.115.54.63
```

輸入 RTXWS 密碼。成功後終端機會停在空白畫面，這表示 tunnel 正在前景執行；不要關閉或在這個視窗繼續輸入其他命令。

若出現 `Address already in use`，代表 Mac 的 `8766` 已被舊 tunnel 佔用。先找出並結束舊 SSH tunnel，再重新執行，不能同時開兩條。

## 六、自我測試［Mac 本機 Terminal 2］

先不要啟動 relay。再開一個新的 **Mac 本機 Terminal**，確認提示字元是 `lichengyan@MacBookPro`：

```bash
cd "/Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta"
```

先用一組小幅度來回移動確認 Server、tunnel 和 Isaac Sim 都正常：

```bash
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis x --dir 1 --step fine
```

看到 `A3 MOVE_DELTA X PLUS FINE PASS` 後再執行回程：

```bash
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis x --dir=-1 --step fine
```

應看到：

```text
A3 MOVE_DELTA X MINUS FINE PASS
```

任何一條若出現 `FAIL`、`rejected` 或 timeout，停止測試，不要自動重送。先看 Server 終端機最後一段輸出。

## 七、啟動 phone relay［Mac 本機 Terminal 2］

自我測試通過後，在剛才執行自我測試的 **Mac 本機 Terminal 2** 執行：

```bash
../a3_mac_sim/.venv/bin/python mac_phone_relay.py
```

成功時應看到類似：

```text
MAC PHONE RELAY READY listen=ws://0.0.0.0:8765 upstream=ws://127.0.0.1:8766
Thomas App URL: ws://192.168.50.13:8765
One App connection; no command buffering or automatic retry.
```

不要關閉這個終端機。若 macOS 詢問是否允許 Python 接受連入連線，允許區域網路連線。

## 八、連線測試［Thomas 手機 App＋Mac relay Terminal］

1. Thomas 在 App 設定 WebSocket URL，例如 `ws://192.168.50.13:8765`。
2. App 建立連線並送出 `hello`。
3. Mac relay 應顯示：

   ```text
   [PHONE CONNECT] peer=(...)
   [RTXWS CONNECT] ws://127.0.0.1:8766
   ```

4. App 必須收到 `welcome`，接著持續收到約 5 Hz 的 `state`。
5. 如果 App 連不上，先確認 Mac IP、同一區網、macOS 防火牆與 relay 是否仍在執行。

## 九、正式功能驗收［Thomas App＋Mac WebRTC＋兩端輸出］

每個測試一次只按一個按鈕，等 App 收到該命令的最終 `ack` 後才執行下一個。WebRTC 畫面、App 顯示、Mac relay、Server 終端機四邊都要觀察。

### 9.1 方向與按鈕映射

先和 Thomas 寫下每個畫面按鈕對應的 `axis` 與 `dir`，不能只憑畫面名稱猜測：

| App 按鈕 | 預期封包 | WebRTC 實際方向 | 結果 |
|---|---|---|---|
| 上 | `axis=?, dir=?` | 待實測 | ☐ |
| 下 | `axis=?, dir=?` | 待實測 | ☐ |
| 左 | `axis=?, dir=?` | 待實測 | ☐ |
| 右 | `axis=?, dir=?` | 待實測 | ☐ |
| 前／後（若有） | `axis=?, dir=?` | 待實測 | ☐ |

對每一個支援方向做正向和反向一組。通過條件：

- App 收到相同 `seq` 的 `ack`。
- `ack.payload.status` 是 `ok` 或合理的 `clamped`。
- App 後續 `state.ee_pos` 有相符變化。
- WebRTC 中方向正確。
- 正反向完成後手臂回到接近起始位置。

### 9.2 coarse 與 fine

若 App 有模式切換，分別確認：

- `coarse` 每次平移目標為 0.020 m。
- `fine` 每次平移目標為 0.005 m。
- `move_delta.payload.step` 必須帶入目前選定的模式。
- App 顯示的模式與回傳 `state.step` 一致。

### 9.3 Gripper

依序測試：

1. 按 Open，App 收到 `ack status=ok`，`state.gripper=open`，WebRTC 看到夾爪張開。
2. 按 Close，App 收到 `ack status=ok`，`state.gripper=closed`，WebRTC 看到夾爪閉合。

### 9.4 快速連按與 busy 防護

1. 發出一個 coarse 移動。
2. 第一個移動尚未完成時再按另一個移動按鈕一次。
3. 第二個命令應收到 `ack status=rejected, reason=busy`。
4. 第二個命令之後不得延遲自行執行。
5. 第一個命令完成後，新的單次命令應可正常執行。

### 9.5 Estop、阻擋與 reset

1. 發出一個 coarse 移動。
2. 手臂正在移動時按 Estop。
3. WebRTC 中手臂應停止；App 應收到 `estop_engaged`，原移動應以 `reason=estop` 結束。
4. Estop 期間再按移動，必須收到 `reason=estop_active`，且手臂不能移動。
5. 按 Estop Reset，App 應收到 `reason=estop_reset`，且 `state.estop=false`。
6. Reset 後再做一個 fine 移動，確認控制恢復。

### 9.6 斷線測試

先用 Mac 測試程式驗證 Server 的斷線安全行為。這項測試要在 relay 尚未接受 Thomas App 連線時執行：

```bash
../a3_mac_sim/.venv/bin/python mac_disconnect_estop.py
```

必須看到：

```text
A3 DISCONNECT ESTOP PASS
```

接著以 App 驗證同一路徑：

1. 發出 coarse 移動，手臂正在移動時讓 App 主動斷線。
2. Mac relay 應顯示 `[PHONE DISCONNECT]`，Server 應進入 Estop，手臂停止。
3. 斷線後不得有舊命令自行執行。
4. App 重新連線後重新送 `hello`，收到的 `state.estop` 應為 `true`。
5. App 送 `estop_reset` 並收到 `state.estop=false` 後，才可恢復移動。
6. 新連線的 `seq` 重新開始後，要在該連線內持續遞增，不可重送同一個 `seq`。

## 十、保存驗收證據［Mac＋VS Code Server Docker］

至少保存：

- WebRTC 畫面錄影，包含按鈕動作、手臂反應與 Estop。
- Thomas App 的 `welcome/state/ack/error` 畫面或 log。
- Mac relay 終端機輸出。
- Mac relay JSONL：

  ```text
  /Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta/mac_phone_relay.jsonl
  ```

- RTXWS 容器內 Server JSONL：

  ```text
  /root/Documents/robotics/data/network_logs/a3_move_delta.jsonl
  ```

在 **VS Code Server 的 Docker Terminal，提示字元 `root@RTXWS`** 查看 Server 最後 50 筆紀錄：

```bash
tail -n 50 /root/Documents/robotics/data/network_logs/a3_move_delta.jsonl
```

## 十一、完成條件

A3 只有在以下項目都留下證據後才能標記完成：

- ☐ Thomas App 經 Mac relay 與 SSH tunnel 收到 `welcome/state`。
- ☐ App 的所有支援方向按鈕與 WebRTC 實際方向一致。
- ☐ coarse/fine（若 App 有提供）符合 20 mm／5 mm。
- ☐ Gripper open/close 通過。
- ☐ 快速連按時第二個命令被 busy 防護拒絕且沒有延遲執行。
- ☐ Estop 能中斷移動、阻擋新移動並成功 reset。
- ☐ 移動中斷線會自動 Estop、不會重播舊命令，重新連線會重新 hello。
- ☐ App、Mac relay、Server JSONL 與 WebRTC 影片都已保存。

## 十二、正常結束［依執行位置操作］

1. **Thomas 手機 App**：先斷線。
2. **Mac 本機 relay Terminal**：按 `Ctrl-C`。
3. **Mac 本機 SSH tunnel Terminal**：按 `Ctrl-C`。
4. **VS Code Server 的 Docker／Isaac Sim Terminal**：按 `Ctrl-C`，等待程式回到 `root@RTXWS` shell。
5. **VS Code Server 的 Docker Terminal**：輸入 `exit` 離開容器。
6. **VS Code Server host Terminal，提示字元 `leo@RTXWS`**：若當天不再使用容器，執行：

   ```bash
   docker stop isaac-sim
   ```

RTXWS 是實驗室共用主機時，關閉整台主機前先依實驗室規則確認沒有其他人使用。

## 常見錯誤定位

| 現象 | 先檢查 |
|---|---|
| Mac client 顯示 connection refused | Isaac Sim 是否有 `A3 WS READY`、SSH tunnel 是否仍在執行 |
| Relay 顯示 `[RTXWS UNAVAILABLE]` | SSH tunnel 是否斷線、Mac `8766` 是否正確轉送 |
| App 連不到 relay | Mac IP 是否改變、手機與 Mac 是否同網、macOS 防火牆 |
| App 已連線但沒有 welcome | App 是否先送符合 v0 schema 的 `hello` |
| `reason=busy` | 上一個命令尚未完成；等待最終 ack，不要自動重送 |
| stale seq／命令無回應 | Thomas App 的 `seq` 是否在同一連線中遞增 |
| `position_tolerance_not_met` | 停止自動測試，查看 Server 的 start/target/end/error |
| WebRTC 看不到動作但 ack=ok | 核對是否看對場景、比較 state.ee_pos 與 Server log |
