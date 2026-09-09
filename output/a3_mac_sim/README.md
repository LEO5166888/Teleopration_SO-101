# A3：Mac 命令讓 Isaac Sim SO-101 上移一次 2 cm

這是第一個通路測試：Mac client → SSH tunnel → 獨立 WebSocket 接收器 → queue → Isaac 主執行緒 IK。接收器與 sim adapter 位於同一個模擬程式內，沿用使用者提供的 S2 場景、位置型 IK、Jacobian 座標轉換、重力補償與關節限幅。不是既有 server_v0.py 的修改版。

先前 N-1 使用 8765；本測試使用 8766。測試封包保留 v/type/ts/seq/payload 信封，臨時 type=test_move_up、payload={}，固定要求 base frame +Z 0.02 m。尚未定義 Thomas 的正式 move_delta、六方向、夾爪或 estop 協定，因此不能將本次通過當作整張 A-3 完成。

## Server（VS Code 終端機）

1. 在先前執行 S2 的容器中操作。若 S2 viewer 還在執行，先回它的終端機按一次 Ctrl+C，等結束；不要同時啟動兩個 WebRTC viewer。
2. 開啟隨附 install_server_short.sh（縮短版），複製整份內容貼到 VS Code 的 Server 終端機。它只會建立 scripts/a3_mac_smoke 內兩份新程式；若檔案已存在且內容不同，會停止，不會覆蓋。
安裝指令使用 `./isaaclab.sh -p -` 選擇 Isaac Lab 的 Python，不依賴容器 PATH 中有 `python3`。縮短版與完整的 `install_server.sh` 產生相同兩份程式；縮短版會先檢查內容雜湊，貼上不完整時不寫入檔案。

3. 在 Isaac Lab 的 Python 環境確認 websockets：

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab
./isaaclab.sh -p -c 'import websockets; print(websockets.__version__)'
```

若出現 `ModuleNotFoundError: websockets`，在同一環境安裝本機驗證過的版本：

```bash
./isaaclab.sh -p -m pip install 'websockets==12.0'
```

4. 啟動以下程式。PUBLIC_IP 沿用先前提供的 S2 啟動值；若你後來成功使用的是別的位址，改成該值。WebRTC 沿用 S2 的觀看方式。

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab
CUDA_VISIBLE_DEVICES=0 \
PUBLIC_IP=140.115.54.63 \
LIVESTREAM=1 \
ENABLE_CAMERAS=1 \
PYTHONUNBUFFERED=1 \
./isaaclab.sh -p \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_mac_smoke/a3_sim_up.py \
--livestream 1 --enable_cameras --ws-host 127.0.0.1 --ws-port 8766
```

等待 `Streaming server started`、`A3 SIM READY` 和 `A3 WS READY ws://127.0.0.1:8766`。啟動時手臂會先回初始姿態並穩定，之後不會自行往返，直到收到命令。

5. 收到命令後應看到 `[A3 EXEC]`、`[A3 MOVE]`，結束時得到 `A3 SIM UP 2CM PASS` 或 FAIL，以及起點、目標、終點。模擬仍會運行並維持最後的關節目標，方便觀看與錄影。重做測試需停止並重新啟動此程式。

JSONL：`/root/Documents/robotics/data/network_logs/a3_smoke.jsonl`。Server log 與 `A3 SIM UP 2CM PASS` 可作為 Server 端截圖。注意觀察的是原程式使用的 gripper body 原點，不是另行校正的指尖 TCP。

## Mac（本地終端機）

1. 新開一個終端機建立 tunnel，讓它保持執行：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8766:127.0.0.1:8766 \
  leo@140.115.54.63
```

成功通常沒有輸出。若出現本地 address already in use，檢查之前是否已有這個 tunnel，勿再開重複 tunnel。

此命令延用先前 N-1 的「SSH 主機 loopback 可達容器服務」前提。8765 可用不保證新 port 8766 也可用。如果建立 tunnel 後，client 連線造成 SSH 印出 `connect failed: Connection refused`，先回傳兩側輸出，確認容器是否共用 host network 或只有映射 8765；這個錯誤不表示 IK 失敗。不要為了測試重建容器或修改防火牆。

2. 新開第二個 Mac 終端機執行 client。本機已建立此獨立虛擬環境並安裝 websockets 12.0：

```bash
cd /Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_mac_sim
./.venv/bin/python mac_test_up.py
```

若移到其他 Mac，先以 `python3 -m venv .venv` 建立環境，再以 `./.venv/bin/python -m pip install 'websockets==12.0'` 安裝。

client 會先驗證 welcome 的 protocol/ready/used，再送一次命令。ACK 只表示接收，client 會等待模擬主迴圈傳回量測，重新計算位移誤差，符合條件才印出：

```text
A3 MAC -> SSH -> SO101 UP 2CM PASS
```

請截圖這個 Mac client 終端機，需含 start_m、target_m、end_m、actual_delta_m、position_error_m；另外錄下 WebRTC 裡手臂向上移動。失敗或逾時時不要自動重送，先看 Server log。本版每次啟動只接受一筆移動，斷線或 client 重啟也不會再次排入移動。

驗收：起點到目標為 base-frame +Z 2 cm，完整三維位置誤差 ≤ 5 mm，Server 結束前至少連續 30 個模擬步符合位置門檻。這是位置門檻，未另外驗證速度收斂。控制採 180 個模擬步漸進移動、180 步保持；有 45 秒執行期限，負載過重也可能逾時。

已完成本機驗證：Python 語法檢查；websockets 12.0 loopback 的 5 組測試，包括錯誤封包、未 ready、ACK 與結果分離、一次性命令、防斷線重送、假 sim 回覆、零位移／錯方向／NaN／缺值拒絕。

尚未完成：RTXWS/SSH 新 port 可達性、Isaac Lab 啟動、GPU/物理動作、WebRTC 觀看和實際上移結果。本機測試使用替代 sim worker，不構成 SO-101 動作通過證據。

通訊 API 參考：[websockets 12.0 server](https://websockets.readthedocs.io/en/12.0/reference/asyncio/server.html)、[client](https://websockets.readthedocs.io/en/12.0/reference/asyncio/client.html)。Isaac 控制部分依使用者提供的 S2 原始程式改寫。
