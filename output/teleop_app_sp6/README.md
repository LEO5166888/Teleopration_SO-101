# teleop-app

> Leo 本機測試分支（2026-10-25）：AR 頁面新增「平移／手腕」明確切換。
> 目前啟動、部署與測試請看 [explicit_v2 說明](../s4_pick_place/EXPLICIT_AR_MODES_20261025.md)。
> 此副本基於 Thomas 的 App；下方保留原說明，其中 stub_server.py 與 docs/ 不包含在這份副本。
> 此版需搭配同一快照的 S4 Server；HTTPS relay 連到 Mac SSH tunnel，無需另外啟動 8765 relay。

手機語音遙操作碩論的 Web App 前端（Thomas）。連線狀態、WebSocket client、信封格式（介面協定 v0）、按住才動的控制（六方向、手腕、夾爪）、急停與解除、ack 回饋。

- 訊息格式合約：《介面協定 v0 Message Schema》（已凍結 2026-09-02，改版走 PR）＋ press-and-hold 擴充：[docs/hold-protocol.md](docs/hold-protocol.md)（Leo 2026-09-09）
- 純靜態頁，無 build 步驟；`stub_server.py` 是本地測試用的假 server
- `orientation.html`：手機姿態 demo（Euler 角與四元數），要用 `python serve_https.py` 開（感測器只給 HTTPS 頁面），手機開 `https://<電腦IP>:8443/orientation.html`，憑證警告按「仍要前往」。給學習票 K-1 與十月陀螺儀票用（姿態模式本身見下方「姿態模式」）

## Demo SOP

前置：一台電腦（跑頁面與 stub）、一支手機、兩者同一個 Wi-Fi。

1. 電腦上啟動假 server（stub）：

   ```bash
   pip install websockets
   python stub_server.py
   ```

2. 電腦上另開一個終端機，啟動靜態頁伺服器：

   ```bash
   python -m http.server 8000
   ```

3. 查電腦的區網 IP（macOS：`ipconfig getifaddr en0`）。
4. 手機瀏覽器開 `http://<電腦IP>:8000`。server 欄會自動帶 `ws://<電腦IP>:8765`，按「連線」。
5. Demo 動線：
   - 連上後右上角變綠色「已連」，紀錄區看到送 `hello`、收 `welcome`
   - 按「送 test」：收到原樣 echo
   - 閘門顯示「可操作」後（收到 `welcome` 且 `state.estop=false`）動作鍵才會亮
   - **按住**方向鍵（前後上下左右）：送 `jog_start`，按住期間每 150 ms 送 `jog_keepalive`，放開送 `jog_stop`；按住中按鈕有黃框，放開後收到兩個 `ack`（stop 的、start 的最終）
   - 按住「手腕 ↑／↓」：`joint_jog_start`／`joint_jog_stop`；按住「夾爪 開／關」：`gripper_jog_start`／`gripper_jog_stop`
   - 「微調（0.5 cm）」切「粗調」：之後的 start 訊息帶 `step:"coarse"`；預設微調
   - 「急停」：連線中隨時可按，按下先停 keepalive 再送 `estop`；閘門變「急停中」，按「解除急停」送 `estop_reset`
   - App 進入背景、視窗失焦、離開頁面、斷線：按住立即結束並送 stop（斷線時 server 端 watchdog 750 ms 接手）
   - 關掉 stub：狀態變「斷線重連中」；重開 stub 自動連回並重新握手
6. 連不上排查：macOS 防火牆詢問要按允許；確認手機跟電腦同網段（訪客 Wi-Fi 常是分開的）。

## 畫面配置

一個螢幕放得下、不用捲動。上方是連線狀態與模式分頁（按鈕／陀螺儀／AR，一次只顯示一種，會記住上次用的）；連線設定連上後自動收起。急停固定在最下方，急停中旁邊才出現「解除急停」。姿態參數與訊息紀錄收在「⚙ 設定」裡。切換分頁時若正按住，會先送 stop。

## 姿態模式（陀螺儀）

按住「按住轉手機」時，用手機轉動控制末端轉向；平移照舊用方向鍵。協定草案（待 Leo 確認）見 [docs/hold-protocol.md](docs/hold-protocol.md) 的 Orientation 一節。

手機只在 HTTPS 頁面給陀螺儀資料，而 HTTPS 頁面又不能連 `ws://`，所以這個模式改用 `serve_https.py`：同一個 port 8443 同時給頁面和 `wss://…/relay`，relay 把訊息原封不動轉給後面的 server。需要 `websockets` 13 以上。

1. 終端機 1：`python3 stub_server.py`
2. 終端機 2：`python3 serve_https.py`（轉給本機 stub）；要接 Leo 的 server 改成 `python3 serve_https.py --upstream ws://<Leo IP>:8765`
3. 手機開 `https://<電腦IP>:8443`，憑證警告按「仍要前往」（Safari：顯示詳細資訊 →「瀏覽此網站」）。server 欄會自動帶 `wss://<電腦IP>:8443/relay`，按連線
4. 切到「陀螺儀」分頁，按「啟用陀螺儀」（iPhone 會跳權限詢問，按允許），按鈕換成「按住轉手機」，小字是「就緒，按住才生效」
5. 按住「按住轉手機」再轉手機：小字顯示目前轉了幾度，stub 的 state 列顯示「姿態 N°」；放開即停
6. 增益、死區、平滑在「⚙ 設定」裡調，按下那一刻讀取，會帶進 `orient_start`

安全行為跟方向鍵一致：一次只能按住一個、放開／背景／失焦／斷線／急停都會送 `orient_stop`；另外陀螺儀 300 ms 沒資料也會自動停。

## AR 模式（WebXR，手機移動帶動整隻手臂）

同一個頁面的「AR」分頁，用支援 WebXR AR 的瀏覽器開才會亮起「進入 AR」：iPhone 用 XRViewer，Android 用 Chrome（Safari 沒有 WebXR AR）。啟動方式同姿態模式（`serve_https.py`），手機開 `https://<電腦IP>:8443/`，連線後切到「AR」分頁按「進入 AR」。

- 按住控制區：手機移動，手臂末端跟著動（IK 帶動整隻手臂）；放開即停
- 夾爪用 AR 畫面下方的「夾爪 開／關」鈕：按住控制區時可以用另一指同時按（1 秒從不動到全開或全關）；沒按住控制區時就是一般的夾爪按住控制
- AR 畫面上有急停、解除急停、重置場景、結束 AR；重置場景在急停中也能按（主畫面也是），收不收由 server 決定
- 追蹤遺失、手機突然動太快（例如掉落）、AR 被結束都會自動送 `pose_stop`
- 協定：`pose_start`／`pose_update`／`pose_stop`，見 [docs/hold-protocol.md](docs/hold-protocol.md) 的 Pose 一節

## WebXR 追蹤測試（AR 對照組可行性）

`xr-test.html`：確認手機＋瀏覽器拿不拿得到 6 自由度姿態（位置＋轉向），做法同 TidyBot++（WebXR `immersive-ar`）。照姿態模式的步驟開 `serve_https.py`，手機開 `https://<電腦IP>:8443/xr-test.html`。iPhone 要用 XR Browser 類的 App（Safari 沒有 WebXR AR），Android 用 Chrome。按「開始 AR」後，按住紫色區把手機直直移 20 cm，看「距離」準不準。

## 對接 Leo 的 server（取代 stub）

姿態模式與 HTTPS 轉接的對接、Leo 自己測試的步驟：[docs/handoff-leo.md](docs/handoff-leo.md)

Leo 的 WebSocket server 起在跑 Isaac Sim 的那台機器上。對接當天：

1. 跟 Leo 要兩個資訊：那台機器的區網 IP 與 server port。
2. 手機連實驗室 Wi-Fi，照 Demo SOP 開頁面，server 欄改填 `ws://<那台機器IP>:<port>`，按連線。
3. 跑同一組驗收：`hello` 收 `welcome`、`test` 收 echo、按鈕指令收 `ack`，他的 server log 看得到正確 JSON。
4. App 程式碼不用改，換網址即完成對接；stub 之後只在單機開發時用。

