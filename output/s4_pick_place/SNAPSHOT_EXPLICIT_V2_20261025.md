# 2026-10-25 可用版本快照：AR 平移／手腕分離

保存原因：Leo 在手機與 Isaac Sim 實際操作後回報「現在順好多了」，要求先保存此版本。
來源為目前工作目錄的檔案，不包含其他簡報、研究文件或歷史 TCP 實驗。

- GitHub：`LEO5166888/Teleopration_SO-101`
- 保存分支：`codex/ar-explicit-modes-20261025`
- Server：`output/s4_pick_place/`
- Mac 網頁與 HTTPS/WSS relay：`output/teleop_app_sp6/`
- 操作行為標記：`control_modes=explicit_v2 default=translation`
- 方向映射保持 `base = [x, y, -z]`，適用目前已校正的手機設定。
- App 改為明確選擇平移／手腕，切換先停，重新按住才開始；不再用傾角自動切換。
- 本快照包含工作目錄原有的錄製支援，沒有用舊版 recorder 覆蓋目前檔案。
- Thomas 的 `WIDM/teleop-app` 未變更。

驗證：53 項 Python、4 項 JavaScript 模式測試，以及隔離 Chrome 頁面切換測試通過。
手機尺寸畫面已檢視；使用者回報手感改善。完整任務成功率、放開後漂移、工作範圍邊界仍待量測。

## 重新取得此版

在新的空目錄 clone，避免覆蓋目前尚未保存的其他工作：

```sh
git clone --branch codex/ar-explicit-modes-20261025 --single-branch \
  https://github.com/LEO5166888/Teleopration_SO-101.git teleop-explicit-v2
```

Server 必須有原本 Isaac Sim／IsaacLab／LeIsaac 環境。
部署及手機重新整理步驟見 [操作說明](EXPLICIT_AR_MODES_20261025.md)。
Mac 頁面請從這份 checkout 的 `output/teleop_app_sp6` 啟動：

```sh
python3 serve_https.py --upstream ws://127.0.0.1:18766
```

18766 是目前 Mac tunnel 的本機埠，轉送 Server 8766；若用其他本機埠需同步調整。
`serve_https.py` 需要支援 `websockets.asyncio` 的 Python 環境；目前 Mac 使用 websockets 16。
HTTPS 憑證、私鑰與既有資料集未上傳，首次啟動依原流程建立本機憑證。
