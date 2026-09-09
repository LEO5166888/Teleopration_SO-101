# SO-101 Shared Autonomy Thesis Workspace

這個 repository 保存碩士論文研究中的 SO-101、Isaac Sim、Mac 遠端控制、示範資料記錄與 shared autonomy 實作。

## 目前內容

- `output/a3_mac_sim/`：Mac 到 Isaac Sim 的初始連線與 smoke test。
- `output/a3_move_delta/`：六方向、夾爪、急停、relay 與鍵盤控制。
- `output/s4_pick_place/`：固定場景 pick-and-place、按住移動、成功判定與自動 reset。
- `output/s3_hdf5/`：30 Hz RGB／狀態／動作 HDF5 contract、驗證與裁切工具。
- `output/presentations/`：研究簡報。

HDF5 示範資料、影片、執行 log、Python 虛擬環境、字型與建置暫存不放入 Git；資料集保存在 RTXWS 的持久化資料目錄。

## 更新 GitHub

在 workspace 根目錄執行：

```bash
./update_github.sh "本次修改摘要"
```

腳本會加入變更、建立 commit，並推送目前分支。第一次使用前必須先設定 Git 作者資訊與 `origin` GitHub remote。
