# S-3 示範資料記錄：啟動說明

更新日期：2026-09-08

## 任務範圍

Notion 票號：S-3「示範資料記錄：相機視角＋狀態＋動作，先落 HDF5」

- Owner：Leo
- 截止日：2026-09-11
- 目前狀態：相機 smoke test 實作完成，等待 RTXWS 實機執行
- DoD：至少一個 episode 可完整重播；action 對到正確 observation 與影格；時間序單調；影像與低維狀態筆數一致；action 無缺值。

## 已確認的前置狀態

- A3 已能從 Thomas App 經 Mac relay 控制 Isaac Sim SO-101。
- S-4 固定方向 pick-and-place 已完成成功判定與成功後自動 reset。
- Thomas 的 R-4 LeRobot dataset 摘要已完成；S-3 固定使用 30 Hz、每步一筆，episode 邊界為 reset 完成到 success，正式輸出只保留成功 episode。

因此 S-3 可以直接在現有 S-4 模擬程式中加入相機與 recorder。HDF5 以可重播的六關節 target 為主要 action，另外保留目前低維操作者控制意圖，供後續 shared-autonomy 分析。

## 實作決定

S-3 第一版使用「現有 A3／S4 模擬程式內的小型 recorder」，不先改用 Isaac Lab `record_demos.py`。

理由：

1. 現有 A3 是直接建立 `SimulationContext`、`Articulation` 和 Differential IK，不是 `record_demos.py` 所需的 manager-based Gym environment。
2. `record_demos.py` 的流程是 device `advance()` → `env.step(actions)` → Recorder Manager；直接套用前必須先建立 SO-101 task、action configuration 和 custom device registration。
3. S-3 的 2 pt DoD 是先得到一個可同步、可重播的 HDF5 episode。小型 recorder 能沿用已驗收的 A3 控制路徑，並保留之後轉成 LeRobot 所需的原始資料。

HDF5 仍採 Isaac Lab 慣例：

```text
data/
  demo_0/
```

每個 timestep 同時保存 observation、RGB 影像、原始操作者命令，以及實際送入模擬器的 joint target。詳細欄位見 `DATA_CONTRACT_DRAFT.md`。

## 執行順序

1. 在 S-4 程式執行固定 RGB camera smoke test，確認能讀到 `camera.data.output["rgb"]` 並輸出有效 PNG。
2. 加入 episode start／stop／discard 控制與 HDF5 writer。
3. 錄一個成功 episode。
4. 用 `validate_s3_hdf5.py` 檢查筆數、時間戳、影像、有限 action 與 metadata。
5. 用 `actions/joint_target` 在相同初始狀態重播，確認動作與影格時間對齊。
6. 將最終欄位對應補進文件並交給 Thomas 做 HDF5 → LeRobot 轉換。

## 第一階段成功條件

- Thomas 已確認 LeRobot v3、30 Hz、每步一筆與影像 key。
- 相機能在 RTXWS Docker 內輸出一張非全黑、非全白的 RGB 影像。
- HDF5 writer 能寫出 `data/demo_0`，並正常關閉檔案。

## RTXWS 相機 smoke test 結果

- 2026-09-09 已在 Isaac Sim Docker 實測通過。
- 輸出：`/root/Documents/robotics/data/s3_camera/front_rgb_smoke.png`
- 格式：`uint8 [480,640,3]`
- 畫面包含 SO-101、紅方塊、綠色目標區與完整操作桌面。

## Recorder 啟動參數

正式啟動 S-4 時加入：

```text
--enable_cameras --record-demos
```

預設輸出：

```text
/root/Documents/robotics/data/demos/s3_s4_demos.hdf5
```

Recorder 在場景 reset 後先進入 armed 狀態；第一個非零操作者命令成為 frame 0，之後的正常停頓仍會保留。資料先寫入暫存 group，成功判定後才發布為 `data/demo_N`。Estop、場景手動 reset、方塊掉落或程式關閉會捨棄進行中的 episode。

舊 episode 若包含很長的起始等待，可用 `trim_s3_hdf5.py` 複製成新檔；預設保留第一個有效操作前 15 筆（0.5 秒），原檔不會被覆寫。

在上述三項完成前，不把臨時 action 維度宣稱為最終 LeRobot contract。

產生第一個 HDF5 後，在 RTXWS Docker 的 Isaac Lab Python 環境執行：

```bash
./isaaclab.sh -p /path/to/validate_s3_hdf5.py /path/to/episode.hdf5
```
