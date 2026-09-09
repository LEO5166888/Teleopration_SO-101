# S-3 HDF5 資料契約 v1

狀態：S-3 錄製契約已凍結；後續由 Thomas 的轉檔程式映射到 LeRobot v3。

## 時間對齊規則

每一列 `t` 表示同一個控制 timestep：

1. 讀取 `observation[t]` 與同一時刻的 RGB 影像。
2. 保存接下來一個控制區間要執行的 `action[t]`。
3. 將 `action[t]` 送入模擬器並前進至下一個 timestep。

因此 `observation[t]` 是執行 `action[t]` 前的狀態。`timestamp[t]` 使用 simulator time，不以 Mac／手機 wall clock 當作主要同步基準。

## HDF5 結構

```text
data/
  attrs:
    total
    env_args
  demo_0/
    attrs:
      num_samples
      success
      schema_version
      fps
      task
      joint_names
      ee_frame
      quaternion_order
    timestamps/
      sim_time_s            float64 [N]
      frame_index           int64   [N]
      source_command_ts_ms  int64   [N]
      source_command_seq    int64   [N]
    obs/
      joint_pos             float32 [N, 6]
      joint_vel             float32 [N, 6]
      ee_pose               float32 [N, 7]
      front_rgb             uint8   [N, H, W, 3]
    actions/
      operator_delta        float32 [N, 5]
      joint_target          float32 [N, 6]
    status/
      estop                 bool    [N]
      command_type          int8    [N]
      step_mode             int8    [N]
```

## 欄位定義

| 欄位 | 單位／順序 | 用途 |
|---|---|---|
| `sim_time_s` | 秒，嚴格遞增 | observation、影像與 action 的主要同步時間 |
| `frame_index` | `0..N-1` | episode 內 frame 編號 |
| `source_command_ts_ms` | 手機封包毫秒；無新封包時沿用目前命令 | 保存來源時間，不用於模擬同步 |
| `source_command_seq` | WebSocket seq | 追查一個按鈕命令涵蓋哪些 frame |
| `joint_pos` | rad；六關節固定順序 | SO-101 實際關節位置 |
| `joint_vel` | rad/s；同一關節順序 | SO-101 實際關節速度 |
| `ee_pose` | `[x,y,z,qw,qx,qy,qz]`；m；base frame | 實際末端位姿 |
| `front_rgb` | HWC、RGB、uint8 | 固定視角相機影像 |
| `operator_delta` | `[dx,dy,dz,d_wrist_flex,d_gripper]` | 下一個 30 Hz 控制區間的低維增量；平移為 base-frame m，關節為 rad |
| `joint_target` | rad；六關節固定順序 | 實際送給 articulation 的 actuator target，用於第一版重播 |
| `estop` | bool | 該 timestep 是否處於 Estop |
| `command_type` | 暫定 `0=hold,1=move_delta,2=gripper` | 避免可變長字串干擾轉檔 |
| `step_mode` | 暫定 `0=none,1=fine,2=coarse` | 保存 5 mm／20 mm 模式 |

六關節建議順序：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

## 為什麼同時保存兩種 action

- `operator_delta` 保存 Thomas App／人的低維意圖，之後可用於 shared-autonomy 與行為克隆研究。
- `joint_target` 保存真正送入 Isaac Sim 的控制 target，可先完成 S-3 的完整重播 DoD。

只存手機按鈕事件會失去內插期間的實際執行動作；只存 joint target 又會失去操作者的原始控制意圖，所以第一版兩者都保留。

## Episode 邊界

- `start`：場景 reset 完成、相機已有有效影像、手臂在固定初始姿態後開始。
- `success stop`：物體進入 S-4 目標區且成功判定成立。
- `discard`：Estop、App 斷線、IK／執行失敗、手動取消或超時。
- 第一版正式輸出只保留 successful episode；失敗 episode 可另存 debug log，不混入訓練資料。

## DoD validator 必查項目

1. 所有 timestep dataset 第一維都等於 `N=num_samples`。
2. `sim_time_s` 全部有限且嚴格遞增。
3. `frame_index` 恰為 `0..N-1`。
4. `front_rgb` 為 `[N,H,W,3]`、dtype `uint8`，且影像不是全黑或全白。
5. `joint_pos`、`joint_vel`、`ee_pose`、`operator_delta`、`joint_target` 都沒有 NaN／Inf。
6. `success=true`。
7. 從固定初始狀態重播 `joint_target` 時，episode 能完成同一任務。

## 與 LeRobot v3 的轉檔邊界

- S-3 固定以 30 Hz 錄製 HDF5，每個 timestep 一筆狀態、RGB 與 action。
- `joint_target` 是 S-3 重播用的標準 action；`operator_delta` 保留低維人類意圖。
- Thomas 的轉檔程式決定 LeRobot 主 `action` 的選用方式，並由 LeRobot 自動產生 timestamp、frame index、episode index、global index 與 task index。
- RGB 固定為 HWC、uint8；LeRobot 端再轉為 `observation.images.front` 影片資料。
