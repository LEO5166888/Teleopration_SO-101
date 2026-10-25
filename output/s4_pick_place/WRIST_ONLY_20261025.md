# AR 前後傾只控制手腕：待模擬器驗證

> 已由 `EXPLICIT_AR_MODES_20261025.md` 的 explicit_v2 取代。
> 下文保留為歷史紀錄；目前不再以角度門檻自動切換。

## 證據與範圍

seq3612、3812 的前後傾操作同時送入公分級 p_rel_m 與 q_rel。
舊 pose 分支同時執行位置 IK 與手腕目標更新，因而會帶動肩膀、手肘。
這次保留已校正的平移映射 `base = [x, y, -z]`，不改手機網頁。

## 行為

- 每次按住開始時，預設仍可平移。
- App 傳來的相對 quaternion 轉成旋轉向量後，X 分量達 1 度時進入手腕模式。
  這是 App 處理後的角度，不等於原始手機傾角。實際 pitch 軸與正負仍需實測確認。
- 進入後固定當下所有其他關節的**目標值**，只更新 wrist_flex。
  完全跳過平移 IK，忽略後續平移和夾爪開合輸入，回正也不退出此模式。
- 放開再按才重新允許平移。觸發門檻前仍可能有短暫平移；平移時若誤傾超過門檻也會鎖定。
  這是自動判斷的限制，不能僅靠這批輸入保證辨識人的意圖。
- 保留原有 wrist flex 正負號、35 度相對限幅、soft joint limits、每步 0.01 rad 限速。
  保留 watchdog、手動急停與非有限值檢查。手腕模式不對舊 XYZ 位置做追蹤誤差檢查，
  因為手腕旋轉會使 gripper 原點／指尖自然畫弧。
- 未進入手腕模式時，subthreshold pitch 不動手腕；原有 roll 控制與獨立陀螺儀模式保留。
- 固定目標不是物理鎖死，仍可能有重力／接觸造成的位移，需用實際關節紀錄確認。

## 備份與本機驗證

修改前完整備份：`../s4_translation_calibrated_before_wrist_20261025_fOXPnP/controller.tar.gz`。

```sh
python3 -m unittest -v test_pose_wrist_only.py test_phone_translation_20261025.py test_s4_task_logic.py test_s4_jog_transport.py
python3 -m py_compile s4_task_logic.py s4_sim_pick_place.py
```

單元測試涵蓋：六方向映射、pitch 門檻、只改 flex、其他目標固定、回正仍鎖定、
新按住可重設、純 roll/yaw 不觸發、角度與步長限制、非法輸入、正負號與等價 quaternion。
本機測試不代表 Isaac 實測通過。

## 部署與實測

先在執行模擬器的終端機 Ctrl+C。Mac 執行：

```sh
cd "/Users/lichengyan/Documents/ChatGPT/碩士論文/output/s4_pick_place"
zsh deploy_base_pose_controller.zsh
```

Server 的 `leo@RTXWS:~$` 執行：

```sh
docker exec -it isaac-sim bash -lc 'bash /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/start_s4_base_pose.sh'
```

如果已在 `root@RTXWS:/isaac-sim#` 容器內，直接執行 `bash /root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/start_s4_base_pose.sh`。

啟動須看到 `pitch_mode=wrist_only_latched_v1` 及 SIM READY / WS READY。
維持現在可用的 Mac HTTPS server 與 SSH tunnel，不另開 relay。
手機重新連線，放開控制鍵後重新按住，前傾、停住、回正、後傾、放開。
提供 `[S4 POSE WRIST-ONLY START]`、`[S4 POSE WRIST-ONLY]`、`S4 POSE STOP`。
核對 pitch_x_deg、wrist_flex_target_deg、wrist_flex_actual_deg 及 arm_actual_delta_deg。
若沒出現 WRIST-ONLY，先確認 q_rel 軸與門檻；不要再盲目翻轉平移方向。
完成手腕測試後，放開再按，回歸測試左右、前後、上下。
