// app.js：手機端遙操作 App 的全部邏輯，單一檔案。
//
// 讀法：照下面十一個區塊的順序讀，每一區開頭都有一段白話說明。
//   1. 信封工具        每則訊息共用的外層格式
//   2. 連線狀態與閘門  三種連線狀態、什麼時候才允許按方向鍵
//   3. 畫面與紀錄      狀態列、按鈕啟停、訊息紀錄
//   4. 送出與 ack 對帳 送出去的訊息怎麼跟 server 的回覆配對
//   5. 按住控制        按下、按住、放開三個階段各送什麼
//   6. WebSocket 連線  從開頁到斷線重連的完整生命週期
//   7. state 廣播      server 每秒五次回報手臂狀態，急停閘門靠它
//   8. 按鈕綁定        把上面的機制接到畫面上的每一顆按鈕
//   9. 姿態模式（IMU） 按住「姿態」鈕時，用手機轉動控制末端轉向
//  10. AR 模式（WebXR）按住控制區時，手機移動帶動整隻手臂，夾爪用下方按鈕
//  11. 畫面模式與設定 三個分頁一次只顯示一種；設定收在對話框裡
//
// 兩條主線貫穿全檔，讀的時候盯著這兩條走：
//   連線生命週期：開頁 → 按連線 → onopen 送 hello → 收 welcome → 收 state → 閘門打開
//                 → 斷線 onclose → 自動重連（1、2、4、8 秒）→ 回到 onopen
//   按住控制：pointerdown 送 *_start → 每 150 ms 送 jog_keepalive → pointerup 送 *_stop
//             任何「失去控制」（背景、失焦、斷線、急停）都走 endHold 收尾

// ===== 1. 信封工具：所有訊息共用（介面協定 v0 ＋ hold 擴充 2026-09-09）=====
// 每則送出去的訊息都長這樣：{ v, type, ts, seq, payload }
//   v        協定版本；server 收到不認得的版本會拒收
//   type     訊息種類（hello、jog_start、estop…）；server 靠它決定怎麼處理
//   ts       手機端送出時間（毫秒）；server 拿來量延遲
//   seq      流水號，每送一則加一；ack 回來時靠它對回是哪一則
//   payload  這種訊息真正的內容；沒有就是空物件
const PROTOCOL_VERSION = 1;
let seq = 0; // 全程遞增、不重用（含 keepalive 與 stop）

function makeEnvelope(type, payload = {}) {
  seq += 1;
  return { v: PROTOCOL_VERSION, type, ts: Date.now(), seq, payload };
}

// ===== 2. 連線狀態與動作閘門 =====
// 連線只有三種狀態，顯示在最上面的狀態列：
//   未連        還沒按連線，或使用者主動按了斷線
//   已連        WebSocket 開著，可以送訊息
//   斷線重連中  使用者要連但目前沒連上（第一次連線中，或斷線後自動重連）
//
// 「已連」不等於「可以動手臂」。方向鍵要三個條件同時成立才啟用（見 motionAllowed）：
//   已連 ＋ 收過 welcome ＋ server 最近一次 state 說 estop=false
// 這三個變數在 setState 離開「已連」時會一起清空，重連後要重新拿一次。
const STATE = {
  DISCONNECTED: "未連",
  CONNECTED: "已連",
  RECONNECTING: "斷線重連中",
};

let ws = null;
let state = STATE.DISCONNECTED;
let wantConnected = false; // 使用者按過連線且沒按斷線
let reconnectTimer = null;
let reconnectDelay = 1000; // ms，指數退避到 8 秒封頂

// 動作閘門：要 welcome 收到、且 state.estop=false 才開放動作鍵
let welcomed = false;
let estopActive = null; // null＝還沒收到 state
let currentStep = "fine"; // 預設 fine（Leo 2026-09-09 要求）
let stepConfig = null; // welcome 帶來的 {coarse_cm, fine_cm}

// ===== 3. 畫面與紀錄：狀態列、按鈕啟停、訊息紀錄 =====
// 這一區不碰協定，只負責「現在什麼能按、什麼不能按」與把訊息印到畫面。
// updateButtons 是唯一決定按鈕啟停的地方；狀態一變就呼叫它，不在別處零散改 disabled。
// setState 是唯一改連線狀態的地方；它順手清閘門變數並呼叫 updateButtons。
const $ = (id) => document.getElementById(id);
const statusEl = $("status");
const logEl = $("log");

// HTTPS 頁面（姿態模式要用）不能連 ws://，改走 serve_https.py 同一個 port 上的 /relay
const DEFAULT_URL =
  location.protocol === "https:"
    ? `wss://${location.host}/relay`
    : `ws://${location.hostname || "localhost"}:8765`;
$("serverUrl").value = localStorage.getItem("serverUrl") || DEFAULT_URL;

function motionAllowed() {
  return state === STATE.CONNECTED && welcomed && estopActive === false;
}

function updateButtons() {
  const connected = state === STATE.CONNECTED;
  $("connectBtn").disabled = wantConnected;
  $("disconnectBtn").disabled = !wantConnected;
  $("helloBtn").disabled = !connected;
  $("testBtn").disabled = !connected;
  const motion = motionAllowed();
  document
    .querySelectorAll(".hold, .mode")
    .forEach((b) => (b.disabled = !motion));
  // 重置場景不受急停限制：急停後常要先重置場景才能解除，由 server 決定收不收
  $("resetSceneBtn").disabled = !(connected && welcomed);
  $("arResetSceneBtn").disabled = !(connected && welcomed);
  $("orientBtn").disabled = !motion || !sensor.enabled;
  $("arClutch").disabled = !motion || !xr.session;
  document
    .querySelectorAll(".ar-grip-btn")
    .forEach((b) => (b.disabled = !motion || !xr.session || arHold?.controlMode === "wrist"));
  $("arStartBtn").disabled = !xr.supported || !!xr.session;
  $("arEstopBtn").disabled = !connected;
  $("arEstopResetBtn").disabled = !(connected && estopActive === true);
  $("estopBtn").disabled = !connected;
  $("estopResetBtn").disabled = !(connected && estopActive === true);
  $("estopResetBtn").hidden = !(connected && estopActive === true); // 急停中才出現
  const gate = $("gate");
  if (!connected) gate.textContent = "未連線";
  else if (!welcomed) gate.textContent = "等 welcome";
  else if (estopActive === null) gate.textContent = "等 state";
  else if (estopActive) gate.textContent = "急停中，按「解除急停」";
  else gate.textContent = "可操作";
  gate.className = "gate " + (motion ? "ok" : "blocked");
}

function setState(next) {
  state = next;
  statusEl.textContent = next;
  statusEl.className =
    "status " +
    {
      [STATE.DISCONNECTED]: "disconnected",
      [STATE.CONNECTED]: "connected",
      [STATE.RECONNECTING]: "reconnecting",
    }[next];
  $("conn").open = next !== STATE.CONNECTED; // 連上就收起連線設定，斷線再展開
  if (next !== STATE.CONNECTED) {
    welcomed = false;
    estopActive = null;
    pendingAck.clear();
  }
  updateButtons();
}

const LOG_MAX = 200; // 手機端只留最近 200 則，避免高頻訊息撐爆 DOM

function addLog(dir, text) {
  const li = document.createElement("li");
  const time = new Date().toLocaleTimeString("zh-TW", { hour12: false });
  li.innerHTML =
    `<span class="dir ${dir}">${dir === "out" ? "送" : dir === "in" ? "收" : "訊"}</span>` +
    `<span class="time">${time}</span><code>${escapeHtml(text)}</code>`;
  logEl.prepend(li);
  while (logEl.children.length > LOG_MAX) logEl.lastChild.remove();
}

function escapeHtml(s) {
  return s.replace(
    /[&<>"']/g,
    (c) =>
      ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[
        c
      ],
  );
}

// ===== 4. 送出與 ack 對帳 =====
// send 是所有訊息的唯一出口：包信封、送出、寫紀錄。socket 沒開就回 null，什麼都不做。
// quiet＝高頻訊息，平常不寫進紀錄（keepalive 每 150 ms、orient_update 每 33 ms，寫了會洗版）；
// 勾「顯示高頻訊息」時照寫，除錯用。
const showHighFreq = () => $("showHighFreq").checked;

function send(type, payload, { quiet = false } = {}) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return null;
  const msg = makeEnvelope(type, payload);
  ws.send(JSON.stringify(msg));
  if (!quiet || showHighFreq()) addLog("out", JSON.stringify(msg));
  return msg.seq;
}

// ack 對帳：server 對每則指令回一個 ack，帶原本的 seq 與 status（ok，或被拒的原因）。
// sendTracked 送出時把 seq 記進 pendingAck；handleAck 收到時用 seq 找回是哪顆按鈕，閃綠或閃紅。
// kind 分兩種：oneshot（一般指令）、start（按住的開始指令，見第 5 區；它的 ack 有特殊意義）。
// pendingAck：seq → {btn, kind}
const pendingAck = new Map();

function sendTracked(btn, type, payload, kind = "oneshot") {
  const s = send(type, payload);
  if (s !== null) pendingAck.set(s, { btn, kind });
  return s;
}

function flash(btn, ok) {
  if (!btn) return;
  const cls = ok ? "flash-ok" : "flash-bad";
  btn.classList.remove("flash-ok", "flash-bad");
  void btn.offsetWidth; // 重新觸發 CSS 動畫
  btn.classList.add(cls);
}

function handleAck(p) {
  const entry = pendingAck.get(p?.seq);
  pendingAck.delete(p?.seq);
  if (p?.status && p.status !== "ok") {
    addLog("in", `指令被拒（seq ${p.seq}）：${p.reason || p.status}`);
  }
  if (!entry) return;
  flash(entry.btn, p.status === "ok");
  // 按住中的 start 收到最終 ack（不論成功或被拒）＝這次按住結束：
  // 例如夾爪碰到物體 server 自動停、watchdog 逾時、被拒。不重送。
  if (hold && entry.kind === "start" && p.seq === hold.startSeq) {
    endHold({
      sendStop: false,
      reason: `server 結束：${p.reason || p.status}`,
    });
  }
}

// ===== 5. 按住控制（press-and-hold）=====
// 方向鍵、手腕鍵、夾爪鍵都是「按著才動、放開就停」，分三個階段：
//   按下  startHold：送 *_start（記進 pendingAck），開一個每 150 ms 跳一次的計時器
//   按住  計時器每 150 ms 送一則 jog_keepalive，告訴 server「我還按著」；不等 ack
//   放開  endHold：關計時器，送 *_stop
// server 端有 750 ms 的看門狗（倒數計時器）：超過 750 ms 沒收到 keepalive 就自己停手臂。
// 所以就算手機當掉、網路斷掉、stop 送不出去，手臂最多也只會多動 750 ms。
//
// 一次只允許一個按住（hold 非 null 時 startHold 直接不理）。
// endHold 是唯一的收尾出口：放開、取消、背景、失焦、斷線、急停全部走它，
// 差別只在要不要送 stop（sendStop）：socket 還活著就送，已經死了就只清本地狀態。
// 姿態模式（第 9 區）也走同一套：只是計時器改成每 33 ms 送 orient_update，它同時當 keepalive。
const KEEPALIVE_MS = 150;
let hold = null; // {btn, stopType, startSeq, timer, pointerId, onEnd}

function sendKeepalive() {
  send("jog_keepalive", {}, { quiet: true });
}

function startHold(
  btn,
  startType,
  stopType,
  payload,
  pointerId,
  { tick = sendKeepalive, intervalMs = KEEPALIVE_MS, onEnd = null } = {},
) {
  if (!motionAllowed() || hold) return;
  const startSeq = sendTracked(btn, startType, payload, "start");
  if (startSeq === null) return;
  btn.classList.add("held");
  hold = { btn, stopType, startSeq, timer: null, pointerId, onEnd };
  hold.timer = setInterval(() => {
    if (!hold || !ws || ws.readyState !== WebSocket.OPEN) return;
    tick();
  }, intervalMs);
}

function endHold({ sendStop = true, reason = "" } = {}) {
  if (!hold) return;
  const h = hold;
  hold = null;
  clearInterval(h.timer);
  h.btn.classList.remove("held");
  if (sendStop) sendTracked(h.btn, h.stopType, {}, "stop");
  if (reason) addLog("sys", reason);
  h.onEnd?.();
}

// 把按住三階段接到一顆按鈕的指標事件上。pointer 事件同時涵蓋觸控與滑鼠。
// payloadFn 是函式不是物件，因為步長（粗調／微調）要在按下的那一刻才讀。
// opts 是給 startHold 的 tick／intervalMs／onEnd，再加 canStart：按下時回傳 false 就不開始。
function bindHold(btn, startType, stopType, payloadFn, opts = {}) {
  const { canStart = () => true, ...holdOpts } = opts;
  btn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    if (btn.disabled || hold || !canStart()) return;
    startHold(btn, startType, stopType, payloadFn(), e.pointerId, holdOpts);
    if (hold && hold.btn === btn) {
      try {
        btn.setPointerCapture(e.pointerId); // 手指滑出按鈕仍算按住，放開才停
      } catch {}
    }
  });
  // 只認開始按住的那根手指，第二指碰同一顆鍵不會誤停第一指
  const release = (e) => {
    if (hold && hold.btn === btn && e.pointerId === hold.pointerId) endHold();
  };
  btn.addEventListener("pointerup", release);
  btn.addEventListener("pointercancel", release);
  btn.addEventListener("pointerleave", release);
  // 捕捉被系統收回（來電、手勢、對話框）＝失去控制，立即停
  btn.addEventListener("lostpointercapture", release);
  btn.addEventListener("contextmenu", (e) => e.preventDefault());
}

// 全域兜底：同一根手指在任何地方放開或取消，都結束按住
["pointerup", "pointercancel"].forEach((t) =>
  window.addEventListener(
    t,
    (e) => {
      if (hold && e.pointerId === hold.pointerId) endHold();
    },
    true,
  ),
);

// 任何「失去控制」的情況都要停：背景、失焦、離開頁面、socket 出事
function stopEverything(reason) {
  endHold({ sendStop: true, reason });
}
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopEverything("App 進入背景，已送 stop");
});
window.addEventListener("blur", () => stopEverything("視窗失焦，已送 stop"));
window.addEventListener("pagehide", () =>
  stopEverything("離開頁面，已送 stop"),
);

// ===== 6. WebSocket 連線：生命週期 =====
// connect 建一條 socket 並掛四個回呼，整個生命週期就是這四個回呼在跑：
//   onopen     連上了：狀態改「已連」，立刻送 hello 自我介紹
//   onmessage  收到訊息：state 走 handleState；welcome 打開閘門；ack 走 handleAck
//   onclose    斷了：清掉按住狀態；使用者還想連就排重連，否則回「未連」
//   onerror    不做事，因為瀏覽器保證 onerror 之後一定接著 onclose
// 重連用指數退避：等 1 秒、2 秒、4 秒、8 秒，之後固定 8 秒；連上後 onopen 把它重設回 1 秒。
function connect() {
  const url = $("serverUrl").value.trim();
  if (!url) return;
  localStorage.setItem("serverUrl", url);
  if (location.protocol === "https:" && url.startsWith("ws://")) {
    addLog("sys", `HTTPS 頁面不能連 ws://，改填 ${DEFAULT_URL}`);
  }

  try {
    ws = new WebSocket(url);
  } catch (e) {
    addLog("sys", `連線失敗：${e.message}`);
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    reconnectDelay = 1000;
    setState(STATE.CONNECTED);
    send("hello", { client: "phone", name: "thomas-app" });
  };

  ws.onmessage = (ev) => {
    // 協定只走 JSON 純文字；binary frame 直接標示不解析
    if (typeof ev.data !== "string") {
      addLog("in", "(忽略非文字 frame)");
      return;
    }
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      addLog("in", ev.data);
      return;
    }
    if (msg.type === "state") {
      handleState(msg.payload || {});
      if (showHighFreq()) addLog("in", ev.data);
      return; // 每秒五則，平常不進紀錄，顯示在狀態列
    }
    addLog("in", ev.data);
    if (msg.type === "ack") handleAck(msg.payload);
    if (msg.type === "welcome") {
      welcomed = true;
      if (msg.payload?.config) stepConfig = msg.payload.config;
      updateModeBtn();
      updateButtons();
    }
  };

  ws.onclose = () => {
    // 斷線＝失去控制：本地清掉按住狀態（socket 已死，stop 送不出去，server 端 watchdog 會接手）
    endHold({ sendStop: false, reason: hold ? "斷線，按住已清除" : "" });
    ws = null;
    if (wantConnected) {
      setState(STATE.RECONNECTING);
      scheduleReconnect();
    } else {
      setState(STATE.DISCONNECTED);
    }
  };

  ws.onerror = () => {
    /* onclose 會跟著觸發，重連交給它 */
  };
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => {
    if (wantConnected) connect();
  }, reconnectDelay);
  reconnectDelay = Math.min(reconnectDelay * 2, 8000);
}

// ===== 7. state 廣播：server 每秒五次回報手臂狀態 =====
// 這是急停閘門唯一的資料來源：estop 變 true 就結束按住並鎖住方向鍵，變 false 才解鎖。
// 其餘欄位只顯示在狀態列：末端位置（公尺轉公分）、夾爪、IK 有沒有解、任務是否成功、第幾個 episode。
function handleState(p) {
  const prevEstop = estopActive;
  if (typeof p.estop === "boolean") estopActive = p.estop;
  if (estopActive && hold) {
    endHold({ sendStop: true, reason: "server 回報急停，已送 stop" });
  }
  if (prevEstop !== estopActive) updateButtons();
  const pos = Array.isArray(p.ee_pos)
    ? p.ee_pos.map((v) => (v * 100).toFixed(1)).join(", ")
    : "?";
  const parts = [
    `ee(cm) ${pos}`,
    `夾爪 ${p.gripper ?? "?"}`,
    `ik ${p.ik ?? "?"}`,
    `estop ${p.estop ?? "?"}`,
  ];
  if (typeof p.task_success === "boolean")
    parts.push(p.task_success ? "任務 ✓" : "任務 …");
  if (typeof p.episode === "number") parts.push(`ep ${p.episode}`);
  if (typeof p.orient_deg === "number") parts.push(`姿態 ${p.orient_deg}°`);
  $("stateBar").textContent = parts.join("　");
}

// ===== 8. 按鈕綁定：把上面的機制接到畫面 =====
// 連線／斷線兩顆改的是 wantConnected（使用者的意圖），不是直接動 socket；
// 這樣 onclose 才分得出「使用者主動斷」與「意外斷線要重連」。
// 方向、手腕、夾爪三組鍵走第 5 區的 bindHold；急停、解除急停、重置場景、切模式是一次性指令，走 sendTracked。
$("connectBtn").onclick = () => {
  wantConnected = true;
  setState(STATE.RECONNECTING);
  connect();
};

$("disconnectBtn").onclick = () => {
  wantConnected = false;
  clearTimeout(reconnectTimer);
  stopEverything("");
  if (ws) ws.close();
  else setState(STATE.DISCONNECTED);
};

$("helloBtn").onclick = () =>
  send("hello", { client: "phone", name: "thomas-app" });
$("testBtn").onclick = () => send("test", { msg: "hello" });
$("clearBtn").onclick = () => {
  logEl.innerHTML = "";
};

function updateModeBtn() {
  const cm = stepConfig
    ? ` ${currentStep === "coarse" ? stepConfig.coarse_cm : stepConfig.fine_cm} cm`
    : "";
  $("modeBtn").textContent = (currentStep === "coarse" ? "粗調" : "微調") + cm;
}
$("modeBtn").onclick = () => {
  currentStep = currentStep === "coarse" ? "fine" : "coarse";
  updateModeBtn();
  sendTracked($("modeBtn"), "mode", { step: currentStep });
};

// 平移六鍵：jog_start {axis, dir, step, frame:"base"}
document.querySelectorAll(".move").forEach((btn) => {
  bindHold(btn, "jog_start", "jog_stop", () => ({
    axis: btn.dataset.axis,
    dir: Number(btn.dataset.dir),
    step: currentStep,
    frame: "base",
  }));
});

// 手腕兩鍵：joint_jog_start {joint:"wrist_flex", dir, step}
document.querySelectorAll(".wrist").forEach((btn) => {
  bindHold(btn, "joint_jog_start", "joint_jog_stop", () => ({
    joint: "wrist_flex",
    dir: Number(btn.dataset.dir),
    step: currentStep,
  }));
});

// 夾爪兩鍵：gripper_jog_start {action, step}
document.querySelectorAll(".grip").forEach((btn) => {
  bindHold(btn, "gripper_jog_start", "gripper_jog_stop", () => ({
    action: btn.dataset.action,
    step: currentStep,
  }));
});

// 急停：獨立、醒目、連線中隨時可按；先本地停 keepalive 再送 estop
function fireEstop() {
  endHold({ sendStop: false });
  sendTracked($("estopBtn"), "estop", {});
}
// 按下就送，不等放開：另一指還按著方向鍵時也能立刻停
$("estopBtn").addEventListener("pointerdown", (e) => {
  e.preventDefault();
  if (!$("estopBtn").disabled) fireEstop();
});
// 鍵盤 Enter／Space 產生的 click 沒有指標（detail 0）才送；指標產生的 click 已在 pointerdown 送過
$("estopBtn").addEventListener("click", (e) => {
  if (e.detail === 0 && !$("estopBtn").disabled) fireEstop();
});
$("estopResetBtn").onclick = () =>
  sendTracked($("estopResetBtn"), "estop_reset", {});
$("resetSceneBtn").onclick = () =>
  sendTracked($("resetSceneBtn"), "reset_scene", {});

// ===== 9. 姿態模式（IMU）：手機轉動控制末端轉向 =====
// 只控「轉向」，平移繼續用按鈕；純 IMU 推位置一定會飄。設計依據見 docs/hold-protocol.md 的 orient 一節。
//   離合器（clutch）：按住「姿態」鈕才生效。按下那一刻記住手機姿態 q0，放開就斷開；
//                     每次按下都重新歸零，飄移只在單次按住期間累積。
//   相對姿態：q_rel = q0⁻¹ · q，「現在比按下時轉了多少」，用手機自己的座標軸表示
//             （x 螢幕右、y 螢幕上、z 垂直螢幕朝外，W3C DeviceOrientation 定義）。
//   處理順序：死區（轉不到 deadband_deg 度當沒動）→ 增益（乘 gain）→ 指數平滑（smoothing 越大越鈍）。
//   App 只送 q_rel；對齊手臂座標、疊加按下時的末端姿態、投影到 SO-101 的腕部，都由 server 做。
// 按住期間每 33 ms 送一則 orient_update（約 30 Hz），它同時當 keepalive，server 看門狗照舊 750 ms。
// 陀螺儀超過 300 ms 沒新資料（感測器卡住、權限被收回）也算失去控制，走 endHold 送 stop。
const ORIENT_INTERVAL_MS = 33;
const SENSOR_STALE_MS = 300;
const D2R = Math.PI / 180;
const IDENTITY = [0, 0, 0, 1];

let sensor = { enabled: false, q: null, t: 0 }; // 最新一筆手機姿態（xyzw）與收到的時間
let orient = null; // 按住中：{q0, out, params}

// 四元數一律 [x, y, z, w]
// DeviceOrientation 的 Euler 順序是 Z（alpha）、X'（beta）、Y''（gamma），內旋；公式同 orientation.html
function eulerToQuat(alpha, beta, gamma) {
  const ca = Math.cos((alpha * D2R) / 2),
    sa = Math.sin((alpha * D2R) / 2);
  const cb = Math.cos((beta * D2R) / 2),
    sb = Math.sin((beta * D2R) / 2);
  const cg = Math.cos((gamma * D2R) / 2),
    sg = Math.sin((gamma * D2R) / 2);
  return [
    sb * ca * cg - cb * sa * sg,
    cb * ca * sg + sb * sa * cg,
    cb * sa * cg + sb * ca * sg,
    cb * ca * cg - sb * sa * sg,
  ];
}

function qMul([ax, ay, az, aw], [bx, by, bz, bw]) {
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

const qConj = ([x, y, z, w]) => [-x, -y, -z, w];

// 轉角（度）：q 與 −q 是同一個朝向，取短的那一邊
function qAngleDeg([x, y, z, w]) {
  return (2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w))) / D2R;
}

// 球面線性插值：從 a 往 b 走 t 比例，走的是轉角而不是四個數各自平均
function qSlerp(a, b, t) {
  let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0) {
    b = b.map((v) => -v);
    dot = -dot;
  }
  if (dot > 0.9995) {
    const q = a.map((v, i) => v + (b[i] - v) * t);
    const n = Math.hypot(...q);
    return q.map((v) => v / n);
  }
  const th = Math.acos(dot);
  const s = Math.sin(th);
  const ka = Math.sin((1 - t) * th) / s,
    kb = Math.sin(t * th) / s;
  return a.map((v, i) => ka * v + kb * b[i]);
}

// 死區＋增益：轉軸不變，轉角改成 gain × (角度 − 死區)；扣掉死區讓輸出從 0 連續長出來，不會一跨過就跳
function shapeRel(q, { gain, deadband_deg }) {
  let [x, y, z, w] = q;
  if (w < 0) [x, y, z, w] = [-x, -y, -z, -w];
  const s = Math.hypot(x, y, z);
  const angle = 2 * Math.atan2(s, w);
  const db = deadband_deg * D2R;
  if (s < 1e-9 || angle <= db) return IDENTITY;
  const a = gain * (angle - db);
  const k = Math.sin(a / 2) / s;
  return [x * k, y * k, z * k, Math.cos(a / 2)];
}

function clampNum(v, lo, hi, fallback) {
  const n = Number(v);
  return Number.isFinite(n) ? Math.min(hi, Math.max(lo, n)) : fallback;
}

// 參數在按下那一刻讀，整段按住期間固定；也原樣放進 orient_start，server 當實驗參數記錄
function readOrientParams() {
  return {
    gain: clampNum($("orientGain").value, 0.1, 2, 0.7),
    deadband_deg: clampNum($("orientDeadband").value, 0, 15, 2.5),
    smoothing: clampNum($("orientSmoothing").value, 0, 0.95, 0.3),
  };
}

function sensorFresh() {
  return sensor.q !== null && performance.now() - sensor.t < SENSOR_STALE_MS;
}

function showOrientInfo(text) {
  $("orientInfo").textContent = text;
}

function onDeviceOrientation(e) {
  if (e.alpha == null) return; // 有事件沒數值：這台沒有陀螺儀
  sensor.q = eulerToQuat(e.alpha, e.beta, e.gamma);
  sensor.t = performance.now();
  if (!hold || hold.btn !== $("orientBtn")) showOrientInfo("就緒，按住才生效");
}

// iPhone 一定要在使用者點擊裡要權限；Android 通常直接給
async function enableSensor() {
  if (!window.isSecureContext) {
    addLog("sys", "姿態模式要用 HTTPS 開頁面（python3 serve_https.py）");
    showOrientInfo("要 HTTPS");
    return;
  }
  try {
    if (typeof DeviceOrientationEvent?.requestPermission === "function") {
      const r = await DeviceOrientationEvent.requestPermission();
      if (r !== "granted") {
        showOrientInfo("沒拿到權限");
        return;
      }
    }
  } catch (err) {
    addLog("sys", `陀螺儀權限失敗：${err.message}`);
    return;
  }
  if (!sensor.enabled) {
    window.addEventListener("deviceorientation", onDeviceOrientation);
  }
  sensor.enabled = true;
  $("sensorBtn").hidden = true; // 拿到權限後，「啟用」鈕換成「按住轉手機」
  $("orientBtn").hidden = false;
  showOrientInfo("等感測器資料…");
  updateButtons();
}

function orientTick() {
  if (!orient) return;
  if (!sensorFresh()) {
    endHold({ reason: "陀螺儀資料中斷，已送 stop" });
    return;
  }
  const { q0, params } = orient;
  const target = shapeRel(qMul(qConj(q0), sensor.q), params);
  orient.out = qSlerp(orient.out, target, 1 - params.smoothing);
  const q_rel = orient.out.map((v) => Number(v.toFixed(5)));
  send("orient_update", { q_rel }, { quiet: true });
  showOrientInfo(`轉了 ${qAngleDeg(orient.out).toFixed(1)}°`);
}

function canStartOrient() {
  if (!sensorFresh()) {
    addLog("sys", "陀螺儀沒有資料，先按「啟用陀螺儀」並確認數值有在跳");
    return false;
  }
  orient = { q0: sensor.q, out: IDENTITY, params: readOrientParams() };
  return true;
}

bindHold(
  $("orientBtn"),
  "orient_start",
  "orient_stop",
  () => ({ frame: "phone_body", rate_hz: 30, ...orient.params }),
  {
    canStart: canStartOrient,
    tick: orientTick,
    intervalMs: ORIENT_INTERVAL_MS,
    // 不論哪種原因結束都清掉 orient，下次按下重新記 q0
    onEnd: () => {
      orient = null;
      showOrientInfo(sensorFresh() ? "就緒，按住才生效" : "等感測器資料…");
    },
  },
);

$("sensorBtn").onclick = enableSensor;

// ===== 10. AR 模式（WebXR）：手機移動帶動整隻手臂 =====
// 陀螺儀只量得到轉向；WebXR 用相機＋慣性感測追蹤，量得到手機移動了多遠（6 自由度）。
// iPhone 的 Safari 沒有 WebXR AR，要用 XRViewer 開；Android 用 Chrome。做法同 TidyBot++。
//   進入 AR：整頁換成 #arOverlay，疊在相機畫面上（dom-overlay），只留急停、結束、控制區。
//   離合器：按住控制區才生效。按下那一刻記住手機位置 p0、姿態 q0；放開就斷開，每次按下重新歸零。
//   每 33 ms 送 pose_update（兼 keepalive）：
//     p_rel_m        p − p0，公尺，WebXR local 座標（x 右、y 上、z 朝後，以進入 AR 時手機的朝向為準）
//     q_rel          q0⁻¹·q，跟姿態模式同一個定義與處理（死區、增益、平滑沿用姿態參數）
//     gripper_delta  按住控制區的同時按住 AR 畫面下方的夾爪鈕：開＝每秒 +1、關＝每秒 −1，
//                    範圍 −1 到 1（±1＝全開／全關），放開夾爪鈕就停在當下的值
//   沒按住控制區時，夾爪鈕就是一般的 gripper_jog（跟按鈕模式一樣），維持「一次只有一個按住」
//   座標對到手臂、位移倍率、工作空間與速度限制、IK 失敗處理都由 server 做（Leo 2026-10-14）。
// 另外三種情況也算失去控制，走 endHold 送 stop：追蹤遺失或 300 ms 沒新姿態、手機突然動太快
// （門檻同 COBALT：每 50 ms 超過 4 cm 或 0.2 rad，例如手機掉落）、AR 被結束。
const POSE_INTERVAL_MS = 33;
const POSE_STALE_MS = 300;
const MAX_SPEED_MPS = 0.04 / 0.05;
const MAX_ANG_SPEED_DPS = 0.2 / 0.05 / D2R;
const GRIP_RATE_PER_S = 1.0;

let xr = { supported: false, session: null, pose: null, t: 0 }; // pose：{p 公尺, q xyzw}
let arHold = null; // 按住中：{p0, q0, out, params, grip, gripDir, gripPointer, gripT, last}
const arModes = new ArControlModes();

function renderArControlMode() {
  const wrist = arModes.selected === "wrist";
  document.querySelectorAll("[data-ar-control]").forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.arControl === arModes.selected));
  });
  $("arModeHint").textContent = wrist
    ? "手腕：前後傾只動手腕，忽略手機平移"
    : "平移：只控制位置，忽略手機傾斜";
  $("arClutchLabel").textContent = wrist ? "按住轉手腕" : "按住平移";
  $("arClutchHint").textContent = wrist
    ? "放開後可操作夾爪；切換模式後請重新按住"
    : "夾爪用下方按鈕；切換模式後請重新按住";
}
document.querySelectorAll("[data-ar-control]").forEach((button) => {
  button.addEventListener("click", () => {
    arModes.select(button.dataset.arControl, () => {
      endHold({ reason: "切換 AR 模式，已停止；請重新按住" });
      arHold = null;
    });
    renderArControlMode();
    updateButtons();
  });
});
renderArControlMode();

function showArInfo(text) {
  $("arInfo").textContent = text;
}

function poseFresh() {
  return xr.pose !== null && performance.now() - xr.t < POSE_STALE_MS;
}

async function checkXrSupport() {
  try {
    xr.supported =
      !!navigator.xr && (await navigator.xr.isSessionSupported("immersive-ar"));
  } catch {
    xr.supported = false;
  }
  $("arSupport").textContent = xr.supported
    ? "這個瀏覽器支援"
    : "這個瀏覽器不支援，見下方說明";
  updateButtons();
}

async function startAR() {
  const overlay = $("arOverlay");
  overlay.hidden = false;
  let session;
  try {
    // dom-overlay 列為選用：XRViewer 比這個規格早，它是把整個網頁直接畫在相機畫面上，
    // 不一定回報 dom-overlay；列成必要的話 XRViewer 會開不了（TidyBot++ 也是這樣用）
    session = await navigator.xr.requestSession("immersive-ar", {
      requiredFeatures: ["local"],
      optionalFeatures: ["dom-overlay"],
      domOverlay: { root: overlay },
    });
  } catch (err) {
    overlay.hidden = true;
    addLog("sys", `開不了 AR：${err.message}`);
    return;
  }
  xr.session = session;
  // 沒有 dom-overlay 時，overlay 改成不透明蓋住主畫面，避免底下的按鈕透出來混淆
  overlay.classList.toggle("opaque", !session.domOverlayState);
  // WebXR 要有 WebGL 圖層才會跑畫格；不畫東西，每格清成透明讓相機畫面透出來
  const gl = document
    .createElement("canvas")
    .getContext("webgl", { xrCompatible: true });
  session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
  const space = await session.requestReferenceSpace("local");
  session.addEventListener("end", () => {
    if (hold) endHold({ reason: "AR 結束，已送 stop" });
    xr.session = null;
    xr.pose = null;
    overlay.hidden = true;
    updateButtons();
  });
  const onFrame = (t, frame) => {
    if (!xr.session) return;
    session.requestAnimationFrame(onFrame);
    gl.bindFramebuffer(
      gl.FRAMEBUFFER,
      session.renderState.baseLayer.framebuffer,
    );
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    const pose = frame.getViewerPose(space);
    if (!pose || pose.emulatedPosition) {
      xr.pose = null; // 下一個 tick 會因為沒有新姿態而停
      showArInfo("追蹤遺失：對著有紋理、光線夠的地方");
      return;
    }
    const { position: p, orientation: o } = pose.transform;
    xr.pose = { p: [p.x, p.y, p.z], q: [o.x, o.y, o.z, o.w] };
    xr.t = performance.now();
    if (!arHold) showArInfo("追蹤中，按住下方區域開始控制");
  };
  session.requestAnimationFrame(onFrame);
  showArInfo("等待追蹤…慢慢移動手機");
  updateButtons();
}

function canStartAr() {
  if (!xr.session || !poseFresh()) {
    showArInfo("還沒追蹤到，先慢慢移動手機");
    return false;
  }
  arHold = {
    controlMode: arModes.selected,
    p0: xr.pose.p,
    q0: xr.pose.q,
    out: IDENTITY,
    params: readOrientParams(),
    grip: 0,
    gripDir: 0,
    gripPointer: null,
    gripT: performance.now(),
    last: { p: xr.pose.p, q: xr.pose.q, t: xr.t },
  };
  updateButtons();
  return true;
}

function arTick() {
  if (!arHold) return;
  if (!poseFresh()) {
    endHold({ reason: "AR 追蹤中斷，已送 stop" });
    return;
  }
  const { p, q } = xr.pose;
  const last = arHold.last;
  if (xr.t > last.t) {
    const dt = (xr.t - last.t) / 1000;
    const speed = Math.hypot(...p.map((v, i) => v - last.p[i])) / dt;
    const angSpeed = qAngleDeg(qMul(qConj(last.q), q)) / dt;
    if (speed > MAX_SPEED_MPS || angSpeed > MAX_ANG_SPEED_DPS) {
      endHold({
        reason: `手機動太快（${speed.toFixed(2)} m/s、${angSpeed.toFixed(0)}°/s），已送 stop`,
      });
      return;
    }
    arHold.last = { p, q, t: xr.t };
  }
  const now = performance.now();
  if (arHold.gripDir) {
    const g =
      arHold.grip +
      (arHold.gripDir * GRIP_RATE_PER_S * (now - arHold.gripT)) / 1000;
    arHold.grip = Math.max(-1, Math.min(1, g));
  }
  arHold.gripT = now;
  const p_rel_m = p.map((v, i) => Number((v - arHold.p0[i]).toFixed(4)));
  const target = shapeRel(qMul(qConj(arHold.q0), q), arHold.params);
  arHold.out = qSlerp(arHold.out, target, 1 - arHold.params.smoothing);
  const q_rel = arHold.out.map((v) => Number(v.toFixed(5)));
  const gripper_delta = Number(arHold.grip.toFixed(3));
  const command = arModes.sample(arHold.controlMode, p_rel_m, q_rel, gripper_delta);
  send("pose_update", command, { quiet: true });
  const cm = p_rel_m.map((v) => (v * 100).toFixed(1)).join(", ");
  $("arReadout").textContent =
    arHold.controlMode === "wrist"
      ? `手腕｜轉 ${qAngleDeg(arHold.out).toFixed(1)}°｜平移已忽略`
      : `平移(cm) ${cm}｜傾斜已忽略｜夾爪 ${gripper_delta.toFixed(2)}`;
}

const arClutch = $("arClutch");
bindHold(
  arClutch,
  "pose_start",
  "pose_stop",
  () => ({ frame: "webxr_local", rate_hz: 30, control_mode: arHold.controlMode, ...arHold.params }),
  {
    canStart: canStartAr,
    tick: arTick,
    intervalMs: POSE_INTERVAL_MS,
    onEnd: () => {
      arHold = null;
      updateButtons();
      document
        .querySelectorAll(".ar-grip-btn")
        .forEach((b) => b.classList.remove("held"));
      $("arReadout").textContent = "";
      if (xr.session) showArInfo(poseFresh() ? "已放開" : "追蹤中斷");
    },
  },
);

// AR 夾爪鈕：按住控制區時，第二指按夾爪鈕只改 gripper_delta 的變化方向，不另開按住；
// 沒按住控制區時，交給下面的 bindHold 當一般 gripper_jog。這個監聽要比 bindHold 先註冊。
document.querySelectorAll(".ar-grip-btn").forEach((btn) => {
  const dir = btn.dataset.action === "open" ? 1 : -1;
  btn.addEventListener("pointerdown", (e) => {
    if (btn.disabled || !arHold) return;
    e.preventDefault();
    e.stopImmediatePropagation();
    arHold.gripDir = dir;
    arHold.gripPointer = e.pointerId;
    btn.classList.add("held");
    try {
      btn.setPointerCapture(e.pointerId);
    } catch {}
  });
  const release = (e) => {
    if (!arHold || e.pointerId !== arHold.gripPointer) return;
    arHold.gripDir = 0;
    arHold.gripPointer = null;
    btn.classList.remove("held");
  };
  ["pointerup", "pointercancel", "lostpointercapture"].forEach((t) =>
    btn.addEventListener(t, release),
  );
  bindHold(btn, "gripper_jog_start", "gripper_jog_stop", () => ({
    action: btn.dataset.action,
    step: currentStep,
  }));
});

// 點 overlay 上的按鈕不要被 WebXR 當成在 AR 場景裡點擊
$("arOverlay").addEventListener("beforexrselect", (e) => e.preventDefault());

// AR 畫面上的急停：跟主畫面那顆一樣，按下就送
$("arEstopBtn").addEventListener("pointerdown", (e) => {
  e.preventDefault();
  if (!$("arEstopBtn").disabled) fireEstop();
});
$("arEstopResetBtn").onclick = () =>
  sendTracked($("arEstopResetBtn"), "estop_reset", {});
$("arResetSceneBtn").onclick = () =>
  sendTracked($("arResetSceneBtn"), "reset_scene", {});
$("arExitBtn").onclick = () => xr.session?.end();
$("arStartBtn").onclick = startAR;
checkXrSupport();

// ===== 11. 畫面模式與設定：一次只顯示一種控制 =====
// 按鈕、陀螺儀、AR 三個分頁；有 data-modes 的區塊只在列出的模式顯示。記住上次用的分頁。
// 切分頁時若正按住，先結束按住（送 stop），不讓看不見的按鈕繼續控制。
const MODES = ["buttons", "gyro", "ar"];

function setMode(mode) {
  if (!MODES.includes(mode)) mode = "buttons";
  if (hold) endHold({ reason: "切換模式，已送 stop" });
  document.querySelectorAll(".tab").forEach((t) => {
    const on = t.dataset.tab === mode;
    t.classList.toggle("active", on);
    t.setAttribute("aria-selected", String(on));
  });
  document.querySelectorAll("[data-modes]").forEach((el) => {
    el.hidden = !el.dataset.modes.split(" ").includes(mode);
  });
  try {
    localStorage.setItem("mode", mode);
  } catch {}
}

document
  .querySelectorAll(".tab")
  .forEach((t) => (t.onclick = () => setMode(t.dataset.tab)));
let savedMode = null;
try {
  savedMode = localStorage.getItem("mode");
} catch {}
setMode(savedMode);

// 設定（姿態參數、訊息紀錄）收在全螢幕對話框裡，平常不佔畫面
$("settingsBtn").onclick = () => $("settings").showModal();
$("settingsCloseBtn").onclick = () => $("settings").close();

updateModeBtn();
setState(STATE.DISCONNECTED);

