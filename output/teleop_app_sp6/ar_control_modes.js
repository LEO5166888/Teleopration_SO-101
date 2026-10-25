// Local test UI: explicit choice, captured for the entire press.
(function (root) {
  class ArControlModes {
    constructor() { this.selected = "translation"; }
    select(mode, stopCurrentHold) {
      if (!["translation", "wrist"].includes(mode)) throw new Error("Unknown AR mode");
      if (mode === this.selected) return false;
      // Stop first; do not restart while a finger is still held down.
      stopCurrentHold();
      this.selected = mode;
      return true;
    }
    sample(mode, p, q, grip) {
      if (mode === "translation") {
        return { p_rel_m: [...p], q_rel: [0, 0, 0, 1], gripper_delta: grip };
      }
      if (mode === "wrist") {
        return { p_rel_m: [0, 0, 0], q_rel: [...q], gripper_delta: 0 };
      }
      throw new Error("Unknown AR mode");
    }
  }
  root.ArControlModes = ArControlModes;
  if (typeof module !== "undefined") module.exports = ArControlModes;
})(globalThis);
