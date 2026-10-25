const assert = require('node:assert/strict');
const { test } = require('node:test');
const ArControlModes = require('./ar_control_modes.js');

test('translation ignores tilt but keeps translation and grip', () => {
  const modes = new ArControlModes();
  assert.equal(modes.selected, 'translation');
  assert.deepEqual(modes.sample(modes.selected, [0.12, -0.02, 0.01], [.1, .2, 0, .97], .4), {
    p_rel_m: [0.12, -0.02, 0.01], q_rel: [0, 0, 0, 1], gripper_delta: .4,
  });
});
test('wrist ignores translation and grip, preserves orientation', () => {
  const modes = new ArControlModes();
  assert.deepEqual(modes.sample('wrist', [0.1, -0.03, 0.05], [.2, 0, 0, .98], .4), {
    p_rel_m: [0, 0, 0], q_rel: [.2, 0, 0, .98], gripper_delta: 0,
  });
});
test('mode change stops before changing selection, with no automatic restart', () => {
  const modes = new ArControlModes();
  const events = [];
  modes.select('wrist', () => events.push(['stop', modes.selected]));
  assert.deepEqual(events, [['stop', 'translation']]);
  assert.equal(modes.selected, 'wrist');
  assert.equal(modes.select('wrist', () => assert.fail('same mode must not stop')), false);
  modes.select('translation', () => events.push(['stop', modes.selected]));
  assert.equal(modes.selected, 'translation');
  assert.equal(events.length, 2);
});
test('invalid mode is rejected without stopping or changing mode', () => {
  const modes = new ArControlModes();
  assert.throws(() => modes.select('auto', () => assert.fail('must not stop')));
  assert.equal(modes.selected, 'translation');
});
