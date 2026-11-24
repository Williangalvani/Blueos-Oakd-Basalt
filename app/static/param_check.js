'use strict';
// The autopilot parameter check banner of the main page (and the Cockpit widget's hint): formatting only, no
// DOM. Input: /api/status autopilot_params (param_checker.ParamChecker.summary).

function paramNumber(v) {
  return typeof v === 'number' && isFinite(v);
}

function paramValueText(value) {
  if (!paramNumber(value)) return '–';
  return Number.isInteger(value) ? String(value) : String(Number(value.toPrecision(6)));
}

function paramAgeText(seconds) {
  if (!paramNumber(seconds)) return '';
  if (seconds < 90) return 'checked ' + seconds.toFixed(0) + ' s ago';
  return 'checked ' + (seconds / 60).toFixed(0) + ' min ago';
}

// What the banner shows. state: "checking" (nothing checked yet), "unreadable" (mavlink2rest or the autopilot
// HEARTBEAT missing), "warn" (mismatches or unreadable parameters), "pending" (all set, a reboot is needed) or
// "ok". rows: [name, current, recommended, needs_reboot] of the mismatches; configure: whether the Configure
// button is offered, enabled unless the vehicle is armed (configureTitle says why not).
function paramCheckView(check) {
  const c = check || {};
  const age = paramAgeText(c.age_s);
  const pending = c.pending_reboot || [];
  const rebootNote = pending.length
    ? 'Reboot the autopilot for ' + pending.join(', ') + ' to take effect (not done automatically).' : '';
  const view = { state: 'ok', title: '', detail: age, rows: [], configure: false, configureEnabled: false,
                 configureTitle: '', notes: [] };
  if (c.checked_at === null || c.checked_at === undefined) {
    return { ...view, state: 'checking', title: 'Checking the autopilot parameters…', detail: '' };
  }
  if (!c.reachable) {
    return { ...view, state: 'unreadable', title: 'Autopilot parameters not checked',
             detail: (c.error || 'mavlink2rest or the autopilot HEARTBEAT is missing') + (age ? ' · ' + age : ''),
             notes: ['Retried in the background, less often while nothing answers.'] };
  }
  const mismatches = c.mismatches || [];
  const unreadable = c.unreadable || [];
  if (mismatches.length || unreadable.length) {
    view.state = 'warn';
    view.title = 'Autopilot parameters differ from the recommended setup';
    view.rows = mismatches.map(m => [m.name, paramValueText(m.current), paramValueText(m.recommended),
                                     !!m.needs_reboot]);
    if (unreadable.length) {
      let note = 'Not readable: ' + unreadable.join(', ') + '.';
      if (mismatches.some(m => m.needs_reboot)) {
        note += ' Some parameters only appear after the ones marked "reboot" are written and the autopilot rebooted.';
      }
      view.notes.push(note);
    }
    if (mismatches.length) {
      view.configure = true;
      view.configureEnabled = c.armed !== true;
      view.configureTitle = c.armed === true
        ? 'The vehicle is ARMED: disarm it first (writes are refused while armed)'
        : 'Write the recommended values (asks for confirmation first)';
    }
    if (rebootNote) view.notes.push(rebootNote);
    return view;
  }
  if (pending.length) {
    return { ...view, state: 'pending', title: 'Reboot the autopilot to finish the setup', notes: [rebootNote] };
  }
  return { ...view, title: 'Autopilot parameters match the recommended setup' };
}

// The confirm dialog's changes for Configure: exactly the mismatches, at their recommended values
function paramConfigureChanges(check) {
  return ((check || {}).mismatches || []).map(m => ({ name: m.name, old: m.current, new: m.recommended,
                                                       reboot: !!m.needs_reboot }));
}

// The Cockpit widget's short hint, or '' when there is nothing to say
function paramWidgetText(check) {
  const view = paramCheckView(check);
  if (view.state === 'warn') return 'autopilot params differ from the recommended setup';
  if (view.state === 'pending') return 'autopilot reboot pending';
  return '';
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { paramCheckView, paramConfigureChanges, paramWidgetText, paramValueText };
}
