// Robot editor (sensors, wheels, motors → a profile saved in this browser) and arena editor
// (route pieces → an arena saved in this browser, with preview, random and built-in loaders, file import/export).
"use strict";
const PORTS = ["A", "B", "C", "D", "E", "F", "S1", "S2", "S3", "S4"];
const EDIT_KEYS = ["sensor_ports", "sensor_pos_mm", "sensor_white", "sensor_black", "motors", "wheel_diameter_mm",
  "axle_track_mm", "gear_ratio"];
const EXAMPLE_ARENA = "# start at (0, 0) heading right\nS 600\nA 150 90\nS 300\nT -90\nS 400\nA 200 -180\nS 500\nM 300 -60 300 60\n";

// ---------------------------------------------------------------- shared
function fitBox(cv, x0, y0, x1, y1) {
  const pad = 20, W = cv.width, Hh = cv.height;
  const s = Math.min((W - 2 * pad) / Math.max(1, x1 - x0), (Hh - 2 * pad) / Math.max(1, y1 - y0));
  const ox = (W - (x1 - x0) * s) / 2, oy = (Hh - (y1 - y0) * s) / 2;
  return { s, a: s, d: -s, e: ox - x0 * s, f: oy + y1 * s };
}
function num(el) { return parseFloat(el.value); }
function freePorts(used, like) {
  // Unused ports, the same kind first (EV3 sensor ports S1–S4 when the others are S ports)
  const s = like && like.startsWith("S");
  return PORTS.filter((p) => !used.includes(p)).sort((a, b) => (a.startsWith("S") === s ? 0 : 1) - (b.startsWith("S") === s ? 0 : 1));
}

// ---------------------------------------------------------------- robot editor
let rWaitBase = null;

function portSelect(v) { return `<select>${PORTS.map((p) => `<option${p === v ? " selected" : ""}>${p}</option>`).join("")}</select>`; }
function addSensorRow(port, fwd, left, white, black) {
  const tr_ = document.createElement("tr");
  tr_.innerHTML = `<td>${portSelect(port)}</td><td><input type="number" step="1" value="${fwd}"></td><td><input type="number" step="1" value="${left}"></td>`
    + `<td><input type="number" step="1" value="${white}"></td><td><input type="number" step="1" value="${black}"></td>`
    + `<td><button class="mini">✕</button></td>`;
  tr_.querySelector("button").title = tr("remove");
  tr_.querySelector("button").addEventListener("click", () => { tr_.remove(); rPreview(); });
  $("rSensors").appendChild(tr_);
}
function addMotorRow(port, side, sign) {
  const tr_ = document.createElement("tr");
  tr_.innerHTML = `<td>${portSelect(port)}</td>`
    + `<td><select><option value="L"${side === "L" ? " selected" : ""}>${esc(tr("sideL"))}</option><option value="R"${side === "R" ? " selected" : ""}>${esc(tr("sideR"))}</option></select></td>`
    + `<td><select><option value="1"${+sign === 1 ? " selected" : ""}>${esc(tr("cw"))}</option><option value="-1"${+sign === -1 ? " selected" : ""}>${esc(tr("ccw"))}</option></select></td>`
    + `<td><button class="mini">✕</button></td>`;
  tr_.querySelector("button").title = tr("remove");
  tr_.querySelector("button").addEventListener("click", () => { tr_.remove(); rPreview(); });
  $("rMotors").appendChild(tr_);
}
function fillRobotForm(f, extra) {
  $("rSensors").innerHTML = "";
  f.sensor_ports.forEach((p, i) => addSensorRow(p, f.sensor_pos_mm[i][0], f.sensor_pos_mm[i][1],
    (f.sensor_white || [])[i] ?? 30, (f.sensor_black || [])[i] ?? 4));
  $("rWheel").value = f.wheel_diameter_mm; $("rAxle").value = f.axle_track_mm; $("rGear").value = f.gear_ratio ?? 1;
  $("rMotors").innerHTML = "";
  Object.entries(f.motors || {}).forEach(([port, ms]) => addMotorRow(port, ms[0], ms[1]));
  $("rExtra").value = extra && Object.keys(extra).length ? JSON.stringify(extra, null, 1) : "";
  $("rErr").textContent = "";
  rPreview();
}
function readSensors() {
  return [...$("rSensors").querySelectorAll("tr")].map((row) => {
    const sel = row.querySelector("select"), ins = row.querySelectorAll("input");
    return { port: sel.value, fwd: num(ins[0]), left: num(ins[1]), white: num(ins[2]), black: num(ins[3]) };
  });
}
function readMotors() {
  return [...$("rMotors").querySelectorAll("tr")].map((row) => {
    const s = row.querySelectorAll("select");
    return { port: s[0].value, side: s[1].value, sign: +s[2].value };
  });
}
function collectRobot() {
  const name = $("rName").value.trim();
  if (!name) return { error: tr("errName") };
  const sensors = readSensors(), motors = readMotors();
  if (!sensors.length || sensors.some((s) => [s.fwd, s.left, s.white, s.black].some(isNaN))) return { error: tr("errSensors") };
  if (!motors.some((m) => m.side === "L") || !motors.some((m) => m.side === "R")) return { error: tr("errMotors") };
  const ports = sensors.map((s) => s.port).concat(motors.map((m) => m.port));
  if (new Set(ports).size !== ports.length) return { error: tr("errPorts") };
  const wheel = num($("rWheel")), axle = num($("rAxle")), gear = num($("rGear"));
  if (!(wheel > 0 && axle > 0 && gear > 0)) return { error: tr("errNumber") };
  let extra = {};
  if ($("rExtra").value.trim()) {
    try { extra = JSON.parse($("rExtra").value); } catch (e) { return { error: tr("errJSON") + e.message }; }
    if (typeof extra !== "object" || Array.isArray(extra) || extra === null) return { error: tr("errJSON") + "{…}" };
  }
  const mot = {};
  motors.forEach((m) => { mot[m.port] = [m.side, m.sign]; });
  const overrides = Object.assign({}, extra, {
    sensor_ports: sensors.map((s) => s.port), sensor_pos_mm: sensors.map((s) => [s.fwd, s.left]),
    sensor_white: sensors.map((s) => s.white), sensor_black: sensors.map((s) => Math.min(s.black, s.white - 1)),
    motors: mot, wheel_diameter_mm: wheel, axle_track_mm: axle, gear_ratio: gear });
  return { name, robot: { base: $("rBase").value, overrides } };
}
function rPreview() {
  const cv = $("rPreview"), g = cv.getContext("2d");
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = css("--mat"); g.fillRect(0, 0, cv.width, cv.height);
  const sensors = readSensors().filter((s) => !isNaN(s.fwd) && !isNaN(s.left));
  const axle = num($("rAxle")) || 100, wheel = num($("rWheel")) || 50;
  if (!sensors.length) return;
  const geom = { sensors: sensors.map((s) => [s.fwd, s.left]), axle, wheel, white: sensors.map((s) => s.white),
    black: sensors.map((s) => s.black), ports: sensors.map((s) => s.port) };
  // robot facing up: local forward → world +y, local left → world −x
  const fw = sensors.map((s) => s.fwd), lf = sensors.map((s) => s.left);
  const fMin = Math.min(-80, -wheel / 2, ...fw) - 30, fMax = Math.max(60, ...fw) + 40;
  const lMax = Math.max(axle / 2 + 14, ...lf.map(Math.abs)) + 30;
  const M = fitBox(cv, -lMax, fMin, lMax, fMax);
  drawRobotShape(g, M, 0, 0, Math.PI / 2, geom, null, null, true);
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = css("--muted"); g.font = "22px -apple-system, sans-serif"; g.textAlign = "center";
  g.fillText("↑ " + tr("front"), cv.width / 2, 26);
  g.textAlign = "left";
}
function openRobotEditor() {
  if (!ready) return;
  const v = $("robotSel").value;
  $("rBase").innerHTML = info.profiles.map((p) => opt(p, (T[lang].profiles[p] ? T[lang].profiles[p] + " — " : "") + p)).join("");
  if (v.startsWith("r:") && robots[v.slice(2)]) {
    const name = v.slice(2), rb = robots[name];
    $("rName").value = name; $("rBase").value = rb.base;
    const extra = {};
    Object.keys(rb.overrides).forEach((k) => { if (!EDIT_KEYS.includes(k)) extra[k] = rb.overrides[k]; });
    fillRobotForm(Object.assign({}, fieldsCache[rb.base] || {}, rb.overrides), extra);
  } else {
    const base = v.slice(2);
    let n = 1;
    while (robots[tr("myRobot") + " " + n]) n++;
    $("rName").value = tr("myRobot") + " " + n; $("rBase").value = base;
    loadBase(base);
  }
  $("robotDlg").showModal();
}
function loadBase(base) {
  if (fieldsCache[base]) fillRobotForm(fieldsCache[base], {});
  else { rWaitBase = base; delete asked[base]; requestFields(base); }
}
H.fields = (name) => { if (rWaitBase === name && $("robotDlg").open) { rWaitBase = null; fillRobotForm(fieldsCache[name], {}); } };

$("editRobot").addEventListener("click", openRobotEditor);
$("rBase").addEventListener("change", () => loadBase($("rBase").value));
$("rAddSensor").addEventListener("click", () => {
  const used = readSensors().map((s) => s.port).concat(readMotors().map((m) => m.port));
  const last = readSensors().pop() || { fwd: 170, left: 0, white: 30, black: 4 };
  addSensorRow(freePorts(used, (readSensors()[0] || {}).port)[0] || "A", last.fwd, last.left - 23, last.white, last.black);
  rPreview();
});
$("rAddMotor").addEventListener("click", () => {
  const used = readSensors().map((s) => s.port).concat(readMotors().map((m) => m.port));
  addMotorRow(freePorts(used, (readMotors()[0] || {}).port)[0] || "A", "L", 1);
  rPreview();
});
$("rLayout").addEventListener("click", () => {
  const n = Math.max(1, Math.min(8, Math.round(num($("rCount")) || 3)));
  const gap = num($("rSpacing")) || 23, fwd = num($("rForward")) || 0;
  const old = readSensors(), motorPorts = readMotors().map((m) => m.port);
  const free = freePorts(motorPorts.concat(old.map((s) => s.port)), (old[0] || {}).port);
  $("rSensors").innerHTML = "";
  for (let i = 0; i < n; i++) {
    const o = old[i] || { port: free.shift() || "A", white: (old[0] || {}).white ?? 30, black: (old[0] || {}).black ?? 4 };
    addSensorRow(o.port, fwd, Math.round(((n - 1) / 2 - i) * gap * 10) / 10, o.white, o.black);
  }
  rPreview();
});
["rWheel", "rAxle", "rGear"].forEach((id) => $(id).addEventListener("input", rPreview));
$("rSensors").addEventListener("input", rPreview);
$("rSensors").addEventListener("change", rPreview);
$("rClose").addEventListener("click", () => $("robotDlg").close());
$("rSave").addEventListener("click", () => {
  const r = collectRobot();
  if (r.error) { $("rErr").textContent = r.error; return; }
  robots[r.name] = r.robot;
  saveStore("lfsim.robots", robots);
  fillSelects();
  $("robotSel").value = "r:" + r.name;
  settingsChanged();
  $("robotDlg").close();
});
$("rDelete").addEventListener("click", () => {
  const name = $("rName").value.trim();
  if (robots[name]) {
    delete robots[name];
    saveStore("lfsim.robots", robots);
    fillSelects();
    $("robotSel").value = "p:" + ($("rBase").value || "spike_3s_geo2");
    settingsChanged();
  }
  $("robotDlg").close();
});

// ---------------------------------------------------------------- arena editor
let aKey = 0, aWait = null, aTimer = null;
let aKeyText = {};          // preview request → the route text it checked
let aValidText = null;      // route text the simulator accepted (null = not checked or rejected)
let aAfter = null;          // "save" / "export" waiting for the check

function round1(v) { return Math.round(v * 10) / 10; }
function arenaToText(spec) {
  const out = [];
  if (spec.line_width_mm) out.push("W " + spec.line_width_mm);
  for (const s of spec.segs || []) out.push(s[0] + " " + s.slice(1).map(round1).join(" "));
  for (const e of spec.extras || []) out.push("M " + e.map((p) => round1(p[0]) + " " + round1(p[1])).join(" "));
  return out.join("\n") + "\n";
}
function parseArena(text) {
  const spec = { segs: [], extras: [] };
  text.split("\n").forEach((raw, i) => {
    const line = raw.replace(/#.*/, "").trim();
    if (!line) return;
    const parts = line.split(/[\s,]+/), k = parts[0].toUpperCase(), v = parts.slice(1).map(Number);
    const bad = (msg) => { throw new Error(tr("errLine", { n: i + 1, msg })); };
    if (v.some(isNaN)) bad(tr("errArgs"));
    if (k === "S") { if (v.length !== 1) bad(tr("errArgs")); spec.segs.push(["S", v[0]]); }
    else if (k === "A") { if (v.length !== 2) bad(tr("errArgs")); spec.segs.push(["A", v[0], v[1]]); }
    else if (k === "T") { if (v.length !== 1) bad(tr("errArgs")); spec.segs.push(["T", v[0]]); }
    else if (k === "M") {
      if (v.length < 4 || v.length % 2) bad(tr("errArgs"));
      const pts = []; for (let j = 0; j < v.length; j += 2) pts.push([v[j], v[j + 1]]);
      spec.extras.push(pts);
    } else if (k === "W") { if (v.length !== 1) bad(tr("errArgs")); spec.line_width_mm = v[0]; }
    else bad(tr("errUnknown", { p: parts[0] }));
  });
  if (!spec.segs.length) throw new Error(tr("errEmpty"));
  return spec;
}
function aClearPreview() {
  const cv = $("aPreview"), g = cv.getContext("2d");
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = css("--mat"); g.fillRect(0, 0, cv.width, cv.height);
  $("aInfo").textContent = "";
}
function aPreview(now) {
  aValidText = null;
  clearTimeout(aTimer);
  const check = () => {
    let spec;
    try { spec = parseArena($("aText").value); } catch (e) { $("aErr").textContent = e.message; aAfter = null; aClearPreview(); return; }
    $("aErr").textContent = "";
    $("aInfo").textContent = tr("checking");
    aKey += 1;
    aKeyText = { [aKey]: $("aText").value };
    worker.postMessage({ type: "track", key: "editor:" + aKey, arena: JSON.stringify({ spec }), robot: robotJSON() });
  };
  if (now) check(); else aTimer = setTimeout(check, 350);
}
function whenValid(action) {
  // Run action now if the current route passed the check, otherwise check first and run it after
  if ($("aText").value === aValidText) { action(); return; }
  aAfter = action;
  aPreview(true);
}
H.editorTrack = (key, image) => {
  if (key !== "editor:" + aKey) return;
  const t = makeTrackImage(image), cv = $("aPreview"), g = cv.getContext("2d");
  drawArena(g, cv, t, fitTransform(cv, t));
  $("aInfo").textContent = tr("arenaInfo", { L: (t.length / 1000).toFixed(1), W: (t.w * t.res / 1000).toFixed(1), H: (t.h * t.res / 1000).toFixed(1) });
  aValidText = aKeyText[aKey];
  if (aAfter && $("aText").value === aValidText) { const f = aAfter; aAfter = null; f(); }
};
H.editorError = (msg) => { $("aErr").textContent = msg; aValidText = null; aAfter = null; aClearPreview(); };
H.randomSpec = (spec, level, seed) => {
  if (aWait !== "random" || !$("arenaDlg").open) return;
  aWait = null;
  $("aText").value = arenaToText(spec);
  if (!$("aName").dataset.touched) $("aName").value = `${tr("myArena")} ${level}-${seed}`;
  aPreview();
};
H.builtinSpec = (spec, name) => {
  if (aWait !== "builtin" || !$("arenaDlg").open) return;
  aWait = null;
  if (!spec) { $("aErr").textContent = tr("noSpec"); $("aText").value = EXAMPLE_ARENA; }
  else { $("aText").value = arenaToText(spec); $("aErr").textContent = ""; }
  if (!$("aName").dataset.touched) $("aName").value = `${tr("myArena")} ${name}`;
  aPreview();
};
function arenaMenus() {
  $("aLevel").innerHTML = LEVELS.map((l) => opt(l, T[lang].levels[l])).join("");
  $("aBuiltin").innerHTML = TRACK_ORDER.filter((t) => !t.startsWith("photo") && info.tracks.includes(t))
    .map((t) => opt(t, T[lang].tracks[t] || t)).join("");
}
function openArenaEditor() {
  if (!ready) return;
  arenaMenus();
  const v = $("arenaSel").value;
  delete $("aName").dataset.touched;
  $("aErr").textContent = ""; $("aInfo").textContent = "";
  if (v.startsWith("a:") && arenas[v.slice(2)]) {
    $("aName").value = v.slice(2); $("aName").dataset.touched = "1";
    $("aText").value = arenaToText(arenas[v.slice(2)]);
    aPreview();
  } else if (v.startsWith("rand:")) {
    $("aLevel").value = v.slice(5); $("aSeedIn").value = $("aseed").value;
    aWait = "random";
    worker.postMessage({ type: "random", level: v.slice(5), seed: Math.max(1, Math.round(+$("aseed").value || 1)) });
  } else {
    const name = v.slice(2);
    if (name.startsWith("photo")) { $("aText").value = EXAMPLE_ARENA; $("aName").value = tr("myArena"); aPreview(); }
    else { $("aBuiltin").value = name; aWait = "builtin"; worker.postMessage({ type: "builtin", name }); }
  }
  $("arenaDlg").showModal();
}
$("editArena").addEventListener("click", openArenaEditor);
$("aText").addEventListener("input", aPreview);
$("aName").addEventListener("input", () => { $("aName").dataset.touched = "1"; });
$("aLoadRandom").addEventListener("click", () => {
  aWait = "random";
  worker.postMessage({ type: "random", level: $("aLevel").value, seed: Math.max(1, Math.round(+$("aSeedIn").value || 1)) });
});
$("aLoadBuiltin").addEventListener("click", () => { aWait = "builtin"; worker.postMessage({ type: "builtin", name: $("aBuiltin").value }); });
$("aClose").addEventListener("click", () => $("arenaDlg").close());
$("aSave").addEventListener("click", () => {
  if (!$("aName").value.trim()) { $("aErr").textContent = tr("errName"); return; }
  whenValid(() => {
    const name = $("aName").value.trim();
    arenas[name] = parseArena($("aText").value);
    saveStore("lfsim.arenas", arenas);
    fillSelects();
    $("arenaSel").value = "a:" + name;
    arenaControls();
    settingsChanged();
    $("arenaDlg").close();
  });
});
$("aDelete").addEventListener("click", () => {
  const name = $("aName").value.trim();
  if (arenas[name]) {
    delete arenas[name];
    saveStore("lfsim.arenas", arenas);
    fillSelects();
    $("arenaSel").value = "t:main";
    arenaControls();
    settingsChanged();
  }
  $("arenaDlg").close();
});
$("aExport").addEventListener("click", () => whenValid(() => {
  const spec = parseArena($("aText").value);
  const name = $("aName").value.trim() || "arena";
  const blob = new Blob([JSON.stringify({ format: "lfsim-arena/1", name, spec }, null, 1)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = name.replace(/[^\w\-가-힣 ]+/g, "_") + ".json";
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}));
$("aImportBtn").addEventListener("click", () => $("aImport").click());
$("aImport").addEventListener("change", async () => {
  const f = $("aImport").files[0];
  if (!f) return;
  try {
    const obj = JSON.parse(await f.text());
    const spec = obj.spec || obj;
    if (!Array.isArray(spec.segs)) throw new Error("no segs");
    $("aText").value = arenaToText(spec);
    $("aName").value = obj.name || f.name.replace(/\.json$/i, ""); $("aName").dataset.touched = "1";
    aPreview();
  } catch (e) { $("aErr").textContent = tr("errJSON") + e.message; }
  $("aImport").value = "";
});
H.lang = () => { if ($("arenaDlg").open) arenaMenus(); };
