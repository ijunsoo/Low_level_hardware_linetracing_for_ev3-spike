// Main page: program, robot and arena pickers, runs in the worker, live drawing and playback.
// The robot and arena editors are in editors.js and use the helpers here.
"use strict";
const $ = (id) => document.getElementById(id);
const view = $("view"), vctx = view.getContext("2d");
const chart = $("chart"), cctx = chart.getContext("2d");

const PROFILE_MAIN = ["spike_3s_geo2", "spike_3s_geo2x", "ev3_3s_geo"];
const TRACK_ORDER = ["easy", "main", "hard", "rightangle", "photo@1.3", "photo", "mrc", "mrc_real"];
const LEVELS = ["easy", "medium", "hard"];
const LIGHT = { RED: "#ff3b30", GREEN: "#34c759", YELLOW: "#ffcc00", ORANGE: "#ff9500", BLUE: "#0a84ff", VIOLET: "#af52de",
  MAGENTA: "#ff2d92", CYAN: "#32d2f0", WHITE: "#ffffff" };

let worker = null, ready = false, busy = false, started = false;
let programs = [], info = { profiles: [], tracks: [] };
const fieldsCache = {};                 // built-in profile → editable fields (for drawing and the editor)
const H = {};                           // hooks set by editors.js
let trackImg = null, trackKey = "", pendingKey = "";
let runGeom = null;                     // robot geometry of the current run
let frames = new Float32Array(0), nRows = 0, cols = 10;
let result = null, lights = [], worlds = [], evalBase = null;   // evalBase: the default world of ▶ Run
let playT = 0, playing = false, lastTs = 0;

// ---------------------------------------------------------------- saved robots and arenas (this browser)
function loadStore(key) { try { return JSON.parse(localStorage.getItem(key) || "{}") || {}; } catch (e) { return {}; } }
function saveStore(key, obj) { try { localStorage.setItem(key, JSON.stringify(obj)); } catch (e) { /* private mode: keep in memory */ } }
const robots = loadStore("lfsim.robots");          // name → {base, overrides}
const arenas = loadStore("lfsim.arenas");          // name → {segs, extras, line_width_mm?}

function robotJSON(v) {
  v = v || $("robotSel").value;
  if (v.startsWith("r:") && robots[v.slice(2)]) return JSON.stringify(robots[v.slice(2)]);
  return JSON.stringify(v.slice(2) || "spike_3s_geo2");
}
function arenaJSON(v) {
  v = v || $("arenaSel").value;
  if (v.startsWith("rand:")) return JSON.stringify({ random: { level: v.slice(5), seed: Math.max(1, Math.round(+$("aseed").value || 1)) } });
  if (v.startsWith("a:") && arenas[v.slice(2)]) return JSON.stringify({ spec: arenas[v.slice(2)] });
  return JSON.stringify(v.slice(2) || "main");
}
function robotGeom(v) {
  // Shape of the selected robot for drawing before a run
  v = v || $("robotSel").value;
  let f = null;
  if (v.startsWith("r:") && robots[v.slice(2)]) f = Object.assign({}, fieldsCache[robots[v.slice(2)].base] || {}, robots[v.slice(2)].overrides);
  else if (v.startsWith("p:")) { f = fieldsCache[v.slice(2)]; if (!f && ready) requestFields(v.slice(2)); }
  if (!f || !f.sensor_pos_mm) return null;
  return { sensors: f.sensor_pos_mm, axle: +f.axle_track_mm, wheel: +f.wheel_diameter_mm, white: f.sensor_white, black: f.sensor_black,
    ports: f.sensor_ports };
}
const asked = {};
function requestFields(name) { if (!asked[name]) { asked[name] = 1; worker.postMessage({ type: "fields", name }); } }

// ---------------------------------------------------------------- language and menus
function applyLang() {
  document.documentElement.lang = lang;
  document.querySelectorAll("[data-i18n]").forEach((el) => { el.textContent = tr(el.dataset.i18n); });
  document.querySelectorAll("[data-i18n-title]").forEach((el) => { el.title = tr(el.dataset.i18nTitle); });
  document.querySelectorAll(".lang button").forEach((b) => b.classList.toggle("on", b.dataset.lang === lang));
  document.title = tr("title");
  fillSelects();
  showResult();
  renderEval();
  if (H.lang) H.lang();
}
document.querySelectorAll(".lang button").forEach((b) => b.addEventListener("click", () => {
  lang = b.dataset.lang;
  try { localStorage.setItem("lfsim.lang", lang); } catch (e) { /* ignore */ }
  applyLang();
}));

function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;" })[c]); }
function opt(v, label) { return `<option value="${esc(v)}">${esc(label)}</option>`; }

function fillSelects() {
  const keep = { prog: $("prog").value, robot: $("robotSel").value, arena: $("arenaSel").value };
  $("prog").innerHTML = programs.map((p) => opt(p.id, p.title[lang] || p.title.en)).join("");
  const others = info.profiles.filter((p) => !PROFILE_MAIN.includes(p));
  const mine = Object.keys(robots).sort();
  $("robotSel").innerHTML = `<optgroup label="${esc(tr("builtIn"))}">`
    + PROFILE_MAIN.filter((p) => info.profiles.includes(p)).map((p) => opt("p:" + p, T[lang].profiles[p])).join("") + "</optgroup>"
    + (mine.length ? `<optgroup label="${esc(tr("myRobots"))}">` + mine.map((n) => opt("r:" + n, n)).join("") + "</optgroup>" : "")
    + (others.length ? `<optgroup label="${esc(tr("moreProfiles"))}">` + others.map((p) => opt("p:" + p, p)).join("") + "</optgroup>" : "");
  const tracks = TRACK_ORDER.filter((t) => t.includes("@") || info.tracks.includes(t));
  const myA = Object.keys(arenas).sort();
  $("arenaSel").innerHTML = `<optgroup label="${esc(tr("builtIn"))}">` + tracks.map((t) => opt("t:" + t, T[lang].tracks[t] || t)).join("") + "</optgroup>"
    + `<optgroup label="${esc(tr("randomArenas"))}">` + LEVELS.map((l) => opt("rand:" + l, T[lang].levels[l])).join("") + "</optgroup>"
    + (myA.length ? `<optgroup label="${esc(tr("myArenas"))}">` + myA.map((n) => opt("a:" + n, n)).join("") + "</optgroup>" : "");
  if (keep.prog) $("prog").value = keep.prog;
  if (keep.robot) $("robotSel").value = keep.robot;
  if (keep.arena) $("arenaSel").value = keep.arena;
  if (!$("robotSel").value) $("robotSel").selectedIndex = 0;
  if (!$("arenaSel").value) $("arenaSel").selectedIndex = 0;
  arenaControls();
}
function arenaControls() {
  const rand = $("arenaSel").value.startsWith("rand:");
  $("aseed").hidden = !rand; $("dice").hidden = !rand;
}

// ---------------------------------------------------------------- worker
function setStatus(s) { $("status").textContent = s; }
function setBusy(b) {
  busy = b;
  $("run").disabled = !ready || b;
  $("eval").disabled = !ready || b;
  $("stop").disabled = !b;
  for (const id of ["prog", "robotSel", "arenaSel", "aseed", "dice", "editRobot", "editArena"]) $(id).disabled = b;
}
function startWorker() {
  ready = false;
  setBusy(false);
  Object.keys(asked).forEach((k) => delete asked[k]);
  worker = new Worker("worker.js");
  worker.onmessage = onWorker;
  worker.postMessage({ type: "init" });
}
function consoleLine(text, cls) {
  const d = document.createElement("div");
  if (cls) d.className = cls;
  d.textContent = text;
  $("console").appendChild(d);
  $("console").scrollTop = $("console").scrollHeight;
}
function floats(u8) { return new Float32Array(u8.slice().buffer); }

function onWorker(e) {
  const m = e.data;
  if (m.type === "status") setStatus(tr(m.key));
  else if (m.type === "ready") {
    info = m.info; ready = true; fillSelects(); setBusy(false); setStatus(tr("ready"));
    PROFILE_MAIN.forEach(requestFields);
    if (!started) {                     // first start: load and run the default program (not after Stop)
      started = true;
      if (!$("prog").value && programs.length) $("prog").value = programs[0].id;
      loadProgram($("prog").value, true);
    } else requestTrack();
  } else if (m.type === "track") {
    if (m.key && m.key.startsWith("editor")) { if (H.editorTrack) H.editorTrack(m.key, m.image); }
    else if (m.key === pendingKey) { trackImg = makeTrackImage(m.image); trackKey = m.key; pendingKey = ""; }
  } else if (m.type === "fields") {
    fieldsCache[m.name] = m.fields;
    if (H.fields) H.fields(m.name, m.fields);
  } else if (m.type === "randomSpec") {
    if (H.randomSpec) H.randomSpec(m.spec, m.level, m.seed);
  } else if (m.type === "builtinSpec") {
    if (H.builtinSpec) H.builtinSpec(m.spec, m.name);
  } else if (m.type === "robot") {
    runGeom = m; cols = m.cols;
  } else if (m.type === "frames") {
    const f = floats(m.data);
    const grown = new Float32Array(frames.length + f.length);
    grown.set(frames); grown.set(f, frames.length);
    frames = grown; nRows = Math.floor(frames.length / cols);
    $("scrub").max = frames[(nRows - 1) * cols].toFixed(3); $("scrub").disabled = false; $("pp").disabled = false;
  } else if (m.type === "console") {
    consoleLine(m.text);
  } else if (m.type === "result") {
    result = m.result; lights = m.lights || [];
    if (m.error) consoleLine(m.error, "err");
    setBusy(false); setStatus(""); showResult();
  } else if (m.type === "world") {
    worlds.push({ k: m.k, result: m.result, trail: floats(m.trail), world: m.world || {} });
    evalBase = m.base || null;
    if (m.error) consoleLine(m.error, "err");
    if (m.k + 2 <= 8) setStatus(tr("evaluating") + " " + (m.k + 2) + "/8");
    renderEval();
  } else if (m.type === "eval_done") {
    setBusy(false); setStatus("");
  } else if (m.type === "error") {
    if (m.context === "track" && m.key && m.key.startsWith("editor")) { if (H.editorError) H.editorError(m.message); return; }
    if (m.context === "track") pendingKey = "";
    consoleLine(m.message, "err");
    if (m.context === "run" || m.context === "eval") { setBusy(false); setStatus(""); }
  }
}

// ---------------------------------------------------------------- program / robot / arena
async function loadPrograms() { programs = await (await fetch("programs.json")).json(); }
function loadProgram(id, autorun) {
  const p = programs.find((x) => x.id === id);
  if (!p) return;
  $("code").value = p.code;
  $("robotSel").value = "p:" + p.profile;
  $("arenaSel").value = "t:" + p.track;
  arenaControls();
  settingsChanged();
  if (autorun) run();
}
function clearResults() {
  // The trail, result, chart, world tests and console belong to the settings they were run with
  frames = new Float32Array(0); nRows = 0; result = null; lights = []; worlds = []; evalBase = null; runGeom = null;
  playT = 0; playing = false;
  $("console").textContent = ""; $("result").innerHTML = ""; $("evalbox").innerHTML = "";
  $("scrub").max = 0; $("scrub").value = 0; $("scrub").disabled = true; $("pp").disabled = true;
}
function settingsChanged() {
  clearResults();
  requestTrack();
}
function requestTrack() {
  if (!ready) return;
  const key = arenaJSON() + "|" + robotJSON();
  if (key === trackKey || key === pendingKey) return;
  pendingKey = key;
  worker.postMessage({ type: "track", key, arena: arenaJSON(), robot: robotJSON() });
}

$("prog").addEventListener("change", () => loadProgram($("prog").value, false));
$("robotSel").addEventListener("change", settingsChanged);
$("arenaSel").addEventListener("change", () => { arenaControls(); settingsChanged(); });
$("aseed").addEventListener("change", settingsChanged);
$("dice").addEventListener("click", () => { $("aseed").value = 1 + Math.floor(Math.random() * 99999); settingsChanged(); });
$("code").addEventListener("keydown", (e) => {
  if (e.key === "Tab") {
    e.preventDefault();
    const ta = e.target, s = ta.selectionStart;
    ta.value = ta.value.slice(0, s) + "    " + ta.value.slice(ta.selectionEnd);
    ta.selectionStart = ta.selectionEnd = s + 4;
  }
});

function resetRun() {
  clearResults();
  playing = true;
}
function opts() {
  return { code: $("code").value, robot: robotJSON(), arena: arenaJSON(),
    offset: +$("offset").value || 0, seed: +$("seed").value || 0, lost: +$("lost").value || 450, limit: +$("limit").value || 40 };
}
function run() {
  if (!ready || busy) return;
  requestTrack(); resetRun(); setBusy(true); setStatus(tr("running"));
  worker.postMessage(Object.assign({ type: "run" }, opts()));
}
$("run").addEventListener("click", run);
$("eval").addEventListener("click", () => {
  if (!ready || busy) return;
  requestTrack(); resetRun(); playing = false; setBusy(true); setStatus(tr("evaluating") + " 1/8");
  worker.postMessage(Object.assign({ type: "eval", worlds: 8 }, opts()));
});
$("stop").addEventListener("click", () => {
  worker.terminate(); setStatus(tr("stopped")); trackKey = ""; pendingKey = ""; startWorker();
});

// ---------------------------------------------------------------- playback and results
$("pp").addEventListener("click", () => {
  if (!nRows) return;
  const tEnd = frames[(nRows - 1) * cols];
  if (!playing && playT >= tEnd - 1e-6 && !busy) playT = 0;
  playing = !playing;
});
$("scrub").addEventListener("input", () => { playT = +$("scrub").value; playing = false; });

function rowAt(t) {
  if (!nRows) return -1;
  let lo = 0, hi = nRows - 1;
  while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (frames[mid * cols] <= t) lo = mid; else hi = mid - 1; }
  return lo;
}
function reasonName(r) { return r === "lost" ? tr("lost_") : tr(r); }
function showResult() {
  if (!result) return;
  const r = result, L = (r.track_mm || (trackImg ? trackImg.length : 0)) / 1000;
  const p = r.track_mm ? Math.round(100 * r.progress_mm / r.track_mm) : 0;
  const txt = r.reason === "finish"
    ? tr("resultFinish", { t: (+r.time_s).toFixed(2), p, L: L.toFixed(1), lat: (+r.mean_lat_mm).toFixed(0) })
    : tr("resultOther", { t: (+r.time_s || 0).toFixed(2), p, L: L.toFixed(1) });
  $("result").innerHTML = `<span class="badge ${esc(r.reason)}">${esc(reasonName(r.reason))}</span> <span>${esc(txt)}</span>`;
}
function renderEval() {
  if (!worlds.length) return;
  const fin = worlds.filter((w) => w.result.reason === "finish");
  const mean = fin.length ? fin.reduce((a, w) => a + w.result.time_s, 0) / fin.length : 0;
  let h = `<p class="note">${esc(tr("evalHead"))}</p>`;
  if (evalBase) h += `<p class="note">${esc(tr("evalBase", { lag: evalBase.cmd_lag_ms.toFixed(0), bat: Math.round(100 * evalBase.battery) }))}</p>`;
  h += `<p><b>${esc(tr("finished", { n: fin.length, N: worlds.length }))}</b>`
    + (fin.length ? ` · ${esc(tr("meanTime", { t: mean.toFixed(2) }))}` : "") + "</p>";
  h += `<table><tr><th>${esc(tr("world"))}</th><th>${esc(tr("reason"))}</th><th>${esc(tr("time"))}</th><th>${esc(tr("cmdLag"))}</th><th>${esc(tr("battery"))}</th></tr>`;
  for (const w of worlds) {
    const r = w.result, x = w.world;
    h += `<tr><td class="n">${w.k + 1}</td><td><span class="badge ${esc(r.reason)}">${esc(reasonName(r.reason))}</span></td><td class="n">${(+r.time_s || 0).toFixed(2)} s</td>`
      + `<td class="n">${x.cmd_lag_ms != null ? (+x.cmd_lag_ms).toFixed(0) + " ms" : ""}</td><td class="n">${x.battery != null ? Math.round(100 * x.battery) + "%" : ""}</td></tr>`;
  }
  $("evalbox").innerHTML = h + "</table>";
}

// ---------------------------------------------------------------- drawing helpers (also used by editors.js)
function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }

function makeTrackImage(t) {
  const c = document.createElement("canvas");
  c.width = t.w; c.height = t.h;
  const g = c.getContext("2d");
  const im = g.createImageData(t.w, t.h);
  const ink = css("--line") || "#1c1e23";
  const rgb = [parseInt(ink.slice(1, 3), 16), parseInt(ink.slice(3, 5), 16), parseInt(ink.slice(5, 7), 16)];
  const d = t.data;
  for (let i = 0; i < d.length; i++) {
    if (d[i]) { const j = i * 4; im.data[j] = rgb[0]; im.data[j + 1] = rgb[1]; im.data[j + 2] = rgb[2]; im.data[j + 3] = 255; }
  }
  g.putImageData(im, 0, 0);
  return { canvas: c, x0: t.x0, y0: t.y0, res: t.res, w: t.w, h: t.h, start: t.start, end: t.end, heading: t.heading || 0, length: t.length };
}
function fitTransform(cv, t) {
  // world (mm, y up) → canvas (px, y down), fitting the arena
  const W = cv.width, Hh = cv.height, pad = 24;
  const ww = t.w * t.res, wh = t.h * t.res;
  const s = Math.min((W - 2 * pad) / ww, (Hh - 2 * pad) / wh);
  const ox = (W - ww * s) / 2, oy = (Hh - wh * s) / 2;
  return { s, a: s, d: -s, e: ox - t.x0 * s, f: oy + (t.y0 + wh) * s };
}
function drawArena(g, cv, t, M) {
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = css("--mat"); g.fillRect(0, 0, cv.width, cv.height);
  if (!t) return;
  g.save();
  g.setTransform(M.a, 0, 0, M.d, M.e, M.f);
  g.drawImage(t.canvas, t.x0, t.y0, t.w * t.res, t.h * t.res);
  g.fillStyle = "#1f9d55"; g.beginPath(); g.arc(t.start[0], t.start[1], 18, 0, 2 * Math.PI); g.fill();
  g.fillStyle = "#d64535"; g.beginPath(); g.arc(t.end[0], t.end[1], 18, 0, 2 * Math.PI); g.fill();
  g.restore();
}
function lightAt(t) {
  let c = null;
  for (const [lt, v] of lights) { if (lt <= t) c = v; else break; }
  return c ? LIGHT[c.replace(/^.*\./, "").toUpperCase()] || null : null;
}
function drawRobotShape(g, M, x, y, h, geom, raws, lightColor, labels) {
  // Robot seen from above in world mm: axle middle at (x, y), heading h (counter-clockwise from +x)
  g.save();
  g.setTransform(M.a, 0, 0, M.d, M.e, M.f);
  g.translate(x, y); g.rotate(h);
  const lats = geom.sensors.map((s) => +s[1]);
  const fwds = geom.sensors.map((s) => +s[0]);
  const fwd = Math.max(...fwds), back = Math.min(0, ...fwds);
  const half = geom.axle / 2, wheelR = geom.wheel / 2;
  g.fillStyle = css("--robot"); g.globalAlpha = 0.9;
  g.fillRect(Math.min(-70, back - 20), -half + 12, Math.max(120, 50 - Math.min(-70, back - 20)), Math.max(20, geom.axle - 24));
  if (fwd > 40) g.fillRect(40, -16, fwd - 40, 32);
  g.fillRect(Math.min(...fwds) - 14, Math.min(...lats) - 14, fwd - Math.min(...fwds) + 28, Math.max(...lats) - Math.min(...lats) + 28);
  g.globalAlpha = 1; g.fillStyle = "#111";
  g.fillRect(-wheelR, half - 6, 2 * wheelR, 18); g.fillRect(-wheelR, -half - 12, 2 * wheelR, 18);
  if (lightColor) { g.fillStyle = lightColor; g.beginPath(); g.arc(-20, 0, 14, 0, 2 * Math.PI); g.fill(); }
  geom.sensors.forEach((s, i) => {
    let v = 235;
    if (raws) {
      const n = Math.max(0, Math.min(1, (raws[i] - geom.black[i]) / Math.max(1e-6, geom.white[i] - geom.black[i])));
      v = Math.round(30 + 225 * n);
    }
    g.fillStyle = `rgb(${v},${v},${v})`; g.strokeStyle = "#ff3b30"; g.lineWidth = 3 / M.s;
    g.beginPath(); g.arc(+s[0], +s[1], 7, 0, 2 * Math.PI); g.fill(); g.stroke();
    if (labels && geom.ports) {
      g.save(); g.translate(+s[0] + 14, +s[1]); g.rotate(-h); g.scale(1, -1);
      g.fillStyle = "#1c1e23"; g.font = `bold ${Math.round(14 / M.s)}px sans-serif`; g.fillText(geom.ports[i] || "", 0, 4 / M.s);
      g.restore();
    }
  });
  g.restore();
}

// ---------------------------------------------------------------- main view
function draw() {
  const M = trackImg ? fitTransform(view, trackImg) : null;
  drawArena(vctx, view, trackImg, M);
  if (!trackImg) return;
  vctx.save();
  vctx.setTransform(M.a, 0, 0, M.d, M.e, M.f);
  for (const w of worlds) {
    const p = w.trail; if (p.length < 4) continue;
    vctx.strokeStyle = w.result.reason === "finish" ? "rgba(31,157,85,.75)" : "rgba(214,69,53,.75)";
    vctx.lineWidth = 3 / M.s; vctx.beginPath(); vctx.moveTo(p[0], p[1]);
    for (let i = 2; i < p.length; i += 2) vctx.lineTo(p[i], p[i + 1]);
    vctx.stroke();
  }
  const row = rowAt(playT);
  if (row > 0) {
    vctx.strokeStyle = css("--trail"); vctx.lineWidth = 3 / M.s; vctx.globalAlpha = 0.8;
    vctx.beginPath(); vctx.moveTo(frames[1], frames[2]);
    const step = Math.max(1, Math.floor(row / 4000));
    for (let i = step; i <= row; i += step) vctx.lineTo(frames[i * cols + 1], frames[i * cols + 2]);
    vctx.stroke(); vctx.globalAlpha = 1;
  }
  vctx.restore();
  const hud = $("hud");
  if (row >= 0 && runGeom) {
    const b = row * cols;
    drawRobotShape(vctx, M, frames[b + 1], frames[b + 2], frames[b + 6], runGeom, frames.subarray(b + 7, b + cols), lightAt(frames[b]));
    hud.hidden = false;
    hud.textContent = `t ${frames[b].toFixed(2)} s   L ${frames[b + 4].toFixed(0).padStart(4)}%  R ${frames[b + 5].toFixed(0).padStart(4)}%   ↔ ${frames[b + 3].toFixed(0)} mm`;
  } else {
    hud.hidden = true;
    const geom = !worlds.length && robotGeom();
    if (geom) drawRobotShape(vctx, M, trackImg.start[0], trackImg.start[1], trackImg.heading, geom, null, null, true);
  }
}
function drawChart() {
  const W = chart.width, Hh = chart.height, g = cctx;
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.clearRect(0, 0, W, Hh);
  const show = nRows ? "" : "none";
  if (chart.style.display !== show) { chart.style.display = show; document.querySelector(".legend").style.display = show; }
  if (!nRows) return;
  const tEnd = Math.max(frames[(nRows - 1) * cols], 1e-3);
  const padL = 46, padR = 10, padT = 10, padB = 22;
  const X = (t) => padL + (W - padL - padR) * t / tEnd;
  let latMax = 50;
  for (let i = 0; i < nRows; i++) latMax = Math.max(latMax, Math.abs(frames[i * cols + 3]));
  const Ylat = (v) => padT + (Hh - padT - padB) * (1 - v / latMax);
  const Yd = (v) => padT + (Hh - padT - padB) * (0.5 - v / 220);
  g.strokeStyle = css("--edge"); g.lineWidth = 1;
  g.beginPath(); g.moveTo(padL, Yd(0)); g.lineTo(W - padR, Yd(0)); g.stroke();
  g.fillStyle = css("--muted"); g.font = "20px -apple-system, sans-serif";
  g.fillText(latMax.toFixed(0) + " mm", 2, padT + 16); g.fillText(tEnd.toFixed(1) + " s", W - padR - 70, Hh - 2);
  const step = Math.max(1, Math.floor(nRows / (W - padL)));
  const line = (col, Y, color) => {
    g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
    for (let i = 0; i < nRows; i += step) { const x = X(frames[i * cols]), y = Y(frames[i * cols + col]); if (i) g.lineTo(x, y); else g.moveTo(x, y); }
    g.stroke();
  };
  line(4, Yd, "rgba(31,157,85,.8)"); line(5, Yd, "rgba(201,138,0,.8)"); line(3, Ylat, "#2f6fdf");
  g.strokeStyle = css("--text"); g.lineWidth = 1.5; g.beginPath(); g.moveTo(X(playT), padT); g.lineTo(X(playT), Hh - padB); g.stroke();
}
function tick(ts) {
  const dt = lastTs ? Math.min(0.1, (ts - lastTs) / 1000) : 0;
  lastTs = ts;
  if (playing && nRows) {
    const tEnd = frames[(nRows - 1) * cols];
    playT = Math.min(playT + dt * +$("speed").value, tEnd);
    if (playT >= tEnd && !busy) playing = false;
  }
  $("pp").textContent = playing ? "⏸" : "▶";
  $("scrub").value = playT; $("tlabel").textContent = playT.toFixed(2) + " s";
  draw(); drawChart();
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- start
applyLang();
loadPrograms().then(() => { fillSelects(); startWorker(); });
requestAnimationFrame(tick);
