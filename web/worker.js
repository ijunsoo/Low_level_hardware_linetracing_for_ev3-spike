// Web Worker: runs the Python simulator in Pyodide so the page stays responsive.
// Robots and arenas are passed as JSON text (see web/runner.py).
// Messages in:  init · track {arena, robot} · fields {name} · random {level, seed} · run {...} · eval {...}
// Messages out: status · ready · track · fields · randomSpec · robot · frames · console · result · world · eval_done · error
const PYODIDE_URL = "https://cdn.jsdelivr.net/pyodide/v0.29.5/full/";
importScripts(PYODIDE_URL + "pyodide.js");

let py = null;

async function init() {
  postMessage({ type: "status", key: "loadingPython" });
  py = await loadPyodide({ indexURL: PYODIDE_URL });
  postMessage({ type: "status", key: "loadingNumpy" });
  await py.loadPackage("numpy");
  postMessage({ type: "status", key: "loadingSim" });
  const bundle = await (await fetch("sim_bundle.json")).json();
  for (const [path, text] of Object.entries(bundle)) {
    const full = "/sim/" + path;
    py.FS.mkdirTree(full.substring(0, full.lastIndexOf("/")));
    py.FS.writeFile(full, text);
  }
  py.runPython("import sys; sys.path.insert(0, '/sim'); import runner");
  const info = py.runPython("runner.info()").toJs({ dict_converter: Object.fromEntries });
  postMessage({ type: "ready", info });
}

function call(name, kwargs) {
  const fn = py.globals.get("runner")[name];
  const out = fn.callKwargs(kwargs);
  const js = out && out.toJs ? out.toJs({ dict_converter: Object.fromEntries }) : out;
  if (out && out.destroy) out.destroy();
  return js;
}

self.onmessage = async (e) => {
  const m = e.data;
  try {
    if (m.type === "init") {
      await init();
    } else if (m.type === "track") {
      const t = call("track_image", { arena_json: m.arena, robot_json: m.robot });
      postMessage({ type: "track", key: m.key, image: t });
    } else if (m.type === "fields") {
      postMessage({ type: "fields", name: m.name, fields: JSON.parse(call("robot_fields", { name: m.name })) });
    } else if (m.type === "builtin") {
      postMessage({ type: "builtinSpec", name: m.name, spec: JSON.parse(call("builtin_spec", { name: m.name })) });
    } else if (m.type === "random") {
      postMessage({ type: "randomSpec", level: m.level, seed: m.seed, spec: JSON.parse(call("random_spec", { level: m.level, seed: m.seed })) });
    } else if (m.type === "run") {
      call("run_program", {
        code: m.code, robot: m.robot, arena_: m.arena, offset: m.offset, seed: m.seed, lost: m.lost, limit: m.limit,
      });
    } else if (m.type === "eval") {
      call("evaluate", { code: m.code, robot: m.robot, arena_: m.arena, worlds: m.worlds, lost: m.lost, limit: m.limit });
    }
  } catch (err) {
    const msg = String(err && err.message ? err.message : err);
    // Python errors arrive as a full traceback: keep the last line (the error itself) for the page
    const last = msg.trim().split("\n").pop();
    postMessage({ type: "error", context: m.type, key: m.key, message: last, detail: msg });
  }
};
