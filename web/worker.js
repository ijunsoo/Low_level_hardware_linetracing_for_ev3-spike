// Web Worker: runs the Python simulator in Pyodide so the page stays responsive.
// Messages in:  {type: "init"} · {type: "track", track, profile} · {type: "run", ...} · {type: "eval", ...}
// Messages out: ready · track · robot · frames · console · result · world · eval_done · error
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
      const t = call("track_image", { name: m.track, profile: m.profile });
      postMessage({ type: "track", track: m.track, profile: m.profile, image: t });
    } else if (m.type === "run") {
      call("run_program", {
        code: m.code, profile: m.profile, track: m.track, offset: m.offset, seed: m.seed,
        lost: m.lost, limit: m.limit,
      });
    } else if (m.type === "eval") {
      call("evaluate", { code: m.code, profile: m.profile, track: m.track, worlds: m.worlds, lost: m.lost, limit: m.limit });
    }
  } catch (err) {
    postMessage({ type: "error", message: String(err && err.message ? err.message : err) });
  }
};
