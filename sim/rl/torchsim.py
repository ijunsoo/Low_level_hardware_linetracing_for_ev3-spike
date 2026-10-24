# =====================================================================
#  torchsim — fastsim + RuleLF3 + RL 환경의 PyTorch 판 (GPU: Apple MPS / CUDA / CPU)
#
#  numpy 판(fastsim.py·rule_lf3.py·env.py)과 같은 모델·같은 로직이다.
#   - MPS는 float64가 없어 물리는 float32, 규칙기반 정수 연산은 int64 (내림 나눗셈 포함)
#   - 시간은 env별 틱 카운터(정수): now_ms = ticks*1000 // sensor_hz  (StopWatch와 같은 정수 ms)
#   - 수천~수만 대를 한 번에 돌려야 GPU 이득이 난다 (커널 호출 수가 대수와 무관)
#
#  사용:
#    env = TorchEnv(8192, make_pool(...), device="mps")
#    obs = env.reset_all(); obs, rew, done, info = env.step(actions)   # 전부 device 텐서
#  검증: python3 -m rl.torchsim --bench / --check (numpy 판과 통계 비교)
# =====================================================================
import math

import numpy as np
import torch

from . import _SIM_DIR  # noqa: F401
import core
from .fastsim import TrackPool, sample_world
from .rule_lf3 import load_constants

F32, I64 = torch.float32, torch.int64


def pick_device(pref="auto"):
    if pref != "auto":
        return torch.device(pref)
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def fdiv(a, b):
    """파이썬 정수 // 와 같은 내림 나눗셈 (음수 포함)."""
    return torch.div(a, b, rounding_mode="floor")


class TorchPool:
    def __init__(self, pool: TrackPool, device):
        d = device
        self.np = pool
        self.n = pool.n
        self.flat = torch.as_tensor(pool.flat, dtype=torch.uint8, device=d)
        self.path = torch.as_tensor(pool.path, dtype=F32, device=d)
        self.s = torch.as_tensor(pool.s, dtype=F32, device=d)
        for k in ("x0", "y0"):
            setattr(self, k, torch.as_tensor(getattr(pool, k), dtype=F32, device=d))
        for k in ("W", "H", "base", "p0", "plen"):
            setattr(self, k, torch.as_tensor(getattr(pool, k), dtype=I64, device=d))
        self.length = torch.as_tensor(pool.length, dtype=F32, device=d)


class TorchSim:
    PROG_WIN = (-10, 60)

    def __init__(self, n, pool, base, device, seed=0):
        self.n, self.d, self.base = n, device, dict(base)
        self.pool = pool if isinstance(pool, TorchPool) else TorchPool(pool, device)
        self.hz = int(round(base["sensor_hz"]))
        self.dt = 1.0 / self.hz
        self.gen = torch.Generator(device=device)
        self.gen.manual_seed(seed)
        self.rng = np.random.default_rng(seed)
        dc = np.asarray(base["dc_table"], float)
        # dc 곡선(구간선형)을 '구간별 기울기 × clamp' 합으로 — interp 대체
        self.dc_x = [float(v) for v in dc[:, 0]]
        self.dc_y0 = float(dc[0, 1])
        self.dc_sl = [float((dc[i + 1, 1] - dc[i, 1]) / max(1e-9, dc[i + 1, 0] - dc[i, 0]))
                      for i in range(len(dc) - 1)]
        self.S = len(base["sensor_ports"])
        self.fp_unit = torch.as_tensor(core._footprint_offsets(1.0), dtype=F32, device=device)
        self.M = 64
        z = lambda: torch.zeros(n, dtype=F32, device=device)  # noqa: E731
        self.a_up, self.a_dn, self.vscale, self.k, self.Wax = z(), z(), z(), z(), z()
        self.a_y = z()                    # 회전(좌우 차) 성분 감쇠 — 0이면 끔 (CPU core.py yaw_tau_s와 같음)
        self.noise, self.fp_r = z(), z()
        self.lag = torch.zeros(n, dtype=I64, device=device)
        self.pos = torch.zeros((n, self.S, 2), dtype=F32, device=device)
        self.fp_off = torch.zeros((n, self.fp_unit.shape[0], 2), dtype=F32, device=device)
        self.white = torch.zeros((n, self.S), dtype=F32, device=device)
        self.black = torch.zeros((n, self.S), dtype=F32, device=device)
        self.x, self.y, self.h, self.wl, self.wr = z(), z(), z(), z(), z()
        self.ticks = torch.zeros(n, dtype=I64, device=device)
        self.ti = torch.zeros(n, dtype=I64, device=device)
        self.pi = torch.zeros(n, dtype=I64, device=device)
        self.max_s = z()
        self.buf = torch.full((n, self.M, self.S), 50, dtype=torch.int32, device=device)
        self.head = 0
        self.raw = torch.full((n, self.S), 50, dtype=torch.int32, device=device)
        self.raw_prev = self.raw.clone()
        self.ar = torch.arange(n, device=device)
        # ★실기 보정 (2026-10-17, core.py와 맞춤): 구동 지연 cmd_lag_ms · 자이로 지연 imu_lag_ms ·
        #   제어 주기(실기 ≈735/s < 센서 1,070Hz — 제어는 일부 틱에서만, 나머지 틱은 직전 모터 명령 유지)
        self.MC = 96
        self.cbuf = torch.zeros((n, self.MC, 2), dtype=F32, device=device)
        # 바퀴 흔들림 (core.py와 같은 모델): 바퀴 회전각에 따른 유효 반지름 출렁임
        self.wob = torch.zeros((n, 2), dtype=F32, device=device)
        self.wph = torch.zeros((n, 2), dtype=F32, device=device)
        self.wang = torch.zeros((n, 2), dtype=F32, device=device)
        self.ybuf = torch.zeros((n, self.MC), dtype=F32, device=device)
        self.chead = 0
        self.clag = torch.zeros(n, dtype=I64, device=device)
        self.ylag = torch.zeros(n, dtype=I64, device=device)
        cc = base.get("call_cost_us") or {}
        if "motor_dc" in cc:            # 스텝 비용 = 센서 읽기 S회 + 자이로 + dc 2회 (core와 같은 모델)
            us = self.S * cc.get("color_read", 47.152) + cc.get("gyro_read", 0) + 2 * cc["motor_dc"]
            ctrl_hz = 1e6 / us
        else:
            ctrl_hz = float(self.hz)     # 옛 프로파일: 매 센서 프레임 제어 (보정 전 동작 그대로)
        self.ctrl_step = torch.full((n,), min(1.0, ctrl_hz / self.hz), dtype=F32, device=device)
        self.ctrl_acc = torch.zeros(n, dtype=F32, device=device)
        self._refresh_track_cache()

    # ---------------- 월드·트랙·리셋 (idx = device LongTensor) ----------------
    def set_world(self, idx, w):
        b, d, m = self.base, self.d, len(idx)
        t = lambda v: torch.as_tensor(np.broadcast_to(v, (m,)).astype(np.float32), device=d)  # noqa: E731
        tau_up = w.get("tau_up", b["motor_tau_s"])
        tau_dn = w.get("tau_dn", b.get("motor_tau_dn_s", b["motor_tau_s"]))
        self.a_up[idx] = t(np.exp(-self.dt / np.asarray(tau_up, float)))
        self.a_dn[idx] = t(np.exp(-self.dt / np.asarray(tau_dn, float)))
        self.vscale[idx] = t(np.asarray(w.get("battery", b["battery_scale"]), float) * b.get("gear_ratio", 1.0))
        self.k[idx] = t(math.pi * np.asarray(w.get("wheel_mm", b["wheel_diameter_mm"]), float) / 360.0)
        self.Wax[idx] = t(b["axle_track_mm"] * np.asarray(w.get("w_eff", b.get("w_eff_scale", 1.0)), float))
        ytau = float(b.get("yaw_tau_s", 0.0))
        self.a_y[idx] = t(math.exp(-self.dt / ytau) if ytau > 0 else 0.0)
        self.noise[idx] = t(w.get("noise", b["sensor_noise"]))
        fr = np.asarray(w.get("footprint_mm", b["sensor_footprint_mm"]), float) / 2.0
        self.fp_r[idx] = t(fr)
        self.fp_off[idx] = self.fp_unit[None] * self.fp_r[idx][:, None, None]
        for key, arr in (("cmd_lag_ms", self.clag), ("imu_lag_ms", self.ylag)):
            v = np.clip(np.round(np.asarray(w.get(key, b.get(key, 0.0)), float) / 1000.0 / self.dt), 0, self.MC - 2)
            arr[idx] = torch.as_tensor(np.broadcast_to(v, (m,)).astype(np.int64), device=d)
        if "wobble" in w:                 # (m, 2) 비율, (m, 2) 위상
            self.wob[idx] = torch.as_tensor(np.asarray(w["wobble"], np.float32), device=d)
            self.wph[idx] = torch.as_tensor(np.asarray(w["wobble_ph"], np.float32), device=d)
        if "ctrl_hz" in w:
            self.ctrl_step[idx] = t(np.minimum(1.0, np.asarray(w["ctrl_hz"], float) / self.hz))
        lag = np.clip(np.round(np.asarray(w.get("lag_ms", b.get("sensor_lag_ms", 0.0)), float) / 1000.0 / self.dt), 0, self.M - 2)
        self.lag[idx] = torch.as_tensor(np.broadcast_to(lag, (m,)).astype(np.int64), device=d)
        pos0 = np.asarray(w["pos_abs"], float) if "pos_abs" in w else np.asarray(b["sensor_pos_mm"], float)[None]
        pos = pos0 + (w["pos_err"] if "pos_err" in w else 0.0)
        self.pos[idx] = torch.as_tensor(np.broadcast_to(pos, (m, self.S, 2)).astype(np.float32), device=d)
        wh = np.asarray(b["sensor_white"][:self.S], float)[None] * (w["white_k"] if "white_k" in w else 1.0)
        bk = np.asarray(b["sensor_black"][:self.S], float)[None] * (w["black_k"] if "black_k" in w else 1.0)
        self.white[idx] = torch.as_tensor(np.broadcast_to(wh, (m, self.S)).astype(np.float32), device=d)
        self.black[idx] = torch.as_tensor(np.broadcast_to(bk, (m, self.S)).astype(np.float32), device=d)

    def _refresh_track_cache(self):
        P = self.pool
        self._tx0, self._ty0 = P.x0[self.ti], P.y0[self.ti]
        self._tW, self._tH, self._tbase = P.W[self.ti], P.H[self.ti], P.base[self.ti]
        self._p0, self._pend = P.p0[self.ti], P.p0[self.ti] + P.plen[self.ti] - 1

    def set_tracks(self, idx, ti):
        self.ti[idx] = ti
        self._refresh_track_cache()

    def reset_pose(self, idx, start_idx, lat_mm, head_deg):
        P = self.pool
        g = P.p0[self.ti[idx]] + start_idx
        g5 = torch.minimum(g + 5, self._pend[idx])
        p0, p1 = P.path[g], P.path[g5]
        h = torch.atan2(p1[:, 1] - p0[:, 1], p1[:, 0] - p0[:, 0])
        self.x[idx] = p0[:, 0] - torch.sin(h) * lat_mm
        self.y[idx] = p0[:, 1] + torch.cos(h) * lat_mm
        self.h[idx] = h + torch.deg2rad(head_deg)
        for a in (self.wl, self.wr):
            a[idx] = 0.0
        self.ticks[idx] = 0
        self.cbuf[idx] = 0.0
        self.wang[idx] = 0.0
        self.ybuf[idx] = 0.0
        self.ctrl_acc[idx] = 0.0
        self.pi[idx] = g
        self.max_s[idx] = P.s[g] - P.s[P.p0[self.ti[idx]]]
        raw = self._measure(idx)
        self.buf[idx] = raw[:, None, :].expand(-1, self.M, -1)
        self.raw[idx] = raw
        self.raw_prev[idx] = raw

    # ---------------- 물리 ----------------
    def _target(self, d):
        a = d.abs()
        v = torch.full_like(a, self.dc_y0)
        for i, sl in enumerate(self.dc_sl):
            v = v + sl * (a.clamp(self.dc_x[i], self.dc_x[i + 1]) - self.dc_x[i])
        return v * self.vscale * torch.sign(d)

    def ctrl_gate(self):
        """이번 틱에 제어 루프가 도는 env (실기 제어 주기 ≈735/s 재현)."""
        self.ctrl_acc = self.ctrl_acc + self.ctrl_step
        g = self.ctrl_acc >= 1.0
        self.ctrl_acc = torch.where(g, self.ctrl_acc - 1.0, self.ctrl_acc)
        return g

    def tick(self, fl, fr):
        dt = self.dt
        self.chead = (self.chead + 1) % self.MC
        self.cbuf[:, self.chead, 0] = fl
        self.cbuf[:, self.chead, 1] = fr
        j = (self.chead - self.clag) % self.MC
        fl, fr = self.cbuf[self.ar, j, 0], self.cbuf[self.ar, j, 1]
        tl, tr = self._target(fl), self._target(fr)
        al = torch.where(tl.abs() < self.wl.abs(), self.a_dn, self.a_up)
        ar = torch.where(tr.abs() < self.wr.abs(), self.a_dn, self.a_up)
        yon = self.a_y > 0
        d1 = (tr - tl) * 0.5 + ((self.wr - self.wl) - (tr - tl)) * 0.5 * self.a_y
        self.wl = tl + (self.wl - tl) * al
        self.wr = tr + (self.wr - tr) * ar
        cm = (self.wl + self.wr) * 0.5
        self.wl = torch.where(yon, cm - d1, self.wl)
        self.wr = torch.where(yon, cm + d1, self.wr)
        vl, vr = self.wl * self.k, self.wr * self.k
        self.wang = self.wang + torch.stack([self.wl, self.wr], 1) * (dt * math.pi / 180.0)
        wf = 1.0 + self.wob * torch.sin(self.wang + self.wph)
        vl, vr = vl * wf[:, 0], vr * wf[:, 1]
        v = (vl + vr) * 0.5
        w = (vr - vl) / self.Wax
        hm = self.h + w * (dt * 0.5)
        self.x = self.x + v * torch.cos(hm) * dt
        self.y = self.y + v * torch.sin(hm) * dt
        self.h = self.h + w * dt
        self.ybuf[:, self.chead] = torch.rad2deg(w)
        self.ticks = self.ticks + 1
        raw = self._measure(None)
        self.head = (self.head + 1) % self.M
        self.buf[:, self.head] = raw
        self.raw_prev = self.raw
        self.raw = self.buf[self.ar, (self.head - self.lag) % self.M]

    def torn(self, p_torn):
        u = torch.rand(self.n, generator=self.gen, device=self.d)
        k = torch.where(u < p_torn * 0.5, 1, torch.where(u < p_torn, 2, 0))
        cols = torch.arange(self.S, device=self.d)[None, :]
        mix = torch.where(cols < k[:, None], self.raw_prev, self.raw)
        return torch.where((k > 0)[:, None], mix, self.raw_prev)

    def now_ms(self):
        return fdiv(self.ticks * 1000, self.hz)

    def yaw_dps(self):
        return self.ybuf[self.ar, (self.chead - self.ylag) % self.MC]

    def _measure(self, idx):
        sl = slice(None) if idx is None else idx
        h = self.h[sl]
        ch, sh = torch.cos(h)[:, None], torch.sin(h)[:, None]
        f, l = self.pos[sl, :, 0], self.pos[sl, :, 1]
        sx = (self.x[sl] - self._tx0[sl])[:, None] + ch * f - sh * l
        sy = (self.y[sl] - self._ty0[sl])[:, None] + sh * f + ch * l
        off = self.fp_off[sl]
        px = sx[:, :, None] + off[:, None, :, 0]
        py = sy[:, :, None] + off[:, None, :, 1]
        W, H = self._tW[sl][:, None, None], self._tH[sl][:, None, None]
        ix = torch.minimum(px.to(I64).clamp_min(0), W - 1)
        iy = torch.minimum(py.to(I64).clamp_min(0), H - 1)
        b = self.pool.flat[self._tbase[sl][:, None, None] + iy * W + ix].to(F32).mean(dim=2)
        nz = torch.randn(b.shape, generator=self.gen, device=self.d) * self.noise[sl][:, None]
        val = self.white[sl] - (self.white[sl] - self.black[sl]) * b + nz
        return val.clamp(0, 100).to(torch.int32)

    def progress(self):
        P = self.pool
        lo, hi = self.PROG_WIN
        off = torch.arange(lo, hi + 1, device=self.d)[None, :]
        idx = torch.minimum(torch.maximum(self.pi[:, None] + off, self._p0[:, None]), self._pend[:, None])
        rf = float(self.base.get("track_ref_mm", 0.0))      # 판정 기준점 (core와 같음)
        rx, ry = self.x + rf * torch.cos(self.h), self.y + rf * torch.sin(self.h)
        dx = P.path[idx, 0] - rx[:, None]
        dy = P.path[idx, 1] - ry[:, None]
        d2 = dx * dx + dy * dy
        j = d2.argmin(dim=1)
        self.pi = idx[self.ar, j]
        nxt = torch.minimum(self.pi + 1, self._pend)
        prv = torch.maximum(self.pi - 1, self._p0)
        tx = P.path[nxt, 0] - P.path[prv, 0]
        ty = P.path[nxt, 1] - P.path[prv, 1]
        cross = tx * (ry - P.path[self.pi, 1]) - ty * (rx - P.path[self.pi, 0])
        lat = d2[self.ar, j].sqrt() * torch.where(cross >= 0, 1.0, -1.0)
        s = P.s[self.pi] - P.s[self._p0]
        self.max_s = torch.maximum(self.max_s, s)
        return s, lat, torch.atan2(ty, tx)

    def wheel_speed_mms(self):
        return self.wl * self.k, self.wr * self.k


class TorchRule:
    """rule_lf3.RuleLF3(= spike_lf3_rule.py)과 같은 로직. 정수 상태는 int64.
    튜닝 상수는 env별 텐서 — set_consts(idx, dict)로 env마다 다른 설정을 줄 수 있다
    (GPU 스윕: 수천 개 설정을 한 번에). 실기 파일의 if 분기는 마스크로 바꿨다."""

    INT_NAMES = ("POWER", "KG", "G_CLAMP", "ALPHA", "D_FILTER", "MAX_STEER_DELTA", "W_NORM",
                 "BLACK_MID", "SIDE_ERR", "GAP_MS", "AHEAD_HOLD_MS", "LOCK_MS", "SEARCH_DEG",
                 "ST_BANK", "RECOVER_POWER", "SPD_K10", "MIN_POWER", "PB_MS", "PB_POWER", "FORCE_MS",
                 "ERR_MODE", "SENSOR_GAP_MM", "CF_GAIN", "SIDE_SEE", "DFLOOR", "KSP", "DELAY_MS", "YAW_PER_STEER", "TAU_MS",
                 "KG2", "CURVE_YAW", "CURVE_TAU_MS", "FF", "G_REL", "CEMA_BANK")   # 연속 라인 위치(ERR_MODE)·지연 보상(KSP) — 실기 파일과 같은 식

    def __init__(self, n, device, consts=None):
        self.n, self.d = n, device
        self.S = 100000
        self.c = {k: torch.zeros(n, dtype=I64, device=device) for k in self.INT_NAMES}
        self.c["USE_SENTINEL"] = torch.zeros(n, dtype=torch.bool, device=device)
        self.c["KP_S"] = torch.zeros(n, dtype=I64, device=device)
        self.c["KD_S"] = torch.zeros(n, dtype=I64, device=device)
        self.c["KP2_S"] = torch.zeros(n, dtype=I64, device=device)
        self.c["KPN_S"] = torch.zeros(n, dtype=I64, device=device)
        self.set_consts(torch.arange(n, device=device), consts or load_constants())
        i = lambda v=0: torch.full((n,), v, dtype=I64, device=device)  # noqa: E731
        b = lambda: torch.zeros(n, dtype=torch.bool, device=device)  # noqa: E731
        self.wv = torch.zeros((n, 3), dtype=I64, device=device)
        self.bv = torch.zeros((n, 3), dtype=I64, device=device)
        self.prev = torch.zeros((n, 3), dtype=I64, device=device)
        self.ar = torch.arange(n, device=device)
        self.rh = torch.zeros((n, 64), dtype=I64, device=device)     # KSP 지연 버퍼 (실기 r_hist)
        self.ysgn = i(-1)
        for nm in ("sL", "sR", "sF", "r_mod", "r_i", "c_ema", "last_error", "d_filt", "prev_steer", "last_good", "lock_side",
                   "t_lock", "t_ahead", "pb_until", "t_step", "bank_dir", "turned", "flips",
                   "steer_out", "error_out", "yellow", "red", "nsteps"):
            setattr(self, nm, i())
        self.side, self.t_lost = i(-1), i(-1)
        self.was_off, self.armed = b(), b()
        self.left = torch.zeros(n, dtype=F32, device=device)
        self.right = torch.zeros(n, dtype=F32, device=device)

    def set_consts(self, idx, consts):
        """consts: {이름: 스칼라 또는 len(idx) 배열}. KP·KD는 실수(내부 ×100000 정수화)."""
        m = len(idx)
        for k, v in consts.items():
            if k in ("KP", "KD", "KP2", "KPN"):
                vv = np.round(np.asarray(v, float) * self.S).astype(np.int64)
                self.c[k + "_S"][idx] = torch.as_tensor(np.broadcast_to(vv, (m,)).copy(), device=self.d)
            elif k == "USE_SENTINEL":
                vv = np.asarray([x in (True, "True", 1) for x in np.broadcast_to(np.asarray(v, object), (m,))])
                self.c[k][idx] = torch.as_tensor(vv, device=self.d)
            elif k in self.INT_NAMES:
                vv = np.asarray(v, float).round().astype(np.int64)
                self.c[k][idx] = torch.as_tensor(np.broadcast_to(vv, (m,)).copy(), device=self.d)

    def set_value(self, idx, white, black):
        self.wv[idx] = white.round().to(I64)
        self.bv[idx] = black.round().to(I64)

    def _norm(self, raw):
        r = torch.minimum(torch.maximum(raw.to(I64), self.bv), self.wv)
        return fdiv((r - self.bv) * 100, self.wv - self.bv)

    def reset(self, idx, raw):
        r = raw[idx].to(I64)
        w, b = self.wv[idx], self.bv[idx]
        n0 = fdiv((torch.minimum(torch.maximum(r, b), w) - b) * 100, w - b)
        self.prev[idx] = r
        self.sL[idx], self.sR[idx] = n0[:, 0], n0[:, 1]
        self.sF[idx] = n0[:, 2]
        self.ysgn[idx] = -1
        self.r_mod[idx] = 0
        self.c_ema[idx] = 0
        self.r_i[idx] = 0
        self.rh[idx] = 0
        for nm, v in (("last_error", 0), ("d_filt", 0), ("prev_steer", 0), ("last_good", 0),
                      ("side", -1), ("lock_side", 0), ("t_lock", 0), ("t_ahead", 0),
                      ("t_lost", -1), ("pb_until", 0), ("t_step", 0), ("bank_dir", 0),
                      ("turned", 0), ("flips", 0), ("steer_out", 0), ("error_out", 0),
                      ("yellow", 0), ("red", 0), ("nsteps", 0)):
            getattr(self, nm)[idx] = v
        self.was_off[idx] = False
        self.armed[idx] = False
        self.left[idx] = 0.0
        self.right[idx] = 0.0

    def step(self, raw, now, gyro, dsteer=None, power=None, dpower=None, gate=None):
        c = self.c
        W = torch.where
        raw = raw.to(I64)
        F = (raw != self.prev).any(dim=1) | (now - self.t_step >= c["FORCE_MS"])
        if gate is not None:
            F = F & gate                            # 이번 틱에 제어 루프가 안 돎 → 직전 명령 유지
        self.prev = W(F[:, None], raw, self.prev)
        dt = now - self.t_step
        n = self._norm(raw)
        nL, nR, nF = n[:, 0], n[:, 1], n[:, 2]
        sL = fdiv(c["ALPHA"] * nL + (1000 - c["ALPHA"]) * self.sL, 1000)
        sR = fdiv(c["ALPHA"] * nR + (1000 - c["ALPHA"]) * self.sR, 1000)
        sF = fdiv(c["ALPHA"] * nF + (1000 - c["ALPHA"]) * self.sF, 1000)
        pair_white = (nL > c["W_NORM"]) & (nR > c["W_NORM"])
        us = c["USE_SENTINEL"]
        fb = us & (nF < c["BLACK_MID"])
        t_ahead = W(fb, now, self.t_ahead)
        armed = self.armed & ~fb
        ahead = us & (now - t_ahead <= c["AHEAD_HOLD_MS"])
        arm = us & ~ahead & ~pair_white & ~armed
        armed = armed | arm
        lock_side = W(arm, 0, self.lock_side)
        pb_until = W(arm, now + c["PB_MS"], self.pb_until)
        red = self.red + arm.to(I64)
        dLm = (100 - c["DFLOOR"] - sL).clamp_min(0)
        dRm = (100 - c["DFLOOR"] - sR).clamp_min(0)
        dFm = (100 - c["DFLOOR"] - sF).clamp_min(0)
        see_side = (dLm + dRm) > c["SIDE_SEE"]
        ysgn = W(see_side, W(dLm > dRm, 1, -1), self.ysgn)
        y_side = fdiv((dLm - dRm) * c["SENSOR_GAP_MM"], (dLm + dFm + dRm).clamp_min(1))
        y_c = fdiv(ysgn * c["CF_GAIN"] * (100 - c["DFLOOR"] - dFm), (100 - c["DFLOOR"]).clamp_min(1))
        ymm = W(see_side, y_side, W(dFm > c["SIDE_SEE"], y_c, ysgn * c["SENSOR_GAP_MM"]))
        m1 = c["ERR_MODE"] == 1
        ysgn = W(m1, ysgn, self.ysgn)
        error = W(m1, fdiv(-ymm * 87, 10), 2 * (sL - sR))
        side = W(error > c["SIDE_ERR"], 1, W(error < -c["SIDE_ERR"], -1, self.side))
        was_off = self.was_off
        setl = (c["LOCK_MS"] > 0) & armed & (lock_side == 0) & ((error > c["SIDE_ERR"]) | (error < -c["SIDE_ERR"]))
        clr = ~setl & (lock_side != 0) & ~was_off & (now - self.t_lock > c["LOCK_MS"])
        lock_side = W(setl, side, W(clr, 0, lock_side))
        t_lock = W(setl, now, self.t_lock)
        online = ~pair_white | ahead
        recap = online & was_off
        was_off = W(recap, False, was_off)
        lock_side = W(recap, 0, lock_side)
        d_filt = W(recap, 0, self.d_filt)
        last_error = W(recap, error, self.last_error)
        prev_steer = W(recap, 0, self.prev_steer)
        d_new = fdiv(c["D_FILTER"] * (error - last_error) + (1000 - c["D_FILTER"]) * d_filt, 1000)
        kp2_on = c["KP2_S"] > 0                   # 곡선 게인 스케줄링 (실기 파일과 같은 식)
        c_new = self.c_ema + fdiv((gyro * 1000 - self.c_ema) * dt, c["CURVE_TAU_MS"].clamp_min(1))
        fcv = torch.clamp(fdiv(c_new.abs(), c["CURVE_YAW"].clamp_min(1)), 0, 1000)
        kp_s = W(kp2_on, c["KP_S"] + fdiv((c["KP2_S"] - c["KP_S"]) * fcv, 1000), c["KP_S"])
        kg = W(kp2_on & (c["KG2"] >= 0), c["KG"] + fdiv((c["KG2"] - c["KG"]) * fcv, 1000), c["KG"])
        steer_on = fdiv(kp_s * error + c["KD_S"] * d_new, self.S)
        steer_on = steer_on + fdiv(c["KPN_S"] * fdiv(error * error.abs(), 200), self.S)   # 비선형 조향 (KPN)
        cc = W(kp2_on, c_new, 0)                  # 곡선 평균 (스케줄링 켰을 때만 — 실기 파일과 같음)
        steer_on = steer_on - fdiv(c["FF"] * fdiv(cc, 1000), 1000 * c["YAW_PER_STEER"].clamp_min(1))   # 곡선 피드포워드
        steer_on = W((sL < c["BLACK_MID"]) & (sR < c["BLACK_MID"]), 0, steer_on)
        ksp_on = c["KSP"] > 0
        r_old = self.rh[self.ar, self.r_i]
        yr = gyro + W(ksp_on, fdiv(c["KSP"] * (self.r_mod - r_old), 1000000), 0)
        yr = yr - fdiv(c["G_REL"] * fdiv(cc, 1000), 1000)   # 감쇠는 곡선 평균과의 차이에만
        g = torch.clamp(fdiv(kg * yr, 1000), -c["G_CLAMP"], c["G_CLAMP"])
        steer_on = steer_on + g
        d_filt = W(online, d_new, d_filt)
        last_error = W(online, error, last_error)
        last_good = W(online, steer_on, self.last_good)
        t_lost = W(online, -1, self.t_lost)
        lost = ~online
        t_lost = W(lost & (t_lost < 0), now, t_lost)
        in_gap = lost & (now - t_lost <= c["GAP_MS"])
        bank = lost & ~in_gap
        enter = bank & ~was_off
        was_off = was_off | enter
        bank_dir = W(enter, W(lock_side != 0, lock_side, side), self.bank_dir)
        turned = W(enter, 0, self.turned)
        flips = W(enter, 0, self.flips)
        cont = bank & ~enter & (c["SEARCH_DEG"] > 0)
        turned = W(cont, turned - bank_dir * gyro * dt, turned)
        flip = cont & (flips < 2) & (turned > c["SEARCH_DEG"] * 1000)
        bank_dir = W(flip, -bank_dir, bank_dir)
        turned = W(flip, 0, turned)
        flips = W(flip, flips + 1, flips)
        steer = W(online, steer_on, W(in_gap, last_good, bank_dir * c["ST_BANK"]))
        pw = W(bank, c["RECOVER_POWER"], c["POWER"])
        if dsteer is not None:
            steer = steer + dsteer.round().to(I64)
        sched = torch.clamp_min(c["POWER"] - fdiv(c["SPD_K10"] * steer.abs(), 10), 0)
        sched = torch.maximum(sched, c["MIN_POWER"])
        pw = W(c["SPD_K10"] > 0, torch.minimum(pw, sched), pw)
        pw = W((now < pb_until) & (pw > c["PB_POWER"]), c["PB_POWER"], pw)
        pwf = pw.to(F32) if power is None else power.to(F32)
        if dpower is not None:                      # 잔차: 규칙기반 파워 ± 보정
            pwf = (pwf + dpower).clamp(0.0, 100.0)
        delta = steer - prev_steer
        msd = c["MAX_STEER_DELTA"]
        steer = W(delta > msd, prev_steer + msd, W(delta < -msd, prev_steer - msd, steer))
        prev_steer = steer
        r_mod = W(ksp_on, self.r_mod + fdiv((-steer * c["YAW_PER_STEER"] * 1000 - self.r_mod) * dt,
                                           c["TAU_MS"].clamp_min(1)), self.r_mod)
        wr = F & ksp_on
        self.rh[self.ar, self.r_i] = W(wr, r_mod, r_old)
        r_n = (fdiv(c["DELAY_MS"] * 735, 1000) + 1).clamp(1, 64)
        self.r_i = W(wr, (self.r_i + 1) % r_n, self.r_i)
        st = steer.clamp(-100, 100).to(F32)
        slope = pwf * 0.02
        left = W(st < 0, torch.round((slope * st + pwf) * 100) / 100, pwf)
        right = W(st < 0, pwf, torch.round((-slope * st + pwf) * 100) / 100)
        c_ema = W((online | (c["CEMA_BANK"] > 0)) & kp2_on, c_new, self.c_ema)
        for nm, val in (("sL", sL), ("sR", sR), ("sF", sF), ("ysgn", ysgn), ("r_mod", r_mod), ("c_ema", c_ema),
                        ("armed", armed), ("t_ahead", t_ahead),
                        ("lock_side", lock_side), ("pb_until", pb_until), ("side", side),
                        ("t_lock", t_lock), ("was_off", was_off), ("d_filt", d_filt),
                        ("last_error", last_error), ("last_good", last_good), ("t_lost", t_lost),
                        ("bank_dir", bank_dir), ("turned", turned), ("flips", flips),
                        ("prev_steer", prev_steer), ("left", left), ("right", right),
                        ("steer_out", steer), ("error_out", error), ("red", red), ("t_step", now)):
            setattr(self, nm, W(F, val, getattr(self, nm)))
        self.yellow = self.yellow + (F & enter).to(I64)
        self.nsteps = self.nsteps + F.to(I64)
        return self.left, self.right

    def step_frame(self, sim, now, gyro, p_torn=None, dsteer=None, power=None, dpower=None, gate=None):
        if p_torn is not None:
            self.step(sim.torn(p_torn), now, gyro, dsteer, power, dpower, gate)
        return self.step(sim.raw, now, gyro, dsteer, power, dpower, gate)


class TorchEnv:
    """rl/env.py LineFollowEnv와 같은 MDP (관측 22·크리틱 12·행동 2). 텐서 입출력."""

    def __init__(self, n, pool, device="auto", profile="spike_3s", dr="rl", mode="residual",
                 decision_ms=5.0, p_torn=(0.0, 2.0 / 3.0), max_s=40.0, start_frac=0.7,
                 dsteer_max=40.0, p_min=20.0, stuck_s=3.0, seed=0, rule_consts=None,
                 w_lat=0.05, w_da=0.01, r_lost=-50.0, r_finish=20.0, layout_pos=None,
                 fixed_start=False, worlds=None, dpower_max=40.0, track_p=None):
        self.dpower_max = dpower_max
        self.track_p = None if track_p is None else np.asarray(track_p, float) / np.sum(track_p)
        self.d = pick_device(device)
        self.layout_pos = None if layout_pos is None else np.asarray(layout_pos, float)
        self.fixed_start = fixed_start
        self.worlds = worlds            # 평가용: env별 고정 월드 dict(parity_test.world 형식) 목록
        self.n = n
        self.base = core.load_profile(profile)
        self.tpool = pool if isinstance(pool, TorchPool) else TorchPool(pool, self.d)
        self.sim = TorchSim(n, self.tpool, self.base, self.d, seed=seed + 1)
        self.rule = TorchRule(n, self.d, rule_consts)
        self.rng = np.random.default_rng(seed)
        self.dr, self.mode = dr, mode
        self.K = max(1, int(round(decision_ms / 1000.0 / self.sim.dt)))
        self.dec_dt = self.K * self.sim.dt
        self.p_torn_rng = p_torn
        self.max_ms = int(max_s * 1000)
        self.start_frac, self.stuck_ms = start_frac, int(stuck_s * 1000)
        self.dsteer_max, self.p_min = dsteer_max, p_min
        self.w_lat, self.w_da, self.r_lost, self.r_finish = w_lat, w_da, r_lost, r_finish
        self.v_ref = float(np.asarray(self.base["dc_table"])[-1, 1] * self.base.get("gear_ratio", 1.0)
                           * math.pi * self.base["wheel_diameter_mm"] / 360.0)
        z = lambda: torch.zeros(n, dtype=F32, device=self.d)  # noqa: E731
        self.s_prev, self.p_torn, self.mark, self.lat = z(), z(), z(), z()
        self.t_mark = torch.zeros(n, dtype=I64, device=self.d)
        self.a_prev = torch.zeros((n, 2), dtype=F32, device=self.d)
        self.hist = torch.zeros((n, 4, 3), dtype=F32, device=self.d)
        self.ep_ret, self.ep_len = z(), torch.zeros(n, dtype=I64, device=self.d)
        self.reset_all()

    def reset_all(self):
        self._reset(torch.arange(self.n, device=self.d))
        return self.obs()

    def _reset(self, idx):
        m = len(idx)
        if m == 0:
            return
        sim, rule, d = self.sim, self.rule, self.d
        if self.worlds is not None:
            ii = idx.cpu().numpy()
            ws = [self.worlds[i % len(self.worlds)] for i in ii]
            w = dict(tau_up=np.array([x['tau_up'] for x in ws]), battery=np.array([x['battery'] for x in ws]),
                     noise=np.array([x['noise'] for x in ws]), lag_ms=np.array([x['lag_ms'] for x in ws]),
                     pos_err=np.stack([x['pos_err'] for x in ws]))
            for key in ('cmd_lag_ms', 'imu_lag_ms', 'w_eff', 'wobble', 'wobble_ph'):
                if key in ws[0]:
                    w[key] = np.array([x[key] for x in ws])
            lat0, head0 = np.array([x['off'] for x in ws]), np.zeros(m)
        else:
            w, lat0, head0 = sample_world(self.rng, self.base, m, self.dr)
        if self.layout_pos is not None:
            w['pos_abs'] = np.broadcast_to(self.layout_pos, (m,) + self.layout_pos.shape).copy()
        sim.set_world(idx, w)
        if self.worlds is not None:
            ti = (idx // len(self.worlds)) % self.tpool.n
        elif self.track_p is not None:                 # 트랙별 표본 비중
            ti = torch.as_tensor(self.rng.choice(self.tpool.n, size=m, p=self.track_p), device=d)
        else:
            ti = torch.as_tensor(self.rng.integers(self.tpool.n, size=m), device=d)
        sim.set_tracks(idx, ti)
        plen = self.tpool.np.plen[ti.cpu().numpy()]
        start = (self.rng.random(m) * self.start_frac * np.maximum(plen - 400, 1)).astype(np.int64)
        if self.fixed_start:
            start = np.zeros(m, np.int64)
        sim.reset_pose(idx, torch.as_tensor(start, device=d),
                       torch.as_tensor(lat0, dtype=F32, device=d),
                       torch.as_tensor(head0, dtype=F32, device=d))
        rule.set_value(idx, sim.white[idx], sim.black[idx])
        rule.reset(idx, sim.raw)
        lo, hi = self.p_torn_rng
        self.p_torn[idx] = torch.as_tensor(self.rng.uniform(lo, hi, m), dtype=F32, device=d)
        self.s_prev[idx] = sim.max_s[idx]
        self.mark[idx] = sim.max_s[idx]
        self.t_mark[idx] = 0
        self.a_prev[idx] = 0.0
        self.ep_ret[idx] = 0.0
        self.ep_len[idx] = 0
        self.hist[idx] = self._norm(sim.raw[idx], idx)[:, None, :]

    def _norm(self, raw, idx=None):
        sl = slice(None) if idx is None else idx
        w, b = self.rule.wv[sl], self.rule.bv[sl]
        r = torch.minimum(torch.maximum(raw.to(I64), b), w)
        return fdiv((r - b) * 100, w - b).to(F32) / 100.0

    def step(self, action):
        sim, rule = self.sim, self.rule
        a = action.to(self.d, F32).clamp(-1, 1)
        dpower = None
        if self.mode == "rule":
            dsteer = power = None
        elif self.mode == "residual2":
            dsteer, power = a[:, 0] * self.dsteer_max, None
            dpower = a[:, 1] * self.dpower_max
        else:
            dsteer = a[:, 0] * self.dsteer_max
            power = (torch.full_like(dsteer, 100.0) if self.mode == "steer_only"
                     else self.p_min + (a[:, 1] + 1) * 0.5 * (100.0 - self.p_min))
        for _ in range(self.K):
            now = sim.now_ms()
            gyro = sim.yaw_dps().trunc().to(I64)
            l, r = rule.step_frame(sim, now, gyro, self.p_torn, dsteer, power, dpower, sim.ctrl_gate())
            sim.tick(l, r)
        s, lat, path_h = sim.progress()
        self.lat, self.path_h = lat, path_h
        rew = (s - self.s_prev) / (self.v_ref * self.dec_dt)
        rew = rew - self.w_lat * (lat / 20.0) ** 2 - self.w_da * ((a - self.a_prev) ** 2).sum(1)
        self.s_prev, self.a_prev = s, a
        now = sim.now_ms()
        moved = sim.max_s > self.mark + 50.0
        self.mark = torch.where(moved, sim.max_s, self.mark)
        self.t_mark = torch.where(moved, now, self.t_mark)
        length = self.tpool.length[sim.ti]
        lost = lat.abs() > core.Sim.LOST_MM
        fin = s > length - core.Sim.FINISH_MM
        stuck = now - self.t_mark > self.stuck_ms
        timeout = now > self.max_ms
        rew = rew + lost.to(F32) * self.r_lost + fin.to(F32) * self.r_finish
        done = lost | fin | stuck | timeout
        self.ep_ret += rew
        self.ep_len += 1
        self.hist = torch.roll(self.hist, 1, dims=1)
        self.hist[:, 0] = self._norm(sim.raw)
        info = {}
        if bool(done.any()):
            idx = done.nonzero().squeeze(1)
            code = torch.where(lost, 1, torch.where(fin, 2, torch.where(stuck, 3, 4)))
            info = dict(idx=idx, code=code[idx], ret=self.ep_ret[idx].clone(),
                        len=self.ep_len[idx].clone(), time_s=now[idx].to(F32) / 1000.0,
                        frac=sim.max_s[idx] / length[idx], track=sim.ti[idx].clone())
            self._reset(idx)
        return self.obs(), rew, done, info

    def obs(self):
        sim, rule = self.sim, self.rule
        now = sim.now_ms()
        vl, vr = sim.wheel_speed_mms()
        since_ahead = ((now - rule.t_ahead).to(F32) / 500.0).clamp(0, 1)
        since_lost = torch.where(rule.t_lost >= 0, ((now - rule.t_lost).to(F32) / 500.0).clamp(0, 1),
                                 torch.zeros_like(vl))
        return torch.cat([
            self.hist.reshape(self.n, 12),
            (rule.error_out.to(F32) / 200.0)[:, None], (rule.steer_out.to(F32) / 100.0)[:, None],
            (sim.yaw_dps() / 400.0)[:, None],
            (vl / self.v_ref)[:, None], (vr / self.v_ref)[:, None],
            self.a_prev, since_ahead[:, None], since_lost[:, None], rule.side.to(F32)[:, None],
        ], dim=1)


    def critic_obs(self):
        """비대칭 크리틱 전용 특권 관측 (실기에선 모름)."""
        sim = self.sim
        path_h = getattr(self, "path_h", sim.h)
        herr = torch.remainder(sim.h - path_h + math.pi, 2 * math.pi) - math.pi
        P = self.tpool
        curv = []
        h0 = self._ph(sim.pi)
        for dmm in (100.0, 200.0, 400.0):
            j = torch.minimum(sim.pi + int(dmm / 2.0), sim._pend)
            dh = self._ph(j) - h0
            curv.append(torch.remainder(dh + math.pi, 2 * math.pi) - math.pi)
        vl, vr = sim.wheel_speed_mms()
        return torch.cat([
            (self.lat / 50.0)[:, None], (herr / math.pi)[:, None], torch.stack(curv, 1) / math.pi,
            ((vl + vr) * 0.5 / self.v_ref)[:, None], (sim.lag.to(F32) * sim.dt * 1000 / 40.0)[:, None],
            (sim.vscale / 2.0)[:, None], (sim.noise / 2.0)[:, None], self.p_torn[:, None],
            (sim.fp_r / 5.0)[:, None], (sim.a_up.log().reciprocal() * -sim.dt / 0.115)[:, None],
        ], dim=1)

    def _ph(self, i):
        P, sim = self.tpool, self.sim
        j = torch.minimum(i + 3, sim._pend)
        i0 = torch.maximum(j - 6, sim._p0)
        return torch.atan2(P.path[j, 1] - P.path[i0, 1], P.path[j, 0] - P.path[i0, 0])


REASONS = {1: "lost", 2: "finish", 3: "stuck", 4: "timeout"}


def _bench(device, ns=(1024, 4096, 16384), steps=30):
    import time
    from .env import make_pool
    pool = make_pool(n_gen=8, include=("photo", "mrc_real"))
    for n in ns:
        env = TorchEnv(n, pool, device=device, mode="residual", seed=1)
        a = torch.zeros((n, 2), device=env.d)
        a[:, 1] = 0.5
        env.step(a)
        if env.d.type == "mps":
            torch.mps.synchronize()
        t0 = time.time()
        for _ in range(steps):
            env.step(a)
        if env.d.type == "mps":
            torch.mps.synchronize()
        dec = n * steps / (time.time() - t0)
        print("%-4s n=%6d: 결정 %8.0f/s → 합산 %6.0fx 실시간 | 1억 결정 ≈ %.2f시간"
              % (env.d.type, n, dec, dec * env.dec_dt, 1e8 / dec / 3600), flush=True)


def _check(device):
    """numpy 판(LineFollowEnv)과 규칙기반 모드 통계 비교 (같은 트랙 풀·같은 DR 분포)."""
    from .env import LineFollowEnv, make_pool
    pool = make_pool(n_gen=6, include=("photo",))
    res = {}
    for name in ("numpy", "torch"):
        n = 256
        env = (LineFollowEnv(n, pool, mode="rule", seed=5) if name == "numpy"
               else TorchEnv(n, pool, device=device, mode="rule", seed=5))
        eps = []
        seen = set()                      # env별 첫 에피소드만 (두 판이 같은 월드·트랙·출발점)
        steps = 0
        while len(seen) < n and steps < 12000:
            act = np.zeros((n, 2)) if name == "numpy" else torch.zeros((n, 2), device=env.d)
            _, _, _, info = env.step(act)
            steps += 1
            if info:
                if name == "numpy":
                    new = [(int(i), r, float(t), float(f)) for i, r, t, f in
                           zip(info["idx"], info["reason"], info["time_s"], info["frac"])]
                else:
                    new = [(int(i), REASONS[int(c)], float(t), float(f)) for i, c, t, f in
                           zip(info["idx"].cpu(), info["code"].cpu(), info["time_s"].cpu(), info["frac"].cpu())]
                for i, r, t, f in new:
                    if i not in seen:
                        seen.add(i)
                        eps.append((r, t, f))
        fin = [e for e in eps if e[0] == "finish"]
        res[name] = dict(n=len(eps), finish=round(len(fin) / max(1, len(eps)), 3),
                         lost=round(sum(e[0] == "lost" for e in eps) / max(1, len(eps)), 3),
                         frac=round(float(np.mean([e[2] for e in eps])), 3),
                         time_fin=round(float(np.mean([e[1] for e in fin])), 2) if fin else None)
        print(name, res[name], flush=True)
    return res


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="auto")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--ns", default="1024,4096,16384")
    args = ap.parse_args()
    if args.check:
        _check(args.device)
    if args.bench:
        _bench(args.device, [int(x) for x in args.ns.split(",")])
