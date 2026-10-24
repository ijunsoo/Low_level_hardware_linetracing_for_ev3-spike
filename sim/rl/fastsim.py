# =====================================================================
#  fastsim — core.Sim과 같은 물리·센서 모델을 N대 동시에 (numpy 벡터화)
#
#  같은 점: 비대칭 1차지연 모터(τ_up/τ_dn)·dc 곡선·배터리·기어, 차동구동 중점적분,
#           16점 발자국 평균·가우시안 노이즈·정수화, 센서별 흰/검·장착 위치,
#           허브 IMU(이상적 요레이트).
#  다른 점(속도를 위해 단순화 — parity_test.py로 영향 확인):
#   - 시간 = 고정 틱 dt = 센서 프레임 주기. 모든 env가 매 틱 새 센서값을 받는다.
#     (core는 read 1회마다 시간이 흐르고, 실기 코드는 프레임 동기로 같은 효과)
#   - UART 지연 = 프레임 단위 링버퍼 (core는 연속시간 FIFO)
#   - 진행도·횡편차는 progress()를 부를 때만 계산
#
#  트랙은 TrackPool(여러 core.Track을 한 배열로)에서 env마다 고른다.
# =====================================================================
import math

import numpy as np

from . import _SIM_DIR  # noqa: F401
import core


class TrackPool:
    """core.Track 여러 개 → 래스터(uint8)와 경로를 한 배열로 이어 붙여 벡터 조회."""

    def __init__(self, tracks, names=None):
        self.names = list(names) if names else ["t%d" % i for i in range(len(tracks))]
        flats, paths, svals = [], [], []
        self.x0, self.y0, self.W, self.H, self.base = [], [], [], [], []
        self.p0, self.plen, self.length = [], [], []
        off = poff = 0
        for tr in tracks:
            assert tr.res == 1.0, "래스터 해상도 1mm만 지원"
            img = (tr.img > 0.5).astype(np.uint8)
            self.x0.append(tr.x0)
            self.y0.append(tr.y0)
            self.H.append(img.shape[0])
            self.W.append(img.shape[1])
            self.base.append(off)
            flats.append(img.ravel())
            off += img.size
            self.p0.append(poff)
            self.plen.append(len(tr.path))
            self.length.append(tr.length)
            paths.append(tr.path)
            svals.append(tr.s)
            poff += len(tr.path)
        self.flat = np.concatenate(flats)
        self.path = np.concatenate(paths)
        self.s = np.concatenate(svals)
        for k in ("x0", "y0", "W", "H", "base", "p0", "plen", "length"):
            setattr(self, k, np.asarray(getattr(self, k)))
        self.n = len(tracks)


class FastSim:
    PROG_WIN = (-10, 60)           # 국소 최근접 탐색창 (경로점 2mm; 결정 주기 5ms 이동 ≤ 10mm)

    def __init__(self, n, pool, base_params, seed=0):
        self.n = n
        self.pool = pool
        self.base = dict(base_params)
        self.rng = np.random.default_rng(seed)
        self.dt = 1.0 / self.base["sensor_hz"]
        dc = np.asarray(self.base["dc_table"], dtype=float)
        self.dc_p, self.dc_v = dc[:, 0], dc[:, 1]
        self.S = len(self.base["sensor_ports"])
        self.fp_unit = core._footprint_offsets(1.0)          # (16, 2), 반지름 1 기준
        self.M = 64                                          # 지연 링버퍼 길이 (≈44ms)
        ar = self.ar = np.arange(n)
        f = np.zeros(n)
        # ---- env별 물리 파라미터 (set_world로 채움) ----
        self.a_up, self.a_dn, self.vscale = f.copy(), f.copy(), f.copy()
        self.k, self.Wax, self.noise, self.fp_r = f.copy(), f.copy(), f.copy(), f.copy()
        self.lag = np.zeros(n, np.int64)
        self.pos = np.zeros((n, self.S, 2))
        self.fp_off = np.zeros((n, len(self.fp_unit), 2), np.float32)   # env별 발자국 점 (mm)
        self.white = np.zeros((n, self.S))
        self.black = np.zeros((n, self.S))
        # ---- 상태 ----
        self.x, self.y, self.h = f.copy(), f.copy(), f.copy()
        self.wl, self.wr = f.copy(), f.copy()
        self.ang_l, self.ang_r = f.copy(), f.copy()
        self.t = f.copy()                                    # 에피소드 시작 후 시간
        self.ti = np.zeros(n, np.int64)
        self.pi = np.zeros(n, np.int64)                      # 경로 인덱스(풀 전체 기준)
        self.max_s = f.copy()
        self.buf = np.full((n, self.M, self.S), 50, np.int16)
        self.head = 0
        self.raw = np.full((n, self.S), 50, np.int16)        # 노출된 센서값 (지연 후)
        self.raw_prev = self.raw.copy()                      # 직전 틱 노출값 (찢어진 읽기용)
        self._tx0 = self._ty0 = self._tW = self._tH = self._tbase = None
        del ar

    # ------------------------------------------------------------ 월드
    def set_world(self, idx, w):
        """idx env들에 월드 파라미터 dict(각 값은 len(idx) 배열 또는 스칼라)를 적용."""
        b = self.base
        tau_up = np.broadcast_to(w.get("tau_up", b["motor_tau_s"]), idx.shape)
        tau_dn = np.broadcast_to(w.get("tau_dn", b.get("motor_tau_dn_s", b["motor_tau_s"])), idx.shape)
        self.a_up[idx] = np.exp(-self.dt / tau_up)
        self.a_dn[idx] = np.exp(-self.dt / tau_dn)
        self.vscale[idx] = (np.broadcast_to(w.get("battery", b["battery_scale"]), idx.shape)
                            * b.get("gear_ratio", 1.0))
        wd = np.broadcast_to(w.get("wheel_mm", b["wheel_diameter_mm"]), idx.shape)
        self.k[idx] = math.pi * wd / 360.0
        self.Wax[idx] = b["axle_track_mm"] * np.broadcast_to(w.get("w_eff", b.get("w_eff_scale", 1.0)), idx.shape)
        self.noise[idx] = np.broadcast_to(w.get("noise", b["sensor_noise"]), idx.shape)
        self.fp_r[idx] = np.broadcast_to(w.get("footprint_mm", b["sensor_footprint_mm"]), idx.shape) / 2.0
        self.fp_off[idx] = (self.fp_unit[None] * self.fp_r[idx][:, None, None]).astype(np.float32)
        lag_ms = np.broadcast_to(w.get("lag_ms", b.get("sensor_lag_ms", 0.0)), idx.shape)
        self.lag[idx] = np.clip(np.round(lag_ms / 1000.0 / self.dt), 0, self.M - 2).astype(np.int64)
        pos = np.asarray(b["sensor_pos_mm"], dtype=float)
        self.pos[idx] = pos[None] + (w["pos_err"] if "pos_err" in w else 0.0)
        wh = np.asarray(b["sensor_white"][:self.S], float)
        bk = np.asarray(b["sensor_black"][:self.S], float)
        self.white[idx] = wh[None] * (w["white_k"] if "white_k" in w else 1.0)
        self.black[idx] = bk[None] * (w["black_k"] if "black_k" in w else 1.0)

    def set_tracks(self, idx, ti):
        self.ti[idx] = ti
        P = self.pool
        self._tx0, self._ty0 = P.x0[self.ti], P.y0[self.ti]
        self._tW, self._tH, self._tbase = P.W[self.ti], P.H[self.ti], P.base[self.ti]

    # ------------------------------------------------------------ 리셋
    def reset_pose(self, idx, start_idx, lat_mm, head_deg):
        """idx env를 트랙 경로 start_idx(트랙 내 인덱스) 위에 세운다."""
        P = self.pool
        g = P.p0[self.ti[idx]] + start_idx
        g5 = np.minimum(g + 5, P.p0[self.ti[idx]] + P.plen[self.ti[idx]] - 1)
        p0, p1 = P.path[g], P.path[g5]
        h = np.arctan2(p1[:, 1] - p0[:, 1], p1[:, 0] - p0[:, 0])
        self.x[idx] = p0[:, 0] - np.sin(h) * lat_mm
        self.y[idx] = p0[:, 1] + np.cos(h) * lat_mm
        self.h[idx] = h + np.radians(head_deg)
        for a in (self.wl, self.wr, self.ang_l, self.ang_r, self.t):
            a[idx] = 0.0
        self.pi[idx] = g
        self.max_s[idx] = P.s[g] - P.s[P.p0[self.ti[idx]]]
        raw = self._measure(idx)
        self.buf[idx] = raw[:, None, :]                       # 지연 버퍼를 현재값으로 채움
        self.raw[idx] = raw
        self.raw_prev[idx] = raw

    # ------------------------------------------------------------ 물리 1틱
    def tick(self, fl, fr):
        """전진 기준 duty(fl, fr)로 dt 진행 → 센서 측정 → 지연 노출값 갱신."""
        dt = self.dt
        tl = np.interp(np.abs(fl), self.dc_p, self.dc_v) * self.vscale * np.sign(fl)
        tr = np.interp(np.abs(fr), self.dc_p, self.dc_v) * self.vscale * np.sign(fr)
        al = np.where(np.abs(tl) < np.abs(self.wl), self.a_dn, self.a_up)
        arr = np.where(np.abs(tr) < np.abs(self.wr), self.a_dn, self.a_up)
        self.wl = tl + (self.wl - tl) * al
        self.wr = tr + (self.wr - tr) * arr
        vl, vr = self.wl * self.k, self.wr * self.k
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.Wax
        hm = self.h + w * dt / 2.0
        self.x += v * np.cos(hm) * dt
        self.y += v * np.sin(hm) * dt
        self.h += w * dt
        self.ang_l += self.wl * dt
        self.ang_r += self.wr * dt
        self.t += dt
        raw = self._measure(None)
        self.head = (self.head + 1) % self.M
        self.buf[:, self.head] = raw
        self.raw_prev = self.raw
        self.raw = self.buf[self.ar, (self.head - self.lag) % self.M]

    def torn(self, p_torn):
        """'찢어진 읽기' 프레임: 실기 코드는 센서를 L→R→F 차례로 읽으므로 새 프레임이
        읽는 도중에 오면 앞쪽 센서는 옛 값, 뒤쪽은 새 값을 본 채로 한 번 더 제어한다
        (core가 재현하는 현상 — 센서 3개면 확률 2/3). env별 확률 p_torn으로 그런 프레임을
        만들어 돌려준다. 해당 없는 env는 이전 노출값 그대로(→ 프레임 동기에서 건너뜀)."""
        old, new = self.raw_prev, self.raw
        u = self.rng.random(self.n)
        k = np.where(u < p_torn / 2, 1, np.where(u < p_torn, 2, 0))     # 옛 값으로 남는 앞쪽 센서 수
        cols = np.arange(self.S)[None, :]
        mix = np.where(cols < k[:, None], old, new)
        return np.where((k > 0)[:, None], mix, old)

    def yaw_dps(self):
        """허브 IMU 요레이트 (deg/s, 좌회전 +) — core.imu_rate_dps와 같은 식."""
        return np.degrees((self.wr - self.wl) * self.k / self.Wax)

    def wheel_speed_mms(self):
        return self.wl * self.k, self.wr * self.k

    def _measure(self, idx):
        sl = slice(None) if idx is None else idx
        h = self.h[sl]
        ch, sh = np.cos(h)[:, None], np.sin(h)[:, None]
        f, l = self.pos[sl, :, 0], self.pos[sl, :, 1]
        # 트랙 래스터 원점 기준 좌표로 바로 계산 (float32로 충분: mm 단위, 1mm 래스터)
        sx = ((self.x[sl] - self._tx0[sl])[:, None] + ch * f - sh * l).astype(np.float32)
        sy = ((self.y[sl] - self._ty0[sl])[:, None] + sh * f + ch * l).astype(np.float32)
        off = self.fp_off[sl]
        px = sx[:, :, None] + off[:, None, :, 0]
        py = sy[:, :, None] + off[:, None, :, 1]
        W, H = self._tW[sl][:, None, None], self._tH[sl][:, None, None]
        ix = np.minimum(np.maximum(px.astype(np.int64), 0), W - 1)
        iy = np.minimum(np.maximum(py.astype(np.int64), 0), H - 1)
        b = self.pool.flat[self._tbase[sl][:, None, None] + iy * W + ix].mean(axis=2)
        nz = self.rng.standard_normal(b.shape) * self.noise[sl][:, None]
        val = self.white[sl] - (self.white[sl] - self.black[sl]) * b + nz
        return np.clip(val, 0, 100).astype(np.int16)

    # ------------------------------------------------------------ 진행도
    def progress(self):
        """국소창 최근접점 → (트랙 내 누적거리 s, 부호 있는 횡편차 lat(+좌), 진행방향 헤딩)."""
        P = self.pool
        lo, hi = self.PROG_WIN
        start = P.p0[self.ti]
        end = start + P.plen[self.ti] - 1
        idx = np.clip(self.pi[:, None] + np.arange(lo, hi + 1)[None, :], start[:, None], end[:, None])
        dx = P.path[idx, 0] - self.x[:, None]
        dy = P.path[idx, 1] - self.y[:, None]
        d2 = dx * dx + dy * dy
        j = np.argmin(d2, axis=1)
        self.pi = idx[self.ar, j]
        nxt = np.minimum(self.pi + 1, end)
        prv = np.maximum(self.pi - 1, start)
        tx = P.path[nxt, 0] - P.path[prv, 0]
        ty = P.path[nxt, 1] - P.path[prv, 1]
        cross = tx * (self.y - P.path[self.pi, 1]) - ty * (self.x - P.path[self.pi, 0])
        lat = np.sqrt(d2[self.ar, j]) * np.where(cross >= 0, 1.0, -1.0)
        s = P.s[self.pi] - P.s[start]
        self.max_s = np.maximum(self.max_s, s)
        path_h = np.arctan2(ty, tx)
        return s, lat, path_h

    def curvature_ahead(self, dists=(100.0, 200.0, 400.0)):
        """경로 앞쪽 헤딩 변화(rad) — 비대칭 크리틱용 특권 정보."""
        P = self.pool
        end = P.p0[self.ti] + P.plen[self.ti] - 1
        h0 = self._path_heading(self.pi, end)
        out = []
        for d in dists:
            j = np.minimum(self.pi + int(d / 2.0), end)
            dh = self._path_heading(j, end) - h0
            out.append((dh + np.pi) % (2 * np.pi) - np.pi)
        return np.stack(out, axis=1)

    def _path_heading(self, i, end):
        P = self.pool
        j = np.minimum(i + 3, end)
        i0 = np.maximum(j - 6, 0)
        return np.arctan2(P.path[j, 1] - P.path[i0, 1], P.path[j, 0] - P.path[i0, 0])


# ------------------------------------------------------------ 도메인 랜덤화
DR_PRESETS = {
    # sweep: sweep_lf3.make_world와 같은 분포 (규칙기반 스윕과 직접 비교용)
    "sweep": dict(tau_k=(0.85, 1.15), tau_dn_ms=None, battery=(0.85, 1.0), noise=(0.5, 2.0),
                  lag_ms=(10.0, 25.0), pos_fwd=3.0, pos_lat=1.5, white_k=None, footprint=None,
                  wheel_k=None, lat0=8.0, head0=0.0),
    # rl: 설계문서 5.6 범위 (실측 전이라 넓게)
    "rl": dict(tau_k=(0.8, 1.2), tau_dn_ms=(20.0, 60.0), battery=(0.85, 1.0), noise=(0.5, 2.0),
               lag_ms=(5.0, 35.0), pos_fwd=3.0, pos_lat=1.5, white_k=(0.85, 1.15),
               footprint=(5.0, 10.0), wheel_k=(0.99, 1.01), lat0=8.0, head0=5.0),
    # geo: 실측 치수 프로파일(spike_3s_geo)용 — 구동/자이로 지연·회전 권한·바퀴 흔들림 랜덤
    "geo": dict(tau_k=(0.85, 1.15), tau_dn_ms=None, battery=(0.85, 1.0), noise=(0.3, 1.5),
                lag_ms=(3.0, 10.0), pos_fwd=4.0, pos_lat=1.5, white_k=(0.9, 1.1),
                footprint=None, wheel_k=(0.99, 1.01), lat0=8.0, head0=5.0,
                cmd_lag_ms=(0.0, 35.0), imu_lag_ms=(0.0, 20.0), w_eff=(1.15, 1.5), wobble=(0.005, 0.035)),
    "none": dict(tau_k=None, tau_dn_ms=None, battery=None, noise=None, lag_ms=None, pos_fwd=0.0,
                 pos_lat=0.0, white_k=None, footprint=None, wheel_k=None, lat0=0.0, head0=0.0),
}


def sample_world(rng, base, m, preset="rl"):
    """m개 env의 월드 파라미터 (FastSim.set_world 형식)."""
    d = DR_PRESETS[preset]
    S = len(base["sensor_ports"])
    u = lambda r: rng.uniform(r[0], r[1], m)  # noqa: E731
    w = {}
    if d["tau_k"]:
        w["tau_up"] = base["motor_tau_s"] * u(d["tau_k"])
    if d["tau_dn_ms"]:
        w["tau_dn"] = u(d["tau_dn_ms"]) / 1000.0
    if d["battery"]:
        w["battery"] = u(d["battery"])
    if d["noise"]:
        w["noise"] = u(d["noise"])
    if d["lag_ms"]:
        w["lag_ms"] = u(d["lag_ms"])
    if d["pos_fwd"] or d["pos_lat"]:
        e = np.zeros((m, S, 2))
        e[:, :, 0] = rng.uniform(-d["pos_fwd"], d["pos_fwd"], (m, S))
        e[:, :, 1] = rng.uniform(-d["pos_lat"], d["pos_lat"], (m, S))
        w["pos_err"] = e
    if d["white_k"]:
        w["white_k"] = rng.uniform(d["white_k"][0], d["white_k"][1], (m, S))
    if d["footprint"]:
        w["footprint_mm"] = u(d["footprint"])
    if d["wheel_k"]:
        w["wheel_mm"] = base["wheel_diameter_mm"] * u(d["wheel_k"])
    for key in ("cmd_lag_ms", "imu_lag_ms", "w_eff"):
        if d.get(key):
            w[key] = u(d[key])
    if d.get("wobble"):
        w["wobble"] = u(d["wobble"])[:, None] * rng.uniform(0.5, 1.5, (m, 2))
        w["wobble_ph"] = rng.uniform(0, 2 * np.pi, (m, 2))
    lat0 = rng.uniform(-d["lat0"], d["lat0"], m) if d["lat0"] else np.zeros(m)
    head0 = rng.uniform(-d["head0"], d["head0"], m) if d["head0"] else np.zeros(m)
    return w, lat0, head0
