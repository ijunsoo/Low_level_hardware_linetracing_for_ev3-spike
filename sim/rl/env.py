# =====================================================================
#  강화학습 환경 (N대 동시, numpy) — 설계문서 5.4 'A1 잔차 정책'
#
#  한 스텝 = 결정 주기(기본 5ms). 그 사이 매 센서 프레임마다 규칙기반(RuleLF3,
#  실기 spike_lf3_rule.py와 같은 로직)이 돌고, 정책은
#    · Δ조향  ∈ [-1,1]×DSTEER_MAX  (규칙 조향에 더함)
#    · 파워   ∈ [-1,1] → [P_MIN, 100] (최종 파워를 덮어씀)
#  을 낸다. mode="steer_only"면 파워는 100 고정(감속 금지 방침), "rule"이면 정책 무시.
#
#  관측 22 (실기에서 잴 수 있는 것만): 센서 3개 정규화값 × 최근 4결정시점(12),
#   규칙 오차·조향(2), 요레이트(1), 좌우 바퀴속도(2), 직전 행동(2),
#   센티넬이 검정 본 뒤 경과·이탈 경과·마지막 방향(3)
#  크리틱 전용 특권 관측: 진짜 횡편차·헤딩오차·전방 곡률·속도·월드 파라미터
#  보상: 진행 Δs/(v_ref·dt) − 0.05(lat/20mm)² − 0.01|Δa|² , 이탈 −50, 완주 +20
#  도메인 랜덤화: fastsim.DR_PRESETS + 찢어진 읽기 확률 p_torn
# =====================================================================
import math

import numpy as np

from . import _SIM_DIR  # noqa: F401
import core
from .fastsim import FastSim, TrackPool, sample_world
from .rule_lf3 import RuleLF3, load_constants

OBS_DIM = 22
CRITIC_DIM = 12
ACT_DIM = 2


class LineFollowEnv:
    def __init__(self, n, tracks, profile="spike_3s", dr="rl", mode="residual",
                 decision_ms=5.0, p_torn=(0.0, 2.0 / 3.0), max_s=40.0, start_frac=0.7,
                 dsteer_max=40.0, p_min=20.0, stuck_s=3.0, seed=0, rule_consts=None,
                 w_lat=0.05, w_da=0.01, r_lost=-50.0, r_finish=20.0):
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.base = core.load_profile(profile)
        self.pool = tracks if isinstance(tracks, TrackPool) else TrackPool(tracks)
        self.sim = FastSim(n, self.pool, self.base, seed=seed + 1)
        self.rule = RuleLF3(n, rule_consts or load_constants())
        self.dr, self.mode = dr, mode
        self.K = max(1, int(round(decision_ms / 1000.0 / self.sim.dt)))
        self.dec_dt = self.K * self.sim.dt
        self.p_torn_rng = p_torn
        self.max_s, self.start_frac, self.stuck_s = max_s, start_frac, stuck_s
        self.dsteer_max, self.p_min = dsteer_max, p_min
        self.w_lat, self.w_da, self.r_lost, self.r_finish = w_lat, w_da, r_lost, r_finish
        dc_v = self.sim.dc_v[-1]
        self.v_ref = dc_v * self.base.get("gear_ratio", 1.0) * math.pi * self.base["wheel_diameter_mm"] / 360.0
        z = np.zeros(n)
        self.t_loop, self.s_prev, self.t_prog = z.copy(), z.copy(), z.copy()
        self.p_torn = z.copy()
        self.a_prev = np.zeros((n, ACT_DIM))
        self.hist = np.zeros((n, 4, 3))
        self.ep_ret, self.ep_len = z.copy(), np.zeros(n, np.int64)
        self.lat = z.copy()
        self._prog_mark = z.copy()
        self.reset_all()

    # ------------------------------------------------------------ 리셋
    def reset_all(self):
        self._reset(np.arange(self.n))
        return self.obs()

    def _reset(self, idx):
        m = len(idx)
        sim, rule = self.sim, self.rule
        w, lat0, head0 = sample_world(self.rng, self.base, m, self.dr)
        sim.set_world(idx, w)
        ti = self.rng.integers(self.pool.n, size=m)
        sim.set_tracks(idx, ti)
        plen = self.pool.plen[ti]
        start = (self.rng.random(m) * self.start_frac * np.maximum(plen - 400, 1)).astype(np.int64)
        sim.reset_pose(idx, start, lat0, head0)
        rule.set_value(idx, np.round(sim.white[idx]), np.round(sim.black[idx]))
        rule.reset(idx, sim.raw)
        lo, hi = self.p_torn_rng
        self.p_torn[idx] = self.rng.uniform(lo, hi, m)
        self.t_loop[idx] = 0.0
        self.t_prog[idx] = 0.0
        self._prog_mark[idx] = sim.max_s[idx]
        self.s_prev[idx] = self.pool.s[sim.pi[idx]] - self.pool.s[self.pool.p0[ti]]
        self.a_prev[idx] = 0.0
        self.ep_ret[idx] = 0.0
        self.ep_len[idx] = 0
        n_now = self._norm(sim.raw[idx], idx)
        self.hist[idx] = n_now[:, None, :]

    def _norm(self, raw, idx=None):
        sl = slice(None) if idx is None else idx
        w, b = self.rule.wv[sl], self.rule.bv[sl]
        return (np.clip(raw.astype(np.int64), b, w) - b) * 100 // (w - b) / 100.0

    # ------------------------------------------------------------ 스텝
    def step(self, action):
        sim, rule = self.sim, self.rule
        a = np.clip(np.asarray(action, float).reshape(self.n, ACT_DIM), -1.0, 1.0)
        if self.mode == "rule":
            dsteer, power = None, None
        else:
            dsteer = a[:, 0] * self.dsteer_max
            if self.mode == "steer_only":
                power = np.full(self.n, 100.0)
            else:
                power = self.p_min + (a[:, 1] + 1.0) / 2.0 * (100.0 - self.p_min)
        for _ in range(self.K):
            now = (self.t_loop * 1000.0).astype(np.int64)
            gyro = np.trunc(sim.yaw_dps()).astype(np.int64)
            l, r = rule.step_frame(sim, now, gyro, self.p_torn, dsteer, power)
            sim.tick(l, r)
            self.t_loop += sim.dt

        s, lat, path_h = sim.progress()
        self.lat = lat
        self.path_h = path_h
        ds = s - self.s_prev
        self.s_prev = s
        rew = ds / (self.v_ref * self.dec_dt)
        rew -= self.w_lat * (lat / 20.0) ** 2
        rew -= self.w_da * ((a - self.a_prev) ** 2).sum(axis=1)
        self.a_prev = a

        length = self.pool.length[sim.ti]
        moved = sim.max_s > self._prog_mark + 50.0
        self._prog_mark = np.where(moved, sim.max_s, self._prog_mark)
        self.t_prog = np.where(moved, self.t_loop, self.t_prog)
        lost = np.abs(lat) > core.Sim.LOST_MM
        fin = s > length - core.Sim.FINISH_MM
        stuck = self.t_loop - self.t_prog > self.stuck_s
        timeout = self.t_loop > self.max_s
        rew = np.where(lost, rew + self.r_lost, rew)
        rew = np.where(fin, rew + self.r_finish, rew)
        done = lost | fin | stuck | timeout
        self.ep_ret += rew
        self.ep_len += 1

        # 관측 기록 갱신 (최근 4결정시점)
        self.hist = np.roll(self.hist, 1, axis=1)
        self.hist[:, 0] = self._norm(sim.raw)

        info = {}
        if done.any():
            idx = np.nonzero(done)[0]
            reason = np.where(lost, "lost", np.where(fin, "finish", np.where(stuck, "stuck", "timeout")))
            info = dict(idx=idx, reason=reason[idx], ret=self.ep_ret[idx].copy(),
                        len=self.ep_len[idx].copy(), time_s=self.t_loop[idx].copy(),
                        frac=(sim.max_s[idx] / length[idx]).copy(),
                        yellow=rule.yellow[idx].copy(), track=sim.ti[idx].copy(),
                        truncated=(timeout & ~lost & ~fin)[idx])
            self._reset(idx)
        return self.obs(), rew, done, info

    # ------------------------------------------------------------ 관측
    def obs(self):
        sim, rule = self.sim, self.rule
        now = (self.t_loop * 1000.0).astype(np.int64)
        vl, vr = sim.wheel_speed_mms()
        since_ahead = np.clip((now - rule.t_ahead) / 500.0, 0, 1)
        since_lost = np.where(rule.t_lost >= 0, np.clip((now - rule.t_lost) / 500.0, 0, 1), 0.0)
        o = np.concatenate([
            self.hist.reshape(self.n, 12),
            (rule.error_out / 200.0)[:, None], (rule.steer_out / 100.0)[:, None],
            (sim.yaw_dps() / 400.0)[:, None],
            (vl / self.v_ref)[:, None], (vr / self.v_ref)[:, None],
            self.a_prev,
            since_ahead[:, None], since_lost[:, None], rule.side[:, None].astype(float),
        ], axis=1)
        return o.astype(np.float32)

    def critic_obs(self):
        """비대칭 크리틱 전용 (실기에선 모르는 값)."""
        sim = self.sim
        path_h = getattr(self, "path_h", sim.h)
        herr = (sim.h - path_h + np.pi) % (2 * np.pi) - np.pi
        curv = sim.curvature_ahead()
        vl, vr = sim.wheel_speed_mms()
        c = np.concatenate([
            (self.lat / 50.0)[:, None], (herr / np.pi)[:, None], curv / np.pi,
            ((vl + vr) / 2 / self.v_ref)[:, None],
            (sim.lag * sim.dt * 1000 / 40.0)[:, None], sim.vscale[:, None] / 2.0,
            (sim.noise / 2.0)[:, None], self.p_torn[:, None],
            (sim.fp_r * 2 / 10.0)[:, None], (-sim.dt / np.log(sim.a_up) / 0.115)[:, None],
        ], axis=1)
        return c.astype(np.float32)


def make_pool(n_gen=32, levels=("medium", "hard"), include=("mrc_real", "photo"), seed0=0):
    """학습용 트랙 풀: 생성 트랙 + (옵션) 알려진 트랙. 평가 홀드아웃 시드는 쓰지 않는다."""
    from . import tracks as gen
    trs, names = [], []
    for i in range(n_gen):
        lv = levels[i % len(levels)]
        sd = seed0 + i
        assert sd < gen.HOLDOUT_SEED0
        trs.append(gen.make_track(gen.generate(sd, lv)))
        names.append("%s%d" % (lv, sd))
    for nm in include:
        trs.append(core.make_track(nm, 20.0))
        names.append(nm)
    return TrackPool(trs, names)
