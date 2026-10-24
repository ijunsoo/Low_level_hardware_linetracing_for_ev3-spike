# =====================================================================
#  규칙기반 3센서 컨트롤러의 벡터화판 (N대 동시) — fastsim·RL 환경용
#
#  spike/spike_lf3_rule.py 의 run() 루프를 한 줄씩 그대로 옮겼다 (정수 연산 포함).
#  튜닝 상수는 실기 파일에서 직접 읽어 오므로(load_constants) 파일 기본값을 바꾸면
#  여기도 따라온다. 로직을 바꾸면 parity_test.py로 core+실기파일과 다시 맞춰 볼 것.
#
#  RL 잔차 훅: step(..., dsteer=, power=) — dsteer는 규칙 조향에 더하고(속도스케줄·
#  슬루 전), power는 최종 파워를 덮어쓴다.
# =====================================================================
import os
import re

import numpy as np

from . import _SIM_DIR

RULE_FILE = os.path.join(os.path.dirname(_SIM_DIR), "spike", "spike_lf3_rule.py")
NAMES = ("POWER", "KP", "KD", "KG", "G_CLAMP", "ALPHA", "D_FILTER", "MAX_STEER_DELTA",
         "W_NORM", "BLACK_MID", "SIDE_ERR", "GAP_MS", "AHEAD_HOLD_MS", "LOCK_MS",
         "SEARCH_DEG", "USE_SENTINEL", "ST_BANK", "RECOVER_POWER", "SPD_K10", "MIN_POWER",
         "PB_MS", "PB_POWER", "FORCE_MS",
         "KP2", "KG2", "CURVE_YAW", "CURVE_TAU_MS", "KPN", "FF", "G_REL", "CEMA_BANK", "ERR_MODE", "SENSOR_GAP_MM", "CF_GAIN", "SIDE_SEE", "DFLOOR", "KSP", "DELAY_MS", "YAW_PER_STEER", "TAU_MS")


def load_constants(path=RULE_FILE):
    src = open(path, encoding="utf-8").read()
    c = {}
    for name in NAMES:
        m = re.search(r"^%s\s*=\s*([^#\n]+?)\s*(#.*)?$" % name, src, re.M)
        if not m:
            raise ValueError("상수 없음: %s" % name)
        c[name] = eval(m.group(1), {})        # 숫자/True/False만 있는 줄
    return c


class RuleLF3:
    def __init__(self, n, consts=None):
        self.n = n
        self.c = dict(consts or load_constants())
        c = self.c
        self.S = 100000
        self.KP_S = round(c["KP"] * self.S)
        self.KD_S = round(c["KD"] * self.S)
        self.A_COMP = 1000 - c["ALPHA"]
        self.D_COMP = 1000 - c["D_FILTER"]
        i = lambda v=0: np.full(n, v, np.int64)  # noqa: E731
        b = lambda v=False: np.full(n, v, bool)  # noqa: E731
        self.wv, self.bv = np.zeros((n, 3), np.int64), np.zeros((n, 3), np.int64)
        self.sL, self.sR = i(), i()
        self.prev = np.zeros((n, 3), np.int64)
        self.last_error, self.d_filt, self.prev_steer, self.last_good = i(), i(), i(), i()
        self.side = i(-1)
        self.was_off, self.armed = b(), b()
        self.lock_side, self.t_lock, self.t_ahead = i(), i(), i()
        self.t_lost, self.pb_until, self.t_step = i(-1), i(), i()
        self.bank_dir, self.turned, self.flips = i(), i(), i()
        self.left = np.zeros(n)
        self.right = np.zeros(n)
        self.steer_out = i()                  # 마지막 조향 (관측용)
        self.error_out = i()
        self.yellow = i()                     # 이탈 진입 횟수 (LED 노랑)
        self.red = i()                        # 코너 예고 횟수 (LED 빨강)
        self.nsteps = i()                     # 프레임 동기를 통과한 제어 스텝 수

    def set_value(self, idx, white, black):
        """env별 VALUE (흰/검) — 실기에서 센서 옮기면 재측정하는 값 = 그 env의 센서 특성."""
        self.wv[idx] = np.asarray(white).astype(np.int64)
        self.bv[idx] = np.asarray(black).astype(np.int64)

    def _norm(self, raw):
        r = np.clip(raw.astype(np.int64), self.bv, self.wv)
        return (r - self.bv) * 100 // (self.wv - self.bv)

    def reset(self, idx, raw):
        """루프 진입 전 초기 읽기 (실기 run()의 pL/pR/pF, sL/sR 초기화)."""
        r = raw[idx].astype(np.int64)
        n0 = np.clip(r, self.bv[idx], self.wv[idx])
        n0 = (n0 - self.bv[idx]) * 100 // (self.wv[idx] - self.bv[idx])
        self.prev[idx] = r
        self.sL[idx], self.sR[idx] = n0[:, 0], n0[:, 1]
        for a, v in ((self.last_error, 0), (self.d_filt, 0), (self.prev_steer, 0),
                     (self.last_good, 0), (self.side, -1), (self.lock_side, 0),
                     (self.t_lock, 0), (self.t_ahead, 0), (self.t_lost, -1),
                     (self.pb_until, 0), (self.t_step, 0), (self.bank_dir, 0),
                     (self.turned, 0), (self.flips, 0), (self.steer_out, 0),
                     (self.error_out, 0), (self.yellow, 0), (self.red, 0)):
            a[idx] = v
        self.was_off[idx] = False
        self.armed[idx] = False
        self.left[idx] = 0.0
        self.right[idx] = 0.0

    def step(self, raw, now, gyro, dsteer=None, power=None):
        """raw (n,3) 노출 센서값, now (n) ms(int, StopWatch), gyro (n) int deg/s(좌회전+).
        반환: (left, right) 전진 기준 duty. 프레임 동기에 걸린 env는 이전 출력 유지."""
        c = self.c
        raw = raw.astype(np.int64)
        fresh = (raw != self.prev).any(axis=1) | (now - self.t_step >= c["FORCE_MS"])
        if not fresh.any():
            return self.left, self.right
        F = fresh
        self.prev[F] = raw[F]
        dt = now - self.t_step
        self.t_step = np.where(F, now, self.t_step)

        n = self._norm(raw)
        nL, nR, nF = n[:, 0], n[:, 1], n[:, 2]
        sL = (c["ALPHA"] * nL + self.A_COMP * self.sL) // 1000
        sR = (c["ALPHA"] * nR + self.A_COMP * self.sR) // 1000

        pair_white = (nL > c["W_NORM"]) & (nR > c["W_NORM"])
        armed, t_ahead = self.armed.copy(), self.t_ahead.copy()
        lock_side, pb_until = self.lock_side.copy(), self.pb_until.copy()
        red = self.red.copy()
        if c["USE_SENTINEL"]:
            fb = nF < c["BLACK_MID"]
            t_ahead = np.where(fb, now, t_ahead)
            armed = armed & ~fb
            ahead = now - t_ahead <= c["AHEAD_HOLD_MS"]
            arm = ~ahead & ~pair_white & ~armed
            armed = armed | arm
            lock_side = np.where(arm, 0, lock_side)
            pb_until = np.where(arm, now + c["PB_MS"], pb_until)
            red = red + arm
        else:
            ahead = np.zeros(self.n, bool)

        error = 2 * (sL - sR)
        side = np.where(error > c["SIDE_ERR"], 1, np.where(error < -c["SIDE_ERR"], -1, self.side))
        t_lock = self.t_lock.copy()
        was_off = self.was_off.copy()
        if c["LOCK_MS"] > 0:
            setl = armed & (lock_side == 0) & ((error > c["SIDE_ERR"]) | (error < -c["SIDE_ERR"]))
            clr = ~setl & (lock_side != 0) & ~was_off & (now - t_lock > c["LOCK_MS"])
            lock_side = np.where(setl, side, np.where(clr, 0, lock_side))
            t_lock = np.where(setl, now, t_lock)
        else:
            clr = (lock_side != 0) & ~was_off & (now - t_lock > c["LOCK_MS"])
            lock_side = np.where(clr, 0, lock_side)

        online = ~pair_white | ahead
        # ---- 라인 보임 ----
        recap = online & was_off
        was_off = np.where(recap, False, was_off)
        lock_side = np.where(recap, 0, lock_side)
        d_filt = np.where(recap, 0, self.d_filt)
        last_error = np.where(recap, error, self.last_error)
        prev_steer = np.where(recap, 0, self.prev_steer)
        d_new = (c["D_FILTER"] * (error - last_error) + self.D_COMP * d_filt) // 1000
        steer_on = (self.KP_S * error + self.KD_S * d_new) // self.S
        steer_on = np.where((sL < c["BLACK_MID"]) & (sR < c["BLACK_MID"]), 0, steer_on)
        g = np.clip((c["KG"] * gyro) // 1000, -c["G_CLAMP"], c["G_CLAMP"])
        steer_on = steer_on + g
        d_filt = np.where(online, d_new, d_filt)
        last_error = np.where(online, error, last_error)
        last_good = np.where(online, steer_on, self.last_good)
        t_lost = np.where(online, -1, self.t_lost)
        # ---- 이탈 ----
        lost = ~online
        t_lost = np.where(lost & (t_lost < 0), now, t_lost)
        in_gap = lost & (now - t_lost <= c["GAP_MS"])
        bank = lost & ~in_gap
        enter = bank & ~was_off
        was_off = was_off | enter
        bank_dir = np.where(enter, np.where(lock_side != 0, lock_side, side), self.bank_dir)
        turned = np.where(enter, 0, self.turned)
        flips = np.where(enter, 0, self.flips)
        if c["SEARCH_DEG"] > 0:
            cont = bank & ~enter
            turned = np.where(cont, turned - bank_dir * gyro * dt, turned)
            flip = cont & (flips < 2) & (turned > c["SEARCH_DEG"] * 1000)
            bank_dir = np.where(flip, -bank_dir, bank_dir)
            turned = np.where(flip, 0, turned)
            flips = np.where(flip, flips + 1, flips)
        steer = np.where(online, steer_on, np.where(in_gap, last_good, bank_dir * c["ST_BANK"]))
        pw = np.where(bank, c["RECOVER_POWER"], c["POWER"]).astype(np.int64)

        if dsteer is not None:
            steer = steer + np.round(dsteer).astype(np.int64)
        if c["SPD_K10"] > 0:
            sched = np.maximum(c["POWER"] - c["SPD_K10"] * np.abs(steer) // 10, c["MIN_POWER"])
            pw = np.minimum(pw, sched)
        pw = np.where((now < pb_until) & (pw > c["PB_POWER"]), c["PB_POWER"], pw)
        pwf = pw.astype(float) if power is None else np.asarray(power, float)

        delta = steer - prev_steer
        steer = np.where(delta > c["MAX_STEER_DELTA"], prev_steer + c["MAX_STEER_DELTA"],
                         np.where(delta < -c["MAX_STEER_DELTA"], prev_steer - c["MAX_STEER_DELTA"], steer))
        prev_steer = steer

        st = np.clip(steer, -100, 100).astype(float)
        slope = pwf * 2 / 100
        left = np.where(st < 0, np.round(slope * st + pwf, 2), pwf)
        right = np.where(st < 0, pwf, np.round(-slope * st + pwf, 2))

        # ---- 프레임 동기: 새 프레임 env만 상태 반영 ----
        for name, val in (("sL", sL), ("sR", sR), ("armed", armed), ("t_ahead", t_ahead),
                          ("lock_side", lock_side), ("pb_until", pb_until), ("side", side),
                          ("t_lock", t_lock), ("was_off", was_off), ("d_filt", d_filt),
                          ("last_error", last_error), ("last_good", last_good),
                          ("t_lost", t_lost), ("bank_dir", bank_dir), ("turned", turned),
                          ("flips", flips), ("prev_steer", prev_steer), ("left", left),
                          ("right", right), ("steer_out", steer), ("error_out", error),
                          ("red", red)):
            cur = getattr(self, name)
            setattr(self, name, np.where(F, val, cur))
        self.yellow = self.yellow + (F & enter)
        self.nsteps = self.nsteps + F
        return self.left, self.right

    def step_frame(self, sim, now, gyro, p_torn=0.0, dsteer=None, power=None):
        """한 틱의 제어: (확률 p_torn으로) 찢어진 읽기 스텝 → 온전한 프레임 스텝."""
        if np.any(p_torn):
            self.step(sim.torn(p_torn), now, gyro, dsteer, power)
        return self.step(sim.raw, now, gyro, dsteer, power)
