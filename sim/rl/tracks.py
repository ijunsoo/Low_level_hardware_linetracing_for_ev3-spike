# =====================================================================
#  규정 기반 랜덤 트랙 생성기 — 강화학습 학습·평가용
#
#  Robotex 한국 LEGO 라인팔로잉 기준: 라인 20mm, 최소 곡률반경 100mm,
#  직각 코너, 교차는 직각으로 지나 직진, 마커(장식선), 필드 3~30m².
#  대회 트랙은 당일 공개라 특정 트랙에 맞추지 않도록 매번 새로 만든다.
#
#  출력은 core.TRACKS와 같은 spec: {"segs": [...], "extras": [...]}
#   ('S', L) 직진 / ('A', R, deg) 원호(+좌) / ('T', deg) 제자리 꺾임(직각·예각)
#  → make_track(spec)으로 core.Track을 만들면 실기 파일(core+목)에도,
#    fastsim에도 같은 트랙을 쓸 수 있다.
#
#  사용:
#    spec = generate(seed=3, level="hard")
#    track = make_track(spec)
#    specs = holdout(50)            # 평가 전용 고정 세트 (학습에 쓰지 말 것)
# =====================================================================
import math

import numpy as np

from . import _SIM_DIR  # noqa: F401  (sim/ 경로 등록)
import core

STEP = 2.0            # core.build_path와 같은 점 간격
CLEAR_MM = 150.0      # 서로 다른 라인 부분 사이 최소 간격 (설계된 교차점 제외)
EDGE_MM = 120.0       # 필드 가장자리 여백
TAIL_MM = 260.0       # 바로 앞 경로는 간격 검사에서 제외 (이어지는 부분)
HOLDOUT_SEED0 = 900000

LEVELS = {
    # r_min: 최소 반경 / p_*: 요소 확률 / field: (최소 W,H), (최대 W,H) mm / length: 목표 길이 mm
    "easy": dict(r_min=300, r_max=700, p_corner=0.0, p_zigzag=0.0, p_loop=0.0,
                 p_swave=0.15, p_marker=0.2, acute=False,
                 field=((1800, 1100), (3200, 2200)), length=(3000, 6000)),
    "medium": dict(r_min=150, r_max=600, p_corner=0.16, p_zigzag=0.0, p_loop=0.08,
                   p_swave=0.15, p_marker=0.3, acute=False,
                   field=((1800, 1100), (3500, 2500)), length=(3500, 8000)),
    "hard": dict(r_min=100, r_max=600, p_corner=0.2, p_zigzag=0.08, p_loop=0.12,
                 p_swave=0.15, p_marker=0.35, acute=True,
                 field=((1900, 1080), (4000, 3000)), length=(4000, 10000)),
}


def _trace(segs, x, y, h):
    """core.build_path와 같은 수식으로 (x,y,h)에서 세그먼트를 이어 그린다."""
    pts = []
    for seg in segs:
        if seg[0] == 'T':
            h += math.radians(seg[1])
            continue
        if seg[0] == 'S':
            L = seg[1]
            n = max(1, int(L / STEP))
            for i in range(1, n + 1):
                pts.append((x + math.cos(h) * L * i / n, y + math.sin(h) * L * i / n))
            x, y = pts[-1]
        else:
            r, adeg = seg[1], seg[2]
            a = math.radians(adeg)
            sgn = 1.0 if a > 0 else -1.0
            cx = x - math.sin(h) * r * sgn
            cy = y + math.cos(h) * r * sgn
            start = h - sgn * math.pi / 2
            n = max(2, int(abs(a) * r / STEP))
            for i in range(1, n + 1):
                th = start + a * i / n
                pts.append((cx + r * math.cos(th), cy + r * math.sin(th)))
            x, y = pts[-1]
            h += a
    return np.asarray(pts, dtype=float).reshape(-1, 2), x, y, h


class _Builder:
    def __init__(self, rng, cfg):
        self.rng = rng
        self.cfg = cfg
        (w0, h0), (w1, h1) = cfg["field"]
        self.W = float(rng.uniform(w0, w1))
        self.H = float(rng.uniform(h0, h1))
        # 시작점 = 왼쪽 가장자리 근처, 동쪽(+x) 출발. 필드는 시작점 기준으로 배치
        ax = EDGE_MM + 60.0
        by = float(rng.uniform(EDGE_MM + 200.0, self.H - EDGE_MM - 200.0))
        self.box = (-ax + EDGE_MM, self.W - ax - EDGE_MM, -by + EDGE_MM, self.H - by - EDGE_MM)
        self.x = self.y = self.h = 0.0
        self.segs = []
        self.extras = []
        self.pts = np.zeros((0, 2))       # 20mm 간격 표본 (간격 검사용)
        self.pts_s = np.zeros(0)          # 각 표본의 누적거리 (마커 = -1e9: 항상 검사)
        self.length = 0.0
        self.last_kind = None
        self.last_straight = 0.0
        self.features = {"corner": 0, "acute": 0, "zigzag": 0, "loop": 0, "swave": 0, "marker": 0}

    # ---- 검사 ----
    def _ok(self, cand, exempt=None):
        if len(cand) == 0:
            return True
        x0, x1, y0, y1 = self.box
        if (cand[:, 0].min() < x0 or cand[:, 0].max() > x1
                or cand[:, 1].min() < y0 or cand[:, 1].max() > y1):
            return False
        if len(self.pts) == 0:
            return True
        old = self.pts_s < self.length - TAIL_MM       # 바로 앞 경로는 제외
        P = self.pts[old]
        if len(P) == 0:
            return True
        c = cand[::5]                                   # 10mm 간격 표본
        if exempt is not None:
            ex, er = exempt
            keep_c = np.hypot(c[:, 0] - ex[0], c[:, 1] - ex[1]) > er
            keep_p = np.hypot(P[:, 0] - ex[0], P[:, 1] - ex[1]) > er
            c, P = c[keep_c], P[keep_p]
            if len(c) == 0 or len(P) == 0:
                return True
        d2 = ((c[:, None, :] - P[None, :, :]) ** 2).sum(-1)
        return float(d2.min()) >= CLEAR_MM ** 2

    def _try(self, segs, kind, exempt=None):
        cand, x, y, h = _trace(segs, self.x, self.y, self.h)
        if not self._ok(cand, exempt):
            return False
        # 채택
        seg_len = float(np.hypot(*np.diff(np.vstack([[self.x, self.y], cand]), axis=0).T).sum())
        s_new = self.length + np.cumsum(np.hypot(*np.diff(np.vstack([[self.x, self.y], cand]), axis=0).T))
        sub = slice(9, None, 10)                        # 20mm 간격
        self.pts = np.vstack([self.pts, cand[sub]])
        self.pts_s = np.concatenate([self.pts_s, s_new[sub]])
        self.segs.extend(segs)
        self.x, self.y, self.h = x, y, h
        self.length += seg_len
        self.last_kind = kind
        self.last_straight = segs[-1][1] if segs[-1][0] == 'S' else 0.0
        return True

    def _marker(self, L):
        # 방금 놓은 직선(길이 L) 위에 장식 마커: 실선 가로줄 또는 양옆 점선
        f = float(self.rng.uniform(0.3, 0.7))
        back = L * (1 - f)
        cx = self.x - math.cos(self.h) * back
        cy = self.y - math.sin(self.h) * back
        nx, ny = -math.sin(self.h), math.cos(self.h)
        if self.rng.random() < 0.5:
            lines = [[(cx + 150 * nx, cy + 150 * ny), (cx - 150 * nx, cy - 150 * ny)]]
        else:
            lines = [[(cx + 40 * nx, cy + 40 * ny), (cx + 150 * nx, cy + 150 * ny)],
                     [(cx - 40 * nx, cy - 40 * ny), (cx - 150 * nx, cy - 150 * ny)]]
        samp = np.asarray([p for ln in lines for p in core._densify(ln, 10.0)])
        x0, x1, y0, y1 = self.box
        if (samp[:, 0].min() < x0 or samp[:, 0].max() > x1
                or samp[:, 1].min() < y0 or samp[:, 1].max() > y1):
            return
        # 다른 라인 부분과 60mm 이상 떨어져야 (자기 직선은 제외)
        far = self.pts_s < self.length - L - 100
        if far.any():
            d2 = ((samp[:, None, :] - self.pts[far][None, :, :]) ** 2).sum(-1)
            if float(d2.min()) < 60.0 ** 2:
                return
        self.extras.extend(lines)
        self.pts = np.vstack([self.pts, samp[::2]])
        self.pts_s = np.concatenate([self.pts_s, np.full(len(samp[::2]), -1e9)])
        self.features["marker"] += 1

    # ---- 요소 ----
    def _straight(self, lo=150, hi=900):
        L = float(self.rng.uniform(lo, hi))
        if self._try([('S', round(L, 1))], "S"):
            if L >= 400 and self.rng.random() < self.cfg["p_marker"]:
                self._marker(L)
            return True
        return False

    def _arc(self):
        c = self.cfg
        R = float(self.rng.uniform(c["r_min"], c["r_max"]))
        deg = float(self.rng.uniform(30, 180)) * (1 if self.rng.random() < 0.5 else -1)
        return self._try([('A', round(R, 1), round(deg, 1))], "A")

    def _corner(self):
        acute = self.cfg["acute"] and self.rng.random() < 0.3
        deg = float(self.rng.uniform(100, 135)) if acute else 90.0
        deg *= 1 if self.rng.random() < 0.5 else -1
        L = float(self.rng.uniform(200, 600))
        if self._try([('T', round(deg, 1)), ('S', round(L, 1))], "corner"):
            self.features["acute" if acute else "corner"] += 1
            return True
        return False

    def _zigzag(self):
        a = float(self.rng.uniform(45, 60)) * (1 if self.rng.random() < 0.5 else -1)
        L = float(self.rng.uniform(150, 300))
        segs = [('T', -a), ('S', L), ('T', 2 * a), ('S', L), ('T', -2 * a), ('S', L),
                ('T', 2 * a), ('S', L), ('T', -a), ('S', 250.0)]
        segs = [(s[0], round(s[1], 1)) for s in segs]
        if self._try(segs, "zigzag"):
            self.features["zigzag"] += 1
            return True
        return False

    def _loop(self):
        # 270° 원호 → 직진 = 진입 직선을 직각으로 가로지름 (mrc의 루프 교차와 같은 구조)
        c = self.cfg
        R = float(self.rng.uniform(max(c["r_min"], 150), 300))
        if self.last_kind != "S" or self.last_straight < R + 60:
            return False
        sgn = 1 if self.rng.random() < 0.5 else -1
        # 교차점 = 원호 시작점에서 진입 직선을 따라 R만큼 뒤
        ex = (self.x - math.cos(self.h) * R, self.y - math.sin(self.h) * R)
        L_out = R + CLEAR_MM + float(self.rng.uniform(100, 300))
        segs = [('A', round(R, 1), 270.0 * sgn), ('S', round(L_out, 1))]
        if self._try(segs, "loop", exempt=(ex, CLEAR_MM + 40.0)):
            self.features["loop"] += 1
            return True
        return False

    def _swave(self):
        c = self.cfg
        R = float(self.rng.uniform(max(c["r_min"], 150), 300))
        th = float(self.rng.uniform(45, 90)) * (1 if self.rng.random() < 0.5 else -1)
        segs = [('A', round(R, 1), round(th, 1)), ('A', round(R, 1), round(-2 * th, 1)),
                ('A', round(R, 1), round(th, 1))]
        if self._try(segs, "swave"):
            self.features["swave"] += 1
            return True
        return False

    def run(self, target):
        if not self._straight(300, 600):
            return False
        fails = 0
        while self.length < target:
            c = self.cfg
            u = self.rng.random()
            special = self.last_kind in ("S",)       # 특수 요소는 직선 뒤에만
            if special and u < c["p_loop"]:
                ok = self._loop()
            elif special and u < c["p_loop"] + c["p_zigzag"]:
                ok = self._zigzag()
            elif special and u < c["p_loop"] + c["p_zigzag"] + c["p_corner"]:
                ok = self._corner()
            elif u < c["p_loop"] + c["p_zigzag"] + c["p_corner"] + c["p_swave"]:
                ok = self._swave()
            elif self.last_kind == "S":
                ok = self._arc()
            else:
                ok = self._straight()
            fails = 0 if ok else fails + 1
            if fails > 40:
                return False
        return self._straight(250, 500)               # 결승 직선


def generate(seed, level="hard", max_tries=400):
    """시드 하나 → 규정에 맞는 트랙 spec (재현 가능)."""
    rng = np.random.default_rng(seed)
    cfg = LEVELS[level]
    for _ in range(max_tries):
        b = _Builder(rng, cfg)
        target = float(rng.uniform(*cfg["length"]))
        if b.run(target) and validate(b):
            return {"segs": b.segs, "extras": b.extras,
                    "meta": dict(seed=seed, level=level, length_mm=round(b.length, 1),
                                 field_mm=(round(b.W), round(b.H)),
                                 area_m2=round(b.W * b.H / 1e6, 2), features=b.features)}
    raise RuntimeError("트랙 생성 실패 seed=%d" % seed)


def validate(b):
    """완성 트랙 전체 재검사: 누적거리가 3×CLEAR 이상 떨어진 부분끼리 CLEAR×0.8 이상
    (설계된 루프 교차점 근처 제외 — 루프 교차는 진입 직선 위에서만 생긴다)."""
    path, _, _, _ = _trace(b.segs, 0.0, 0.0, 0.0)
    seg = np.diff(np.vstack([[0.0, 0.0], path]), axis=0)
    s = np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))
    P, S = path[9::10], s[9::10]
    d = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(-1))
    far = np.abs(S[:, None] - S[None, :]) > 3 * CLEAR_MM
    bad = far & (d < 0.8 * CLEAR_MM)
    if not bad.any():
        return True
    # 위반 쌍이 전부 '루프 교차'(직각 교차, 거리 ≈ 0 근처 국소 영역)인지 확인
    i, j = np.nonzero(bad)
    return b.features["loop"] > 0 and bool((d[i, j] < 0.8 * CLEAR_MM).all()) and \
        _crossings_ok(P, S, i, j)


def _crossings_ok(P, S, i, j):
    # 위반 점들이 교차점 몇 개 주변(반경 CLEAR+60) 안에만 몰려 있으면 설계된 교차로 본다
    pts = P[np.concatenate([i, j])]
    centers = []
    for p in pts:
        if not any(np.hypot(*(p - c)) < CLEAR_MM + 60 for c in centers):
            centers.append(p)
    return len(centers) <= 2 * 8


def make_track(spec, line_width_mm=20.0):
    return core.Track(core.build_path(spec["segs"]), line_width_mm, extras=spec.get("extras"))


def holdout(n=50, level="hard"):
    """평가 전용 고정 세트 — 학습 시드와 겹치지 않게 HOLDOUT_SEED0부터."""
    return [generate(HOLDOUT_SEED0 + i, level) for i in range(n)]


if __name__ == "__main__":
    import sys
    import time
    lvl = sys.argv[1] if len(sys.argv) > 1 else "hard"
    t0 = time.time()
    for sd in range(8):
        sp = generate(sd, lvl)
        m = sp["meta"]
        print("seed %d: %.1fm, 필드 %s (%.1fm²), %s" % (sd, m["length_mm"] / 1000, m["field_mm"],
                                                    m["area_m2"], m["features"]))
    print("%.2fs" % (time.time() - t0))
