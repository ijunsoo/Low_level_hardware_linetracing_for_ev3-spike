# =====================================================================
#  라인트레이서 2D 시뮬레이터 코어 (SPIKE Prime · EV3 공용)
# ---------------------------------------------------------------------
#  - 차동구동 운동학 + 비대칭 1차지연 모터모델(dc→속도 곡선, τ_up/τ_dn)
#  - 컬러센서 N개: 트랙 래스터 위 발자국(원) 평균 + 노이즈 + UART 지연
#  - 시간 진행 = '호출 비용' 모델: 센서 읽기·자이로·모터 명령이 각자
#    비용만큼 시뮬 시간을 흘린다. 기본값은 SPIKE 실측 그대로
#    (센서 read 1회 = 1/(loop_hz×센서수), 모터·IMU = 0)
#  - 하드웨어 프로파일: robot_params.json(= spike_4s) + profiles/*.json
#    ("extends"로 상속). load_profile("ev3_3s") 처럼 쓴다.
#  - pybricks 목(sim/pybricks)이 SIM에 연결: SPIKE(PrimeHub·PUPDevice·
#    pupdevices.Motor), EV3(EV3Brick·ev3devices.Motor/ColorSensor/GyroSensor)
# =====================================================================
import codecs
import json
import collections
import math
import os
import sys

import numpy as np

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
PROFILES_DIR = os.path.join(PKG_DIR, "profiles")


def _utf8_stdio():
    # Windows: 출력이 파일·파이프로 가면 stdout이 cp1252/cp949 같은 코드페이지라 한글 print(실기 코드
    # 출력 포함)에서 UnicodeEncodeError가 난다 → UTF-8로 바꾼다. 이미 UTF-8이면(macOS·Linux·Windows
    # 콘솔) 그대로. PYTHONIOENCODING을 직접 정했으면 건드리지 않는다.
    if os.environ.get("PYTHONIOENCODING"):
        return
    for s in (sys.stdout, sys.stderr):
        try:
            if codecs.lookup(s.encoding).name != "utf-8":
                s.reconfigure(encoding="utf-8", errors=s.errors)
        except (AttributeError, LookupError, TypeError, ValueError, OSError):
            pass        # None(pythonw)·StringIO 등 바꿀 수 없는 스트림


_utf8_stdio()

# pybricks 목(mock) 모듈이 여기로 연결한다. run_sim.py가 Sim을 만들어 넣음.
SIM = None


class SimDone(Exception):
    def __init__(self, reason):
        self.reason = reason
        super().__init__(reason)


DEFAULT_PARAMS = {
    # --- 기하 (자로 실측해서 고칠 것) ---
    "wheel_diameter_mm": 56.0,
    "axle_track_mm": 112.0,
    "sensor_forward_mm": 60.0,     # 바퀴축 → 센서 앞거리 (sensor_pos_mm 없을 때)
    "sensor_spacing_mm": 32.0,     # 이웃 센서 중심 간격 (sensor_pos_mm 없을 때)
    "sensor_footprint_mm": 14.0,   # 센서가 보는 원 지름 (라인스캔으로 피팅)
    "line_width_mm": 19.0,
    # --- 모터 (MRC_spike_calibration.py + fit_params.py로 피팅) ---
    "dc_table": [[0, 0], [8, 0], [20, 190], [40, 450],
                 [60, 680], [80, 870], [100, 1010]],   # [duty%, 모터축 정상속도 deg/s]
    "motor_tau_s": 0.15,           # 1차지연 시정수 (가속 특성, 측정 기어비 기준)
    "gear_ratio": 1.0,             # 바퀴속도 = 모터속도 × 이 값 (기어업 상정용)
    "battery_scale": 1.0,          # 전압에 따른 속도 배율 (도메인 랜덤화용)
    # --- 센서 (실측 VALUE 그대로, sensor_ports 순서) ---
    "sensor_white": [64, 52, 53, 61],
    "sensor_black": [9, 7, 6, 8],
    "sensor_noise": 1.0,           # 반사값 가우시안 노이즈 표준편차
    "sensor_hz": 1331.0,           # 실측: 센서 갱신 속도
    "loop_hz": 5211.0,             # 실측: 제어 루프 속도 (센서 4개 읽기 기준)
    # --- 포트 매핑 (실기 배선: lm=B(CCW), rm=F, 센서 좌→우 A/C/E/D) ---
    "left_motor_port": "B",
    "right_motor_port": "F",
    "sensor_ports": ["A", "C", "E", "D"],   # 좌→우 순서 (c1..c4) — 개수 = 센서 수
    "left_mount_sign": -1,         # 좌측이 미러 장착 (dc+ = 물리 CCW = 전진)
    "right_mount_sign": 1,
    # --- 플랫폼 확장 (기본값 = 위 SPIKE 모델과 비트 단위로 동일) ---
    "platform": "spike",           # spike | ev3 (기록용)
    "pybricks_api": "v4",          # v2(ev3dev) | v3 | v4 — media.ev3dev 존재 여부 등
    # 모터 포트맵 {포트: [측 'L'|'R', 장착부호]}. 없으면 left/right_motor_port로 구성.
    # 장착부호: 물리 duty × 부호 = 전진 기준 duty. 한 측에 2개 = 4모터 스키드.
    "motors": None,
    # 호출 비용(µs) {color_read, gyro_read, motor_dc, motor_read}.
    # 없으면 color_read = 1/(loop_hz×센서수), 나머지 0.
    "call_cost_us": None,
    # 외장 자이로(EV3 GyroSensor) 모델. None이면 장착 안 됨.
    # {"source": "sensor", "port": "S4", "mount": 1(빨간점 위), "lag_ms", "noise_dps",
    #  "bias_dps", "max_dps", "update_hz"}
    "gyro": None,
    "w_eff_scale": 1.0,            # 유효 윤거 배율 (스키드 회전 미끄러짐)
    "mu": None,                    # 횡가속 한계 μ (None = 마찰 무한, 슬립 없음)
    "brake_tau_s": None,           # brake()/hold() 감속 시정수 (None = dc 0과 동일)
    "coast_tau_s": None,           # stop() 감속 시정수 (None = dc 0과 동일)
    "speed_est_tau_s": 0.0,        # Motor.speed() 추정 지연 (1차 필터 시정수)
    "battery_mv": 8000,            # hub.battery.voltage() 기준값 (× battery_scale)
}


def load_params(path=None):
    p = dict(DEFAULT_PARAMS)
    path = path or os.path.join(PKG_DIR, "robot_params.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            p.update(json.load(f))
    return p


# ------------------------- 하드웨어 프로파일 ------------------------- #

# spike_4s = 현행 실기(SPIKE 허브 + EV3 컬러센서 4개). fit_params.py가 갱신하는
# robot_params.json을 그대로 쓴다.
PROFILE_ALIASES = {"spike_4s": os.path.join(PKG_DIR, "robot_params.json")}


def _profile_path(name):
    return PROFILE_ALIASES.get(name) or os.path.join(PROFILES_DIR, name + ".json")


def _read_profile(name, depth=0):
    if depth > 8:
        raise ValueError("profile 'extends' loop: %s" % name)
    path = _profile_path(name)
    if not os.path.exists(path):
        raise FileNotFoundError("profile not found: %s (%s)" % (name, path))
    with open(path, encoding="utf-8") as f:      # 프로파일에 한글 주석 — Windows 기본 코드페이지로는 못 읽음
        d = json.load(f)
    parent = d.pop("extends", None)
    if parent:
        merged = _read_profile(parent, depth + 1)
        merged.update(d)
        return merged
    return d


def load_profile(name):
    """프로파일 이름 → 파라미터. "extends"는 얕은 병합(중첩 dict는 통째 교체)."""
    p = dict(DEFAULT_PARAMS)
    p.update(_read_profile(name))
    p["profile"] = name
    return p


def list_profiles():
    names = set(PROFILE_ALIASES)
    if os.path.isdir(PROFILES_DIR):
        names |= {f[:-5] for f in os.listdir(PROFILES_DIR) if f.endswith(".json")}
    return sorted(names)


# ------------------------- 트랙 ------------------------- #

def build_path(segments, step=2.0):
    """세그먼트 리스트 → 2mm 간격 폴리라인.
    ('S', 길이mm) 직진 / ('A', 반경mm, 각도deg) 곡선(+=좌회전)
    ('T', 각도deg) 제자리 방향전환 → 예각(뾰족한) 코너"""
    pts = [(0.0, 0.0)]
    x = y = h = 0.0
    for seg in segments:
        if seg[0] == 'T':
            h += math.radians(seg[1])
            continue
        if seg[0] == 'S':
            L = seg[1]
            n = max(1, int(L / step))
            for i in range(1, n + 1):
                pts.append((x + math.cos(h) * L * i / n,
                            y + math.sin(h) * L * i / n))
            x, y = pts[-1]
        else:
            r, adeg = seg[1], seg[2]
            a = math.radians(adeg)
            sgn = 1.0 if a > 0 else -1.0
            cx = x - math.sin(h) * r * sgn
            cy = y + math.cos(h) * r * sgn
            start = h - sgn * math.pi / 2
            n = max(2, int(abs(a) * r / step))
            for i in range(1, n + 1):
                th = start + a * i / n
                pts.append((cx + r * math.cos(th), cy + r * math.sin(th)))
            x, y = pts[-1]
            h += a
    return np.asarray(pts)


TRACKS = {
    # 직선-완만커브: 빠른 동작확인용
    "easy": [('S', 600), ('A', 200, 90), ('S', 400), ('A', 200, -90), ('S', 400)],
    # 직선 + 90도코너 + S자 시케인 + 코너: 기본 튜닝 코스
    "main": [('S', 500), ('A', 150, 90), ('S', 150), ('A', 150, 90), ('S', 80),
             ('A', 120, -50), ('A', 120, 100), ('A', 120, -50),
             ('S', 200), ('A', 130, 90), ('S', 150)],
    # 급커브(r80) 포함: 이탈복구/뱅킹 검증용
    "hard": [('S', 400), ('A', 80, 90), ('S', 100), ('A', 80, -90), ('S', 100),
             ('A', 80, -90), ('S', 100), ('A', 80, 90), ('S', 300)],
    # 실제 경기장 재현 (KakaoTalk_Photo_2026-07-04): 시계방향 1랩.
    # 상단직선(교차마커 2개) → 물방울 루프 → 우측면 → 지그재그(예각)
    # → S웨이브 → 좌하단 루프 → 8자 프레첼(교차 2회) → 좌변 복귀
    "mrc": {
        # 실기 피드백 반영: 25×25cm 로봇이 라인 사이를 지나다닐 만큼
        # 특징 간격이 넓음 → 전체 1.6배 스케일 (커브도 실제만큼 완만해짐)
        "segs": [
            ('S', 2400),                                   # 상단 직선 (마커 구간)
            ('T', -90), ('S', 320),                        # 우상단 직각 → 남하
            ('A', 240, -290), ('S', 320), ('A', 240, -70),   # 물방울 루프 (자기교차)
            ('S', 560),                                    # 우측면 남하
            ('A', 160, -90), ('S', 400),                   # 우하단 모서리 → 서진
            ('T', -60), ('S', 270), ('T', 120), ('S', 270),   # 지그재그 (예각 W)
            ('T', -120), ('S', 270), ('T', 120), ('S', 270),
            ('T', -60), ('S', 160),
            ('A', 175, 80), ('A', 175, -160), ('A', 175, 80),  # S웨이브
            ('S', 350),
            # 자기 교차 3곳: 사진의 '대각선 8자' 모양 유지 + 교차각은 직각
            # (루프 진입 직선 ⊥ 출구 방향이면 모양이 대각선이어도 교차는 90°)
            ('A', 200, 270), ('S', 120),                   # 좌하단 루프 (직각교차, 북쪽 탈출)
            ('A', 150, -45), ('S', 430),                   # 북동 대각선 진입
            ('A', 220, -270), ('S', 340),                  # 프레첼 위 로브 (직각교차, 북서 탈출)
            ('A', 220, 270), ('S', 60),                    # 프레첼 아래 로브 (직각교차, 북동 탈출)
            ('A', 260, 135), ('S', 912),                   # 크게 감아 서진 (개활지)
            ('T', -90), ('S', 570),                        # 좌변 북상
            ('A', 145, -90), ('S', 120),                   # 좌상단 → 시작점 '뒤'에서 합류
        ],
        # 교차 마커 (주행 라인이 아닌 장식선 — 센서에는 검정으로 보임)
        "extras": [
            [(720, 40), (720, 150)], [(720, -40), (720, -150)],   # 점선형 마커
            [(1520, 150), (1520, -150)],                          # 실선 마커
        ],
    },
}


# 실물 스케일 변형: mrc는 1.6배 확대판이므로 1/1.6로 되돌린 트랙 (커브가 빡빡)
def _scale_spec(spec, k):
    segs = []
    for s in spec["segs"]:
        if s[0] == 'T':
            segs.append(s)
        elif s[0] == 'S':
            segs.append(('S', s[1] * k))
        else:
            segs.append(('A', s[1] * k, s[2]))
    return {"segs": segs,
            "extras": [[(x * k, y * k) for x, y in e]
                       for e in spec.get("extras", [])]}


TRACKS["mrc_real"] = _scale_spec(TRACKS["mrc"], 1 / 1.6)
# 날카로운 직각만 있는 시험 트랙 (실기 '직각에서 직진' 재현용, 2026-10-17): 좌 90° → 우 90° → 우 90°
TRACKS["rightangle"] = [('S', 1000), ('T', 90), ('S', 800), ('T', -90), ('S', 800), ('T', -90), ('S', 800)]


def _densify(polyline, step=2.0):
    # 꼭짓점 목록 → step 간격 점 목록 (extras 스탬핑용)
    pts = [tuple(polyline[0])]
    for (x0, y0), (x1, y1) in zip(polyline, polyline[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / step))
        for i in range(1, n + 1):
            pts.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n))
    return pts


class Track:
    def __init__(self, path, line_width_mm=19.0, margin=150.0, mm_per_px=1.0,
                 extras=None, stamp_pts=None, stamp_path=True):
        """path: 진행도·이탈 판정용 중심선. extras: 장식선(폴리라인, 보간해 찍음).
        stamp_pts: 그대로 찍을 검정 점군(사진 트랙). stamp_path=False면 중심선은
        판정에만 쓰고 래스터에는 점군만 찍는다."""
        self.path = np.asarray(path, dtype=float)
        d = np.diff(self.path, axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        self.length = float(self.s[-1])

        parts = [self.path] + [np.asarray(e, dtype=float) for e in (extras or [])]
        if stamp_pts is not None:
            parts.append(np.asarray(stamp_pts, dtype=float))
        allpts = parts[0] if len(parts) == 1 else np.vstack(parts)
        self.res = mm_per_px
        self.x0 = float(allpts[:, 0].min() - margin)
        self.y0 = float(allpts[:, 1].min() - margin)
        W = int((allpts[:, 0].max() + margin - self.x0) / mm_per_px) + 1
        H = int((allpts[:, 1].max() + margin - self.y0) / mm_per_px) + 1
        self.img = np.zeros((H, W), np.float32)

        # 폴리라인을 따라 라인폭/2 반지름 원을 찍어 라인 래스터 생성
        r = line_width_mm / 2.0 / mm_per_px
        ri = int(math.ceil(r))
        yy, xx = np.mgrid[-ri:ri + 1, -ri:ri + 1]
        self._disk = ((xx * xx + yy * yy) <= r * r).astype(np.float32)
        self._ri = ri
        if stamp_path:
            self._stamp(self.path)
        for e in (extras or []):
            self._stamp(_densify(e))   # 교차 마커 등 장식선 (주행 경로 아님)
        if stamp_pts is not None:
            self._stamp(np.asarray(stamp_pts, dtype=float))

    def _stamp(self, pts):
        ri = self._ri
        for px, py in pts:
            cx = int((px - self.x0) / self.res)
            cy = int((py - self.y0) / self.res)
            self.img[cy - ri:cy + ri + 1, cx - ri:cx + ri + 1] = np.maximum(
                self.img[cy - ri:cy + ri + 1, cx - ri:cx + ri + 1], self._disk)

    def blackness(self, xs, ys):
        ix = np.clip(((xs - self.x0) / self.res).astype(int), 0, self.img.shape[1] - 1)
        iy = np.clip(((ys - self.y0) / self.res).astype(int), 0, self.img.shape[0] - 1)
        return self.img[iy, ix]


def _load_photo_track(line_width_mm, scale=1.0):
    """경기장 사진에서 추출한 실물 트랙 (photo_track.py → viz/track_pts.json).
    실측 195cm 매트 스케일, 코스 6.89m, 서쪽 출발. 마커 포함.
    scale: 트랙 확대 배율 (라인 폭은 그대로) — 이름 "photo@1.15" = 1.15배. 사진 추출 루프 반지름(최소 ≈76mm)이
    한국 규정 최소 곡률 반경 100mm보다 작아, 실제 경기장이 더 클 수 있는지 보는 용도"""
    with open(os.path.join(PKG_DIR, "viz", "track_pts.json"), encoding="utf-8") as f:
        d = json.load(f)
    c = d["coarse"]
    path = np.asarray(_densify(list(zip(c["px"], c["py"])))) * scale
    blk = np.asarray(d["black"], dtype=float).reshape(-1, 2) * scale
    return Track(path, line_width_mm, stamp_pts=blk, stamp_path=False)


TRACK_LOADERS = {"photo": _load_photo_track}


def track_names():
    return sorted(set(TRACKS) | set(TRACK_LOADERS))


def make_track(name="main", line_width_mm=19.0):
    if name.startswith("photo@"):
        return _load_photo_track(line_width_mm, float(name[6:]))
    if name in TRACK_LOADERS:
        return TRACK_LOADERS[name](line_width_mm)
    spec = TRACKS[name]
    if isinstance(spec, dict):
        return Track(build_path(spec["segs"]), line_width_mm,
                     extras=spec.get("extras"))
    return Track(build_path(spec), line_width_mm)


# ------------------------- 시뮬레이터 ------------------------- #

def _footprint_offsets(radius, n_ring=10):
    # 발자국(원) 평균용 샘플점: 중심 1 + 내륜 5 + 외륜 n_ring
    pts = [(0.0, 0.0)]
    for i in range(5):
        a = 2 * math.pi * i / 5
        pts.append((0.5 * radius * math.cos(a), 0.5 * radius * math.sin(a)))
    for i in range(n_ring):
        a = 2 * math.pi * i / n_ring
        pts.append((0.95 * radius * math.cos(a), 0.95 * radius * math.sin(a)))
    return np.asarray(pts)


class Sim:
    LOST_MM = 200.0     # 라인 중심에서 이만큼 벗어나면 이탈 판정 (뱅킹 복구 여지 포함)
    FINISH_MM = 40.0    # 트랙 끝에서 이 거리 안이면 완주
    STALL_CALLS = 200000  # 시간이 안 흐르는 목 호출이 이만큼 이어지면 무한루프로 보고 종료

    def __init__(self, params=None, track=None, start_lateral_mm=0.0,
                 time_limit_s=25.0, seed=0, start_heading_deg=0.0):
        self.p = params or load_params()
        self.track = track or make_track("main", self.p["line_width_mm"])
        self.rng = np.random.default_rng(seed)
        self.time_limit = time_limit_s

        # 시작 자세: 트랙 시작점, 진행방향 정렬 (+ 좌측으로 lateral 오프셋)
        p0, p1 = self.track.path[0], self.track.path[5]
        self.h = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        nx, ny = -math.sin(self.h), math.cos(self.h)
        self.x = float(p0[0] + nx * start_lateral_mm)
        self.y = float(p0[1] + ny * start_lateral_mm)
        if start_heading_deg:
            self.h += math.radians(start_heading_deg)

        self.t = 0.0
        self.wl = self.wr = 0.0     # 바퀴 속도 (deg/s, 전진+)
        self.ang_l = self.ang_r = 0.0   # 바퀴 누적각 (deg, 전진+) — 모터각 = 바퀴각/기어비
        self.events = []            # (t, 종류, 값) — 목의 LED 등 디버그 이벤트

        # ---- 모터 포트맵: {포트: (측, 장착부호)} ----
        self._motors = self._build_motor_map()
        self._side_ports = {'L': [], 'R': []}
        for port, (side, _) in self._motors.items():
            self._side_ports[side].append(port)
        if not self._side_ports['L'] or not self._side_ports['R']:
            raise ValueError("the profile needs both a left and a right drive motor: %s" % self._motors)
        self._mphys = {pt: 0.0 for pt in self._motors}   # 물리 duty (클램프됨)
        # ★구동 지연 (실기 블랙박스 2026-10-17: 조향 명령 → 요레이트 응답에 순수 지연 24~40ms)
        #   cmd_lag_ms > 0이면 모터 명령이 그만큼 늦게 적용됨. 기본 0 = 기존과 동일
        self._cmd_lag = float(self.p.get("cmd_lag_ms", 0.0) or 0.0) / 1000.0
        self._cmdq = collections.deque()
        # ★IMU(요레이트) 측정 지연: imu_lag_ms > 0이면 imu_rate_dps()가 그만큼 예전 값을 돌려줌. 기본 0 = 기존
        self._imu_lag = float(self.p.get("imu_lag_ms", 0.0) or 0.0) / 1000.0
        self._imuq = collections.deque()          # (t, 요레이트 deg/s)
        self._mmode = {pt: "dc" for pt in self._motors}  # dc | coast | brake | hold
        self._spd_est = {'L': 0.0, 'R': 0.0}             # Motor.speed() 추정 (바퀴 deg/s)
        self._noadv = 0

        # ---- 센서 ----
        ports = list(self.p["sensor_ports"])
        self._n = len(ports)
        n_reads = max(1, self._n)
        self.read_dt = 1.0 / (self.p["loop_hz"] * n_reads)
        self.sensor_dt = 1.0 / self.p["sensor_hz"]
        self._next_sensor = 0.0
        self._port_idx = {pt: i for i, pt in enumerate(ports)}
        for key in ("sensor_white", "sensor_black"):
            if len(self.p[key]) < self._n:
                raise ValueError("%s has %d entries but there are %d sensors" % (key, len(self.p[key]), self._n))
        sp = self.p["sensor_spacing_mm"]
        pos = self.p.get("sensor_pos_mm")   # 센서별 [전방mm, 측방mm(+좌)] — 분리배치용
        if pos:
            if len(pos) != self._n:
                raise ValueError("sensor_pos_mm has %d entries but there are %d sensors" % (len(pos), self._n))
            self._fwd = [float(q[0]) for q in pos]
            self._lat = [float(q[1]) for q in pos]
        else:
            self._fwd = [float(self.p["sensor_forward_mm"])] * self._n
            self._lat = [((self._n - 1) / 2 - i) * sp for i in range(self._n)]  # +y(좌)가 0번
        self._fp = _footprint_offsets(self.p["sensor_footprint_mm"] / 2.0)
        self._raw = [50] * self._n
        # ★센서 전송지연 (EV3 UART, 실기 흔들림 주범): 측정→read() 노출까지 지연
        self._lag = self.p.get("sensor_lag_ms", 0.0) / 1000.0
        self._lagq = []             # (노출시각, rawN) FIFO
        # ★비동기 센서 (실기 진단 2026-10-17: EV3 컬러센서는 각자 ≈1,070Hz로 따로 갱신)
        #   sensor_async=true면 센서마다 자기 주기(sensor_hz × (1±sensor_hz_jitter))·임의 위상으로
        #   따로 샘플한다. 기본(false)은 기존처럼 모든 센서가 한 프레임에 함께 갱신.
        self._async = bool(self.p.get("sensor_async", False))
        self._latest = None         # 가장 최근에 샘플한 센서값 벡터 (지연 큐에 들어가기 전)

        # ---- 호출 비용 (s) ----
        cc = self.p.get("call_cost_us") or {}
        self._cost = {
            "color_read": cc["color_read"] / 1e6 if "color_read" in cc else self.read_dt,
            "gyro_read": cc.get("gyro_read", 0.0) / 1e6,
            "motor_dc": cc.get("motor_dc", 0.0) / 1e6,
            "motor_read": cc.get("motor_read", 0.0) / 1e6,
        }

        # ---- 구동 확장 ----
        self._W = float(self.p["axle_track_mm"])
        # ★바퀴 흔들림 (2026-10-17, 실기 사진: 긴 차축 끝에 바퀴 → 휠 런아웃·유격):
        #   바퀴마다 유효 반지름이 바퀴 회전각에 따라 ±wheel_wobble 비율로 출렁임 → 한 바퀴 돌 때마다
        #   좌우 속도차 → 요 흔들림 → 앞 170mm 센서가 좌우로 흔들림. 별도 난수열이라 기존 월드 불변.
        wob = float(self.p.get("wheel_wobble", 0.0))
        wr_ = np.random.default_rng(int(self.p.get("wobble_seed", 0)) + 77)
        self._wob = [wob * wr_.uniform(0.5, 1.5), wob * wr_.uniform(0.5, 1.5)] if wob > 0 else None
        self._wob_ph = [wr_.uniform(0, 2 * math.pi), wr_.uniform(0, 2 * math.pi)]
        if self.p.get("w_eff_scale", 1.0) != 1.0:
            self._W *= self.p["w_eff_scale"]
        self._mu = self.p.get("mu")
        self._slip_omega = None     # 슬립 중일 때 몸체 각속도 (rad/s)
        # ★회전 부하 감속 turn_load (2026-10-24, 사용자 가설 "실기는 회전할 때 관성·부하로 느려진다"):
        #   좌우 바퀴 속도차가 클수록(급회전·제자리 회전) 두 바퀴가 함께 느려짐 → 전진·회전이 같은 비율로 줄어
        #   회전 반지름은 바퀴 속도 그대로. w_eff_scale(미끄러짐: 회전만 줄고 전진은 그대로 → 반지름이 커짐)의 대안.
        #   배율 = 1 − turn_load × min(1, |vR−vL| / (2 × 최고속도)). 0 = 끔
        self._tload = float(self.p.get("turn_load", 0.0))

        # ---- 외장 자이로 (EV3 GyroSensor) ----
        g = self.p.get("gyro") or {}
        self._gsrc = g.get("source", "none")
        self._gport = g.get("port")
        if self._gsrc == "sensor":
            self._gmount = g.get("mount", 1)
            self._glag = g.get("lag_ms", 0.0) / 1000.0
            self._gnoise = g.get("noise_dps", 0.0)
            self._gbias = g.get("bias_dps", 0.0)
            self._gmax = g.get("max_dps")
            self._gdt = 1.0 / g.get("update_hz", 1000.0)
            self._grng = np.random.default_rng([int(seed), 7919])  # 센서 노이즈 RNG와 분리
            self._gq = []           # (노출시각, 레이트int, 각도float)
            self._gval = (0, 0.0)
            self._gang = 0.0        # 센서 내부 적분각 (반시계+, 바이어스 포함)
            self._next_gyro = 0.0

        dc = np.asarray(self.p["dc_table"], dtype=float)
        self._dc_p, self._dc_v = dc[:, 0], dc[:, 1]
        self._v2max = 2.0 * self._target_fwd(100.0) * math.pi * self.p["wheel_diameter_mm"] / 360.0   # turn_load 기준

        self._pi = 0                # 트랙 진행 인덱스 (국소 최근접 탐색)
        self.max_progress_mm = 0.0  # 최대 진행 (역주행해도 줄지 않음 — 사망 지점 분석용)
        self._lat_now = 0.0
        self._lat_sum = 0.0
        self._lat_n = 0

        self.trace = []             # (t, x, y, lat, dutyL, dutyR, h, raw...)
        self._next_trace = 0.0
        self.result = None
        self._update_sensors()
        if self._async:
            arng = np.random.default_rng([int(seed), 104729])   # 센서 노이즈 RNG와 분리
            jit = float(self.p.get("sensor_hz_jitter", 0.02))
            self._dt_s = [1.0 / (self.p["sensor_hz"] * (1.0 + arng.uniform(-jit, jit)))
                          for _ in range(self._n)]
            self._next_s = [arng.uniform(0.0, d) for d in self._dt_s]

    # ---- 모터 ----
    def _build_motor_map(self):
        m = self.p.get("motors")
        if m:
            return {str(pt): (v[0], int(v[1])) for pt, v in m.items()}
        return {self.p["left_motor_port"]: ('L', self.p["left_mount_sign"]),
                self.p["right_motor_port"]: ('R', self.p["right_mount_sign"])}

    def _side_fwd(self, side):
        # 측의 전진 기준 duty = 각 모터 (물리 duty × 장착부호)의 평균
        ports = self._side_ports[side]
        if len(ports) == 1:
            pt = ports[0]
            return self._mphys[pt] * self._motors[pt][1]
        return sum(self._mphys[pt] * self._motors[pt][1] for pt in ports) / len(ports)

    def _side_mode(self, side):
        modes = {self._mmode[pt] for pt in self._side_ports[side]}
        return modes.pop() if len(modes) == 1 else "dc"

    def _target_fwd(self, d):
        # 전진 기준 duty → 목표 바퀴속도 (deg/s)
        v = float(np.interp(abs(d), self._dc_p, self._dc_v))
        v *= self.p["battery_scale"] * self.p.get("gear_ratio", 1.0)
        return math.copysign(v, d) if d else 0.0

    def motor_check(self, port):
        if port not in self._motors:
            raise ValueError("no motor on port %s in this profile (motors: %s)"
                             % (port, sorted(self._motors)))

    def _set_motor(self, port, duty, mode):
        if self._cmd_lag > 0:
            self._cmdq.append((self.t + self._cmd_lag, port, duty, mode))
        else:
            self._mphys[port] = duty
            self._mmode[port] = mode

    def motor_dc(self, port, physical_duty):
        self._set_motor(port, max(-100.0, min(100.0, float(physical_duty))), "dc")
        self.charge("motor_dc")

    def motor_stop(self, port, mode):
        self._set_motor(port, 0.0, mode)
        self.charge("motor_dc")

    def motor_run(self, port, physical_speed):
        """run(speed) 근사: 목표 모터속도를 내는 개루프 duty로 환산 (폐루프 아님)."""
        v = abs(physical_speed) / max(1e-9, self.p["battery_scale"])
        xs, ps = [0.0], [0.0]
        for d, s in zip(self._dc_p, self._dc_v):    # 단조증가 구간만으로 역함수
            if s > xs[-1]:
                xs.append(float(s))
                ps.append(float(d))
            elif s == 0.0:
                ps[0] = float(d)                   # 데드존 끝 duty
        duty = float(np.interp(v, xs, ps)) if v > 0 else 0.0
        self.motor_dc(port, math.copysign(duty, physical_speed))

    def motor_angle_phys(self, port):
        # 물리 모터축 누적각 (모터 자기 기준 시계+). 바퀴각/기어비 × 장착부호.
        self.charge("motor_read")
        side, mount = self._motors[port]
        ang = self.ang_l if side == 'L' else self.ang_r
        return ang / self.p.get("gear_ratio", 1.0) * mount

    def motor_speed_phys(self, port):
        self.charge("motor_read")
        side, mount = self._motors[port]
        if self.p.get("speed_est_tau_s", 0.0) > 0:
            w = self._spd_est[side]
        else:
            w = self.wl if side == 'L' else self.wr
        return w / self.p.get("gear_ratio", 1.0) * mount

    # 하위호환: 예전 목(측 단위 물리 duty)
    def set_duty(self, role, physical_duty):
        for pt in self._side_ports[role]:
            self._mphys[pt] = max(-100.0, min(100.0, float(physical_duty)))
            self._mmode[pt] = "dc"

    def set_forward_duties(self, left, right):
        # 컨트롤러용: '전진+' 기준 duty → 각 모터의 물리 duty로 변환
        for side, val in (('L', left), ('R', right)):
            for pt in self._side_ports[side]:
                self._set_motor(pt, max(-100.0, min(100.0, float(val * self._motors[pt][1]))), "dc")

    # ---- 호출 비용 / 이벤트 ----
    def charge(self, kind):
        dt = self._cost[kind]
        if dt > 0:
            self.advance(dt)
        else:
            self._noadv += 1
            if self._noadv > self.STALL_CALLS:
                self._finish("stall")

    def poll(self):
        # 시간이 안 흐르는 목 호출(버튼·IMU·배터리·StopWatch): 이것만 도는 무한 반복도 STALL_CALLS 뒤 stall로 끝낸다
        self._noadv += 1
        if self._noadv > self.STALL_CALLS:
            self._finish("stall")

    def log_event(self, kind, value):
        self.events.append((round(self.t, 4), kind, value))

    # ---- 자이로 ----
    def imu_rate_dps(self):
        """허브 내장 IMU 요레이트 (deg/s, 좌회전+). imu_lag_ms > 0이면 그만큼 지난 값."""
        if self._imu_lag > 0:
            tq = self.t - self._imu_lag
            while len(self._imuq) > 1 and self._imuq[1][0] <= tq:
                self._imuq.popleft()
            if self._imuq and self._imuq[0][0] <= tq:
                return self._imuq[0][1]
            return 0.0
        return self._imu_rate_now()

    def _imu_rate_now(self):
        if self._slip_omega is not None:
            return math.degrees(self._slip_omega)
        if self._wob is not None:
            return math.degrees(getattr(self, "_w_true", 0.0))
        kw = math.pi * self.p["wheel_diameter_mm"] / 360.0
        return math.degrees((self.wr - self.wl) * kw / self._W)

    def gyro_check(self, port):
        if self._gsrc != "sensor" or port != self._gport:
            raise OSError("simulator: no gyro sensor on port %s (profile gyro.port=%s)"
                          % (port, self._gport))

    def gyro_read(self):
        """외장 자이로 (레이트 int deg/s, 각도 deg) — 둘 다 반시계+ × 장착부호."""
        self.charge("gyro_read")
        while self._gq and self._gq[0][0] <= self.t:
            self._gval = self._gq.pop(0)[1:]
        rate, ang = self._gval
        return rate * self._gmount, ang * self._gmount

    def _gyro_step(self, dt):
        true = self.imu_rate_dps()
        self._gang += (true + self._gbias) * dt
        if self.t >= self._next_gyro:
            self._next_gyro += self._gdt
            v = true + self._gbias
            if self._gnoise:
                v += float(self._grng.normal(0.0, self._gnoise))
            if self._gmax is not None:
                v = max(-self._gmax, min(self._gmax, v))
            sample = (int(v), self._gang)
            if self._glag > 0:
                self._gq.append((self.t + self._glag,) + sample)
            else:
                self._gval = sample

    # ---- 시간 전진 ----
    def sensor_read(self, port):
        self.advance(self._cost["color_read"])
        while self._lagq and self._lagq[0][0] <= self.t:   # 지연 만료분 노출
            self._raw = self._lagq.pop(0)[1]
        return self._raw[self._port_idx[port]]

    def color_check(self, port):
        if port not in self._port_idx:
            raise OSError("simulator: no color sensor on port %s (profile sensor_ports=%s)"
                          % (port, list(self._port_idx)))

    def wait_ms(self, ms):
        end = self.t + ms / 1000.0
        while self.t < end:
            self.advance(min(self.read_dt, end - self.t))

    def advance(self, dt):
        self._noadv = 0
        while self._cmdq and self._cmdq[0][0] <= self.t:      # 지연 만료된 모터 명령 적용
            _, pt, d, m = self._cmdq.popleft()
            self._mphys[pt] = d
            self._mmode[pt] = m
        # ★비대칭 1차지연: 가속 tau_up(실측 115ms), 감속 tau_dn(역기전력 제동
        #   ~35ms, 캘리브레이션 확정) — 목표속도 크기가 현재보다 작아지는(감속)
        #   바퀴는 훨씬 빨리 수렴. 실기 "브레이크가 시뮬보다 강함" 반영.
        tau = self.p["motor_tau_s"]
        tau_dn = self.p.get("motor_tau_dn_s", tau)
        a_up = math.exp(-dt / tau)
        a_dn = math.exp(-dt / tau_dn)
        tl = self._target_fwd(self._side_fwd('L'))
        tr = self._target_fwd(self._side_fwd('R'))
        al = a_dn if abs(tl) < abs(self.wl) else a_up
        ar = a_dn if abs(tr) < abs(self.wr) else a_up
        if self.p.get("brake_tau_s") or self.p.get("coast_tau_s"):
            al = self._stop_alpha('L', dt, al)
            ar = self._stop_alpha('R', dt, ar)
        ytau = self.p.get("yaw_tau_s", 0.0)
        if ytau > 0:
            # ★회전 성분 시정수 (2026-10-24 실기 블랙박스 5개 맞춤: 조향→요레이트 τ ≈0.05s, 지연 ≈36ms, w_eff ≈1.5
            #   — τ 0.1이면 오차 84°/s, 0.05면 71°/s). 바닥 마찰이 회전에만 브레이크로 걸려 좌우 차(회전)는
            #   모터 시정수보다 빨리 붙고 빨리 멈춤. 전진 성분은 아래 바퀴별 계산 그대로. 0 = 끔
            d1 = (tr - tl) / 2.0 + ((self.wr - self.wl) - (tr - tl)) / 2.0 * math.exp(-dt / ytau)
        self.wl = tl + (self.wl - tl) * al
        self.wr = tr + (self.wr - tr) * ar
        if ytau > 0:
            c = (self.wl + self.wr) / 2.0
            self.wl, self.wr = c - d1, c + d1

        k = math.pi * self.p["wheel_diameter_mm"] / 360.0   # deg/s → mm/s
        vl, vr = self.wl * k, self.wr * k
        if self._wob is not None:
            vl *= 1.0 + self._wob[0] * math.sin(math.radians(self.ang_l) + self._wob_ph[0])
            vr *= 1.0 + self._wob[1] * math.sin(math.radians(self.ang_r) + self._wob_ph[1])
        v = (vl + vr) / 2.0
        w = (vr - vl) / self._W
        if self._tload > 0:
            kld = 1.0 - self._tload * min(1.0, abs(vr - vl) / self._v2max)
            v *= kld
            w *= kld
        if self._mu is not None:
            w = self._slip_limit(v, w)
        self._w_true = w                                     # 자이로가 보는 실제 요레이트 (바퀴 흔들림 포함)
        self.x += v * math.cos(self.h + w * dt / 2.0) * dt
        self.y += v * math.sin(self.h + w * dt / 2.0) * dt
        self.h += w * dt
        self.ang_l += self.wl * dt
        self.ang_r += self.wr * dt
        self.t += dt

        est_tau = self.p.get("speed_est_tau_s", 0.0)
        if est_tau > 0:
            a = math.exp(-dt / est_tau)
            self._spd_est['L'] = self.wl + (self._spd_est['L'] - self.wl) * a
            self._spd_est['R'] = self.wr + (self._spd_est['R'] - self.wr) * a
        if self._gsrc == "sensor":
            self._gyro_step(dt)
        if self._imu_lag > 0:
            self._imuq.append((self.t, self._imu_rate_now()))

        if self._async:
            for i in range(self._n):
                if self.t >= self._next_s[i]:
                    self._next_s[i] += self._dt_s[i]
                    self._sample_one(i)
        if self.t >= self._next_sensor:
            self._next_sensor += self.sensor_dt
            self._update_sensors()
        if self.t >= self._next_trace:
            self._next_trace += 0.005
            # duty는 '전진+' 기준으로 기록. 뒤에 heading·센서생값도 기록(시각화용)
            self.trace.append((self.t, self.x, self.y, self._lat_now,
                               self._side_fwd('L'), self._side_fwd('R'),
                               self.h, *self._raw))
        if self.t > self.time_limit:
            self._finish("timeout")

    def _stop_alpha(self, side, dt, default):
        mode = self._side_mode(side)
        tau = None
        if mode in ("brake", "hold"):
            tau = self.p.get("brake_tau_s")
        elif mode == "coast":
            tau = self.p.get("coast_tau_s")
        return math.exp(-dt / tau) if tau else default

    def _slip_limit(self, v, w):
        # 횡가속 |v·ω| ≤ μg — 넘으면 몸체 회전이 바퀴 차이만큼 못 따라감(언더스티어)
        amax = self._mu * 9810.0
        if v != 0.0 and abs(v * w) > amax:
            w = math.copysign(amax / abs(v), w)
            self._slip_omega = w
        else:
            self._slip_omega = None
        return w

    # ---- 센서/판정 ----
    def _update_sensors(self):
        # 공통 프레임마다: (동기 모드) 센서 전부 샘플 + 진행도·이탈 판정. 비동기면 판정만.
        if not self._async or self._latest is None:
            self._sample_all()
        self._check_progress()

    def _sample_one(self, i):
        ch, sh = math.cos(self.h), math.sin(self.h)
        f, lat = self._fwd[i], self._lat[i]
        sx = self.x + ch * f - sh * lat
        sy = self.y + sh * f + ch * lat
        b = float(self.track.blackness(sx + self._fp[:, 0], sy + self._fp[:, 1]).mean())
        w, bl = self.p["sensor_white"][i], self.p["sensor_black"][i]
        fresh = list(self._latest)
        fresh[i] = int(np.clip(w - (w - bl) * b + self.rng.normal(0.0, self.p["sensor_noise"]), 0, 100))
        self._latest = fresh
        if self._lag > 0:
            self._lagq.append((self.t + self._lag, fresh))
        else:
            self._raw = fresh

    def _sample_all(self):
        ch, sh = math.cos(self.h), math.sin(self.h)
        noise = self.rng.normal(0.0, self.p["sensor_noise"], self._n)
        fresh = [0] * self._n
        for i, lat in enumerate(self._lat):
            f = self._fwd[i]
            sx = self.x + ch * f - sh * lat
            sy = self.y + sh * f + ch * lat
            b = float(self.track.blackness(sx + self._fp[:, 0],
                                           sy + self._fp[:, 1]).mean())
            w, bl = self.p["sensor_white"][i], self.p["sensor_black"][i]
            fresh[i] = int(np.clip(w - (w - bl) * b + noise[i], 0, 100))
        self._latest = fresh
        if self._lag > 0:
            self._lagq.append((self.t + self._lag, fresh))
        else:
            self._raw = fresh

    def _check_progress(self):
        # 진행도/이탈 판정 (국소 최근접점 탐색)
        lo = max(0, self._pi - 25)
        hi = 60 if self.p.get("track_ref_mm", 0.0) > 0 else 150   # 센서 기준이면 앞 창을 좁힘 (8자 교차에서 다른 선으로 튀지 않게, GPU와 같음)
        win = self.track.path[lo:self._pi + hi]
        # 판정 기준점: track_ref_mm 앞 (센서가 바퀴축보다 멀리 앞에 있으면 커브에서 바퀴축은 원래 안쪽으로 질러감 →
        #   바퀴축 기준이면 센서가 라인 위인데도 이탈로 오판. 실측 치수 프로파일은 센서 위치 170mm)
        rf = self.p.get("track_ref_mm", 0.0)
        rx, ry = self.x + rf * math.cos(self.h), self.y + rf * math.sin(self.h)
        d2 = (win[:, 0] - rx) ** 2 + (win[:, 1] - ry) ** 2
        j = int(np.argmin(d2))
        self._pi = lo + j
        s_now = float(self.track.s[self._pi])
        if s_now > self.max_progress_mm:
            self.max_progress_mm = s_now
        self._lat_now = math.sqrt(float(d2[j]))
        self._lat_sum += self._lat_now
        self._lat_n += 1

        if self._lat_now > self.LOST_MM:
            self._finish("lost")
        if self.track.s[self._pi] > self.track.length - self.FINISH_MM:
            self._finish("finish")

    def _finish(self, reason):
        # 첫 결과를 유지: 프로그램의 finally(정지 시 출력·wait)가 시간을 더 흘려도 덮어쓰지 않음
        if self.result is not None:
            raise SimDone(self.result["reason"])
        self.result = {
            "reason": reason,
            "time_s": round(self.t, 3),
            "progress_mm": round(float(self.track.s[self._pi]), 1),
            "track_mm": round(self.track.length, 1),
            "mean_lat_mm": round(self._lat_sum / max(1, self._lat_n), 2),
        }
        raise SimDone(reason)


# ------------------------- 컨트롤러 실행기 (스윕용) ------------------------- #

def steering_dc_duties(steering, power):
    # MRC_line_following.py의 steering_dc()와 동일 (lm=left, rm=right)
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        return round(slope * steering + power, 2), power
    return power, round(-slope * steering + power, 2)


def run_controller(sim, ctrl_factory, calib_reads=10):
    """실코드와 같은 순서로 실행: 정지상태 error_offset 보정 → 제어 루프.
    ctrl_factory(error_offset) → step(r1..rN)이 (steering, power)를 반환하는 객체.
    오프셋 보정: 컨트롤러가 calib_value(r)를 가지면 그걸 쓰고, 없으면 4센서
    기존 식((1+2)-(3+4), norm4 보유 시 정규화 스케일). 그 외 센서 수는 0."""
    ports = sim.p["sensor_ports"]
    try:
        probe = ctrl_factory(0)
        total = 0
        for _ in range(calib_reads):
            r = [sim.sensor_read(pt) for pt in ports]
            if hasattr(probe, "calib_value"):
                total += probe.calib_value(r)
            elif hasattr(probe, "norm4"):
                # 생값으로 재면 센서 비대칭 +30가량이 바이어스로 들어가는 버그 → 정규화
                n = probe.norm4(r)
                total += (n[0] + n[1]) - (n[2] + n[3])
            elif len(r) == 4:
                total += (r[0] + r[1]) - (r[2] + r[3])
            sim.wait_ms(50)
        ctrl = ctrl_factory(round(total / calib_reads))

        r = [sim.sensor_read(pt) for pt in ports]
        if hasattr(ctrl, "prime"):
            ctrl.prime(*r)          # 실코드의 루프 진입 전 ps1..ps4 초기 읽기 재현

        k = math.pi * sim.p["wheel_diameter_mm"] / 360.0
        while True:
            r = [sim.sensor_read(pt) for pt in ports]
            if hasattr(ctrl, "on_gyro"):
                # 실기의 hub.imu.angular_velocity(Axis.Z)에 해당 (deg/s, 좌회전 +)
                ctrl.on_gyro(sim.imu_rate_dps())
            if hasattr(ctrl, "on_distance"):
                # 주행거리 mm. ang_l/ang_r는 이미 '바퀴' 각이라 기어비를 곱하지 않는다
                # (2026-10-17 수정: 예전엔 기어비를 한 번 더 곱해 거리가 기어배로 부풀었음)
                ctrl.on_distance((abs(sim.ang_l) + abs(sim.ang_r)) / 2.0 * k)
            steering, power = ctrl.step(*r)
            l, rr = steering_dc_duties(steering, power)
            sim.set_forward_duties(l, rr)
    except SimDone:
        return sim.result
