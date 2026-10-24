# =====================================================================
#  실기 컨트롤러들의 1루프=1스텝 이식본 (스윕 전용 사본).
#  - V1Controller: MRC_line_following.py의 pid_control()
#  - V5Controller: MRC_line_following_v5.py (정규화 + 프레임동기 + 위치기반 이탈)
#  실기 파일 자체는 run_sim.py가 수정 없이 그대로 실행한다.
#  원본 로직을 바꾸면 여기도 같이 맞출 것.
# =====================================================================


class V1Controller:
    def __init__(self, power, dec_v, st, kp, ki, kd, value, w_offset,
                 error_offset=0):
        S = 100000
        self.S = S
        self.KP = round(kp * S)
        self.KI = round(ki * S)
        self.KD = round(kd * S)

        self.ALPHA, self.A_COMP, self.E_S = 700, 300, 1000

        self.wh = [value[0], value[2], value[4], value[6]]
        self.b = [value[1], value[3], value[5], value[7]]

        self.integral = 0
        self.max_error = sum(self.wh[i] - self.b[i] for i in range(4)) // 8
        self.last_error = 0

        self.power = power
        self.dec_v = dec_v
        self.st = st
        self.repower = power
        self.direction = 0
        self.on_count = 0
        self.ON_CONFIRM = 2
        self.line_off = False

        self.prev_steering = 0
        self.MAX_STEER_DELTA = 15

        self.l_black = (self.wh[0] + self.b[0]) // 2
        self.r_black = (self.wh[3] + self.b[3]) // 2
        self.cross = sum(self.b) // 4 + 20
        self.w = [self.wh[i] - w_offset for i in range(4)]

        self.error_offset = error_offset
        self.ps = None

    def prime(self, r1, r2, r3, r4):
        # 원본의 루프 진입 전 ps1..ps4 초기 읽기
        self.ps = [r1, r2, r3, r4]

    def step(self, r1, r2, r3, r4):
        if self.ps is None:
            self.ps = [r1, r2, r3, r4]
        raw = [r1, r2, r3, r4]

        s = [0] * 4
        for i in range(4):
            s[i] = (self.ALPHA * min(max(raw[i], self.b[i]), self.wh[i])
                    + self.A_COMP * self.ps[i]) // self.E_S
        self.ps = s

        raw_off = all(s[i] > self.w[i] for i in range(4))

        if raw_off:
            self.on_count = 0
            self.line_off = True
        else:
            if self.line_off:
                self.on_count += 1
                if self.on_count >= self.ON_CONFIRM:
                    self.line_off = False
                    self.on_count = 0
                    self.integral = 0
                    self.last_error = (s[0] + s[1]) - (s[2] + s[3]) - self.error_offset

        error = (s[0] + s[1]) - (s[2] + s[3]) - self.error_offset

        if not self.line_off:
            self.integral += error
            if self.KI != 0:
                max_integral = (self.S * 100 - abs(self.max_error * self.KP)) // self.KI
                self.integral = max(-max_integral, min(max_integral, self.integral))

        diff = error - self.last_error
        steering = (self.KP * error + self.KI * self.integral + self.KD * diff) // self.S
        self.last_error = error

        if s[0] < self.l_black:
            self.direction = 0
        elif s[3] < self.r_black:
            self.direction = 1

        if self.line_off:
            self.repower = max(50, self.repower - self.dec_v)
            if self.on_count == 0:
                steering = -self.st if self.direction == 0 else self.st
        else:
            self.repower = self.power
            if sum(s) < self.cross:
                steering = 0

        delta = steering - self.prev_steering
        if delta > self.MAX_STEER_DELTA:
            steering = self.prev_steering + self.MAX_STEER_DELTA
        elif delta < -self.MAX_STEER_DELTA:
            steering = self.prev_steering - self.MAX_STEER_DELTA
        self.prev_steering = steering

        return steering, self.repower


class V5Controller:
    """MRC_line_following_v5.py의 pid_control() 이식본 (정규화 스케일)."""

    def __init__(self, value, error_offset=0, power=100, recover_power=70,
                 kp=0.08, ki=0.00005, kd=1.0, i_limit=15,
                 st_bank=75, max_steer_delta=15, ema_alpha=700, d_filter=300,
                 gap_limit=10, edge_err=80, w_norm=82, black_mid=50,
                 cross_sum=60, edge_bias=0):
        # edge_bias: 추종 타깃을 라인 '가장자리'로 옮기는 오프셋(정규화 스케일).
        # 라인폭 < 센서간격이라 정중앙은 무신호 데드밴드(불안정) →
        # 가장자리 추종이 연속 기울기를 줘 고속에서 훨씬 안정적.
        S = 100000
        self.S = S
        self.KP_S = round(kp * S)
        self.KI_S = round(ki * S)
        self.KD_S = round(kd * S)
        self.E_S = 1000
        self.ALPHA = ema_alpha
        self.A_COMP = self.E_S - ema_alpha
        self.D_FILTER = d_filter
        self.D_COMP = self.E_S - d_filter

        self.WH = [value[0], value[2], value[4], value[6]]
        self.BK = [value[1], value[3], value[5], value[7]]
        self.RG = [self.WH[i] - self.BK[i] for i in range(4)]

        self.error_offset = error_offset + edge_bias
        self.POWER = power
        self.RECOVER_POWER = recover_power
        self.I_LIMIT = i_limit
        self.ST_BANK = st_bank
        self.MAX_STEER_DELTA = max_steer_delta
        self.GAP_LIMIT = gap_limit
        self.EDGE_ERR = edge_err
        self.W_NORM = w_norm
        self.BLACK_MID = black_mid
        self.CROSS_SUM = cross_sum

        self.integral = 0
        self.last_error = 0
        self.d_filt = 0
        self.repower = power
        self.direction = 0
        self.was_off = False
        self.prev_steering = 0
        self.gap = 0
        self.last_good_steer = 0
        self.raw_prev = None
        self.ps = None
        self._out = (0, power)

    def norm4(self, r):
        return [(min(max(r[i], self.BK[i]), self.WH[i]) - self.BK[i]) * 100
                // self.RG[i] for i in range(4)]

    def prime(self, r1, r2, r3, r4):
        self.raw_prev = (r1, r2, r3, r4)
        self.ps = self.norm4((r1, r2, r3, r4))

    def step(self, r1, r2, r3, r4):
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out          # 프레임 동기: 중복 프레임 → 모터 유지
        self.raw_prev = raw

        n = self.norm4(raw)
        s = [0] * 4
        for i in range(4):
            s[i] = (self.ALPHA * n[i] + self.A_COMP * self.ps[i]) // self.E_S
        self.ps = s

        raw_off = all(v > self.W_NORM for v in n)
        error = (s[0] + s[1]) - (s[2] + s[3]) - self.error_offset

        if not raw_off:
            if self.was_off:
                self.last_error = error
                self.integral = 0
                self.d_filt = 0
            self.was_off = False
            self.gap = 0

            if s[0] < self.BLACK_MID:
                self.direction = 0
            elif s[3] < self.BLACK_MID:
                self.direction = 1

            self.integral += error
            if self.KI_S != 0:
                mi = self.I_LIMIT * self.S // self.KI_S
                self.integral = max(-mi, min(mi, self.integral))

            diff = error - self.last_error
            self.d_filt = (self.D_FILTER * diff + self.D_COMP * self.d_filt) // self.E_S
            steering = (self.KP_S * error + self.KI_S * self.integral
                        + self.KD_S * self.d_filt) // self.S
            self.last_error = error

            if sum(s) < self.CROSS_SUM:
                steering = 0
            self.repower = self.POWER
            self.last_good_steer = steering
        else:
            self.gap += 1
            if self.gap <= self.GAP_LIMIT:
                steering = self.last_good_steer
                self.repower = self.POWER
            elif abs(self.last_error) < self.EDGE_ERR:
                steering = self.last_good_steer
                self.repower = self.POWER
            else:
                self.was_off = True
                self.integral = 0
                steering = -self.ST_BANK if self.direction == 0 else self.ST_BANK
                self.repower = self.RECOVER_POWER

        delta = steering - self.prev_steering
        if delta > self.MAX_STEER_DELTA:
            steering = self.prev_steering + self.MAX_STEER_DELTA
        elif delta < -self.MAX_STEER_DELTA:
            steering = self.prev_steering - self.MAX_STEER_DELTA
        self.prev_steering = steering

        self._out = (steering, self.repower)
        return self._out


class V6Controller(V5Controller):
    """v5 + ★코너 피벗 (시간순서 기반 코너/교차 구별)
    1) 바깥 센서(n1/n4) 하나만 진한 검정 → '그쪽에 급코너 가능성' 기억
       (양쪽 동시 검정 = 교차마커 → 기억 안 함)
    2) 그 직후 전체 흰색(라인 소실) → 코너 확정 → 저파워 피벗
       (교차선이었다면 중앙 라인이 유지되므로 기억만 만료 → 오발동 없음)
    3) 중앙 센서가 라인 재포착 → PID 리셋 후 복귀 (타임아웃 시 포기)"""

    def __init__(self, value, corner_black=25, mem_frames=100,
                 turn_st=95, turn_power=45, reacquire=50,
                 turn_timeout=2000, reacq_confirm=1,
                 settle_frames=350, settle_power=55, white_confirm=10, **kw):
        super().__init__(value, **kw)
        self.CORNER_BLACK = corner_black
        self.MEM_FRAMES = mem_frames      # 진검정 기억 유지 프레임수
        self.TURN_ST = turn_st
        self.TURN_POWER = turn_power
        self.REACQUIRE = reacquire
        self.TURN_TIMEOUT = turn_timeout  # 피벗 포기 프레임수
        self.REACQ_CONFIRM = reacq_confirm    # 재포착 확인 연속 프레임수
        self.SETTLE_FRAMES = settle_frames    # 복귀 후 감속 정착 프레임수
        self.SETTLE_POWER = settle_power      # 정착 중 파워
        self.turning = False
        self.turn_dir = 0                 # 0=좌, 1=우
        self.mem_dir = None
        self.mem_age = 0
        self.turn_frames = 0
        self.WHITE_CONFIRM = white_confirm    # 피벗 진입 전 전체흰색 연속 확인
        self.reacq_cnt = 0
        self.settle = 0
        self.white_cnt = 0

    def step(self, r1, r2, r3, r4):
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out          # 프레임 동기
        n = self.norm4(raw)

        if self.turning:
            self.raw_prev = raw
            self.ps = n[:]            # EMA 리프라임 (복귀 시 미분 튐 방지)
            self.turn_frames += 1
            if n[1] < self.REACQUIRE or n[2] < self.REACQUIRE:
                self.reacq_cnt += 1   # ★연속 프레임 확인 (스치는 재포착 방지)
            else:
                self.reacq_cnt = 0
            if self.reacq_cnt >= self.REACQ_CONFIRM:
                # 중앙 재포착 확정 → PID 리셋 + 감속 정착 모드로 복귀
                self.turning = False
                self.integral = 0
                self.d_filt = 0
                self.last_error = 0
                self.was_off = False
                self.gap = 0
                self.prev_steering = 0
                self.last_good_steer = 0
                self.mem_dir = None
                self.reacq_cnt = 0
                self.settle = self.SETTLE_FRAMES
                self._out = (0, self.SETTLE_POWER)
                return self._out
            if self.turn_frames > self.TURN_TIMEOUT:
                self.turning = False   # 포기 → 기존 뱅킹 로직에 맡김
                self.mem_dir = None
                return super().step(r1, r2, r3, r4)
            st = -self.TURN_ST if self.turn_dir == 0 else self.TURN_ST
            self.prev_steering = st
            self._out = (st, self.TURN_POWER)
            return self._out

        # ── NORMAL: 진검정 기억 갱신 ── #
        left_deep = n[0] < self.CORNER_BLACK and n[3] > 60
        right_deep = n[3] < self.CORNER_BLACK and n[0] > 60
        if left_deep:
            self.mem_dir, self.mem_age = 0, 0
        elif right_deep:
            self.mem_dir, self.mem_age = 1, 0
        elif self.mem_dir is not None:
            self.mem_age += 1
            if self.mem_age > self.MEM_FRAMES:
                self.mem_dir = None

        # 라인 소실 '지속' + 기억 신선 → 코너 확정, 피벗 진입
        # (센서 틈으로 인한 짧은 전체흰색 깜빡임은 걸러냄)
        if all(v > self.W_NORM for v in n) and self.mem_dir is not None:
            self.white_cnt += 1
        else:
            self.white_cnt = 0
        if self.white_cnt >= self.WHITE_CONFIRM:
            self.white_cnt = 0
            self.turning = True
            self.turn_dir = self.mem_dir
            self.turn_frames = 0
            self.raw_prev = raw
            self.ps = n[:]
            st = -self.TURN_ST if self.turn_dir == 0 else self.TURN_ST
            self.prev_steering = st
            self._out = (st, self.TURN_POWER)
            return self._out

        return super().step(r1, r2, r3, r4)


class V7Controller(V6Controller):
    """v6 + ★기어업(고속) 대응:
    - 속도 스케줄링: repower = POWER - spd_k×|조향| (직선=풀듀티, 흔들리면 자동감속)
    - 코너 확정 시 역토크 브레이크(BRAKE_FRAMES, BRAKE_DUTY) 후 피벗
    - 뱅킹/정착 파워도 스케줄 상한 적용"""

    def __init__(self, value, spd_k=0.8, min_power=35,
                 brake_frames=40, brake_duty=-40, **kw):
        super().__init__(value, **kw)
        self.SPD_K = spd_k
        self.MIN_POWER = min_power
        self.BRAKE_FRAMES = brake_frames
        self.BRAKE_DUTY = brake_duty
        self.brake_cnt = 0

    def step(self, r1, r2, r3, r4):
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out
        n = self.norm4(raw)

        # ── 브레이크 단계 (코너 확정 직후 감속) ── #
        if self.brake_cnt > 0:
            self.raw_prev = raw
            self.ps = n[:]
            self.brake_cnt -= 1
            if self.brake_cnt == 0:
                self.turning = True     # 감속 끝 → 피벗 시작
                self.turn_frames = 0
                st = -self.TURN_ST if self.turn_dir == 0 else self.TURN_ST
                self.prev_steering = st
                self._out = (st, self.TURN_POWER)
            else:
                self._out = (0, self.BRAKE_DUTY)   # 양쪽 역토크 브레이크
            return self._out

        if self.turning:
            return super().step(r1, r2, r3, r4)

        # ── NORMAL: 진검정 기억 + 코너 확정(흰색 지속) ── #
        left_deep = n[0] < self.CORNER_BLACK and n[3] > 60
        right_deep = n[3] < self.CORNER_BLACK and n[0] > 60
        if left_deep:
            self.mem_dir, self.mem_age = 0, 0
        elif right_deep:
            self.mem_dir, self.mem_age = 1, 0
        elif self.mem_dir is not None:
            self.mem_age += 1
            if self.mem_age > self.MEM_FRAMES:
                self.mem_dir = None

        if all(v > self.W_NORM for v in n) and self.mem_dir is not None:
            self.white_cnt += 1
        else:
            self.white_cnt = 0
        if self.white_cnt >= self.WHITE_CONFIRM:
            self.white_cnt = 0
            self.turn_dir = self.mem_dir
            self.mem_dir = None
            self.raw_prev = raw
            self.ps = n[:]
            if self.BRAKE_FRAMES > 0:
                self.brake_cnt = self.BRAKE_FRAMES
                self._out = (0, self.BRAKE_DUTY)
            else:
                self.turning = True
                self.turn_frames = 0
                st = -self.TURN_ST if self.turn_dir == 0 else self.TURN_ST
                self.prev_steering = st
                self._out = (st, self.TURN_POWER)
            return self._out

        out = V5Controller.step(self, r1, r2, r3, r4)

        # ── ★속도 스케줄링: 조향이 클수록 감속 ── #
        sched = self.POWER - self.SPD_K * abs(out[0])
        sched = max(self.MIN_POWER, sched)
        pw = min(out[1], sched)
        if self.settle > 0:
            self.settle -= 1
            pw = min(pw, self.SETTLE_POWER)
        out = (out[0], pw)
        self._out = out
        return out


class V8Controller(V7Controller):
    """v7 + ★구간(오도메트리) 모드: 코스를 미리 아는 대회 특성 활용.
    지그재그 구간에서는 코너피벗/뱅킹을 끄고 직진 관통한다
    (봉우리 높이 ~147mm < 이탈한계, 다리들이 주기적으로 센서를 지나감).
    거리는 좌우 모터 각도 평균으로 추정 (실기: lm.angle()/rm.angle())."""

    def __init__(self, value, zones=((3450, 4450),), zone_steer_cap=15, **kw):
        super().__init__(value, **kw)
        self.ZONES = zones             # (시작mm, 끝mm) 직진관통 구간들
        self.ZONE_CAP = zone_steer_cap # 구간 내 조향 상한 (다리 추종 방지)
        self.dist_mm = 0.0

    def on_distance(self, mm):
        self.dist_mm = mm

    def in_zone(self):
        return any(a <= self.dist_mm <= b for a, b in self.ZONES)

    def step(self, r1, r2, r3, r4):
        if not self.in_zone():
            return super().step(r1, r2, r3, r4)

        # ── 관통 모드 ── #
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out
        n = self.norm4(raw)
        self.raw_prev = raw
        self.ps = n[:]
        self.turning = False       # 구간 진입 시 피벗 상태 해제
        self.brake_cnt = 0
        self.mem_dir = None
        self.white_cnt = 0

        if all(v > self.W_NORM for v in n):
            st = 0                 # 라인 안 보임 → 기저 방향 유지
        else:
            err = (n[0] + n[1]) - (n[2] + n[3]) - self.error_offset
            st = max(-self.ZONE_CAP, min(self.ZONE_CAP,
                     (self.KP_S * err) // self.S))
        self.prev_steering = st
        self.last_good_steer = 0
        self.last_error = 0
        self._out = (st, self.POWER)
        return self._out


class V9Controller(V8Controller):
    """v8 + ★코스맵: 경기 코스를 미리 아는 것을 최대 활용.
    - 코너 피벗은 맵의 코너 창(window) 안에서만 무장 (창 안: 즉시 발동)
      → 루프/교차/웨이브에서의 가짜 피벗을 원천 차단
    - 지그재그 창은 직진 관통 (V8)
    - 나머지 전 구간: 순수 PID + 갭/뱅킹 (풀듀티)"""

    def __init__(self, value, corner_windows=((1350, 1800),), **kw):
        super().__init__(value, **kw)
        self.CORNER_WINDOWS = corner_windows

    def corner_armed(self):
        return any(a <= self.dist_mm <= b for a, b in self.CORNER_WINDOWS)

    def step(self, r1, r2, r3, r4):
        if self.in_zone():
            return V8Controller.step(self, r1, r2, r3, r4)   # 지그재그 관통
        if self.turning or self.brake_cnt > 0 or self.corner_armed():
            return V7Controller.step(self, r1, r2, r3, r4)   # 코너 창: 피벗 무장

        # ── 그 외 전 구간: 피벗 비무장 (기억도 안 쌓음) ── #
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out
        self.mem_dir = None
        self.white_cnt = 0
        out = V5Controller.step(self, r1, r2, r3, r4)
        self._out = out
        return out


class V10Controller(V8Controller):
    """v8 + ★오도메트리 선행 피벗: 코스맵의 코너 좌표에서 라인 소실을
    기다리지 않고 피벗을 선제 발동 → 감지 지연(~100mm) 만큼 관성 예산 확보.
    scripted = ((발동거리mm, 방향 0좌/1우), ...) — 각 항목은 1회만 발동."""

    def __init__(self, value, scripted=((1430, 1),), **kw):
        super().__init__(value, **kw)
        self.scripted = sorted(scripted)
        self.next_sc = 0

    def step(self, r1, r2, r3, r4):
        if (self.next_sc < len(self.scripted)
                and not self.turning and self.brake_cnt == 0
                and self.dist_mm >= self.scripted[self.next_sc][0]):
            d = self.scripted[self.next_sc][1]
            self.next_sc += 1
            self.turning = True
            self.turn_dir = d
            self.turn_frames = 0
            self.mem_dir = None
            self.white_cnt = 0
            raw = (r1, r2, r3, r4)
            if self.raw_prev is None:
                self.prime(r1, r2, r3, r4)
            self.raw_prev = raw
            self.ps = self.norm4(raw)
            st = -self.TURN_ST if d == 0 else self.TURN_ST
            self.prev_steering = st
            self._out = (st, self.TURN_POWER)
            return self._out
        return super().step(r1, r2, r3, r4)
