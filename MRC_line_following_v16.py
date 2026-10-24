from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis, Color
from pybricks.tools import wait

# =============================================================
#  v16: 계보 통합판 (2026-07-11, τ_dn=35ms 물리 재검증 기반)
#
#  τ_dn(감속 비대칭) 반영 시뮬 슛아웃 결과:
#   - v12 아키텍처(정규화+투센서 PID+슬루)가 직선 압승
#     : 위빙 std 0.2mm·2.1m 직선 2.0초 (v14는 11mm·6.8초)
#   - 단 v12는 직각코너 결정적 사망 (깨끗한 추종 = 풀속 코너 진입)
#   - 해법 = v13 트립와이어 사전감속 (실기+시뮬 검증: 코너 0/6→6/6)
#
#  합성 내용:
#   [v12] norm4 정규화, 안쪽 페어(inner2) PID, EMA/미분필터,
#         코너 방향기억, GAP 유지, 뱅킹, 조향비례감속, 슬루 리미터
#   [v13.1] 바깥센서 트립와이어(생값 OR EMA잔상) → 사전감속 PB_POWER
#   [v13.2] 진짜 이탈 첫 진입 시 단락제동 brake() 펄스
#   [v14] 자이로 기여 클램프(재포착 스핀 폭주 차단), 부호 자가판별 v2
#
#  하드웨어 전제: 안쪽 c2,c3 = 전방 90mm(라인트레이싱),
#                바깥 c1,c4 = 전방 170mm(코너 조기경보), 기어 2배속
# =============================================================

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
c1 = PUPDevice(Port.A)    # 좌바깥 (전방 170mm, 코너감지 전담)
c2 = PUPDevice(Port.C)    # 좌안쪽 (전방 90mm, 라인트레이싱)
c3 = PUPDevice(Port.B)    # 우안쪽 (전방 90mm, 라인트레이싱)
c4 = PUPDevice(Port.D)    # 우바깥 (전방 170mm, 코너감지 전담)

def steering_dc(steering, power):
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        st_B = round(slope * steering + power, 2)
        st_C = power
    else:
        st_B = power
        st_C = round(-slope * steering + power, 2)
    lm.dc(st_B)
    rm.dc(st_C)

KG_S = 200    # 자이로 댐핑 ×1000 (τ_dn 스캔 상위권 공통값)

# ★자이로 부호 자가판별 v2 (v14 검증판): duty 40→60 승급 + |평균|30deg/s 검증
def gyro_sign_check():
    for duty in (40, 60):
        lm.dc(-duty)
        rm.dc(duty)            # 제자리 좌회전 (반시계)
        wait(150)
        total = 0
        for _ in range(20):
            total += hub.imu.angular_velocity(Axis.Z)
            wait(10)
        lm.dc(duty)
        rm.dc(-duty)           # 되돌리기
        wait(350)
        lm.dc(0)
        rm.dc(0)
        wait(300)
        if total > 600:
            return 1
        if total < -600:
            return -1
    hub.speaker.beep()         # 판별 실패 경고 (기본 +1 사용)
    return 1

def pid_control(power, kp, kd, value):
    S = 100000
    KP = round(kp * S)
    KD = round(kd * S)

    ALPHA  = 831           # EMA (v12 챔피언)
    A_COMP = 1000 - ALPHA
    E_S    = 1000
    D_FILTER = 351         # 미분 저역필터
    D_COMP   = 1000 - D_FILTER

    # ── 구조 파라미터 (τ_dn 스캔으로 재확정) ── #
    MAX_STEER_DELTA = 61   # 프레임당 조향 변화 제한 (프레임=센서 1.4kHz 동기)
    GAP_LIMIT   = 4        # 이탈 후 직전조향 유지 프레임
    MEM_FRAMES  = 400      # 코너 방향기억 유효 프레임 (~285ms @1.4kHz)
    CORNER_BLACK = 36      # 바깥센서 진검정 문턱 (정규화 0~100)
    CORNER_EMA   = 45      # EMA 잔상용 완화 문턱 (+25%p, 가로줄 스침 포착)
    BLACK_MID   = 50
    W_NORM      = 82       # 흰색(이탈) 문턱 (정규화)
    ST_BANK     = 65       # 이탈복구 뱅킹 조향
    RECOVER_POWER = 64     # 뱅킹 중 파워
    SPD_K10     = 8        # 조향비례 감속 0.8 — 스캔에서 완주의 핵심 기여.
                           # 감속 미허용이면 0으로 (완주율 하락 감수)
    MIN_POWER   = 70       # 감속 하한 (SPD_K10>0일 때만 의미)
    G_CLAMP     = 40       # 자이로 기여 상한 (v14: 재포착 스핀 폭주 차단)
    PB_FRAMES   = 300      # ★트립와이어 사전감속 유지 프레임 (~285ms)
    PB_POWER    = 35       # ★사전감속 파워
    BRAKE_FRAMES = 0       # 이탈 단락제동 펄스 — 스캔 결과 무익 판정, OFF
                           # (트립와이어 사전감속이 코너를 이미 해결. 130≈95ms)

    wh1, wh2, wh3, wh4 = value[0], value[2], value[4], value[6]
    b1,  b2,  b3,  b4  = value[1], value[3], value[5], value[7]
    rg1, rg2, rg3, rg4 = wh1 - b1, wh2 - b2, wh3 - b3, wh4 - b4

    integral   = 0
    last_error = 0
    d_filt     = 0
    direction  = 0
    mem_dir    = None      # 코너 방향기억 (0=좌, 1=우)
    mem_age    = 0
    gap        = 0
    was_off    = False
    prev_steering   = 0
    last_good_steer = 0
    pb_cnt     = 0         # 사전감속 잔여 프레임
    brake_cnt  = 0         # 단락제동 잔여 프레임

    # ── 정규화: 각 센서 0(검정)~100(흰색) — v5 검증 픽스 ── #
    def norm4(r1, r2, r3, r4):
        n1 = (min(max(r1, b1), wh1) - b1) * 100 // rg1
        n2 = (min(max(r2, b2), wh2) - b2) * 100 // rg2
        n3 = (min(max(r3, b3), wh3) - b3) * 100 // rg3
        n4 = (min(max(r4, b4), wh4) - b4) * 100 // rg4
        return n1, n2, n3, n4

    pr1 = c1.read(mode=0)[0]
    pr2 = c2.read(mode=0)[0]
    pr3 = c3.read(mode=0)[0]
    pr4 = c4.read(mode=0)[0]
    ps1, ps2, ps3, ps4 = norm4(pr1, pr2, pr3, pr4)

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]

        # 프레임 동기: 센서 갱신(1.4kHz)이 루프(5kHz)보다 느림
        if r1 == pr1 and r2 == pr2 and r3 == pr3 and r4 == pr4:
            continue
        pr1, pr2, pr3, pr4 = r1, r2, r3, r4

        n1, n2, n3, n4 = norm4(r1, r2, r3, r4)

        # EMA (정규화값 기준)
        s1 = (ALPHA * n1 + A_COMP * ps1) // E_S
        s2 = (ALPHA * n2 + A_COMP * ps2) // E_S
        s3 = (ALPHA * n3 + A_COMP * ps3) // E_S
        s4 = (ALPHA * n4 + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        # ── ★트립와이어: 바깥 한쪽만 진검정 = 전방 170mm 직각 가로줄 ──
        #   가로줄 스침 ~12ms를 생값이 놓칠 수 있어 생값 OR EMA잔상.
        #   마커/교차는 양쪽 동시 검정이라 면역.
        left_deep  = (n1 < CORNER_BLACK or s1 < CORNER_EMA) and n4 > 60
        right_deep = (n4 < CORNER_BLACK or s4 < CORNER_EMA) and n1 > 60
        if left_deep or right_deep:
            if mem_dir is None:          # 신선한 첫 감지 → 사전감속 발동
                pb_cnt = PB_FRAMES
                hub.light.on(Color.RED)  # 디버그: 사전감속 = 빨강
            mem_dir = 0 if left_deep else 1
            mem_age = 0
        elif mem_dir is not None:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = None
                hub.light.on(Color.GREEN)   # 디버그: 코너 모드 종료 = 초록

        # ── 이탈 판정: 안쪽 페어 둘 다 흰색 (정규화 생값) ── #
        lost = n2 > W_NORM and n3 > W_NORM

        # ── 오차 = 안쪽 페어 차이 (투센서) ── #
        error = 2 * (s2 - s3)

        if not lost:
            if was_off:            # 재포착 → PID 리셋 (미분 튐 방지)
                integral = 0
                d_filt = 0
                last_error = error
                prev_steering = 0
            was_off = False
            gap = 0

            if n1 < BLACK_MID and n4 > 60:
                direction = 0
            elif n4 < BLACK_MID and n1 > 60:
                direction = 1

            integral += error
            diff = error - last_error
            d_filt = (D_FILTER * diff + D_COMP * d_filt) // E_S
            steering = (KP * error + KD * d_filt) // S
            last_error = error

            # 교차점: 안쪽 페어 동시 진검정 → 직진 유지
            if s2 < BLACK_MID and s3 < BLACK_MID:
                steering = 0

            # ★자이로 댐핑 + 클램프 (v14: 재포착 고속회전 폭주 차단)
            g = (KG_EFF * int(hub.imu.angular_velocity(Axis.Z))) // 1000
            if g > G_CLAMP:
                g = G_CLAMP
            elif g < -G_CLAMP:
                g = -G_CLAMP
            steering += g
            out_power = power
            last_good_steer = steering
        else:
            gap += 1
            if gap <= GAP_LIMIT:       # 짧은 공백: 직전 조향 유지
                steering = last_good_steer
                out_power = power
            else:                      # 진짜 이탈
                if not was_off:        # ★첫 진입 → 단락제동 펄스
                    brake_cnt = BRAKE_FRAMES
                    hub.light.on(Color.YELLOW)   # 디버그: 이탈 브레이크 = 노랑
                was_off = True
                d = mem_dir if mem_dir is not None else direction
                steering = -ST_BANK if d == 0 else ST_BANK
                out_power = RECOVER_POWER

        # 조향비례 감속 (SPD_K10=0이면 no-op)
        if SPD_K10 > 0:
            sched = power - (SPD_K10 * abs(steering)) // 10
            if sched < MIN_POWER:
                sched = MIN_POWER
            if out_power > sched:
                out_power = sched

        # ★트립와이어 사전감속 적용
        if pb_cnt > 0:
            pb_cnt -= 1
            if out_power > PB_POWER:
                out_power = PB_POWER

        # Slew Rate Limiter
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        # ★단락제동 펄스 (조향보다 우선)
        if brake_cnt > 0:
            brake_cnt -= 1
            lm.brake()
            rm.brake()
        else:
            steering_dc(steering, out_power)

# --------------------------- 메인 --------------------------- #

# 실기 재측정 VALUE (2026-07-11 기준, 센서 이동 시 재측정할 것)
value = [28, 2, 26, 2, 25, 2, 21, 1]

KG_EFF = KG_S * gyro_sign_check()   # 자이로 부호 자동 보정 (시작 시 0.7초)
# ★τ_dn 스캔 챔피언 (180조합 + 12월드 재검증, 2026-07-11):
#   완주 8/12·29.3s (동조건 v14 = 0/12). kp0.268 kd6.2 kg200 msd61
#   bank65 rec64 spdk0.8 pb300f/35p brake OFF mem400
pid_control(100, 0.268, 6.2, value)
