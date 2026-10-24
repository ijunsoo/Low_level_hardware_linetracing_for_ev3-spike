from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Button, Color, Axis
from pybricks.tools import wait, StopWatch

# =============================================================
#  spike_diag: 실기 타이밍 진단 — 시뮬(core·fastsim) 보정용, SPIKE Prime
#
#  재는 것
#   [1] 호출 비용: PUPDevice.read / IMU / Motor.dc 1회 (µs)
#   [2] 센서 갱신 구조: 센서별 갱신 간격, 여러 센서가 '같이' 바뀌는지,
#       읽는 도중 프레임이 바뀌는 '찢어진 읽기' 비율 → 시뮬 p_torn
#   [3] 실제 제어 스텝 수: 라인을 따라가며 프레임 동기(값이 바뀐 루프)를 통과한 횟수/초
#   [4] 센서 지연: 직각 라인을 속도별로 지나가며 '감지 위치'가 속도에 따라 밀리는
#       기울기 = 지연 (ms). 모터각 기준이라 Motor.angle() 자체의 지연이 빠진 '상대' 값
#   [5] 센서 지연(자이로 기준): 라인 위 제자리 좌우 회전 — 반시계·시계 방향 검정 구간 중심 차
#       = 2 × 회전속도 × 지연. 바퀴 미끄러짐·모터각 지연과 무관한 '절대'에 가까운 값
#   [6] 구동 반응 지연: 정지 상태에서 갑자기 제자리 회전 명령 → 모터 엔코더와 자이로가 각각 몇 ms 뒤
#       반응하는지 1ms 간격 기록 (블랙박스로 본 '명령→회전 30ms 지연'이 모터 쪽인지 자이로 쪽인지 가름)
#
#  사용: Pybricks Code에서 실행 → 안내에 따라 왼쪽 버튼으로 진행 →
#        출력 전체를 diag_log.txt로 저장 → PC: python3 sim/diag_parse.py diag_log.txt
#  배선: 3센서 일렬(spike_lf3_rule과 같은 포트) 기준, 읽는 순서 = 좌→가운데→우.
#        4센서 실기면 SENSORS 줄을 아래 주석 것으로 바꿀 것
# =============================================================

hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
SENSORS = [PUPDevice(Port.C), PUPDevice(Port.A), PUPDevice(Port.B)]     # 좌 C · 가운데 A · 우 B
# 4센서(v16 계열) 배선이면: [PUPDevice(Port.A), PUPDevice(Port.C), PUPDevice(Port.B), PUPDevice(Port.D)]

AUTO = False            # True = 버튼 안 기다림 (시뮬에서 돌릴 때만 — diag_parse.py가 켬)
STAGES = (1, 2, 3, 4, 5, 6)   # 돌릴 단계 (지연만: (5,) / 구동 반응만: (6,))
UPD_ITERS = 12000       # [2] 기록할 루프 수 (≈2.5초)
LAG_DUTIES = (25, 40, 55, 70, 85)
LAG_REPS = 2            # [4] 속도마다 반복 횟수
LAG_TRAVEL_DEG = 600    # [4] 1회 주행 모터각 상한 (출발점 기준, 기어2·56mm 바퀴 ≈ 0.6m)
RET_DUTY = 35           # [4] 출발점 복귀 듀티
K_SYNC = 2              # [4] 좌우 모터각 차(도) × K_SYNC 만큼 듀티 보정 → 직진 유지
IMU_RATES = (150, 250, 350)   # [5] 제자리 회전 목표 요레이트 (deg/s)
IMU_SWEEPS = 3          # [5] 속도마다 왕복 횟수
IMU_AMP = 40            # [5] 회전 범위 ±도 (센서 3개가 모두 라인을 지나가게)
WHEEL_MM = 56
GEAR = 20 / 12          # 바퀴각 = 모터각 × GEAR — 모터 20T가 바퀴 12T를 돌림 (직결이면 1)
AXLE_MM = 72            # 바퀴 간격 (목표 회전속도 환산용 — 틀려도 실제 속도는 자이로로 잼)

N = len(SENSORS)
ALL = (1 << N) - 1


def ask(msg):
    print("#", msg, "→ 왼쪽 버튼")
    if AUTO:
        SIM_RESET()             # 시뮬: 로봇을 출발점에 다시 세움 (실기에선 사람이 함)
        wait(300)
        return
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)
    while hub.buttons.pressed():
        wait(10)
    wait(400)


def read_all():
    return [s.read(0)[0] for s in SENSORS]


def go(duty, al, ar):
    # 좌우 모터각을 같게 유지하며 duty로 주행 (음수 = 후진). al·ar = 방금 읽은 모터각
    c = (al - ar) * K_SYNC
    lm.dc(max(-100, min(100, duty - c)))
    rm.dc(max(-100, min(100, duty + c)))


# ------------------------- [1] 호출 비용 ------------------------- #
def stage_cost():
    sw = StopWatch()
    n = 3000
    t0 = sw.time()
    for _ in range(n):
        pass
    empty = sw.time() - t0
    s0 = SENSORS[0]
    t0 = sw.time()
    for _ in range(n):
        s0.read(0)
    rd = sw.time() - t0
    t0 = sw.time()
    for _ in range(n):
        hub.imu.angular_velocity()
    imu = sw.time() - t0
    t0 = sw.time()
    for _ in range(n):
        lm.dc(0)
    dc = sw.time() - t0
    t0 = sw.time()
    for _ in range(n):
        sw.time()
    tm = sw.time() - t0
    # µs/회 = (총 ms - 빈 루프 ms) × 1000 / n
    print("COST,read_us,%d,imu_us,%d,dc_us,%d,stopwatch_us,%d,empty_loop_us,%d" % (
        (rd - empty) * 1000 // n, (imu - empty) * 1000 // n, (dc - empty) * 1000 // n,
        (tm - empty) * 1000 // n, empty * 1000 // n))


# ------------------------- [2] 센서 갱신 구조 ------------------------- #
def record_masks(iters):
    # 루프마다 '이번 루프에서 값이 바뀐 센서' 비트마스크를 기록 (시간축 = 루프 번호)
    masks = bytearray(iters)
    prev = read_all()
    sw = StopWatch()
    t0 = sw.time()
    for k in range(iters):
        m = 0
        for i in range(N):
            v = SENSORS[i].read(0)[0]
            if v != prev[i]:
                m |= 1 << i
                prev[i] = v
        masks[k] = m
    return masks, sw.time() - t0


def analyze_masks(masks, ms, tag):
    iters = len(masks)
    cnt = [0] * N
    hist = [0] * (ALL + 1)
    last = [-1] * N
    gaps = [[0] * 16 for _ in range(N)]     # 센서별 갱신 간격 히스토그램 (루프 수, 15+는 마지막 칸)
    torn = 0
    for k in range(iters):
        m = masks[k]
        hist[m] += 1
        for i in range(N):
            if m >> i & 1:
                cnt[i] += 1
                if last[i] >= 0:
                    g = k - last[i]
                    gaps[i][g if g < 15 else 15] += 1
                last[i] = k
        # 찢어진 읽기: 이번 루프에 '뒤쪽 센서들'만 바뀌고 다음 루프에 '앞쪽 나머지'가 바뀜
        if 0 < m < ALL and k + 1 < iters:
            nxt = masks[k + 1]
            low = m & -m                    # 바뀐 센서 중 가장 앞(읽기 순서상)
            prefix = low - 1                # 그보다 앞쪽 센서들
            if prefix and (m | prefix) == ALL and nxt & prefix == prefix:
                torn += 1
    print("UPD,%s,iters,%d,ms,%d" % (tag, iters, ms))
    print("UPDRATE,%s,%s" % (tag, ",".join("%d" % (c * 1000 // max(1, ms)) for c in cnt)))
    print("MASKHIST,%s,%s" % (tag, ",".join(str(h) for h in hist)))
    print("TORN,%s,%d" % (tag, torn))
    for i in range(N):
        print("GAPS,%s,%d,%s" % (tag, i, ",".join(str(g) for g in gaps[i])))
    # 원본 마스크 (루프마다 16진 1글자) — PC가 센서쌍별 '한 루프 앞섬' 빈도로 찢어짐을 추정
    hexd = "0123456789abcdef"
    for a in range(0, iters, 200):
        print("MASKS,%s,%d,%s" % (tag, a, "".join(hexd[m] for m in masks[a:a + 200])))


def stage_update():
    ask("[2] 라인 위에 놓고, 실행 중 손으로 로봇을 천천히 좌우로 흔들어 주세요")
    hub.light.on(Color.RED)
    masks, ms = record_masks(UPD_ITERS)
    hub.light.on(Color.GREEN)
    analyze_masks(masks, ms, "hand")


# ------------------------- [3] 실제 제어 스텝 수 ------------------------- #
def stage_steps():
    ask("[3] 직선 라인 위에 정렬해 두세요 (2.5초 라인을 따라갑니다)")
    # 진단용 최소 P 제어 — 목적은 '주행 중 프레임 동기 통과 횟수' 측정
    w = read_all()
    hub.light.on(Color.RED)
    prev = read_all()
    sw = StopWatch()
    loops = steps = 0
    masks = bytearray(9000)
    while sw.time() < 2500:
        r = read_all()
        m = 0
        for i in range(N):
            if r[i] != prev[i]:
                m |= 1 << i
        if loops < len(masks):
            masks[loops] = m
        loops += 1
        if m == 0:
            continue
        prev = r
        steps += 1
        # 바깥/안쪽 구분 없이 좌반 - 우반 밝기 차이로 조향 (진단용)
        half = N // 2
        e = sum(r[:half]) - sum(r[N - half:])
        st = max(-60, min(60, e * 3))
        if st < 0:
            lm.dc(50 + st)
            rm.dc(50)
        else:
            lm.dc(50)
            rm.dc(50 - st)
    ms = sw.time()
    lm.brake()
    rm.brake()
    hub.light.on(Color.GREEN)
    print("STEPS,loops,%d,steps,%d,ms,%d" % (loops, steps, ms))
    analyze_masks(masks[:min(loops, len(masks))], ms * min(loops, len(masks)) // max(1, loops), "drive")
    del w


# ------------------------- [4] 센서 지연 ------------------------- #
def stage_lag():
    # 한 번만 놓으면 속도마다 라인을 지나간 뒤 출발점으로 스스로 돌아온다 → 모든 런의 기준이 같아
    # '감지 위치 vs 속도' 기울기가 순수한 지연이 된다 (런마다 사람이 다시 놓으면 놓은 위치 차이가 섞임)
    deg_mm = 3.14159 * WHEEL_MM / 360 * GEAR      # 모터 1도당 전진 mm
    print("LAGCFG,deg_mm,%d" % round(deg_mm * 1000))
    print("LAGMODE,return")
    ask("[4] 흰 바닥, 직각 라인 20cm 앞에 라인과 수직으로 한 번만 놓아 주세요 (라인 뒤로 30cm 비우기)")
    lm.reset_angle(0)
    rm.reset_angle(0)
    base = read_all()                       # 흰 바닥 기준값 (출발점)
    for duty in LAG_DUTIES:
        for rep in range(LAG_REPS):
            a0 = (lm.angle() + rm.angle()) // 2
            state = [0] * N                 # 0=흰 1=라인 위 2=통과
            lo = list(base)
            hit = [None] * N
            out = [None] * N
            while True:
                al = lm.angle()
                ar = rm.angle()
                go(duty, al, ar)
                a = (al + ar) // 2
                r = read_all()
                sp = (lm.speed() + rm.speed()) // 2
                for i in range(N):
                    if r[i] < lo[i]:
                        lo[i] = r[i]
                    # 히스테리시스: 흰 기준 50% 아래 = 진입, 75% 위로 회복 = 이탈
                    # (한 문턱만 쓰면 가장자리에서 노이즈로 진입 직후 '이탈'이 찍힘)
                    if state[i] == 0 and r[i] < base[i] // 2:
                        state[i] = 1
                        hit[i] = (a, sp)
                    elif state[i] == 1 and r[i] > base[i] * 3 // 4:
                        state[i] = 2
                        out[i] = (a, sp)
                if min(state) == 2 or a > LAG_TRAVEL_DEG:
                    break
            lm.brake()
            rm.brake()
            print("LAGRUN,%d,%d,%d" % (duty, rep, a0))
            for i in range(N):
                if hit[i] and out[i]:
                    print("LAG,%d,%d,%d,%d,%d,%d,%d,%d" % (duty, i, hit[i][0], hit[i][1],
                                                         out[i][0], out[i][1], base[i], lo[i]))
                else:
                    print("LAG,%d,%d,miss" % (duty, i))
            wait(300)
            while True:                     # 출발점으로 복귀 (직진 유지하며 후진)
                al = lm.angle()
                ar = rm.angle()
                if al + ar <= 0:
                    break
                go(-RET_DUTY, al, ar)
                wait(1)                     # 1ms 간격으로 확인 (시뮬 시간도 흐르게)
            lm.brake()
            rm.brake()
            wait(400)


# ------------------------- [5] 센서 지연 (자이로 기준) ------------------------- #
def find_yaw():
    # (spike_lf3_rule.py와 같음) 정지 상태에서 중력이 실리는 축 = 상하축 → '좌회전 = +' 요레이트
    for _ in range(40):
        wait(5)
    ax = hub.imu.acceleration(Axis.X)
    ay = hub.imu.acceleration(Axis.Y)
    az = hub.imu.acceleration(Axis.Z)
    if abs(ax) >= abs(ay) and abs(ax) >= abs(az):
        return Axis.X, (1 if ax > 0 else -1)
    if abs(ay) >= abs(az):
        return Axis.Y, (1 if ay > 0 else -1)
    return Axis.Z, (1 if az > 0 else -1)


def stage_imu():
    # 라인 위에서 제자리로 좌우 회전하며, 센서가 라인을 지나는 순간의 '자이로 적분 각도'를 잰다.
    # 검정 구간 중심이 반시계면 +ω·지연, 시계면 −ω·지연만큼 밀림 → 두 방향 차 = 2ω·지연.
    # 문턱·발자국 같은 정적 치우침은 속도와 무관한 절편으로 빠지도록 회전속도 3가지로 회귀.
    ask("[5] 직선 라인 위에 평소 출발처럼(라인 방향, 가운데 센서가 라인 위) 놓아 주세요 — 제자리 좌우 회전")
    yax, ysg = find_yaw()
    sw = StopWatch()
    b = 0.0                                   # 자이로 바이어스 (정지 0.3초 평균)
    for _ in range(60):
        b += hub.imu.angular_velocity(yax) * ysg
        wait(5)
    b /= 60
    st = [0.0, sw.time()]                     # 적분 각도(도, 반시계 +), 직전 시각(ms)

    def upd():
        now = sw.time()
        rate = hub.imu.angular_velocity(yax) * ysg - b
        st[0] += rate * (now - st[1]) / 1000
        st[1] = now
        return rate

    k_w = AXLE_MM / (WHEEL_MM * GEAR)        # 요레이트(deg/s) → 모터 속도(deg/s)
    lo = [999] * N
    hi = [-1] * N

    def turn(rate_cmd, target, thr, rows, tag):
        # rate_cmd>0 = 반시계. 적분각이 target을 넘을 때까지 회전, 그 뒤 0.25초 정지하며 계속 적분
        state = [0] * N
        ent = [None] * N
        if thr:                                 # 시작부터 검정 위인 센서는 이번 회전에서 제외
            r = read_all()
            for i in range(N):
                if r[i] < thr[i][1]:
                    state[i] = 2
        lm.run(-rate_cmd * k_w)
        rm.run(rate_cmd * k_w)
        t0 = sw.time()
        while (st[0] < target) if rate_cmd > 0 else (st[0] > target):
            w = upd()
            r = read_all()
            for i in range(N):
                if r[i] < lo[i]:
                    lo[i] = r[i]
                if r[i] > hi[i]:
                    hi[i] = r[i]
                if thr:
                    if state[i] == 0 and r[i] < thr[i][0]:
                        state[i] = 1
                        ent[i] = (st[0], w)
                    elif state[i] == 1 and r[i] > thr[i][1]:
                        state[i] = 2
                        rows.append((tag, i, ent[i][0], ent[i][1], st[0], w))
            if sw.time() - t0 > 4000:          # 안 돌면(마찰·배선) 중단
                break
        lm.brake()
        rm.brake()
        t1 = sw.time()
        while sw.time() - t1 < 250:
            upd()
            wait(1)                             # 1ms 간격 (시뮬 시간도 흐르게)

    # 1) 느린 왕복으로 센서별 흰·검 범위 → 대칭 문턱 (중간값 ± 범위의 10%)
    turn(120, IMU_AMP, None, None, 0)
    turn(-120, -IMU_AMP, None, None, 0)       # 측정 왕복은 항상 −AMP에서 시작
    thr = [((lo[i] + hi[i]) // 2 - (hi[i] - lo[i]) // 10, (lo[i] + hi[i]) // 2 + (hi[i] - lo[i]) // 10)
           for i in range(N)]
    print("IMUCFG,amp,%d,bias_mdps,%d,%s" % (IMU_AMP, round(b * 1000),
                                             ",".join("%d/%d" % (lo[i], hi[i]) for i in range(N))))
    # 2) 속도별 왕복: 각 통과의 (방향, 센서, 진입각, 진입속도, 이탈각, 이탈속도)
    for rc in IMU_RATES:
        rows = []
        for _ in range(IMU_SWEEPS):
            turn(rc, IMU_AMP, thr, rows, 1)
            turn(-rc, -IMU_AMP, thr, rows, -1)
        for tag, i, a_in, w_in, a_out, w_out in rows:
            print("IMUX,%d,%d,%d,%d,%d,%d,%d" % (rc, tag, i, round(a_in * 100), round(w_in),
                                               round(a_out * 100), round(w_out)))
    turn(120, 0, None, None, 0)                 # 처음 방향으로 복귀


# ------------------------- [6] 구동 반응 지연 ------------------------- #
def stage_step():
    ask("[6] 평평한 바닥에 놓아 주세요 — 제자리 회전을 갑자기 시작했다 멈추기를 6번 반복")
    yax, ysg = find_yaw()
    sw = StopWatch()
    for rep in range(3):
        for sgn in (1, -1):
            lm.brake()
            rm.brake()
            wait(500)
            a0l = lm.angle()
            a0r = rm.angle()
            lm.dc(-100 * sgn)                    # sgn=1: 반시계(좌회전) 제자리 회전
            rm.dc(100 * sgn)
            t0 = sw.time()
            buf = []
            last = -1
            while True:
                t = sw.time() - t0
                if t > 150:
                    break
                if t != last:                    # 1ms에 한 번 기록
                    last = t
                    buf.append((t, lm.angle() - a0l, rm.angle() - a0r, int(hub.imu.angular_velocity(yax) * ysg)))
                wait(1)                          # (시뮬 시간도 흐르게 — 기록은 어차피 1ms 간격)
            lm.brake()
            rm.brake()
            tm = -1                              # 엔코더가 2도 넘게 움직인 첫 시각
            tg = -1                              # 자이로가 40°/s 넘은 첫 시각
            for b in buf:
                if tm < 0 and (abs(b[1]) >= 2 or abs(b[2]) >= 2):
                    tm = b[0]
                if tg < 0 and abs(b[3]) >= 40:
                    tg = b[0]
            print("STEP,%d,%d,%d,%d" % (sgn, rep, tm, tg))
            print("STEPD," + ";".join("%d:%d:%d:%d" % b for b in buf[:90:2]))
            wait(300)


print("DIAG,spike,sensors,%d" % N)
if 1 in STAGES:
    stage_cost()
if 2 in STAGES:
    stage_update()
if 3 in STAGES:
    stage_steps()
if 4 in STAGES:
    stage_lag()
if 5 in STAGES:
    stage_imu()
if 6 in STAGES:
    stage_step()
print("# 완료 — 출력 전체를 diag_log.txt로 저장하세요")
