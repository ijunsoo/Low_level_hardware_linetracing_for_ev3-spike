# =====================================================================
#  SPIKE Prime 하드웨어 보정 스크립트 (Pybricks)
# ---------------------------------------------------------------------
#  시뮬레이터(sim/)에 넣을 "내 로봇의 진짜 스펙"을 측정한다.
#    [1] 배터리 전압
#    [2] 모터 가속(스텝응답): dc 20/40/60/80/100 각각에 대해
#        시간-각도 로그 → 정상속도·가속시간(tau)·데드존 피팅용
#    [3] 라인 스캔: 라인을 천천히 통과하며 4센서 반사값 기록
#        → 센서 발자국(footprint) 폭 + 흰/검 실측값
#    [4] 루프/센서 갱신속도 벤치마크
#
#  사용법:
#    1. Pybricks Code(beta.pybricks.com)에서 이 파일 실행
#    2. 각 단계 안내가 뜨면 로봇을 배치하고 << 왼쪽 버튼 >> 으로 진행
#       (가운데 버튼은 프로그램 정지라서 안 씀)
#    3. 끝나면 터미널 출력 전체를 복사해 calib_log.txt 로 저장
#    4. PC에서: python3 sim/fit_params.py calib_log.txt
#
#  주의: [2]는 로봇이 실제로 달린다. 파워당 직선 1.5m 확보하고,
#        매번 제자리로 옮긴 뒤 왼쪽 버튼을 누를 것.
# =====================================================================

import gc

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Button
from pybricks.tools import wait, StopWatch

# ------------------------- 장치 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.B, Direction.COUNTERCLOCKWISE)
rm = Motor(Port.F)
c1 = PUPDevice(Port.A)    # 좌측부터 1
c2 = PUPDevice(Port.C)
c3 = PUPDevice(Port.E)
c4 = PUPDevice(Port.D)

# ------------------------- 측정 설정 ------------------------- #
POWERS    = (20, 40, 60, 80, 100)  # 스텝응답 측정할 dc 값
STEP_MS   = 1200                   # 가속+정속 구간 기록 시간
COAST_MS  = 600                    # 전원 끊고 관성 감속 기록 시간
SAMPLE_MS = 3                      # 샘플 간격 (StopWatch 해상도 1ms)
SCAN_DC   = 25                     # 라인 스캔 주행 파워 (실측 데드존 dc≈17 위여야 함)
SCAN_DEG  = 500                    # 라인 스캔 주행량 (모터 각도, 56mm휠≈244mm)


def wait_left(msg):
    # 왼쪽 버튼을 눌렀다 뗄 때까지 대기 (가운데 버튼 = 프로그램 정지 예약이라 회피)
    print("#", msg, "→ 왼쪽 버튼")
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)
    while hub.buttons.pressed():
        wait(10)
    wait(400)   # 손 떼는 흔들림이 측정에 안 섞이게 잠깐 대기


def step_test(power):
    # dc(power) 스텝을 걸고 시간-각도를 메모리에 기록 후 한번에 출력.
    # (측정 중 print하면 타이밍이 흔들려서 기록과 출력을 분리)
    gc.collect()   # 이전 측정의 잔여 메모리 정리 (작은 힙 대비)
    ts, las, ras = [], [], []
    lm.reset_angle(0)
    rm.reset_angle(0)
    sw = StopWatch()
    nxt = 0
    lm.dc(power)
    rm.dc(power)
    while sw.time() < STEP_MS:
        t = sw.time()
        if t >= nxt:
            ts.append(t); las.append(lm.angle()); ras.append(rm.angle())
            nxt += SAMPLE_MS
    # 전원 차단(코스트) — 감속 특성도 함께 기록
    lm.stop()
    rm.stop()
    while sw.time() < STEP_MS + COAST_MS:
        t = sw.time()
        if t >= nxt:
            ts.append(t); las.append(lm.angle()); ras.append(rm.angle())
            nxt += SAMPLE_MS
    lm.brake()
    rm.brake()
    for i in range(len(ts)):
        # t<=STEP_MS 구간이 구동, 이후가 코스트 (fit_params.py가 구분)
        print("STEP,%d,%d,%d,%d" % (power, ts[i], las[i], ras[i]))


def line_scan():
    # 라인에 수직으로 천천히 진입-통과하며 모터각도+4센서 반사값 기록.
    # → 발자국 폭, 흰/검 실측값. (센서 좌우 간격은 자로 재는 게 정확)
    # x축이 각도라 타이밍이 안 중요함 → 메모리에 안 쌓고 즉시 print (MemoryError 방지).
    # 각도 2도(휠 56mm 기준 ≈1mm)마다 1줄 기록.
    gc.collect()
    lm.reset_angle(0)
    sw = StopWatch()
    last = -999
    lm.dc(SCAN_DC)
    rm.dc(SCAN_DC)
    while lm.angle() < SCAN_DEG:
        a = lm.angle()
        if a - last >= 2:
            last = a
            print("SCAN,%d,%d,%d,%d,%d" % (a,
                  c1.read(0)[0], c2.read(0)[0],
                  c3.read(0)[0], c4.read(0)[0]))
        if sw.time() > 20000:
            print("# 스캔 타임아웃 — SCAN_DC(%d)가 데드존이면 값을 키우세요" % SCAN_DC)
            break
    lm.brake()
    rm.brake()


def benchmark():
    # 실제 제어 루프와 같은 '4센서 읽기' 루프의 속도와,
    # 센서값이 실제로 바뀐(새 프레임) 속도를 측정.
    # ※ 측정 중 로봇을 라인 위에서 손으로 천천히 움직여야 갱신이 제대로 잡힌다.
    sw = StopWatch()
    loops = 0
    changes = 0
    prev = None
    while sw.time() < 2000:
        r = (c1.read(0)[0], c2.read(0)[0], c3.read(0)[0], c4.read(0)[0])
        loops += 1
        if r != prev:
            changes += 1
            prev = r
    print("HZ,loop_reads,%d,sensor_updates,%d" % (loops // 2, changes // 2))


# ------------------------- 실행 순서 ------------------------- #
# 시작 단계: 1=배터리+가속(스텝응답), 2=라인스캔, 3=벤치마크
# 가속 측정은 이미 끝났으므로 스캔부터 재개. 처음부터 다시 하려면 1로.
START_STAGE = 2

print("BATT,%d" % hub.battery.voltage())          # mV

if START_STAGE <= 1:
    for p in POWERS:
        wait_left("[가속] 직선 1.5m 앞에 배치, dc(%d) 발진" % p)
        step_test(p)

if START_STAGE <= 2:
    wait_left("[스캔] 4센서가 라인 '앞' 흰바닥 위에 오게, 라인에 수직 배치")
    line_scan()

wait_left("[벤치] 시작 후 2초간 로봇을 라인 위에서 손으로 천천히 움직이기")
benchmark()

print("BATT,%d" % hub.battery.voltage())          # 측정 후 전압(강하 확인)
print("# 완료 — 출력 전체를 calib_log.txt로 저장하세요")
