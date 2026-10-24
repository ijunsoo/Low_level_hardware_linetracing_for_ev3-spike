"""
fixed_main 기반 + 실제 ToF 벽감지 버전.
fixed_main(잘 동작하는 베이스)에서 딱 세 군데만 바꿈:
  1) PaHub + VL53L0X 4개 하드웨어 추가
  2) get_open_side: dg 배열 조회 → 실제 ToF 읽기
  3) drive(): 전방 ToF 긴급정지 추가
turn/탐색/main 구조는 fixed_main과 동일.
"""


import time
from machine import I2C, Pin

import M5
M5.begin()  # I2C 초기화 전에 가장 먼저 실행
# 콜드(하드)부팅 settle — M5.begin()이 시작한 FT6336 터치 init/reset 해제가 끝나기 전에
# 아래에서 I2C(0)를 재생성하면 그 init 트랜잭션이 중간에 끊겨 터치가 죽는다(콜드부팅만
# 실패, 소프트리셋은 이전 부팅 init이 남아 정상). 충분히 기다린 뒤 버스를 잡는다.
time.sleep_ms(500)
from basex import DriveBase
from VL53L0X import VL53L0X
from devices import PaHub, IMU, Touch, reset_cores3_touch, ColorSensor
from astar.grid import *
from astar.Astar import get_directions
from astar.visualize import create_grid
from logger import log_error, init_logging
from utils import *
init_logging()


# -----------------------------------------------------------------------
# 제너레이터 협력 스케줄러 (asyncio 대체)
# 제약: ① 두 스레드가 I2C → 259  ② async 함수 안에서 I2C → 에러  ③ 락은 핫루프에 느림.
# → 스레드/asyncio를 모두 없애고 순수 제너레이터(yield) 협력 멀티태스킹으로 돌린다.
#   모든 I2C는 이 단일 루프 컨텍스트에서 각 호출이 직접(machine.I2C로) 수행 — 동시
#   접근이 구조적으로 불가능. uasyncio의 숨은 타이머/스케줄러 훅도 없어 통제가 명확하다.
# 변환 규칙: async def→def, yield→yield,
#   await asyncio.sleep_ms(n)→yield from gsleep(n), await f()→yield from f().
# -----------------------------------------------------------------------

def gsleep(ms):
    # ms가 0 이하이면 대기 시간 없이 단 한 번만 다른 제너레이터에게 순서를 양보한다.
    if ms <= 0:
        yield
        return

    end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        yield


def grun(gen):
    # 제너레이터 하나를 끝까지 구동한다 (최상위 엔트리용).
    # 매 틱 imu.update()로 자이로 적분을 직접 갱신한다(연속 샘플링 필요한 유일한 항목).
    # 모터/엔코더/터치/ToF는 각 호출 지점이 그때그때 직접 I2C에 접근한다.
    while True:
        imu.update()
        try:
            next(gen)
        except StopIteration:
            break


def grace(*gens):
    # 여러 제너레이터를 라운드로빈으로 돌리다 하나라도 끝나면 그 인덱스를 반환하고
    # 나머지는 close()로 중단한다. asyncio.create_task + done 폴링을 대체.
    # 여기서도 매 틱 imu.update()로 자이로 적분을 펌프한다(메인 루프 단일 컨텍스트).
    gens = list(gens)
    while True:
        imu.update()
        for i, g in enumerate(gens):
            try:
                next(g)
            except StopIteration:
                for j, other in enumerate(gens):
                    if j != i:
                        other.close()
                return i


def wait_touch(hold_threshold=300, detect_hold=True, debug=False):
    # devices.Touch.wait_for_touch의 제너레이터 판 — is_touching()(캐시, I2C 아님)만 본다.
    press_time = 0
    was_touching = False
    while True:
        touching = touch.is_touching()
        now = time.ticks_ms()
        if not was_touching and touching:
            press_time = now
        if was_touching and not touching:
            if detect_hold:
                if time.ticks_diff(now, press_time) >= hold_threshold:
                    return 2
                else:
                    return 1
            else:
                return True
        was_touching = touching
        yield from gsleep(10)


# -----------------------------------------------------------------------
# 하드웨어 초기화
# -----------------------------------------------------------------------

# 메인 버스(하드웨어 I2C id=0). IMU·DriveBase·Touch가 이 i2c 객체를 직접 공유한다.
# 콜드부팅 터치 실패의 진짜 원인은 I2C 인스턴스 충돌이 아니라 FT6336 RST(AW9523 P0 bit0)가
# 안 풀린 것 → Touch.__init__의 reset_cores3_touch()가 AW9523로 직접 리셋을 풀어 해결한다.
# (이전의 I2C(0) 이중생성 트릭은 헛다리였으므로 제거하고 버스를 한 번만 만든다.)
_i2c = I2C(0, scl=Pin(11), sda=Pin(12), freq=800000)

# Touch 를 가장 먼저 생성한다 — __init__의 reset_cores3_touch()가 콜드부팅에서 죽어 있는
# FT6336 RST(AW9523 P0 bit0)를 풀어 깨운다. IMU/DriveBase init 트래픽보다 앞서 깨워야
# 진단(touch_aw9523_test.py)에서 검증된 "I2C 생성 → 곧바로 리셋" 순서와 일치한다.
touch = Touch(_i2c)                        # 터치 패널 (FT6336)
imu   = IMU(_i2c, gyro_deadzone=0.7)       # 자이로 데드존 0.7도: 미세 노이즈 무시
robot = DriveBase(
    _i2c,
    right_motors=[2, 3], left_motors=[1, 4],
    wheel_diameter=62.4,   # 바퀴 직경 (mm)
    axle_track=112,        # 좌우 바퀴 간격 (mm)
    right_counter=True,    # 우측 모터 엔코더 방향 반전
    gear=[12, 28]          # 기어비: 구동 12T → 피동 28T
)

# ToF 4개 (PaHub 하드웨어 I2C id=1 너머). 포트: 앞5 오4 뒷1 왼0.
# 실제 i2c 초기화/센서 생성/start는 main()에서 수행한다.
pahub = PaHub(sda=2, scl=1)
tof1 = tof2 = tof3 = tof4 = None

# 바닥 컬러센서 (TCS3472, 같은 PaHub 채널 2). ToF가 0/1/4/5 → 남는 2번.
# 실제 생성은 main()에서 pahub.start() 뒤에 한다.
COLOR_CH = 2
color = None

# 분류 결과 → LCD 표시색(RGB888). 글자 없이 색으로만 표기한다.
TILE_RGB = {
    "black": 0x202020, "silver": 0xC0C0C0, "blue": 0x0000FF,
    "red": 0xFF0000, "white": 0xFFFFFF,
}

WALL_MM = 120  # 벽으로 판정하는 거리 임계값 (mm)
STOP_MM = 40  # 주행 중 충돌 방지 긴급 정지 임계값 (mm)

# 후방 벽정렬(회전 직후 재앵커링)
ALIGN_BACK_MM = 35     # 이 거리(mm)까지 후진해 평평한 벽에 밀착
ALIGN_TIMEOUT = 2000   # 후진 정렬 타임아웃 (ms)
ALIGN_PUSH_MS = 300    # 밀착 후 추가로 미는 시간(ms) — 양측 동일 출력으로 스퀘어 맞춤

# -----------------------------------------------------------------------
# 상수 / 타입 정의
# -----------------------------------------------------------------------

# 셀의 4방향 통행 가능 여부를 담는 네임드튜플
azimuth = namedtuple('azimuth', ['n', 's', 'w', 'e'])

_DIRS = [direction(0,-1), direction(1,0), direction(0,1), direction(-1,0)]  # N E S W

def look_dir(h, d):
    # 현재 heading h 에서 방향벡터 d 를 향하기 위해 회전해야 할 각도
    return [0, -90, 180, 90][(_DIRS.index(d) - h) % 4]

def next_dir(h, i):
    # h=현재heading, i: 0=좌 1=전 2=우
    return _DIRS[(h + i - 1) % 4]

# -----------------------------------------------------------------------
# 전역 상태
# -----------------------------------------------------------------------

lastYaw     = 0   # 직진 PID 이전 프레임 편차 (미분항 계산용)
lastTurnYaw = 0   # 회전 PID 이전 프레임 편차
gyroAngle   = 0   # 누적 목표 각도 (회전할 때마다 갱신)
heading     = 0   # 로봇의 논리적 진행 방향 (0=N 1=E 2=S 3=W)

# 전후면 반전 플래그.
# True 이면 후면이 진행 방향 — 실제 180도 회전 대신 모터 방향만 반전한다.
is_reversed = True

pos      = [0, 0]   # 현재 그리드 좌표 (절대)
startPos = [0, 0]   # 시작 셀의 그리드 좌표 (그리드 확장 시 함께 갱신)

openList   = []                    # 아직 방문하지 않은 이웃 방향 스택
closedList = [direction(0, 0)]     # 이미 방문한 셀 좌표 목록

# 프로그램 재시작 시 이전 상태를 복원하기 위한 체크포인트
checkpoint = {
    "grid": None,       # 탐색 그리드
    "node": None,       # 마지막 노드
    "dir": None,        # 다음 이동 방향
    "openList": None,
    "closedList": None,
    "startPos": None,
    "gyroAngle": None,
    "heading": None,
}

# -----------------------------------------------------------------------
# PID 제어
# -----------------------------------------------------------------------

# A: 오도메트리 직진 보정 가중치 (deg당 조향량). 0이면 비활성. 실측 튜닝 필요.
ODO_STRAIGHT_GAIN = 2.0

# 직진 기준속도 — 감속(파워 ↓) 시 조향을 곡률 일정하게 비례 축소(steering ∝ speed)하는 분모.
# drive()의 순항 속도(=150)와 맞춘다. speed가 이 값 미만이면 robot.drive()가 조향을
# speed/REF 로 줄여 저속에서 κ=steering/speed 폭주(좌우 흔들림)를 막는다.
# 턴(pid_turn_control, speed=0)에는 fade_below 를 넘기지 않으므로 영향 없음 → 자이로 그대로 회전.
STRAIGHT_REF_SPEED = 150

def odo_yaw():
    # 좌우 바퀴 주행차로 추정한 헤딩 변화(deg). 자이로와 독립 → 자기간섭/적분 드리프트 면역.
    # robot.reset()(=drive 시작) 기준이라 직진 중엔 0이어야 정상. 양수=CCW(좌)로 틀어짐.
    l = (robot.current_encoder[robot.left_ports[0] - 1] - robot.left_ref) * robot.l_counter
    r = (robot.current_encoder[robot.right_ports[0] - 1] - robot.right_ref) * robot.r_counter
    return (r - l) * robot._deg_to_mm / robot.axle_track * 57.29577951308232


def pid_control(speed, gain, kp, kd):
    global lastYaw
    yaw = -(imu.angle() - gyroAngle)
    diff = yaw - lastYaw
    lastYaw = yaw
    # A: 자이로 PID + 오도메트리 직진 보정(독립 신호). odo_yaw>0(좌로 드리프트)면 우로 보정.
    # 감속 시 조향을 speed 에 비례 축소(곡률 일정)해 정지 직전 좌우 흔들림 제거.
    robot.drive(speed, (kp * yaw + kd * diff) * gain - ODO_STRAIGHT_GAIN * odo_yaw(),
                fade_below=STRAIGHT_REF_SPEED)


def pid_turn_control(dest, gain, kp, kd):
    global lastTurnYaw
    yaw  = dest - (imu.angle() - gyroAngle)
    diff = yaw - lastTurnYaw
    lastTurnYaw = yaw
    robot.drive(0, (kp * yaw + kd * diff) * gain)


def reset_turn():
    print("reset_turn 시작 angle=%.1f" % imu.angle())
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 5000:
        if abs(imu.angle() - gyroAngle) < 0.1:
            break
        pid_turn_control(gyroAngle, 0.26, 17.2, 4)
        yield
    robot.brake()
    print("reset_turn 완료 angle=%.1f" % imu.angle())


def align_back(rev):
    # 회전 직후 후방에 벽이 있으면 그 벽에 평평히 붙여 자세/헤딩을 재앵커링한다.
    # 센서가 면당 1개뿐이라 각도 측정은 못 한다 → 평평한 벽에 양측 동일 출력(조향 0)으로
    # 밀어붙여 섀시를 기계적으로 스퀘어시킨 뒤, imu 각을 목표(gyroAngle)에 맞춰 적분 드리프트 제거.
    global lastYaw

    # 진행 전방의 반대편이 후방 센서 (drive의 front: not reversed=tof4 / reversed=tof1).
    rear = tof1 if not is_reversed else tof4
    if rear is None:
        yield from robot.straight(150, 20 * rev)
        return

    d = rear.get()
    if d > WALL_MM:   # 후방에 벽 없음 → 정렬 생략
        print("후방 벽 없음 → 정렬 생략", d)
        return
    print("후방 벽 정렬 시작", d)

    # 후방으로 천천히 후진해 벽에 붙인다 (모터 부호는 전진의 반대 = -rev). 후진량은 distance로 측정.
    rev = -1 if is_reversed else 1
    robot.reset()
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < ALIGN_TIMEOUT:
        if rear.get() <= ALIGN_BACK_MM:
            break
        robot.drive(-120 * rev, 0)
        yield from gsleep(1)

    # 평평히 밀착되도록 잠깐 더 민다 — 양측 동일 출력이라 벽 반력으로 스퀘어가 맞춰진다.
    push = time.ticks_add(time.ticks_ms(), ALIGN_PUSH_MS)
    while time.ticks_diff(push, time.ticks_ms()) > 0:
        robot.drive(-100 * rev, 0)
        yield from gsleep(1)
    robot.brake()

    # 후진한 거리 (straight가 내부에서 reset 하므로 전진 전에 읽어둔다).
    back_dist = abs(robot.distance())

    # 재앵커링 — 물리적으로 그리드 축에 정렬됐으므로 imu 각을 목표(gyroAngle)로 맞춘다.
    imu.set_angle(gyroAngle)
    lastYaw = 0

    # 후진한 만큼 다시 전진해 셀 중앙으로 복귀.
    yield from robot.straight(150, back_dist * rev)
    robot.brake()
    print("후방 벽 정렬 완료, 복귀 %.0fmm angle=%.1f" % (back_dist, imu.angle()))


def turn(dest, duration=2000):
    global heading, lastYaw, gyroAngle, is_reversed

    if dest == 0:
        return

    if abs(dest) == 180:
        is_reversed = not is_reversed
        heading = (heading + 2) % 4
        return

    # 턴 시작 전 뒤로 15mm — 셀 중앙 확보 후 포인트턴.
    # is_reversed면 진행 방향이 뒤집히므로 부호도 반전.
    rev = -1 if is_reversed else 1
    yield from robot.straight(150, -15 * rev)

    robot.brake()
    i = 0
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < duration:
        remaining = dest - (imu.angle() - gyroAngle)
        # ratio = min(1.0, abs(remaining) / abs(dest))
        ratio = 1
        pid_turn_control(dest, 0.15, 17.2, 10)
        if abs(remaining) < 0.5 and i == 0:
            start = time.ticks_ms()
            i = 1
        # print(imu.angle(), gyroAngle, remaining, ratio)
        yield

    gyroAngle += dest
    lastYaw = 0
    robot.brake()
    print("turn finished")
    yield from gsleep(100)

    # 턴 종료 후 앞으로 15mm — 후진분 복원 (is_reversed면 부호 반전).
    robot.brake()

    # 회전 직후 후방에 벽이 있으면 그 벽에 붙여 자세/헤딩 재앵커링 (없으면 자동 생략).
    yield from align_back(rev)
    # await wait(100)
    # robot.stop(False)
    # await wait(100)
    # await imu.calibrate_with_mag()
    # await wait(100)

    heading = (heading + (-1 if dest == 90 else 1)) % 4

# -----------------------------------------------------------------------
# 주행
# -----------------------------------------------------------------------

def drive(speed, destMM):
    i = 0
    lastError = imu.angle()
    actual_speed = -speed if is_reversed else speed
    sensor = tof4 if not is_reversed else tof1  # 진행방향 전방 센서(전/후 swap)

    robot.reset()

    while True:
        dist = abs(robot.distance())
        if dist > destMM:
            break
        ratio = dist / destMM
        pid_control(actual_speed * (1 - ratio * 0.7), 2, 1.8, 3.6)

        angle = imu.angle()
        diff = angle - lastError
        lastError = angle
        if abs(diff) * 5 > 6 and not i:
            destMM += 10
            i = 1


        t = sensor.get()
        if t <= STOP_MM:
            print("전방 벽(TOF) → 긴급 정지", t)
            break
        # print(t)

        # 주행 중엔 검정(구멍)만 본다 → 감지 시 즉시 정지 후 후진(규정 3.2: 구멍 진입 금지).
        if color is not None and color.is_black():
            print("구멍(검정 타일) 감지 → 긴급 정지/후진")
            M5.Lcd.fillScreen(TILE_RGB["black"])
            robot.brake()
            rev = -1 if is_reversed else 1
            yield from robot.straight(150, -60 * rev)
            break

        yield from gsleep(1)

    robot.brake()
    robot.reset()
    print("drive finished")


# -----------------------------------------------------------------------
# 바닥 컬러 (RCJ 3.2 Floor) — 타일색 분류 + 규정 동작
# -----------------------------------------------------------------------

def blue_stop():
    # 규정: 파랑(웅덩이) 진입 시 다음 타일로 가기 전 5초 정지. 글자 없이 파랑/검정 깜빡임.
    robot.brake()
    for s in range(5, 0, -1):
        M5.Lcd.fillScreen(0x0000FF if s % 2 else 0x000000)
        yield from gsleep(1000)


def check_floor():
    # 정지(도착) 상태 측정 — 빨강/파랑/은색만 5회 다수결로 감지(노이즈 제거).
    # 검정(구멍)은 주행 중 drive()에서 처리하고, 흰색(일반)은 무시한다.
    # red(위험구역 입구)/silver(체크포인트)는 색 표시만, blue(웅덩이)는 5초 정지.
    if color is None:
        return
    name = color.detect_stop()
    if name is None:
        return
    M5.Lcd.fillScreen(TILE_RGB.get(name, 0x000000))
    print("floor:", name, color.MEANING.get(name, "?"))
    if name == "blue":
        yield from blue_stop()

# -----------------------------------------------------------------------
# 그리드 계산
# -----------------------------------------------------------------------

def get_open_side(gx, gy):
    # 실제 ToF로 좌/전/우 개방 판정. 전/후·좌/우 물리 swap 반영:
    #   not reversed: 전방=tof4, 우=tof3, 좌=tof2
    #   reversed:     전방=tof1, 논리좌=tof3, 논리우=tof2
    # try:
    print("get_open_side")

    for _ in range(3):
        pahub.get([tof4, tof3, tof2, tof1])

    if not is_reversed:
        d_f, d_r, d_l = pahub.get([tof4, tof3, tof2])
    else:
        d_f, d_l, d_r = pahub.get([tof1, tof3, tof2])
    print(d_f, d_r, d_l)
    sides = [int(d_l > WALL_MM), int(d_f > WALL_MM), int(d_r > WALL_MM)]

    for i in range(3):
        nb = direction(gx + next_dir(heading, i).x, gy + next_dir(heading, i).y)
        if nb in closedList:
            sides[i] = 0
        elif sides[i] and next_dir(heading, i) in openList:
            openList.remove(next_dir(heading, i))

    return sides


def sides_to_wall(sides, isFirst=False):
    # isFirst=True: 시작 셀은 후면이 항상 벽(막힌 면에 붙여 출발)
    backward = 0 if isFirst else 1
    n = s = w = e = 0
    if   heading == 0: n, s, w, e = sides[1], backward, sides[0], sides[2]
    elif heading == 1: n, s, w, e = sides[0], sides[2], backward, sides[1]
    elif heading == 2: n, s, w, e = backward, sides[1], sides[2], sides[0]
    elif heading == 3: n, s, w, e = sides[2], sides[0], sides[1], backward
    # 시각적 레이아웃: bit0=N벽, bit1=S벽, bit2=W벽, bit3=E벽 (1=벽 있음, 0=통로)
    return (1-n) | ((1-s) << 1) | ((1-w) << 2) | ((1-e) << 3)


def check_open_dir(gx, gy, isFirst=False):
    sides = get_open_side(gx, gy)
    return sides, sides_to_wall(sides, isFirst)


def _expand_grid(grid, _node, dir):
    dx = dy = tx = ty = gx = gy = 0
    old_h, old_w = len(grid), len(grid[0])
    if dir.x != 0:
        tx = -dir.x
        if not (0 <= _node.x + dir.x < old_w):
            gx = 1
            if dir.x < 0: dx = 1
    if dir.y != 0:
        ty = -dir.y
        if not (0 <= _node.y + dir.y < old_h):
            gy = 1
            if dir.y < 0: dy = 1
    return dx, dy, tx, ty, gx, gy


def _build_grid(grid, dx, dy, gx, gy):
    tGrid = [[NNode() for _ in range(len(grid[0]) + gx)]
             for _ in range(len(grid) + gy)]
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            n = grid[y][x]
            n.set_xy(n.x + dx, n.y + dy)
            tGrid[y + dy][x + dx] = n
    return tGrid


def modify_grid(_node, dir, grid):
    global openList, closedList

    dx, dy, tx, ty, gx, gy = _expand_grid(grid, _node, dir)

    openList   = [direction(e[0] + tx, e[1] + ty) for e in openList]
    closedList = [direction(e[0] + dx, e[1] + dy) for e in closedList]

    nx, ny = _node.x + dir.x + dx, _node.y + dir.y + dy
    sides, _wall = check_open_dir(nx, ny)

    _node = Node(nx, ny, _wall, None)
    tGrid = _build_grid(grid, dx, dy, gx, gy)
    tGrid[_node.y][_node.x] = _node

    for i, cond in enumerate(sides):
        if cond:
            openList.append(next_dir(heading, i))
    closedList.append(direction(_node.x, _node.y))

    return _node, tGrid, dx, dy


def follow_next_dir(_dir, _node, _grid):
    isGriding = True
    _node = _node.copy()
    try:
        _grid[_node.pos(_dir)[1]][_node.pos(_dir)[0]]
    except Exception:
        isGriding = False

    dirs = get_directions(_grid, dir_to_pos(_node.pos()), dir_to_pos(_node.pos(_dir)), isGriding)
    yield from gsleep(200)

    for __dir in dirs:
        if __dir in (direction(0,2), direction(0,-2), direction(2,0), direction(-2,0)):
            half = direction(__dir.x // 2, __dir.y // 2)
            _node.add_xy(half.x, half.y)
            _dir = add_dir(_dir, -half.x, -half.y)
            openList[:] = [add_dir(e, -half.x, -half.y) for e in openList]
            create_grid(grid_to_int(_grid), direction(_node.x, _node.y), heading)
            continue

        yield from turn(look_dir(heading, __dir))
        yield from gsleep(200)
        yield from drive(150, 300)
        yield from check_floor()
        yield from gsleep(200)

        _node.add_xy(__dir.x, __dir.y)
        _dir = add_dir(_dir, -__dir.x, -__dir.y)
        openList[:] = [add_dir(e, -__dir.x, -__dir.y) for e in openList]
        create_grid(grid_to_int(_grid), direction(_node.x, _node.y), heading)

    return _dir, _node


def grid_to_int(grid):
    return [[n.wall for n in row] for row in grid]


def dir_to_pos(d):
    return [d.x, d.y]


def add_dir(d, x, y):
    return direction(d.x + x, d.y + y)

# -----------------------------------------------------------------------
# 메인 탐색 루프
# -----------------------------------------------------------------------

def start_program():
    global pos, startPos, checkpoint, openList, closedList, heading, gyroAngle

    if checkpoint["grid"] is None:
        pos = startPos = [0, 0]
        openList, closedList = [], [direction(0, 0)]
        heading = gyroAngle = 0
        yield from reset_turn()

        sides, wall = check_open_dir(0, 0, True)
        openList = [next_dir(heading, i) for i, ok in enumerate(sides) if ok]

        node = Node(0, 0, wall, None)
        grid = [[node]]
        dir  = openList.pop()
        create_grid(grid_to_int(grid), direction(node.x, node.y), heading)
    else:
        node       = checkpoint["node"]
        dir        = checkpoint["dir"]
        grid       = checkpoint["grid"]
        openList   = checkpoint["openList"]
        closedList = checkpoint["closedList"]
        startPos   = checkpoint["startPos"]
        gyroAngle  = checkpoint["gyroAngle"]
        heading    = checkpoint["heading"]
        yield from reset_turn()

    while True:
        # checkpoint["grid"]      = grid
        # checkpoint["node"]      = node
        # checkpoint["dir"]       = dir
        # checkpoint["openList"]  = list(openList)
        # checkpoint["closedList"]= list(closedList)
        # checkpoint["startPos"]  = list(startPos)
        # checkpoint["gyroAngle"] = gyroAngle
        # checkpoint["heading"]   = heading

        if dir not in (direction(0,-1), direction(0,1), direction(-1,0), direction(1,0)):
            dir, node = yield from follow_next_dir(dir, node, grid)
        elif node.wall in (7, 11, 13, 14):
            one_exit = {7: direction(1,0), 11: direction(-1,0),
                        13: direction(0,1), 14: direction(0,-1)}[node.wall]
            if dir != one_exit:
                dir, node = yield from follow_next_dir(dir, node, grid)

        yield from turn(look_dir(heading, dir))

        yield from drive(150, 300)
        yield from check_floor()

        yield from gsleep(200)

        node, grid, dx, dy = modify_grid(node, dir, grid)
        pos = node.pos()
        startPos[0] += dx
        startPos[1] += dy

        create_grid(grid_to_int(grid), direction(node.x, node.y), heading)

        if not openList:
            break

        dir = openList.pop()
        yield

    print("탐색 완료")
    yield from follow_next_dir(
        direction(startPos[0] - node.x, startPos[1] - node.y), node, grid
    )


# -----------------------------------------------------------------------
# 엔트리포인트
# -----------------------------------------------------------------------

_touch_result = None

def _wait_touch():
    global _touch_result
    _touch_result = yield from wait_touch(hold_threshold=2000, detect_hold=True)


def main():
    try:
        # I2C 장치 응답 확인 — 0x22=BASEX, 0x38=FT6336, 0x69=BMI270
        yield from gsleep(1000)
        print("scan:", _i2c.scan())

        # ToF(id=1) 초기화 — PaHub I2C 컨트롤러 생성 후 센서 생성/start.
        # 읽기는 get_open_side/drive에서 그때그때 직접 한다(폴링 스레드 없음).
        global tof1, tof2, tof3, tof4, color
        pahub.start()
        # 2점 보정: 각 센서를 25mm·150mm 벽에 대고 raw를 측정해 offset_25/offset_150에 채운다.
        # 미측정이면 기본값(25/150)이라 raw 그대로 나온다 → 측정 후 값만 바꾸면 자동 보정.
        tof1 = VL53L0X(pahub, channel=5, offset_25=56,  offset_150=190)  # 정면
        tof2 = VL53L0X(pahub, channel=4, offset_25=92,  offset_150=221)  # 우측
        tof3 = VL53L0X(pahub, channel=0, offset_25=120, offset_150=245)  # 좌측
        tof4 = VL53L0X(pahub, channel=1, offset_25=81,  offset_150=225)  # 후면
        tof1.start()
        tof2.start()
        tof3.start()
        tof4.start()
        # 바닥 컬러센서 — PaHub 같은 버스 채널 2. 생성 시 _init()이 채널 전환 후 자동 실행.
        color = ColorSensor(pahub, channel=COLOR_CH)
        print("init 완료")

        imu.reset_angle()
        # await imu.calibrate_with_mag()  # 시작 방향을 지자계 오프셋으로 저장 (_mag_initial)



        # 콜드부팅 대응 — 첫 터치를 읽기 "직전"에 FT6336을 한 번 더 리셋한다.
        # Touch.__init__의 리셋 이후 pahub.start()/ToF init 과정에서 FT6336이 다시
        # 죽는 경우가 있어, 진단(touch_aw9523_test.py)에서 검증된 "읽기 직전 리셋" 순서를
        # 여기서 재현한다. reset_cores3_touch 내부에 부팅 대기(300ms)가 포함돼 있다.
        reset_cores3_touch(_i2c)

        # 첫 터치로 탐색 시작 — 화면 색으로 터치 상태 표시(시리얼 없이 눈으로 확인).
        #   준비(대기)=빨강, 터치 중=흰색, 터치 끝(시작)=검정
        M5.Lcd.fillScreen(0xFF0000)
        _was = False
        while True:
            _t = touch.is_touching()
            if _t and not _was:
                M5.Lcd.fillScreen(0xFFFFFF)   # 누르는 동안 흰색
            elif _was and not _t:
                M5.Lcd.fillScreen(0x000000)   # 떼면 검정 → 탐색 시작
                break
            _was = _t
            yield from gsleep(10)
        print("터치 감지 — 탐색 시작")

        while True:
            # start_program 과 터치 감시를 제너레이터 협력으로 동시 구동.
            # 먼저 끝난 쪽 인덱스를 받는다(0=탐색 완료, 1=터치 중단). 나머지는 grace가 close.
            idx = grace(start_program(), _wait_touch())
            # idx = grace(drive(200, 300), _wait_touch())

            if idx == 0:
                # 탐색 정상 완료
                break

            # 터치로 중단 — 결과에 따라 재시작 방식 결정
            result = _touch_result
            print("touch result:", result)

            if result == 2:  # 길게 누름: 저장 데이터 초기화 후 재시작
                checkpoint["grid"] = None
            # 짧게 누름(1): 체크포인트부터 재시작 — 루프 계속

    except Exception as e:
        log_error(e, "tof_f_main")
        print("오류:", e)
    finally:
        robot.close()


try:
    grun(main())
    # grun(drive(200, 300))
except KeyboardInterrupt:
    robot.stop(lock=False)
