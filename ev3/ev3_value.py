from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch
try:
    from pybricks.iodevices import LUMPDevice   # 빨간 불: 센서 모드를 직접 골라 읽음
except ImportError:
    LUMPDevice = None

# =============================================================
#  ev3_value: EV3 3센서 빛값 측정 → ev3_final.py에 붙여 넣기
#
#  사용
#   1) 실행 → 로봇을 라인 위에 놓고 왼쪽 버튼
#   2) 브릭 빨간 불 4초 동안, 로봇을 바닥에 댄 채 라인을 가로질러 좌우로 천천히 여러 번 쓸기
#      (센서 3개가 모두 라인을 완전히 지나가게)
#   3) 초록 불 → 콘솔과 브릭 화면에 VALUE 줄이 나옴 → ev3_final.py 맨 아래 line_trace(VALUE=...)에
#      아무 버튼이나 누르면 끝
#  RED_LIGHT는 ev3_final.py와 같게 맞출 것
#   False(기본) = RGB 반사광: ColorSensor.reflection() (Pybricks 4.1: 빨강·초록·파랑 세 색 반사광의 평균)
#   True = 빨간 불 반사광: 주황 불 4초 동안 한 번 더 쓸어서 VALUE_RED(raw 모드 3)·VALUE0(반사광 % 모드 0) 두 줄
#          → ev3_final.py 고급 설정의 VALUE_RED·VALUE0 줄에
#  경기장 매트·조명에서 잴 것. 포트: 좌 S1 · 가운데 S2 · 우 S3 (출력 순서 = 좌, 우, 가운데)
#  흰 = 밝은 쪽 기록들의 중앙값(전형적인 흰 바닥), 검 = 하위 3% 값
#  (예전처럼 흰 = 상위 3%로 잡으면 흰색이 실제 바닥보다 높아져, 흰 바닥을 '흰색 아님'으로 읽고
#   라인 이탈을 늦게 알아챔 — SPIKE 실기 2026-10-24, spike_value.py와 같은 방식)
# =============================================================

RED_LIGHT = False

ev3 = EV3Brick()
MS = 4000
NAMES = ("좌", "우", "가운데")
RED = RED_LIGHT and LUMPDevice is not None     # 펌웨어에 LUMPDevice가 없으면 RGB로
if RED:
    SENS = (LUMPDevice(Port.S1), LUMPDevice(Port.S3), LUMPDevice(Port.S2))     # 좌, 우, 가운데
else:
    SENS = (ColorSensor(Port.S1), ColorSensor(Port.S3), ColorSensor(Port.S2))


def show(*a):
    # 브릭 화면에 글자 표시 (화면 기능이 없으면 그냥 넘어감)
    try:
        ev3.screen.print(*a)
    except Exception:
        pass


def read(i, raw):
    if not RED:
        return int(SENS[i].reflection())    # RGB 반사광 % (ev3_final.py와 같게 정수로)
    if raw:
        a = SENS[i].read(3)                 # 모드 3 raw 반사광: 두 값의 차 (SPIKE와 같은 방식)
        return abs(a[0] - a[1])
    return SENS[i].read(0)[0]               # 모드 0 반사광 %


def measure(raw):
    top = 1023 if raw else 100
    tag = "(빨간 불 raw)" if raw else ("(빨간 불 %)" if RED else "(RGB %)")
    for i in range(3):                      # 모드를 바꾼 직후 값은 버림
        read(i, raw)
    wait(100)
    hist = [[0] * (top + 1) for _ in SENS]
    n = 0
    sw = StopWatch()
    while sw.time() < MS:
        for i in range(3):
            v = read(i, raw)
            hist[i][0 if v < 0 else (top if v > top else v)] += 1
        n += 1

    def pct(h, q):                          # 누적 q/1000 지점의 값
        k = n * q // 1000
        c = 0
        for v in range(len(h)):
            c += h[v]
            if c > k:
                return v
        return top

    out = []
    ok = True
    for i in range(3):
        h = hist[i]
        hi = pct(h, 970)
        b = pct(h, 30)
        k0 = (hi + b) // 2
        tot = sum(h[k0:])                   # 흰색 = 중간값보다 밝은 기록들의 중앙값
        c = 0
        w = hi
        for k in range(k0, len(h)):
            c += h[k]
            if c * 2 >= tot:
                w = k
                break
        print(NAMES[i], "흰", w, "검", b, "대비", w - b, tag)
        if w - b < (45 if raw else 15):     # raw 45 ≈ 반사광 % 15 (SPIKE 실측 raw ≈ % × 3)
            ok = False
        out += [w, b]
    return out, ok


print("# 라인 위에 놓고 왼쪽 버튼 → 브릭 빨간 불" + ("·주황 불" if RED else "") + " 동안 라인을 가로질러 좌우로 쓸기")
if RED_LIGHT and not RED:
    print("# 펌웨어에 LUMPDevice 없음 → RGB 반사광으로 잼 (ev3_final.py도 RGB로 달림)")
show("LEFT = START")
while Button.LEFT not in ev3.buttons.pressed():
    wait(10)
while ev3.buttons.pressed():
    wait(10)

if RED:
    ev3.light.on(Color.RED)
    show("SWEEP 1/2 (red raw)")
    value, ok1 = measure(True)
    ev3.light.on(Color.ORANGE)
    show("SWEEP 2/2 (red %)")
    value0, ok2 = measure(False)
    name, where = "VALUE_RED", "고급 설정 VALUE_RED 줄에"
else:
    ev3.light.on(Color.RED)
    show("SWEEP (RGB)")
    value, ok1 = measure(False)
    value0, ok2 = None, True
    name, where = "VALUE", "맨 아래 line_trace(VALUE=...)에"
ev3.light.on(Color.GREEN)

print("%s = [%d, %d, %d, %d, %d, %d]   # ev3_final.py %s" % ((name,) + tuple(value) + (where,)))
if value0:
    print("VALUE0 = [%d, %d, %d, %d, %d, %d]  # ev3_final.py 고급 설정 VALUE0 줄에" % tuple(value0))
if not (ok1 and ok2):
    print("# 대비가 작음: 센서 3개가 라인을 다 지나가게 쓸었는지, 센서 높이(바닥에서 8~12mm)를 확인")
show("DONE")
show(name, ",".join(str(x) for x in value))
if value0:
    show("VALUE0", ",".join(str(x) for x in value0))
if not (ok1 and ok2):
    show("LOW CONTRAST")
while not ev3.buttons.pressed():            # 화면을 보고 적을 수 있게 버튼을 누를 때까지 기다림
    wait(10)
