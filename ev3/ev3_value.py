from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch

# =============================================================
#  ev3_value: 3센서 빛값(VALUE) 측정 → ev3_lf3_rule.py / ev3_lf3_rl.py에 붙여넣기
#
#  사용: 실행 → 라인 위에 놓고 왼쪽 버튼 → 빨간불 4초 동안 로봇을 바닥에 댄 채
#        라인을 가로질러 좌우로 천천히 여러 번 쓸어 주세요 (센서 3개가 모두 라인을
#        완전히 지나가게) → 초록불 → 출력된 VALUE 줄을 파일의 VALUE = [...] 줄과 바꾸기.
#        경기장 매트·조명에서 잴 것.
#  포트 = 사용자 EV3 로봇(2026-10-24, ev3_lf3_rule_e8.py): 좌 S1, 가운데 S2, 우 S3
#  (예전 ev3_lf3_rule.py는 좌 S1·우 S2·가운데 S3 가정 — 그 파일엔 이 VALUE 순서가 안 맞음)
#  흰 = 상위 3% 값, 검 = 하위 3% 값 (튀는 값 한두 개에 흔들리지 않게)
# =============================================================

hub = EV3Brick()
SENS = (ColorSensor(Port.S1), ColorSensor(Port.S3), ColorSensor(Port.S2))   # 좌 S1, 우 S3, 가운데 S2 (출력 순서 = VALUE 순서)
NAMES = ("좌", "우", "가운데")
MS = 4000

print("# 라인 위에 놓고 왼쪽 버튼 → 빨간불 동안 라인을 가로질러 좌우로 쓸기")
while Button.LEFT not in hub.buttons.pressed():
    wait(10)
while hub.buttons.pressed():
    wait(10)
hub.light.on(Color.RED)
hist = [[0] * 101 for _ in SENS]
n = 0
sw = StopWatch()
while sw.time() < MS:
    for i in range(3):
        v = int(SENS[i].reflection())      # EV3 펌웨어에 따라 실수(34.0)로 나옴 → 정수로
        hist[i][0 if v < 0 else (100 if v > 100 else v)] += 1
    n += 1
hub.light.on(Color.GREEN)


def pct(h, q):
    # 누적 q/1000 지점의 값
    k = n * q // 1000
    c = 0
    for v in range(101):
        c += h[v]
        if c > k:
            return v
    return 100


out = []
ok = True
for i in range(3):
    w = pct(hist[i], 970)
    b = pct(hist[i], 30)
    print(NAMES[i], "흰", w, "검", b, "대비", w - b)
    if w - b < 15:
        ok = False
    out += [w, b]
print("VALUE = [%d, %d, %d, %d, %d, %d]" % tuple(out))
if not ok:
    print("# 주의: 대비가 15 미만인 센서가 있음 — 라인을 다 못 지났거나 센서가 너무 높음. 다시 재기")
