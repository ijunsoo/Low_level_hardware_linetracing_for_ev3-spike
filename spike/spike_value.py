from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch

# =============================================================
#  spike_value: 3센서 빛값(VALUE) 측정 → spike_lf3_rule.py / spike_lf3_rl.py에 붙여넣기
#
#  사용: 실행 → 라인 위에 놓고 왼쪽 버튼 → 빨간불 4초 동안 로봇을 손으로 들지 말고
#        바닥에 댄 채 라인을 가로질러 좌우로 천천히 여러 번 쓸어 주세요
#        (센서 3개가 모두 라인을 완전히 지나가게) → 초록불 → 출력된 VALUE 줄을
#        파일의 VALUE = [...] 줄과 바꾸기. 경기장 매트·조명에서 잴 것.
#  포트는 spike_lf3_rule.py와 같음: 좌 C, 우 B, 가운데 A
#  흰 = 밝은 쪽 기록들의 중앙값(전형적인 흰 바닥), 검 = 하위 3% 값 — 흰색을 너무 높게 잡으면
#       실제 흰 바닥이 '흰색 아님'으로 읽혀 이탈을 늦게 알아챔 (실기 2026-10-24)
# =============================================================

hub = PrimeHub()
SENS = (PUPDevice(Port.C), PUPDevice(Port.B), PUPDevice(Port.A))     # 좌, 우, 가운데
NAMES = ("좌", "우", "가운데")
MS = 4000
RAW_MODE = False    # ★spike_lf3_rule.py에서 RAW_MODE = True로 쓸 거면 여기도 True (모드 3 REF-RAW, 0~1020)
TOP = 1023 if RAW_MODE else 100
STEP = 4 if RAW_MODE else 1          # raw는 4카운트 칸으로 모아 셈 (메모리 절약)

print("# 라인 위에 놓고 왼쪽 버튼 → 빨간불 동안 라인을 가로질러 좌우로 쓸기")
while Button.LEFT not in hub.buttons.pressed():
    wait(10)
while hub.buttons.pressed():
    wait(10)
hub.light.on(Color.RED)
hist = [[0] * (TOP // STEP + 1) for _ in SENS]
n = 0
sw = StopWatch()
while sw.time() < MS:
    for i in range(3):
        if RAW_MODE:
            a = SENS[i].read(3)
            v = abs(a[0] - a[1])
        else:
            v = SENS[i].read(0)[0]
        v = 0 if v < 0 else (TOP if v > TOP else v)
        hist[i][v // STEP] += 1
    n += 1
hub.light.on(Color.GREEN)


def pct(h, q):
    # 누적 q/1000 지점의 값
    k = n * q // 1000
    c = 0
    for v in range(len(h)):
        c += h[v]
        if c > k:
            return v * STEP
    return TOP


out = []
ok = True
for i in range(3):
    hi = pct(hist[i], 970)
    b = pct(hist[i], 30)
    mid = (hi + b) // 2
    h = hist[i]
    k0 = mid // STEP
    tot = sum(h[k0:])                              # 흰색 = 중간값보다 밝은 기록들의 중앙값 (전형적인 흰 바닥)
    c = 0
    w = hi
    for k in range(k0, len(h)):
        c += h[k]
        if c * 2 >= tot:
            w = k * STEP
            break
    print(NAMES[i], "흰", w, "검", b, "대비", w - b)
    if w - b < (60 if RAW_MODE else 15):
        ok = False
    out += [w, b]
print("VALUE = [%d, %d, %d, %d, %d, %d]" % tuple(out) + ("   # RAW_MODE = True 용" if RAW_MODE else ""))
print("# 참고: 모드 0에서 흰색이 40 미만이면 센서가 바닥에서 너무 높음 — 8~12mm로 내리고 빛 가리개를 달면 대비가 커짐")
if not ok:
    print("# 주의: 대비가 15 미만인 센서가 있음 — 라인을 다 못 지났거나 센서가 너무 높음. 다시 재기")
