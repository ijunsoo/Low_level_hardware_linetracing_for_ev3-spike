from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Color, Port
from pybricks.tools import wait

# =============================================================
#  spike_hub_test: 허브가 코드를 실행하는지 + 배터리 + 포트별 장치 확인 (2026-10-24)
#
#  실행되면: 초록 불 → 화면 H → 삑 소리. 콘솔(PC 앱)에 배터리 전압과 포트 A~F 상태가 나옴.
#  이것도 아무 반응이 없으면 코드 문제가 아니라 허브(배터리·펌웨어) 문제.
#  E·F = 모터, A·B·C = 컬러센서(가운데·우·좌)가 '장치 있음'으로 나와야 정상
# =============================================================

hub = PrimeHub()
hub.light.on(Color.GREEN)
hub.display.char("H")
hub.speaker.beep()
v = hub.battery.voltage()
print("허브 동작 OK — 배터리", v, "mV", "(7200 아래면 충전)" if v < 7200 else "")
for p, name in ((Port.A, "A 가운데 센서"), (Port.B, "B 우 센서"), (Port.C, "C 좌 센서"),
                (Port.D, "D"), (Port.E, "E 좌 모터"), (Port.F, "F 우 모터")):
    try:
        d = PUPDevice(p)
        print(name, ": 장치 있음 (id", d.info()["id"], ")")
    except OSError:
        print(name, ": 비어 있음 또는 응답 없음")
    wait(100)
hub.display.char("K")
print("끝 — 화면 K")
wait(3000)
