from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Button, Color, Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# 장치설정
hub = PrimeHub()
lm = Motor(Port.E, Direction.COUNTERCLOCKWISE)
rm = Motor(Port.F)
c1 = PUPDevice(Port.A)    # 좌측부터 1 컬러센서
c2 = PUPDevice(Port.C)    # 2 컬러센서
c3 = PUPDevice(Port.B)    # 3 컬러센서
c4 = PUPDevice(Port.D)    # 4 컬러센서

# 함수
def normalzing(value, w_offset):
    # value 리스트에서 흰색/검정색 기준값 분리
    white_ref = value[::2]    # [value[0], value[2], value[4], value[6]]
    black_ref = value[1::2]   # [value[1], value[3], value[5], value[7]]

    # EMA 필터 변수 초기화 -> 0 < alpha < 1
    alpha = 0.3
    prev_s = [c1.read(mode=0)[0], c2.read(mode=0)[0], c3.read(mode=0)[0], c4.read(mode=0)[0]]

    while True:
        # 센서값 읽기
        raw = [c1.read(mode=0)[0], c2.read(mode=0)[0], c3.read(mode=0)[0], c4.read(mode=0)[0]]

        # Clamp: calibration 범위(black_ref[i] ~ white_ref[i])로 제한
        clamped = [
            min(max(raw[i], black_ref[i]), white_ref[i])
            for i in range(4)
        ]

        # EMA 적용
        filtered = [
            alpha * clamped[i] + (1 - alpha) * prev_s[i]
            for i in range(4)
        ]
        # 소수점 1자리 반올림
        filtered = [int(f) for f in filtered]
        prev_s = filtered.copy()
        s1, s2, s3, s4 = filtered
        w1 = value[0]-w_offset; w2 = value[2]-w_offset
        w3 = value[4]-w_offset; w4 = value[6]-w_offset
        print(w1, w2, w3, w4)
        if s1>w1 and s2>w2 and s3>w3 and s4>w4:
            hub.speaker.beep(500, 10)
        # print(s1, s2, s3, s4)
        wait(100)

# sensor_input.py 파일의 실행결과를 복사해서 붙혀넣는다
value = [64, 9, 52, 7, 53, 6, 61, 8]

# 네개의 센서 중 하나라도 검은선 위에 있는 경우 비프음이 울리지 않고 
# 센서 모두 흰색에 있을때 비프음이 울리는 가장 큰 값을 찾는다
w_offset = 17
# 메인프로그램
normalzing(value, w_offset)