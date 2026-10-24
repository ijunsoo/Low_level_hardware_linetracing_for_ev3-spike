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

# 반사광강도 측정 함수
def detect_reflect():
    reflect_value = [0] * 8
    # 백색 측정 - 왼쪽버튼 눌렀다 땜
    while not Button.LEFT in hub.buttons.pressed():
        pass
    hub.speaker.beep()
    set_1=set(); set_2=set(); set_3=set(); set_4=set()
    while Button.LEFT in hub.buttons.pressed():
        set_1.add(c1.read(mode=0)[0])
        set_2.add(c2.read(mode=0)[0])
        set_3.add(c3.read(mode=0)[0])
        set_4.add(c4.read(mode=0)[0])
    reflect_value[0] = min(set_1)
    reflect_value[2] = min(set_2)
    reflect_value[4] = min(set_3)
    reflect_value[6] = min(set_4)

    # 검은선 측정 - 오른쪽버튼 눌렀다 땜
    while not Button.RIGHT in hub.buttons.pressed():
        pass
    hub.speaker.beep(700)
    set_1=set(); set_2=set(); set_3=set(); set_4=set()
    while Button.RIGHT in hub.buttons.pressed():
        set_1.add(c1.read(mode=0)[0])
        set_2.add(c2.read(mode=0)[0])
        set_3.add(c3.read(mode=0)[0])
        set_4.add(c4.read(mode=0)[0])
    reflect_value[1] = min(set_1)
    reflect_value[3] = min(set_2)
    reflect_value[5] = min(set_3)
    reflect_value[7] = min(set_4)
    
    print(reflect_value)

detect_reflect()