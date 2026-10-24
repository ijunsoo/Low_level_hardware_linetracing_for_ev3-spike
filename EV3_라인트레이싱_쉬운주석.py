#!/usr/bin/env pybricks-micropython
# ↑ 이 줄은 "이 파일은 EV3 로봇용 파이썬 프로그램이에요"라고 알려주는 약속이에요.

# ================================================================
# 준비물 가져오기 (요리하기 전에 재료를 꺼내는 것과 같아요)
# pybricks라는 도구 상자에서 모터, 센서 같은 부품을 다루는 도구를 꺼내요.
# ================================================================
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# EV3 본체(로봇의 뇌)를 준비하고, "삑" 소리를 내서 시작을 알려요.
ev3 = EV3Brick()
ev3.speaker.beep()

######################################### device setting #########################################
# 로봇에 붙어있는 부품(모터, 센서)이 어느 구멍(포트)에 꽂혀 있는지 알려주는 부분이에요.
'''
좌상(lmu, A, +), 좌하(lmd, B, -), 우상(rmu, C, -), 우하(rmd, D, +)
'''
# 이 로봇은 바퀴가 4개라서 모터도 4개예요. (왼쪽 2개 + 오른쪽 2개)
# Direction.COUNTERCLOCKWISE는 "이 모터는 반대로 돌려줘"라는 뜻이에요.
# 모터가 꽂힌 방향이 서로 달라서, 이렇게 맞춰줘야 다 같이 앞으로 가요.
lmu = Motor(Port.A)                             # 왼쪽 위 모터
lmd = Motor(Port.B, Direction.COUNTERCLOCKWISE) # 왼쪽 아래 모터
rmu = Motor(Port.C, Direction.COUNTERCLOCKWISE) # 오른쪽 위 모터
rmd = Motor(Port.D)                             # 오른쪽 아래 모터

# 컬러센서 3개: 바닥을 내려다보면서 "얼마나 밝은지(하얀지 검은지)"를 숫자로 알려줘요.
lc = ColorSensor(Port.S1)                       # 왼쪽 눈 (왼쪽 컬러센서)
cc = ColorSensor(Port.S2)                       # 가운데 눈 (중앙 컬러센서)
rc = ColorSensor(Port.S3)                       # 오른쪽 눈 (오른쪽 컬러센서)
# ul = UltrasonicSensor(Port.S4)                # 초음파센서 (지금은 안 써서 꺼둠)

############################################ function ############################################
# 자주 쓰는 동작들을 "함수"라는 이름표 붙은 상자에 담아두는 부분이에요.
# 이름만 부르면 상자 안의 동작이 실행돼요.

def normalizing(reflect_list):
    # [센서 점수 바꾸기 함수]
    # 센서마다 보는 눈이 조금씩 달라요. 어떤 센서는 흰색을 49라고 하고,
    # 어떤 센서는 64라고 해요. 그래서 셋 다 똑같은 기준으로 바꿔줘요:
    #   "가장 검을 때 = 0점, 가장 하얄 때 = 100점"
    # 계산 방법: (지금 값 - 검은색 값) ÷ (흰색 값 - 검은색 값) × 100
    # 이렇게 하면 세 센서를 공평하게 비교할 수 있어요.
    lv = int((lc.reflection() - reflect_list[1]) / (reflect_list[0] - reflect_list[1])*100)  # 왼쪽 점수
    cv = int((cc.reflection() - reflect_list[3]) / (reflect_list[2] - reflect_list[3])*100)  # 가운데 점수
    rv = int((rc.reflection() - reflect_list[5]) / (reflect_list[4] - reflect_list[5])*100)  # 오른쪽 점수
    return lv, cv, rv   # 세 점수를 돌려줘요

def lm(power):
    # 왼쪽 모터 2개를 같은 힘(power)으로 돌려요. (왼쪽 바퀴 담당)
    lmu.dc(power)
    lmd.dc(power)

def rm(power):
    # 오른쪽 모터 2개를 같은 힘(power)으로 돌려요. (오른쪽 바퀴 담당)
    rmu.dc(power)
    rmd.dc(power)

def stop():
    # 모터 4개를 모두 꽉 잡아서 멈춰요. (브레이크!)
    # 확실하게 멈추려고 5번 반복해서 브레이크를 걸어요.
    for i in range(5):
        lmu.stop(Stop.BRAKE)
        lmd.stop(Stop.BRAKE)
        rmu.stop(Stop.BRAKE)
        rmd.stop(Stop.BRAKE)

def steering_drive(power, steering):
    # [핸들 함수] 자동차 핸들처럼 로봇을 조종해요.
    #   power    = 얼마나 빨리 갈지 (속도)
    #   steering = 핸들을 얼마나 꺾을지 (-100 ~ 100)
    #     0    → 직진
    #     100  → 오른쪽으로 제자리 돌기 (오른쪽 바퀴가 뒤로 돌아요)
    #     -100 → 왼쪽으로 제자리 돌기
    # 원리: 오른쪽으로 돌고 싶으면 오른쪽 바퀴를 천천히, 왼쪽 바퀴를 빨리 돌리면 돼요.
    steering = max(-100, min(100, steering))   # 핸들 값이 -100~100을 넘지 않게 잘라줘요
    if -100 > steering:
        st_B = -power
        st_C = power
    elif -100 <= steering < 0:
        # 왼쪽으로 꺾기: 오른쪽 바퀴는 그대로, 왼쪽 바퀴만 천천히
        st_B = ((power * 2) / 100) * steering + power
        st_C = power
    elif 0 == steering:
        # 핸들이 가운데: 양쪽 바퀴 똑같이 → 직진
        st_B = power
        st_C = power
    elif 0 < steering <= 100:
        # 오른쪽으로 꺾기: 왼쪽 바퀴는 그대로, 오른쪽 바퀴만 천천히
        st_B = power
        st_C = (-(power * 2) / 100) * steering + power
    elif 100 < steering:
        st_B = power
        st_C = -power
    # st_B = 왼쪽 바퀴 힘, st_C = 오른쪽 바퀴 힘
    lm(st_B)
    rm(st_C)

# 라인이 마지막으로 어느 쪽에 있었는지 기억하는 메모장이에요. ('l'=왼쪽, 'r'=오른쪽)
line = ''

def pid_control(st, speed, kp, kd, dec):
    # [라인 따라가기 함수 - 이 프로그램의 심장!]
    # 검은 선을 따라가도록 핸들을 계속 조금씩 조절해요.
    #   st    = 선을 놓쳤을 때 꺾을 핸들 크기
    #   speed = 달리는 속도
    #   kp    = 선에서 벗어난 만큼 핸들을 꺾는 세기 (많이 벗어나면 많이 꺾어요)
    #   kd    = 갑자기 벗어나기 시작하면 미리 꺾는 세기 (덜컹거림을 줄여줘요)
    #   dec   = 선을 놓쳤을 때 속도를 얼마나 줄일지
    global last_error, line   # 함수 밖에 있는 메모장(변수)도 쓰겠다는 뜻이에요
    respeed = speed

    # 1단계: 세 눈(센서)의 밝기 점수를 읽어요. (0=검정, 100=흰색)
    lv, cv, rv = normalizing(reflect_list)

    # 2단계: 얼마나 벗어났는지 계산해요.
    # error = 왼쪽 점수 - 오른쪽 점수 (가운데는 양쪽에 다 들어가서 서로 지워져요)
    #   error가 0    → 선이 정확히 가운데! 그대로 직진
    #   error가 +    → 왼쪽이 더 하얗다 = 선이 오른쪽에 있다 → 오른쪽으로 꺾자
    #   error가 -    → 오른쪽이 더 하얗다 = 선이 왼쪽에 있다 → 왼쪽으로 꺾자
    error = (lv + cv) - (cv + rv)

    # 3단계: 아까보다 얼마나 "빠르게" 벗어나는 중인지도 계산해요.
    # (자전거 탈 때 넘어지려고 하면 미리 핸들을 트는 것과 같아요)
    differential = error - last_error

    # 4단계: 핸들을 얼마나 꺾을지 정해요.
    # (벗어난 정도 × kp) + (벗어나는 빠르기 × kd)
    steering = (kp * error) + (kd * differential)

    # 5단계: 선을 완전히 놓쳤는지 확인해요.
    # 세 눈이 전부 76점보다 밝으면 = 셋 다 흰 바닥만 보고 있다 = 선을 놓쳤다!
    if lv > 76 and cv > 76 and rv > 76:
        respeed = respeed - dec   # 속도를 살짝 줄이고
        # 선이 마지막으로 어느 쪽에 있었는지 기억을 보고, 그쪽으로 크게 꺾어서 다시 찾아요.
        if line == 'l':
            steering = -st        # 왼쪽에 있었으면 왼쪽으로!
        else:
            steering = st         # 오른쪽에 있었으면 오른쪽으로!
    else:
        respeed = speed           # 선이 보이면 원래 속도로 달려요

    # 6단계: 정한 속도와 핸들 값으로 실제로 달려요!
    steering_drive(respeed, steering)

    # 7단계: 지금 선이 어느 쪽에 있는지 메모해둬요. (나중에 놓쳤을 때 쓰려고)
    # 점수가 47보다 낮다 = 그 눈이 검은 선을 보고 있다는 뜻이에요.
    if lv < 47 or rv < 47:
        if lv < 47:
            line = 'l'   # "선이 왼쪽에 있어!" 라고 메모
        else:
            line = 'r'   # "선이 오른쪽에 있어!" 라고 메모

    # 8단계: 이번에 벗어난 정도를 기억해둬요. (다음번 3단계 계산에 필요해요)
    last_error = error

def move(steering, power, angle):
    # [정해진 만큼 움직이기 함수]
    # 바퀴가 angle(도)만큼 돌 때까지, 정해진 핸들과 속도로 움직이고 멈춰요.
    # 바퀴가 얼마나 돌았는지 세면, 로봇이 얼마나 갔는지 알 수 있어요.
    lmu.reset_angle(0)   # 바퀴 회전 숫자를 0으로 리셋 (줄자를 0에 맞추는 것)
    while True:
        steering_drive(power, steering)
        if steering < 0:                  # 왼쪽으로 돌 때는 바퀴가 거꾸로 돌아서
            if lmu.angle() < -angle:      # 마이너스 쪽으로 세요
                break
        else:
            if lmu.angle() > angle:       # 목표만큼 돌았으면 멈춰요
                break
    stop()

def move_black(steering, power):
    # [검은 선 찾을 때까지 가기 함수]
    # 왼쪽이나 오른쪽 눈이 검은색(50점 미만)을 볼 때까지 계속 가다가 멈춰요.
    while True:
        lv, cv, rv = normalizing(reflect_list)
        steering_drive(power, steering)
        if lv < 50 or rv < 50:
            break
    stop()

########################################## main program ##########################################
# 여기서부터 진짜 프로그램이 시작돼요!

# 미리 측정해둔 센서 값이에요. (숙제처럼 먼저 재놓은 값)
# 순서: 왼쪽눈이 본 흰색, 왼쪽눈이 본 검은색, 가운데눈 흰색, 가운데눈 검은색, 오른쪽눈 흰색, 오른쪽눈 검은색
reflect_list = [49,3, 62,6, 64, 5]

# "지난번에 얼마나 벗어났었지?"를 기억하는 메모장. 처음엔 0으로 시작해요.
last_error = 0

# 무한 반복! 로봇이 멈출 때까지 계속 선을 따라가요.
while True:
    # 라인 따라가기 실행! (놓쳤을 때 핸들 67, 속도 100, kp 0.14, kd 1.4, 감속 0.1)
    pid_control(67, 100, 0.14, 1.4, 0.1)

    # ↓ 아래는 장애물을 만나면 피해가는 코드인데, 지금은 잠자고 있어요(주석 처리).
    # 초음파센서로 앞에 물건이 229mm보다 가까이 있는지 확인하고,
    # 진짜 장애물이 맞는지 10번 다시 확인한 뒤, 옆으로 빙 돌아서 피해가요.
    # if ul.distance() < 229:
    #     # 장애물
    #     stop()
    #     obstacle = 0
    #     for i in range(10):
    #         if ul.distance() < 229:
    #             obstacle+=1
    #     # print(obstacle)
    #     if obstacle > 4:
    #         # wait(100)
    #         move(100, 100, 290)      # 오른쪽으로 휙 돌고
    #         wait(100)
    #         move(0, 90, 110)         # 앞으로 가고
    #         wait(100)
    #         move(-100, 100, 130)     # 왼쪽으로 휙 돌고
    #         move_black(-30, 90)      # 검은 선을 찾을 때까지 가고
    #         move(0, 100, 70)         # 조금 더 가서
    #         move_black(100, 50)      # 선 위로 돌아와요!
