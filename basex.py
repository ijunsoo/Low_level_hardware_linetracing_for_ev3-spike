import ustruct as struct
import math
import time

BASEX_ADDR = 0x22

# 서보 제어 레지스터
SERVO1_ANGLE_ADDR = 0x00  # 0~180
SERVO2_ANGLE_ADDR = 0x01  # 0~180
SERVO1_PULSE_ADDR = 0x10  # (uint16_t) 500~2500
SERVO2_PULSE_ADDR = 0x12  # (uint16_t) 500~2500


class DriveBase:
    # ══════════════════════════════════════════════════════════════════
    #  EV3 Medium 모터 PID 설정값 — 실측 펌웨어/드라이버 출처 병합
    # ------------------------------------------------------------------
    #  ev3dev (lego-linux-drivers, motors/ev3_motor_defs.c):
    #     MEDIUM  speed   = {p:1000, i:60, d:0}   (speed 게인은 대/중 모터 공통)
    #             position= {p:160000,i:0, d:0}   (position P 는 Large 80000 의 2배)
    #  LEGO 공식 펌웨어 (d_pwm.c, dRegulateSpeed, TYPE_MINITACHO=Medium):
    #     속도 P×4 / I=err/3(×0.9 누설=anti-windup) / D=Δerr×80
    #             → 중 모터는 저관성이라 'D(감쇠)'를 크게 쓴다(오버슈트 억제)
    #  leJOS (JavaMotorRegulator):  HOLD = {p:2, i:0.02, d:8}
    #             → 위치유지에 소량 D 로 버징/헌팅 억제
    # ------------------------------------------------------------------
    #  [BASEX 1바이트(0~255) 레지스터 매핑 원칙]
    #   전달되는 건 '절대값'이 아니라 '비율'. P 는 BASEX 동작영역 크기로 두고,
    #   I·D 는 EV3 비율로 환산한다. (EV3 원값은 고정소수점이라 그대로 못 옮김)
    # ══════════════════════════════════════════════════════════════════
    # ── 속도 PID (reg n+0x09/0A/0B) : ev3dev 1000/60/0 비율 이식
    SPEED_P = 22                  # BASEX 동작영역 기준(검증값). 비율의 분모 역할
    SPEED_I = 1                   # = round(SPEED_P * 60/1000) → EV3 속도 I/P=0.06
    SPEED_D = 0                   # ev3dev=0. 고속 오버슈트 시 1~3 (LEGO 공식 D 근거)

    # ── 위치 PID (reg n+0x01/02/03) : ev3dev 순수 P, Medium=Large×2
    POS_P = 24                    # = 12(Large) × 2  (160000 vs 80000)
    POS_I = 0
    POS_D = 0                     # 정지 헌팅/버징 시 1~2 (leJOS HOLD d:8 근거)
    POS_MAX_SPEED = 100           # 위치모드 최대속도(reg n+0x08). Medium은 Large보다 빠름

    def __init__(self, i2c, right_motors, left_motors,
                 wheel_diameter, axle_track, gear=None, left_counter=False, right_counter=False,
                 max_speed=600, motor_max=128):
        self._i2c = i2c
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.circum = math.pi * self.wheel_diameter

        self.l_counter = -1 if left_counter else 1
        self.r_counter = -1 if right_counter else 1

        gear = gear if gear else []
        self.ratio = gear[-1] / gear[0] if gear else 1
        self.counts_per_mm = 360 / self.circum * self.ratio
        self._deg_to_mm = self.circum / 360 / self.ratio
        self._half_track = self.axle_track / 2

        self.left_ports = left_motors
        self.right_ports = right_motors
        self.all_ports = self.left_ports + self.right_ports

        self.current_encoder = [0, 0, 0, 0]
        self.speed_scale = motor_max / max_speed
        self._speed_regs = [0x5C + m * 0x10 for m in range(4)]
        self._speed_buf = bytearray(1)
        self._enc_buf = bytearray(4)   # 엔코더 직접읽기 버퍼 재사용

        # 모터 컨트롤러(BASEX) 직접 초기화
        for p in self.all_ports:
            n = 0x50 + (p - 1) * 0x10
            i2c.writeto_mem(BASEX_ADDR, n, b'\x01')
            time.sleep_us(100)
        for p in self.all_ports:
            n = 0x50 + (p - 1) * 0x10
            # 위치 PID — EV3 Medium(position_pid_k={p:160000,i:0,d:0}) 성격: 순수 P.
            # 값은 클래스 상수(POS_*)에서 — 출처/유도는 위 상수 정의부 참조.
            i2c.writeto_mem(BASEX_ADDR, n + 0x01, bytes([self.POS_P]))
            time.sleep_us(100)
            i2c.writeto_mem(BASEX_ADDR, n + 0x02, bytes([self.POS_I]))
            time.sleep_us(100)
            i2c.writeto_mem(BASEX_ADDR, n + 0x03, bytes([self.POS_D]))
            time.sleep_us(100)
            # 위치모드 최대속도(reg n+0x08) — Medium은 Large보다 빨라 상한을 명시 설정
            i2c.writeto_mem(BASEX_ADDR, n + 0x08, bytes([self.POS_MAX_SPEED]))
            time.sleep_us(100)
        for p in self.all_ports:
            n = 0x50 + (p - 1) * 0x10
            i2c.writeto_mem(BASEX_ADDR, n, b'\x02')
            time.sleep_us(100)
        for p in self.all_ports:
            n = 0x50 + (p - 1) * 0x10
            # 속도 PID — EV3 Medium(speed_pid_k={p:1000,i:60,d:0}, 전 모터 공통).
            # 값은 클래스 상수(SPEED_*)에서 — 출처/유도는 위 상수 정의부 참조.
            i2c.writeto_mem(BASEX_ADDR, n + 0x09, bytes([self.SPEED_P]))
            time.sleep_us(100)
            i2c.writeto_mem(BASEX_ADDR, n + 0x0A, bytes([self.SPEED_I]))
            time.sleep_us(100)
            i2c.writeto_mem(BASEX_ADDR, n + 0x0B, bytes([self.SPEED_D]))
            time.sleep_us(100)

        # 시작 기준 엔코더를 직접 읽어 ref 설정.
        self.update_encoders()
        self.left_ref = self.current_encoder[self.left_ports[0] - 1]
        self.right_ref = self.current_encoder[self.right_ports[0] - 1]

    def _read_encoder(self, motor):
        n = 0x30 + (motor - 1) * 0x04
        self._i2c.readfrom_mem_into(BASEX_ADDR, n, self._enc_buf)
        return struct.unpack('>i', self._enc_buf)[0]

    def update_encoders(self):
        # distance()/odo_yaw가 쓰는 좌·우 기준 모터 엔코더만 직접 읽어 갱신한다.
        # 메인 제너레이터 루프 단일 컨텍스트라 충돌 없음. 노이즈성 실패는 직전 값 유지.
        enc = self.current_encoder[:]
        for motor in (self.left_ports[0], self.right_ports[0]):
            try:
                enc[motor - 1] = self._read_encoder(motor)
            except OSError:
                pass
        self.current_encoder = enc

    def _clamp(self, value, min_val=-128, max_val=127):
        return max(min_val, min(max_val, value))

    def write(self, reg, data):
        self._i2c.writeto_mem(BASEX_ADDR, reg, data)

    def set_mode(self, motor, value):
        n = 0x50 + (motor - 1) * 0x10
        self._i2c.writeto_mem(BASEX_ADDR, n, value)

    def set_motor_pwm(self, motor, value):
        n = 0x20 + (motor - 1) * 0x01
        try:
            self._i2c.writeto_mem(BASEX_ADDR, n, struct.pack('b', self._clamp(int(value))))
        except OSError:
            pass   # 종료/노이즈 시 모터쓰기 실패가 예외로 전파되지 않게 (close 경로 보호)

    def set_position_pid(self, motor, p, i, d):
        n = 0x50 + (motor - 1) * 0x10
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x01, bytes([int(p)]))
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x02, bytes([int(i)]))
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x03, bytes([int(d)]))

    def set_position_point(self, motor, target):
        n = 0x50 + (motor - 1) * 0x10
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x04, struct.pack('<i', int(target)))

    def set_position_max_speed(self, motor, speed):
        n = 0x50 + (motor - 1) * 0x10
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x08, bytes([self._clamp(abs(int(speed)), 0, 127)]))

    def set_motor_pid(self, motor, p, i, d):
        n = 0x50 + (motor - 1) * 0x10
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x09, bytes([int(p)]))
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x0A, bytes([int(i)]))
        self._i2c.writeto_mem(BASEX_ADDR, n + 0x0B, bytes([int(d)]))

    def set_motor_speed(self, motor, value):
        # 즉시 쓰기 — 미러/플러시 없이 호출 즉시 모터 속도 레지스터에 반영.
        idx = motor - 1
        self._speed_buf[0] = self._clamp(int(value)) & 0xFF
        try:
            self._i2c.writeto_mem(BASEX_ADDR, self._speed_regs[idx], self._speed_buf)
        except OSError:
            pass

    def set_servo_angle(self, servo, angle):
        addr = SERVO1_ANGLE_ADDR if servo == 1 else SERVO2_ANGLE_ADDR
        self._i2c.writeto_mem(BASEX_ADDR, addr, bytes([self._clamp(int(angle), 0, 180)]))

    def set_servo_pulse(self, servo, pulse):
        addr = SERVO1_PULSE_ADDR if servo == 1 else SERVO2_PULSE_ADDR
        self._i2c.writeto_mem(BASEX_ADDR, addr, struct.pack('<H', self._clamp(int(pulse), 500, 2500)))

    def distance(self):
        self.update_encoders()   # 호출 즉시 엔코더 직접 읽기 — 항상 최신값
        l_deg = self.current_encoder[self.left_ports[0] - 1] - self.left_ref
        r_deg = self.current_encoder[self.right_ports[0] - 1] - self.right_ref
        return (l_deg * self.l_counter + r_deg * self.r_counter) * self._deg_to_mm / 2

    def reset(self):
        self.update_encoders()   # ref도 최신 엔코더 기준으로
        self.left_ref = self.current_encoder[self.left_ports[0] - 1]
        self.right_ref = self.current_encoder[self.right_ports[0] - 1]

    def drive_init(self):
        for p in self.all_ports:
            self.set_mode(p, b'\x02')
        time.sleep_ms(20)

    def drive(self, speed, steering, fade_below=0):
        # 기본 동작은 원본 그대로 — 조향은 속도와 무관(제자리/자이로 턴 정상 동작).
        #
        # fade_below > 0 (선택): '라인주행을 정지시킬 때'만 켠다. 그 속도 이하로
        #   감속하는 구간에서만 조향(좌우차동)을 0까지 줄여 정지 직전 좌우 흔들림을
        #   없앤다. 제자리 턴(speed≈0)에는 절대 쓰지 말 것 — 회전이 사라진다.
        #   예) 라인주행 정지 페이즈: robot.drive(v, steer, fade_below=60)
        scaled_speed = speed * self.speed_scale
        turn_rate_rad = math.radians(steering)
        diff = turn_rate_rad * self._half_track
        if fade_below and abs(speed) < fade_below:
            diff *= abs(speed) / fade_below   # 정지 직전 저속에서만 조향 페이드아웃
        v_l = scaled_speed - diff
        v_r = scaled_speed + diff

        for p in self.left_ports:
            self.set_motor_speed(p, int(v_l * self.l_counter))
        for p in self.right_ports:
            self.set_motor_speed(p, int(v_r * self.r_counter))

        return v_l, v_r

    def _odo_yaw(self):
        # 좌우 바퀴 주행차로 추정한 헤딩 변화(deg). reset() 기준이라 직진 중엔 0이어야 정상.
        # 양수 = CCW(좌)로 틀어짐. 자이로 없이 엔코더만으로 직진 보정에 사용.
        l = (self.current_encoder[self.left_ports[0] - 1] - self.left_ref) * self.l_counter
        r = (self.current_encoder[self.right_ports[0] - 1] - self.right_ref) * self.r_counter
        return (r - l) * self._deg_to_mm / self.axle_track * 57.29577951308232

    def straight(self, speed, mm, kp=2.0, decel=0, min_speed=20):
        # 엔코더 오도메트리만으로 mm만큼 직진하는 협력 제너레이터.
        # IMU 없이 좌우 주행차(_odo_yaw)를 0으로 맞춰 직진성을 유지한다.
        # 호출부: yield from robot.straight(speed, mm)
        #
        # decel>0 (mm): 목표 decel mm 전부터 속도를 min_speed 까지 선형 감속하며,
        #   '헤딩보정 조향'도 같은 비율로 0까지 페이드한다. 저속에서 조향이 그대로면
        #   곡률 κ=steering/speed 가 폭주해 좌우로 흔들리는데(감속 구간 핑퐁),
        #   조향을 속도와 함께 줄여 곧게 감속·정지시킨다. decel=0 이면 기존 동작(등속+급정지).
        target = abs(mm)
        actual_speed = -speed if mm < 0 else speed
        eff_min = min(abs(min_speed), abs(actual_speed))   # min_speed 가 speed 초과 방지

        self.reset()
        while True:
            dist = abs(self.distance())   # distance()가 내부에서 엔코더 직접 갱신
            if dist >= target:
                break

            v = actual_speed
            # odo_yaw>0(좌로 드리프트)면 우로 보정 → 조향 부호 반대로.
            steer = -kp * self._odo_yaw()

            if decel > 0:
                remain = target - dist
                if remain < decel:
                    ratio = remain / decel                 # 감속 구간에서 1→0
                    mag = eff_min + (abs(actual_speed) - eff_min) * ratio
                    v = mag if actual_speed >= 0 else -mag
                    steer *= ratio                         # 조향도 같이 페이드 → 흔들림 제거

            self.drive(v, steer)
            yield

        self.brake()
        self.reset()

    def stop(self, lock=True):
        for p in self.all_ports:
            self.set_motor_speed(p, 0)
            if not lock:
                self.set_motor_pwm(p, 0)

    def brake(self):
        self.stop()

    def close(self):
        self.stop(False)