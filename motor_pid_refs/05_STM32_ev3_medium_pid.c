/* =====================================================================
 *  05_STM32_ev3_medium_pid.c
 *  STM32(베어메탈 C / HAL) 에서 "EV3 미디엄 모터"를 닫힌루프 PID 제어
 * ---------------------------------------------------------------------
 *  ※ EV3 미디엄 모터는 STM32 입장에선 그냥 "9V DC 모터 + 쿼드러처 엔코더"다.
 *     따라서 LEGO/ev3dev/Pybricks/leJOS의 펌웨어 PID는 '그대로' 못 돌리고,
 *     아래처럼 STM32 표준 패턴으로 직접 구현한다.
 *     - 구조(이산 PID + anti-windup): ev3dev tm_pid_update 와 동일 사상
 *     - 엔코더 읽기: STM32 TIMx 엔코더 모드(하드웨어 4체배)
 *     - 구동: 부호-크기(sign-magnitude) H브리지(TB6612/DRV8833)
 *
 *  [배선 요약]
 *    EV3 케이블 핀1,2  -> H브리지 모터 출력 (AO1/AO2)
 *    EV3 케이블 핀3    -> GND
 *    EV3 케이블 핀4    -> +5V (엔코더 전원, 3.3V도 가능)
 *    EV3 케이블 핀5(A) -> TIM_ENC CH1 핀  (예: TIM2_CH1 / PA0)
 *    EV3 케이블 핀6(B) -> TIM_ENC CH2 핀  (예: TIM2_CH2 / PA1)
 *    STM32 PWM 핀      -> H브리지 PWMA
 *    STM32 GPIO 2개    -> H브리지 AIN1/AIN2 (방향)
 *
 *  [엔코더 분해능]
 *    EV3 미디엄: 채널당 180펄스/회전.
 *    STM32 엔코더모드 TI12 = 4체배 -> 720 counts / 출력축 1회전.
 *    (LEGO 펌웨어는 2체배라 360 = "1°당 1 count" 로 보고함)
 *
 *  CubeMX 설정:
 *    - TIM2: Combined Channels -> Encoder Mode (TI1 and TI2)
 *    - TIM3: PWM Generation CH1 (10~20kHz, ARR = MAX_PWM)
 *    - TIM6(또는 SysTick): 제어 루프 주기(여기선 1kHz, dt=0.001s) 인터럽트
 * ===================================================================== */

#include "main.h"   // CubeMX 생성 헤더(htim2, htim3 핸들 등)
#include <stdlib.h>

/* ------------------------- 하드웨어 상수 ------------------------- */
#define ENC_CPR          720.0f   // 4체배 counts/회전 (EV3 미디엄)
#define CONTROL_HZ       1000.0f  // 제어 루프 주파수
#define DT               (1.0f / CONTROL_HZ)   // 0.001 s
#define MAX_PWM          1000      // TIM3 ARR 값 = 듀티 최대치

extern TIM_HandleTypeDef htim2;   // 엔코더 타이머
extern TIM_HandleTypeDef htim3;   // PWM 타이머

/* ===================================================================
 *  1) 이산 PID  (ev3dev tm_pid_update 와 같은 사상: P + I + D, anti-windup)
 *     - 위치형(positional), D는 setpoint kick 방지 위해 '측정값'으로 미분
 *     - 출력은 ±MAX_PWM 로 클램프, 포화 시 적분 누적 되돌림(back-calc)
 * =================================================================== */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_meas;     // 측정값 기반 미분용
    float out_min, out_max;
} PID_t;

void PID_Init(PID_t *p, float Kp, float Ki, float Kd, float omin, float omax)
{
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->out_min = omin; p->out_max = omax;
}

float PID_Update(PID_t *p, float setpoint, float meas, float dt)
{
    float error = setpoint - meas;

    /* P */
    float P = p->Kp * error;

    /* I */
    p->integral += error * dt;
    float I = p->Ki * p->integral;

    /* D : 측정값으로 미분 (setpoint 계단 입력 시 D 킥 방지) */
    float D = -p->Kd * (meas - p->prev_meas) / dt;
    p->prev_meas = meas;

    float out = P + I + D;

    /* 클램프 + anti-windup (이번 스텝 적분 되돌림) */
    if (out > p->out_max) {
        out = p->out_max;
        p->integral -= error * dt;
    } else if (out < p->out_min) {
        out = p->out_min;
        p->integral -= error * dt;
    }
    return out;
}

/* ===================================================================
 *  2) 엔코더 읽기 (STM32 TIM 엔코더 모드)
 *     - 16bit 카운터 차분을 int16_t 로 캐스팅하면 오버플로 자동 처리
 *     - 매 루프 delta(counts/loop) -> 속도(counts/s, RPM 등) 환산
 * =================================================================== */
static uint16_t g_last_cnt = 0;
long  g_position = 0;        // 누적 위치(counts, 멀티턴)

/* 매 제어주기 호출: 이번 주기 동안의 엔코더 증분 반환 */
int16_t Encoder_Delta(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t  delta = (int16_t)(now - g_last_cnt);  // 부호있는 차분 = 방향+오버플로 처리
    g_last_cnt = now;
    g_position += delta;
    return delta;
}

/* counts/loop -> RPM */
float Counts_To_RPM(int16_t delta_per_loop)
{
    return (delta_per_loop / ENC_CPR) * (60.0f * CONTROL_HZ);
}

/* ===================================================================
 *  3) H브리지 구동 (부호-크기): PID 부호=방향, 크기=듀티
 *     TB6612/DRV8833 기준: AIN1/AIN2 로 방향, PWMA 로 듀티
 * =================================================================== */
void Motor_Drive(int32_t u)
{
    if (u >  MAX_PWM) u =  MAX_PWM;
    if (u < -MAX_PWM) u = -MAX_PWM;

    if (u >= 0) {  /* 정방향 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)u);
    } else {       /* 역방향 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(-u));
    }
}

/* ===================================================================
 *  4) 사용 예
 * =================================================================== */
PID_t  speed_pid;     // 속도 루프
PID_t  pos_pid;       // 위치 루프
float  target_rpm   = 0.0f;
long   target_count = 0;
volatile int use_position_mode = 0;

void Control_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    /* 시작 게인(반드시 본인 모터/전압/CPR/주기로 재튜닝!)
       절차: Ki=Kd=0 -> Kp 올려 약한 오버슈트 -> 0.5~0.6배로 -> Ki로 정상상태오차 제거 -> 필요시 소량 Kd */
    PID_Init(&speed_pid, 0.8f, 6.0f, 0.0f,  -MAX_PWM, MAX_PWM);   // 속도: D는 0에서 시작 권장
    PID_Init(&pos_pid,   8.0f, 0.0f, 0.4f,  -MAX_PWM, MAX_PWM);   // 위치: P+D 위주
}

/* 제어 루프 ISR: TIM6 1kHz 인터럽트에서 호출 (CONTROL_HZ/DT 일치 필수) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    int16_t delta = Encoder_Delta();   // g_position 도 여기서 갱신됨

    float u;
    if (use_position_mode) {
        u = PID_Update(&pos_pid, (float)target_count, (float)g_position, DT);
    } else {
        float rpm = Counts_To_RPM(delta);
        u = PID_Update(&speed_pid, target_rpm, rpm, DT);
    }
    Motor_Drive((int32_t)u);
}

/* =====================================================================
 *  참고 출처(실제 코드/사양 확인):
 *   - EV3 모터 핀아웃/엔코더 사양:
 *       lechnology.com/2017/03/using-lego-mindstorms-motors-with-beaglebone-blue/
 *       biasedlogic.com  (3rd-party motor with EV3/NXT)
 *       LEGO Hardware Developer Kit (PDF)
 *   - STM32 TIM 엔코더 모드:
 *       controllerstech.com/incremental-encoder-with-stm32/
 *       deepbluembedded.com/stm32-timer-encoder-mode-...
 *   - 이산 PID(위치형, anti-windup):
 *       steppeschool.com/blog/pid-implementation-in-c
 *   - 부호-크기 H브리지 구동(STM32+엔코더+PID 실제 repo):
 *       github.com/agnunez/stm32duino_2DCMotor_Encoders
 *   - EV3 모터를 STM32 PWM으로 구동(TB6612, 전기적 실증):
 *       en.depfields.com/dcmotor-pwmcontrol/
 *   - PID 구조 원형(EV3 미디엄 게인 명시):
 *       ev3dev/lego-linux-drivers  motors/tacho_motor_helper.c, ev3_motor_defs.c
 * ===================================================================== */
