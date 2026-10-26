# 물리 엔진을 넣으면 시뮬이 실기처럼 될까?

*2026-10-26 검토 · 도구: [`sim/tools/physics_check.py`](../sim/tools/physics_check.py) · [English](PHYSICS_ENGINE.md)*

**결론: 엔진만으로는 안 됩니다.** SPIKE 라지 모터는 공개된 값이 있고, 그 값으로 계산하면 시뮬의 직진 반응이 물리식 그대로 나옵니다. 하지만 회전은 바퀴가 매트에서 얼마나 미끄러지는지가 좌우합니다. 이 값은 어디에도 공개돼 있지 않고 물리 엔진도 모릅니다. 로봇의 블랙박스 기록에서만 얻을 수 있습니다.

## 1. 짐작한 값으로 돌린 MuJoCo

[MuJoCo](https://mujoco.org) 3.17로 우리 로봇을 만들었습니다.
- 바퀴 80mm, 바퀴 사이 205mm
- 센서 바 밑 볼 캐스터, 컬러센서는 바퀴축 앞 170mm
- DC 모터 2개가 기어를 거쳐 바퀴를 돌림

실기와는 [`pivot_resp.py`](../sim/tools/pivot_resp.py)와 같은 지표로 비교했습니다. 제자리 회전을 명령한 뒤 20·40·60ms에 회전 속도가 얼마나 붙는지입니다.

처음에는 모르는 값(모터 멈춤 토크, 모터 관성, 바퀴·캐스터 마찰, 무게중심, 회전 직전 속도)을 112가지로 바꿔 봤습니다. 이 시험은 도구에 넣지 않았습니다. `--mujoco`는 4장의 공개값 판을 돌립니다.
- 1배 기어의 60ms 회전 속도가 약 40~150°/s로 흔들렸습니다.
- 2배 기어는 1배보다 0.9~1.4배 빨리 붙었습니다. 실기는 1.75배입니다(2배 회전 1번 기준).
- 가장 가까운 조합도 오차가 30%였고, 그마저 실기 숫자를 보고 고른 값입니다.

짐작한 값으로는 엔진이 지금 손으로 맞춘 모형보다 낫지 않습니다.

## 2. 공개된 자료

| 출처 | 값 |
|---|---|
| Pybricks 펌웨어 [`servo_settings.c`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/motor/servo_settings.c) (`model_technic_l_angular`) | Pybricks가 라지 모터를 측정해 만든 모델입니다. `observer.c`의 배율로 풀면: <ul><li>볼트당 토크 0.0623 N·m</li><li>역기전력 항 0.0256 N·m·s/rad</li><li>출력축 기준 관성 0.00122 kg·m² (회전자와 안쪽 기어박스)</li><li>마찰 0.023 N·m</li></ul>모터 혼자일 때 시정수는 48ms입니다. |
| Pybricks [`motor_model.py`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/doc/control/motor_model.py) | 위 숫자를 만드는 방법입니다. 이 파일의 미디엄 모터 실험값으로 계산하면 펌웨어의 미디엄 모터 숫자가 그대로 나옵니다. 푸는 방법이 맞다는 확인입니다(`physics_check.py`가 둘 다 출력). |
| Pybricks [`battery.c`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/battery.c), [`pb_type_motor.c`](https://github.com/pybricks/pybricks-micropython/blob/master/pybricks/common/pb_type_motor.c) | `Motor.dc(duty)`는 배터리 전압 × duty를 그대로 겁니다. 전압 보정은 없고 상한은 9V입니다. |
| [LEGO 기술 사양서 45602](https://assets.education.lego.com/v3/assets/blt293eea581807678a/bltb9abb42596a7f1b3/5f8801b5f4c5ce0e93db1587/le_spike-prime_tech-fact-sheet_45602_1hy19.pdf?locale=en-us) | 7.2V 기준: <ul><li>무부하: 175rpm ±15%, 135mA</li><li>멈춤: 25 N·cm, 1.9A</li></ul>엔코더: 한 바퀴 360칸, 100Hz로 갱신. |
| [Philo의 모터 비교표](https://www.philohome.com/motors/motorcomp.htm) | 무게 71g. 9V 기준: <ul><li>무부하: 213rpm</li><li>멈춤: 33.5 N·cm, 1.8A</li></ul> |
| 판매처 목록 ([BuyWisely](https://buywisely.com.au/product/lego-education-45601-spike-prime-large-hub)) | 라지 허브 약 170g. |

## 3. 공개값으로 설명되는 것

**직진.** 바퀴가 미끄러지지 않으면 직진 시정수는 τ = (J + M·(r·g)²/2) / k입니다.
- J는 모터 관성, M은 로봇 무게, r은 바퀴 반지름, g는 기어비, k는 역기전력 항입니다.
- 로봇 무게는 부품 무게로 어림해 약 0.57~0.72kg입니다.

Pybricks 값을 넣으면 1배 기어에서 **0.10~0.11초**가 나옵니다. 시뮬의 1배 프로파일이 이미 쓰는 0.10초와 같고, MuJoCo도 같은 값을 냅니다(88~114ms). LEGO 사양서의 멈춤 토크로는 0.18~0.21초로 두 배 느립니다. 그래서 맞는 쪽은 Pybricks 모델입니다.

**회전.** 실기 블랙박스로 맞춘 회전 시정수는 0.05초입니다. 모터 혼자일 때 시정수(0.048초)와 같습니다. 바퀴가 바닥을 꽉 잡았다면 로봇 자체의 관성 때문에 0.1초쯤 나와야 합니다.
- 그러니까 제자리 회전 중에는 바퀴가 많이 미끄러져서, 모터가 로봇 무게를 거의 느끼지 못합니다.
- 실기가 바퀴 속도로 계산한 것보다 약 1.5배 덜 도는 것(`w_eff_scale` 1.5)도 아마 같은 이유입니다.

## 4. 공개값으로 설명되지 않는 것

**바퀴 미끄러짐.** 바퀴가 얼마나 미끄러지는지는 우리 타이어와 매트에 달린 값이고, 이 값이 회전을 정합니다.

**2배 기어의 회전.** 1배 시정수를 미끄러짐 없는 물리식 비율로 늘리면 직진 0.263초, 회전 0.13초가 됩니다.
- 이 값이면 SPIKE 대회용 완성판이 기본 월드의 첫 직각에서 10.3초를 헤맵니다. 실기는 잘 완주합니다.
- 회전은 0.10초로 두고 직진만 0.263초로 바꾸면 결과가 지금과 거의 같습니다.

| 2배 모형 (직진 / 회전) | main | hard | rightangle | photo@1.3 |
|---|---|---|---|---|
| 지금 `spike_3s_geo2x` (0.20 / 0.10초) | 2.4초 · 8/8 | 2.0초 · 8/8 | 4.0초 · 8/8 | 이탈 · 1/8 |
| 미끄러짐 없는 물리식 (0.263 / 0.13초) | 2.6초 · 8/8 | 2.1초 · 8/8 | **10.3초** · 8/8 | 이탈 · 1/8 |
| 직진만 물리식 (0.263 / 0.10초) | 2.3초 · 8/8 | 2.0초 · 8/8 | 4.1초 · 8/8 | 이탈 · 2/8 |

칸마다 기본 월드 결과, 그다음 무작위 월드 8개 중 완주 수입니다.

**MuJoCo도 회전은 안 맞습니다.** 공개 모터값을 넣어도 MuJoCo에서는 바퀴가 바닥에 붙어 있어서, 회전이 실기보다 느리게 붙습니다.

| 회전 속도 증가 (°/s) | 20ms | 40ms | 60ms | 60ms의 2배 ÷ 1배 |
|---|---|---|---|---|
| MuJoCo 1배 | 6~34 | 33~70 | 58~101 | |
| 실기 1배 | 54 | 84 | 114 | |
| MuJoCo 2배 vs 1배 | | | | 0.83~1.24 |
| 실기 2배 vs 1배 | | | | 1.75 |

**비교할 실기 자료도 부족합니다.**
- 실기 2배 회전은 0.39초짜리 블랙박스에 있는 1번뿐입니다.
- 1배 회전은 다른 로봇에서 나왔습니다. 센서가 바퀴축 앞 130mm인 로봇입니다(로그 `E7_130`).

## 5. 결론

- **모터는 따로 잴 필요가 없습니다.** Pybricks 모델을 쓰면 됩니다.
- **무게**는 부품 무게로 어림하거나 저울로 재면 됩니다.
- **바퀴 미끄러짐**은 블랙박스 기록으로만 얻을 수 있습니다.
  - 1배 로봇은 이미 충분합니다.
  - 2배 로봇은 대회용 완성판을 몇 번 더 달려야 합니다. 정지 버튼을 누르면 마지막 2초를 출력합니다.
- **지금은 물리 엔진을 넣을 가치가 없습니다.**
  - 엔진도 미끄러짐을 모릅니다.
  - MuJoCo 파이썬 패키지는 브라우저 시뮬(Pyodide)에서 돌지 않습니다.
  - GPU 대량 시뮬에는 따로 구현해야 합니다.
  - 지금 모형이 이미 맞는 구조입니다: 직진과 회전 시정수가 따로 있습니다.

## 다시 돌려 보기

```bash
python sim/tools/physics_check.py            # 모터 모델 풀기·검산, 시정수 vs 프로파일
pip install mujoco
python sim/tools/physics_check.py --mujoco   # MuJoCo 로봇 vs 실기 블랙박스
python sim/tools/physics_check.py --final    # SPIKE 완성판: 지금 2배 모형 vs 물리식 2배 모형
```
