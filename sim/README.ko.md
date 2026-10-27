# 시뮬레이터 (CPU · GPU) · 강화학습

[English](README.md) | 한국어

실기와 같은 Pybricks 코드 파일을 PC에서 실행하는 디지털 트윈입니다.
다른 점은 두 가지예요: 빛값을 시뮬 센서에 맞춰야 하고, 센서는 반사광으로 읽습니다(raw 모드 3은 없음). 둘 다 `--sub`로 파일을 고치지 않고 바꿀 수 있습니다.
모터 가속(비대칭 1차 지연), 회전 미끄러짐, 센서 발자국·노이즈·지연, 실측 루프 속도를 반영합니다.
SPIKE Prime(+EV3 컬러센서)과 EV3(Pybricks v4) 둘 다 지원합니다.

## 목차

1. [준비](#1-준비)
2. [구성](#2-구성)
3. [프로파일과 트랙](#3-프로파일과-트랙)
4. [실행과 평가](#4-실행과-평가)
5. [분석 도구 (tools/)](#5-분석-도구-tools)
6. [회귀 테스트](#6-회귀-테스트)
7. [GPU 시뮬 · 강화학습 (rl/)](#7-gpu-시뮬--강화학습-rl)
8. [실기로 보정하기](#8-실기로-보정하기)
9. [한계](#9-한계)

---

## 1. 준비

| 용도 | 파이썬 |
|---|---|
| CPU 시뮬 | numpy가 있는 파이썬 — `/opt/homebrew/bin/python3.13` |
| GPU 시뮬·강화학습 | `~/.venvs/linefollowing` — numpy, torch(MPS), mpy-cross |
| 그림 (`--png`) | matplotlib가 있는 파이썬 |

프로젝트 안 `.venv`는 iCloud 폴더라 파일이 내려가면 `import numpy`에 몇 분이 걸립니다. 위 파이썬을 쓰세요.

---

## 2. 구성

| 경로 | 역할 |
|---|---|
| `core.py` | 물리(모터·회전 미끄러짐)·센서·트랙·시뮬 루프 |
| `pybricks/` | Pybricks 목 — 실기 파일이 그대로 `import` (SPIKE `PrimeHub`·`PUPDevice`, EV3 `EV3Brick`·`ColorSensor`·`LUMPDevice`) |
| `profiles/` | 하드웨어 프로파일 (`extends`로 상속) |
| `run_sim.py` | 실기 파일 하나를 달려 보고 그림 저장 |
| `eval_lf3.py`, `recap_stats.py` | 여러 월드 평가, 재포착 통계 |
| `sweep_lf3.py` | 실기 파일 상수 스윕 (CPU) |
| `regress.py` | 회귀 테스트 |
| `bb_parse.py` | 실기 블랙박스 해석 |
| `diag_parse.py` | 실기 진단 로그 해석 → 프로파일 보정 |
| `photo_track.py` | 경기장 사진 → 트랙 (`viz/track_pts.json`) |
| `tools/` | 분석 도구 ([5장](#5-분석-도구-tools)) |
| `rl/` | GPU 시뮬·GPU 스윕·강화학습 ([7장](#7-gpu-시뮬--강화학습-rl)) |
| `viz/gpu_view.html` | GPU 시뮬 재생 화면 |
| `runs_geo/` | 루트 README에 쓴 그림 |

---

## 3. 프로파일과 트랙

`--profile 이름`으로 고릅니다.

| 프로파일 | 내용 |
|---|---|
| `spike_3s_geo2` | **SPIKE 1배 기어 기본.** 실측 치수(센서 3개 23mm 간격 @170mm, 윤거 205mm, 바퀴 80mm) + 회전 보정(시정수 0.05s, w_eff 1.5) |
| `spike_3s_geo2x` | **SPIKE 2배 기어 기본.** geo2 + 기어 2배 + 모터·회전 시정수 ×2 (토크 한계) |
| `spike_3s_geo2_15x` | **SPIKE 1.5배 기어 (권장, 아직 실측 없음).** geo2 + 기어 1.5배(`gear_ratio` 2.5), 시정수 0.142 / 0.071초 = 실측 1배·2배 사이를 모터 물리식으로 보간. 대회용 완성판 상수를 이 기어에 맞춤 |
| `ev3_3s_geo` | EV3 (포트 S1·S3·S2, 모터 A·B / C·D, 자이로 없음) |
| `spike_3s_geo`, `spike_3s_real`, `spike_3s`, `ev3_3s`, `spike_4s`(= `robot_params.json`) | 위 프로파일들의 상속 단계 (예전 측정) |
| `spike_3s1_geo`, `spike_4row_geo`, `spike_3s1f_geo` (+ `_geo2`) | 센서 4개 실험: 3개 + D 읽기만 / 4개 일렬 / 3개 + 앞 가운데 |

geo2·geo2x·geo2_15x로 평가할 때는 `--weff 1.4,1.6`을 함께 줍니다.

| 트랙 | 내용 |
|---|---|
| `easy`, `main`, `hard` | 기본 곡선 트랙 |
| `rightangle` | 직선 뒤 직각 3번 |
| `mrc`, `mrc_real` | 대회형 트랙 (1.6배 / 실물 크기) |
| `photo` | 경기장 사진에서 뽑은 트랙 (195cm 매트, 6.89m) |
| `photo@배율` | 사진 트랙 확대. `photo@1.3`이면 원 반지름 ≈124mm로 실기 원 크기와 맞음 |

---

## 4. 실행과 평가

```bash
python3.13 sim/run_sim.py --list                                                        # 프로파일·트랙 목록
python3.13 sim/run_sim.py spike/spike_lf3_rule.py --profile spike_3s_geo2 --track photo
python3.13 sim/run_sim.py ev3/ev3_lf3_rule.py --profile ev3_3s_geo --track hard
```

- **센서 모드**: 시뮬 센서 목은 반사광(모드 0)만 지원합니다. `RAW_MODE = True`인 파일은 `RAW_MODE False`와 시뮬 밝기 `VALUE`(SPIKE `[34, 6, 22, 3, 21, 3]`)로 바꿔서 돌립니다 (아래 `--base`).
- **최종판**(`spike_final.py`, `ev3_final.py`)은 바꿀 값이 마지막 `line_trace(...)` 한 줄에 있어서 `--base`로 `VALUE`·`AUTO_STOP`을 못 바꿉니다. 사본에서 그 줄을 고쳐(`VALUE=[34, 6, 22, 3, 21, 3]`, `AUTO_STOP=False`) 평가하고, `RAW_MODE`·`AUTO_CAL`은 `--base`로 바꿉니다.
  EV3 최종판은 RGB 반사광(`ColorSensor`)이 기본이라 `RAW_MODE`를 안 바꿔도 되고, 빨간 불 경로는 `--base '{"RED_LIGHT": true, "RAW_MODE": false, "VALUE0": [34, 6, 22, 3, 21, 3]}'`로 돌립니다.

---

## 5. 분석 도구 (tools/)

| 파일 | 역할 |
|---|---|
| `corner_eval.py` | 변형 평가: 완주·시간·최대 이탈·뱅킹·피벗·루프 속도. `--base` 공통 상수, `--dump` 월드별 JSON |
| `paired.py` | `--dump` 결과를 같은 월드끼리 짝지어 비교 |
| `loop_trace.py` | 원·교차로 구간의 결과와 바퀴축·센서 궤적 그림 |
| `ra_trace.py` | 직각 트랙 첫 직각: 센서가 지나친 거리, 재진입 각도 |
| `stop_test.py` | 정지 버튼 시험: 정지 후 출력량, 프로그램이 붙잡는 시간, 그동안 바퀴가 더 굴러간 거리 |
| `step_dump.py` | 한 월드의 스텝별 센서·오차·조향·위치 |
| `pivot_resp.py` | 제자리 회전 뒤 회전 속도가 붙는 정도: 실기 블랙박스 vs 시뮬 |
| `physics_check.py` | 공개된 SPIKE 모터값(Pybricks 펌웨어 모델) → 직진·회전 시정수 vs 프로파일. `--mujoco` MuJoCo 로봇, `--final` 물리식 2배 모형으로 완성판 주행 ([`docs/PHYSICS_ENGINE.ko.md`](../docs/PHYSICS_ENGINE.ko.md)) |
| `make4.py`, `make_guard.py`, `make_sent.py`, `make_bank.py` | 센서 4개·원 대책·앞 센티넬·곡선 강제 회전 실험 변형 생성 → `tools/variants/` (git 제외) |

```bash
B='{"RAW_MODE": false, "VALUE": [34, 6, 22, 3, 21, 3], "AUTO_CAL": false}'
python3.13 sim/tools/corner_eval.py '[["85", {"ST_BANK": 85}], ["75", {}]]' --src spike/spike_lf3_rule_e8r10_170.py \
  --profile spike_3s_geo2x --fwd 170 --weff 1.4,1.6 --base "$B" \
  --tracks rightangle,main,hard,photo@1.3 --lost 450 --worlds 32 --dump /tmp/bank.json
python3.13 sim/tools/paired.py "/tmp/bank.json:85" "/tmp/bank.json:75"
python3.13 sim/tools/pivot_resp.py diag_logs/bb_2026-10-24_E8R_2x_short.txt --sim spike_3s_geo2x
python3.13 sim/tools/physics_check.py --final
```

`--fwd 170`은 센서 3개를 그 거리 일렬로 놓고, `--fwd 0`은 프로파일의 센서 위치를 그대로 씁니다 (센서 4개 프로파일은 반드시 0).

---

## 6. 회귀 테스트

`core.py`나 목을 고친 뒤에는 꼭 돌립니다. 결과를 궤적 해시까지 골든(`regress_golden.json`)과 비교합니다.

```bash
python3.13 sim/regress.py              # 비교 (15건, 약 10초)
python3.13 sim/regress.py --only lf3   # 일부만
python3.13 sim/regress.py --capture    # 의도한 변경 뒤 골든 갱신
```

케이스: 센서 응답곡선, SPIKE·EV3 목 스모크(`tests/`), 3센서 규칙 파일(`spike_lf3_rule.py`, `ev3_lf3_rule.py`), **SPIKE↔EV3 동일성**(같은 물리·입출력에서 두 파일 궤적이 같은지).

---

## 7. GPU 시뮬 · 강화학습 (rl/)

| 파일 | 역할 |
|---|---|
| `tracks.py` | 규정 기반 랜덤 트랙 생성 (라인 20mm, 최소 R100, 직각·예각·루프 교차·지그재그·S자·마커) |
| `fastsim.py` | core와 같은 물리·센서를 numpy로 N대 동시에 |
| `torchsim.py` | 같은 시뮬의 PyTorch판 (Apple GPU). 16,384대 동시 약 65만 결정/초 |
| `rule_lf3.py` | 실기 3센서 규칙 로직의 벡터화판 (상수는 `spike/spike_lf3_rule.py`에서 읽음) |
| `sweep_gpu.py`, `sweep_geo.py` | GPU 대규모 상수·배치 탐색 (상위 설정만 CPU core로 재검증) |
| `env.py`, `ppo.py` | 강화학습 환경, PPO (잔차 정책, 비대칭 액터-크리틱) |
| `export_policy.py` | 학습 정책 → 정수 연산 Pybricks 실기 파일 |
| `parity_test.py`, `cpu_eval_ckpt.py`, `replay_test.py`, `watch_train.py` | CPU·GPU 일치 확인, 체크포인트 평가, 재생, 학습 지켜보기 |

```bash
cd sim
~/.venvs/linefollowing/bin/python -m rl.torchsim --check --bench                   # numpy판과 일치·속도
~/.venvs/linefollowing/bin/python -m rl.sweep_gpu --n 192 --no-slow --top-out top.json
python3.13 sweep_lf3.py --phase validate --configs top.json --out val.json         # 상위 설정 CPU 재검증
~/.venvs/linefollowing/bin/python -m rl.ppo --layout row3 --mode steer_only --actor-hid 16 --act relu
~/.venvs/linefollowing/bin/python -m rl.export_policy rl/runs/<이름>/policy.pt
```

- **`--no-slow`**: 파워 100 고정 (감속 없음 방침). `ppo --mode steer_only` = 조향만 보정.
- **실기 배포**: 신경망을 정수(×1000) 고정소수점 직선 코드로 생성해 SPIKE·EV3 모두 float 없이 돕니다. 내보낼 때 중간값 범위를 계산해 오버플로가 없음을 확인하고, 생성 코드와 기준식을 극단값 포함 4,000개 입력으로 자가검사합니다.
- **남겨 둔 학습 결과**: `runs/row3_res2_h16_v3` → `spike/spike_lf3_rl.py`, `ev3/ev3_lf3_rl.py` · `runs/geo_steer_v2` → `spike/spike_lf3_rl_geo.py`

RL v3 (일렬 3센서, 잔차 정책) vs 규칙기반, CPU core 100월드:

| | mrc 완주 | photo 완주 |
|---|---|---|
| 규칙기반 (당시 기본값) | 26/100 · 16.91s | 90/100 · 8.96s |
| RL v3, 정책 계산 1ms | 52/100 · 15.75s | 98/100 · 8.71s |

> RL v3는 2배 기어·토크 한계 이전의 동역학으로 학습했습니다. 지금 로봇에 쓰려면 `spike_3s_geo2x` 조건으로 다시 학습해야 합니다.

---

## 8. 실기로 보정하기

- **블랙박스**: 실기 파일은 4ms마다 [센서 3개, 조향, 파워, 상태, 회전 속도]를 2초(500개) 기록해 두고, 멈추면 `BB`·`BBD` 줄로 출력합니다 → `python3.13 sim/bb_parse.py 로그.txt --events`
- **진단**: `spike/spike_diag.py`(EV3 `ev3/ev3_diag.py`) 출력 → `python3.13 sim/diag_parse.py 로그.txt --sim spike_3s`로 실기와 시뮬의 호출 비용·센서 갱신·지연을 비교합니다.
- **지금까지 맞춘 값**:
  - 실기 루프 ≈ 734 제어/초
  - 회전: 시정수 ≈0.05s, 구동 지연 ≈36ms, w_eff ≈1.5 (블랙박스 5개) → `spike_3s_geo2`
  - 2배 기어: 제자리 회전이 붙는 속도가 1배와 비슷 → 시정수 ×2 → `spike_3s_geo2x` (실기 2배 회전 1번엔 ×2·×3이 똑같이 맞고, ×2여야 최종판이 실기처럼 완주 — 대략값)

---

## 9. 한계

- **시뮬 1등 ≠ 실기 1등.** 후보를 추리는 용도입니다.
- 슬립은 횡가속 한계(언더스티어)만 있고, 전압 강하·바닥 마찰 편차는 도메인 랜덤화로 흡수합니다.
- 센서는 반사광만 재현합니다. raw 모드(모드 3)·컬러는 없습니다.
- 원(작은 루프)은 1배 `photo`에서 실기보다 훨씬 비관적입니다 (원이 실기보다 작음). 원 관련 비교는 `photo@1.3`으로 합니다.
- 실기에서 라인 하나가 두 센서 사이에 걸치면 둘 다 30~45%로 읽혀 '가짜 직각 신호'가 생겼는데, 시뮬은 이걸 재현하지 못했습니다.
- `spike_3s_geo2x`의 토크 한계(시정수 ×2)는 실기 2배 회전 1번에 맞춘 대략값입니다. 2초 블랙박스가 더 모이면 다시 맞춥니다.
- 물리 엔진(MuJoCo)은 검토 후 넣지 않았습니다. 모터는 공개값으로 맞지만, 회전은 엔진도 모르는 바퀴 미끄러짐이 좌우합니다. [`docs/PHYSICS_ENGINE.ko.md`](../docs/PHYSICS_ENGINE.ko.md) 참고.
