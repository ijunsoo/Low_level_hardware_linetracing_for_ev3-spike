# 라인트레이서 시뮬레이터 (SPIKE Prime · EV3 공용)

실기 코드(Pybricks)를 **수정 없이** PC에서 실행하는 디지털 트윈. 모터 가속
(비대칭 1차지연), 센서 발자국·노이즈·UART 지연, 실측 루프·센서 속도까지 반영한다.
SPIKE Prime 허브(+EV3 컬러센서)와 EV3 브릭(Pybricks v4 네이티브) 둘 다 지원.

## 준비

`.venv`는 iCloud 동기 폴더(Documents) 안에 있어 파일이 클라우드로 내려가면(dataless)
`import numpy`에만 몇 분이 걸린다. 아래 중 하나로 실행한다.

```bash
/opt/homebrew/bin/python3.13 sim/run_sim.py ...     # numpy 로컬 설치됨 (matplotlib 없음 → --no-plot)
python3 -m venv ~/.venvs/linefollowing && ~/.venvs/linefollowing/bin/pip install numpy matplotlib
```

## 하드웨어 프로파일

`--profile 이름`으로 고른다. `"extends"`로 다른 프로파일을 상속(얕은 병합).

| 프로파일 | 파일 | 구성 |
|---|---|---|
| `spike_4s` | `robot_params.json` | 현행 실기: SPIKE 허브 + EV3 컬러 4(하이브리드 90/170mm) + 허브 IMU. `fit_params.py`가 갱신 |
| `spike_3s` | `profiles/spike_3s.json` | SPIKE 허브 + 컬러 3 (C·B 페어 @90mm, A 전방 센티넬 @130mm) + 허브 IMU |
| `ev3_3s` | `profiles/ev3_3s.json` | EV3(Pybricks v4) + 컬러 3 (S1·S2 페어, S3 센티넬) + 자이로 S4 + 모터 4. **모터·비용·자이로는 실측 전 임시값** |
| `ev3_row3_ref` | `profiles/ev3_row3_ref.json` | `EV3_라인트레이싱_쉬운주석.py` 재현: 컬러 3 일렬 + 모터 4, Pybricks v2 API |
| `spike_3s_real` | `profiles/spike_3s_real.json` | 2026-10-17 실기 측정 동역학: 기어 1.667, 회전 권한, 구동 지연, 센서 비동기 1,070Hz |
| `spike_3s_geo` | `profiles/spike_3s_geo.json` | 실측 치수: 3개 일렬 23mm 간격 @170mm, 윤거 205mm, 바퀴 80mm |
| `spike_3s_geo2` | `profiles/spike_3s_geo2.json` | **지금 기본 평가 모델.** geo + 회전 보정(`yaw_tau_s` 0.05, w_eff 1.5, 실기 블랙박스 5개로 맞춤). 평가할 때 `--weff 1.4,1.6` |
| `ev3_3s_geo` | `profiles/ev3_3s_geo.json` | EV3판 실측 치수: 포트 S1·S3·S2, 모터 A·B / C·D, 자이로 없음 |
| `spike_3s1_geo`, `spike_4row_geo`, `spike_3s1f_geo` (+ `_geo2`) | `profiles/` | 센서 4개 실험: 3개 + D 읽기만 / 4개 일렬(±11.5·±34.5mm) / 3개 + 앞 가운데(195mm) |

주요 파라미터 (기본값은 기존 SPIKE 모델과 비트 단위로 동일):

| 키 | 의미 |
|---|---|
| `sensor_ports`, `sensor_pos_mm`, `sensor_white/black` | 센서 개수 = 포트 수. 위치는 센서별 [전방mm, 측방mm(+좌)] |
| `motors` | `{포트: [측 "L"\|"R", 장착부호]}` — 물리 duty × 부호 = 전진 duty. 한 측 2개 = 4모터 |
| `call_cost_us` | 호출 비용 µs `{color_read, gyro_read, motor_dc, motor_read}`. 없으면 color_read = 1/(loop_hz×센서수) |
| `gyro` | 외장 자이로(EV3) `{source: "sensor", port, mount, lag_ms, noise_dps, bias_dps, max_dps, update_hz}` |
| `w_eff_scale` | 유효 윤거 배율 (스키드 회전 미끄러짐) |
| `mu` | 횡가속 한계 μ — 넘으면 회전이 덜 됨(언더스티어). None = 슬립 없음 |
| `brake_tau_s`, `coast_tau_s` | `brake()/hold()`, `stop()` 감속 시정수. None = `dc(0)`과 같음 |
| `speed_est_tau_s` | `Motor.speed()` 추정 지연 |
| `pybricks_api` | `v2`일 때만 `pybricks.media.ev3dev` import 가능 (v4 실기와 동일하게 실패) |

## 실행

```bash
python3 sim/run_sim.py --list                                   # 프로파일·트랙 목록
python3 sim/run_sim.py MRC_line_following_v16.py --profile spike_4s \
    --sub "KG_EFF = KG_S * gyro_sign_check()=>KG_EFF = KG_S"     # 시작 스핀 아티팩트 우회
python3 sim/run_sim.py EV3_라인트레이싱_쉬운주석.py --profile ev3_row3_ref --track photo
python3 sim/run_sim.py sim/tests/ev3_smoke.py --profile ev3_3s --no-plot
```

트랙: `easy`, `main`, `hard`, `mrc`(1.6배), `mrc_real`(실물 스케일), `photo`(경기장 사진 추출, 195cm 매트·6.89m),
`rightangle`(직선 뒤 직각 3번), `photo@배율`(사진 트랙 확대 — `photo@1.3`이면 원 반지름 ≈124mm·원 중심→교차점 ≈178mm로 실기 원 크기와 맞음).
결과에 LED 이벤트 수가 함께 찍힌다 (v16의 노랑 = 이탈 진입).

## 목(mock) 지원 범위

| 플랫폼 | 모듈 | 지원 |
|---|---|---|
| SPIKE | `hubs.PrimeHub` | `imu.angular_velocity/heading/acceleration`, light, speaker, battery |
| SPIKE | `iodevices.PUPDevice` | EV3 컬러센서 `read(mode=0)` |
| 공통 | `pupdevices.Motor`, `ev3devices.Motor` | `dc/run/stop/brake/hold/angle/speed/reset_angle` — **angle·speed는 모터축 기준(바퀴÷기어비)** |
| EV3 | `hubs.EV3Brick` | light, speaker, battery, buttons, screen(스텁). 내장 IMU 없음 |
| EV3 | `ev3devices.ColorSensor` | `reflection()` (+ color/ambient/rgb 근사) |
| EV3 | `ev3devices.GyroSensor` | `speed/angle/reset_angle`, `positive_direction` 규약(기본 시계방향 +) |
| EV3 | `media.ev3dev`, `robotics.DriveBase` | v2에서만 import / 주행 명령 미지원 |

`run_angle` 같은 위치제어는 미지원(NotImplementedError). 포트가 프로파일과 안 맞으면 실기처럼 에러.

## 회귀 테스트

core·목을 고친 뒤에는 반드시 돌린다. 궤적 해시까지 골든(`regress_golden.json`)과 비교.

```bash
python3 sim/regress.py                 # 비교 (21케이스, 약 20초)
python3 sim/regress.py --only v16,ev3  # 일부만
python3 sim/regress.py --capture       # 의도한 변경 후 골든 갱신
```

케이스: v16 6월드·v14 3월드(실기 파일 무수정), GeoController 3월드, V5, 센서 응답곡선,
SPIKE 3센서·EV3 스모크(`sim/tests/`), EV3 참고 파일(사진 트랙). 골든은 위 homebrew 파이썬
(numpy 2.3)으로 저장됨 — 다른 numpy 버전이면 난수열이 달라질 수 있다.

## 3센서 규칙기반·배치 스윕 (2단계)

실기 파일: `spike/spike_lf3_rule.py`(SPIKE), `ev3/ev3_lf3_rule.py`(EV3) — 입출력만 다르고 로직은 같다.
`regress.py`의 `lf3_parity`가 같은 물리·입출력 조건에서 두 파일의 궤적이 같은지 확인한다.

```bash
python3 sim/sweep_lf3.py --phase screen      # 셀 × 32설정 × (2월드 × mrc·photo)
python3 sim/sweep_lf3.py --phase validate    # 셀별 기본값 + 상위 3 → 12월드 × mrc·photo
python3 sim/sweep_lf3.py --phase screen --cells pf130,v11 --n 8
```

| 셀 | 내용 |
|---|---|
| `pf130` / `pf110` / `pf150` | 페어(±12.5 @90mm) + 센티넬(중앙 @130/110/150mm) |
| `pf130_fp12`, `pf130_fp12_lowc` | 발자국 12mm (대비 그대로 / 흰-검 폭 60%로 비관 가정) |
| `pair2` | 센티넬 무시(`USE_SENTINEL = False`) — 센티넬 기여 확인용 |
| `row3` | 일렬 3개(±25·0 @90mm) — 참고 EV3 파일 방식 |
| `v11` · `v12` · `v16` | 4센서 기준선 (현재 배선·현재 VALUE로 맞춤, 같은 예산으로 튜닝) |
| `v14` | 마지막 실기 검증판 — 기본값만 |

튜닝값은 소스의 `NAME = 값` 줄만 치환해 실기 파일 그대로 실행한다. 모든 셀이 같은
월드(`make_world`: 모터 τ·배터리·노이즈·센서지연 10~25ms·장착오차·시작오프셋)를 쓰고,
최대진행이 5초간 안 늘면 `stuck`으로 끊는다. 결과: `sweep_lf3_screen.json`, `sweep_lf3_validate.json`.

## 실기 진단 (시뮬 보정)

같은 진단 프로그램을 실기와 시뮬에서 돌려 '지문'을 비교한다.

```bash
# 실기: Pybricks Code에서 spike/spike_diag.py (EV3는 ev3/ev3_diag.py) 실행 → 출력 저장
python3 sim/diag_parse.py diag_log.txt                     # 실기 값 해석 + 프로파일 제안
python3 sim/diag_parse.py --sim spike_4s                   # 시뮬 지문 (파서 자체 검증)
python3 sim/diag_parse.py diag_log.txt --sim spike_4s      # 실기 vs 시뮬
python3 sim/diag_parse.py --sim ev3_3s --diag ev3/ev3_diag.py
```

| 단계 | 재는 것 | 시뮬 파라미터 |
|---|---|---|
| [1] | read·IMU(자이로)·dc 호출 비용 | `call_cost_us` |
| [2] | 센서별 갱신 간격, 동시 갱신 여부, '찢어진 읽기'(읽는 도중 프레임 바뀜) 비율 | `p_torn` (fastsim) |
| [3] | 라인 주행 중 프레임 동기를 통과한 실제 제어 횟수/초 | 검증용 |
| [4] | **한 번만 놓으면** 직각 라인을 5가지 속도×2회 통과하고 매번 출발점으로 스스로 복귀(좌우 모터각 동기 직진) → 감지 위치의 속도 기울기 = 센서 지연 | `sensor_lag_ms` |
| [5] | **라인 위 제자리 좌우 회전**(±40°, 요레이트 3단×왕복 3회) → 반시계·시계 검정구간 중심 차 = 2×회전속도×지연. 각도는 자이로 적분이라 바퀴 미끄러짐·`Motor.angle()` 지연과 무관 | `sensor_lag_ms` (권장) |

시뮬로 검증함: `spike_4s`에서 read 47µs·루프 5,303Hz·찢어짐 0.74~0.77(이론 0.75)·지연 15.0~15.6ms(정답 15)를 되찾는다.
`ev3_3s`(임시값)에서도 read 49·자이로 50·dc 20µs·찢어짐 0.66(이론 2/3)·지연 15~17ms.
자동 복귀판 [4]도 시뮬에서 지연 15.5ms(정답 15)를 되찾는다 (한 번 놓고 10런, 복귀 오차 −11°로 매번 같음).
[5]는 시뮬에서 15.5 ± 0.3ms(정답 15). [4]의 실기 한계: 빠른 런 뒤 급제동 미끄러짐으로 출발점이
앞으로 밀린다(758°/s 뒤 ≈18°) → 파서는 '앞 런이 350°/s 이하인 연속 쌍'만으로 추정하고, 값은 모터각 대비 상대 지연.

빛값: `spike/spike_value.py` (EV3 `ev3/ev3_value.py`) — 4초 동안 라인을 가로질러 손으로 쓸면 센서별
상위 3%=흰·하위 3%=검을 재서 `VALUE = [...]` 줄을 출력 (시뮬 손 쓸기 시험: 정답과 ±1).

**첫 실기 측정 (2026-10-17, SPIKE + EV3 컬러센서 3개, `diag_logs/spike_2026-10-17.txt`)**

| 항목 | 실기 | 시뮬 `spike_3s` |
|---|---|---|
| 호출 비용 µs | read 28 · IMU 30 · dc 20 · StopWatch 9 | read 47(루프 오버헤드 포함 실효값) · 나머지 0 |
| 센서 프레임 | 센서별 ≈1,070Hz, **서로 독립(비동기)** — 동시 변화 비율 0.29~0.52, 찢어짐 구조 없음 | 1,457Hz 공통 프레임 |
| 값 변화/s (주행) | 센서별 349 · 238 · 298 (프레임의 ≈1/3만 값이 바뀜 = 노이즈 작음) | ≈1,070 |
| 진단 [3] 제어 스텝/s | 778 | 2,087 |
| 지연 | **무효** — 옛 [4]는 속도마다 다시 놓아 놓은 위치 차(≈36mm)가 기울기에 섞임 → 자동 복귀판으로 재측정 필요 | 15ms (DR 10~25) |
| 흰/검 (흰 바닥 기준) | 60/8 · 39/5 · 39/4 | 26/2 · 25/2 · 28/2 |

2차 측정(`diag_logs/spike_2026-10-17_2.txt`, 자동복귀 [4]): 저속 쌍 추정 −3.5 ± 1.4ms(모터각 대비) →
센서 지연은 모터각 읽기 지연과 비슷하거나 작음 = 시뮬 가정 15ms보다 훨씬 작을 가능성 → [5]로 절대값 확인 중.

실기 규칙·RL 파일은 정지 버튼을 누르면 `STAT,steps,…,ms,…`(실제 제어 스텝 수)를 출력한다 →
스텝당 파이썬 연산 비용(시뮬에 없음)을 이 값으로 맞출 예정.

## 실기 블랙박스 · 실기형 동역학

- 규칙·RL 실기 파일은 4ms마다 [생값 3개, 조향, 파워, 상태비트, 요레이트]를 500개(2초) 링버퍼에 기록하고,
  **정지 버튼**을 누르거나 `AUTO_STOP`으로 멈추면 `BB`·`BBD` 줄로 최신 기록부터 출력 → `python3 sim/bb_parse.py 로그.txt [--events]`.
  (E8R7은 1000개 = 4초였음. E8R8부터 2초로 줄이고, 정지하면 모터를 먼저 세움. `BB_PRINT = False`면 `STAT` 한 줄만)
- `profiles/spike_3s_real.json` = 실기 측정으로 맞춘 동역학: 기어 20T→12T(1.667), 모터 V(100)≈1,021°/s(최고속 0.83m/s),
  블랙박스 직각 장면에서 피팅한 회전 권한 ≈750°/s(옛 시뮬의 0.30배, 유효 윤거 1.77배), τ_up 0.2·τ_dn 0.15, 센서 비동기 1,070Hz.
  옛 동역학에선 실기 '직각에서 직진'이 안 보였음(로봇이 3배 빨리 돎). sweep 설정 `_PROFILE`·`_WEFF`(회전 권한 불확실성)·`_MU`,
  `--tracks photo,rightangle`(날카로운 직각 시험 트랙)로 이 조건 검증.

## 강화학습 준비 (3단계, `sim/rl/`)

| 파일 | 역할 |
|---|---|
| `rl/tracks.py` | 규정 기반 랜덤 트랙 생성기(라인 20mm·최소 R100·직각·예각·루프 직각교차·지그재그·S웨이브·마커, easy/medium/hard). 홀드아웃 시드 ≥ 900000은 평가 전용 |
| `rl/fastsim.py` | core와 같은 물리·센서를 N대 동시에 (numpy). 틱 = 센서 프레임, 지연 = 프레임 링버퍼, `torn()` = 찢어진 읽기 재현 |
| `rl/rule_lf3.py` | `spike_lf3_rule.py` 로직의 벡터화판. 상수는 실기 파일에서 직접 읽음 |
| `rl/env.py` | RL 환경: 관측 22·크리틱 12·행동 2(Δ조향·파워), 5ms 결정, 도메인 랜덤화 |
| `rl/parity_test.py` | core+실기 파일 vs fastsim+RuleLF3 비교, `--bench` 속도 |

```bash
python3 -m rl.tracks hard             # (sim/에서) 트랙 8개 생성 요약
python3 -m rl.parity_test --worlds 3 --tracks mrc,photo,gen1
```

- **일치**: `p_torn = 2/3`(core와 같은 찢어진 읽기)이면 직선 2.5m 도달 5.13 vs 5.03s, 랜덤 월드
  완주 여부 8/9 일치. `p_torn = 0`이면 지연 15ms에서 16% 느려짐 → core는 센서를 차례로 읽다
  프레임이 바뀌면 같은 프레임에서 한 번 더 제어하기 때문 (core 제어 2,100~2,300회/s vs 실기 벤치 '값
  바뀐 루프' 1,457회/s). 실기 값은 진단 [3]으로 확인, 그 전까지 RL은 p_torn ∈ [0, 2/3] 랜덤화.
- **속도**: 1,024대 동시 약 11,700결정/s (합산 56배 실시간, 1억 결정 ≈ 2.4시간).
  4단계에서 numba로 컴파일하면 10배 이상 기대 (iCloud 밖 venv 필요).

## GPU · 강화학습 (4단계)

학습용 가상환경은 iCloud 밖에 있다: `~/.venvs/linefollowing` (numpy, torch 2.14 + MPS, mpy-cross).

| 파일 | 역할 |
|---|---|
| `rl/torchsim.py` | fastsim·규칙기반·학습환경의 PyTorch판 (Apple GPU=MPS). env별 설정·배치 지원 |
| `rl/sweep_gpu.py` | GPU 대규모 스윕 — 배치 × 설정 수천 개를 한 번에 (상위만 core로 재검증) |
| `rl/ppo.py` | PPO 잔차 정책 학습 (비대칭 액터-크리틱). `--mode residual2` = Δ조향·Δ파워를 규칙기반 위에 |
| `rl/export_policy.py` | 학습 정책 → **Pybricks 실기 파일** `spike/spike_lf3_rl.py`, `ev3/ev3_lf3_rl.py` |

```bash
cd sim
~/.venvs/linefollowing/bin/python -m rl.torchsim --check --bench      # numpy판과 일치·속도
~/.venvs/linefollowing/bin/python -m rl.sweep_gpu --n 192 --worlds 100,101,102,103
python3 sweep_lf3.py --phase validate --configs top.json --out sweep_lf3_validate_gpu.json
~/.venvs/linefollowing/bin/python -m rl.ppo --layout row3 --cfg best.json --actor-hid 16 --act relu
~/.venvs/linefollowing/bin/python -m rl.export_policy rl/runs/<이름>/policy.pt
```

- **속도**: GPU 16,384대 동시 65만 결정/s (numpy판의 약 55배), 1억 결정 ≈ 2.6분.
- **조건 옵션**: `sweep_gpu --lag 2,15 --noise 0.2,1.0 --no-slow --rank finish --top-out top.json`
  (`--no-slow` = POWER 100·SPD_K10 0·RECOVER_POWER 100·PB 0 고정, 사용자 방침 '감속 없이 파워 100'),
  `--configs top.json --worlds 200:264`로 후보만 많은 월드에서 재평가. `ppo --mode steer_only --lag --noise`
  = 조향만 보정(파워는 규칙 100 그대로)·학습/평가 지연 범위. core 쪽 `sweep_lf3` 설정 dict의 `_LAG_MS`·`_NOISE`·
  `_ASYNC`(센서별 독립 갱신)·`_HZ`·`_PROFILE`·`_POLICY_US` = 시뮬 조건(난수열이 같아 월드끼리 짝 비교).
- **일치**: GPU판 vs numpy판, 같은 256개 첫 에피소드에서 완주 80.1% vs 79.3%, 도달·시간 동일 수준.
- **Pybricks 배포**: 신경망은 정수(×1000) 고정소수점 직선 코드로 생성 → float·numpy 없이 SPIKE·EV3 둘 다
  (EV3는 FPU 없음). `mpy-cross`(MicroPython 1.29) 컴파일 통과, 정수/float 정책 차이 ≤ 0.005,
  생성 파일을 시뮬 목에서 무수정 실행해 검증.
  - 오버플로 없음 보장: 내보낼 때 정수 중간값 최악을 구간 계산(`check_bounds`)해 small int(2³⁰)의 절반을
    넘으면 실패. 정규화는 범위를 곱하기 전에 봐서 곱이 ±8,000,000 이하 (v3: 최악 2.5억 = 한계의 23%).
  - 결정마다 리스트를 안 만듦(관측 22개를 인자로, 센서 이력은 변수 12개) → GC 멈칫 없음.
    생성 코드 == 정수 기준식(`policy_int`)을 극단값 포함 4,000개 입력으로 내보낼 때마다 자가검사.
  - 시작 시 `POLICY_US`(정책 1회 µs)를 출력 → 시뮬에선 `sweep_lf3` 설정 `{"_POLICY_US": µs}`
    (또는 프로파일 `call_cost_us.policy`)로 계산하는 동안 센서·모터가 멈추는 것을 재현.
- **v3 (row3, residual2, 액터 16-16 ReLU) core 100월드** — 12월드는 표본이 작아 순위가 뒤집혔었음:

  | | mrc 완주 | photo 완주 |
  |---|---|---|
  | 규칙 row3 (배포 기본값) | 26/100 16.91s | 90/100 8.96s |
  | RL v3, 정책 0ms | 43/100 15.63s | 100/100 8.67s |
  | RL v3, 정책 1ms | 52/100 15.75s | 98/100 8.71s |
  | RL v3, 정책 2ms | 45/100 15.70s | 96/100 8.80s |

  mrc 실패는 규칙·RL 공통으로 세 곳: 2.5m 비스듬한 교차(RL만 가끔), 6.0m 지그재그 예각 꼭짓점
  (모든 센서가 흰색이 되기 직전 꼭짓점에 오른쪽 센서가 마지막으로 걸려 '마지막 본 쪽' 기억이 오른쪽으로
  바뀜 → 반대로 210° 돌아 온 길을 역주행 → stuck), 8.0m 여러 선이 만나는 분기(다른 선을 따라감 → lost).

## 기타 도구

| 경로 | 역할 |
|---|---|
| `controller.py` | 실기 pid_control 이식본 V1~V10 (스윕용). V8~V10 구간 좌표는 옛 8.5m 트랙 기준 |
| `sweep*.py`, `sweep_geo.py` | 게인·배치 탐색 + 도메인 랜덤화. `sweep_geo` 지그재그 구간 트릭은 꺼짐(아래) |
| `fit_params.py` | `MRC_spike_calibration.py` 로그 → `robot_params.json` 피팅 |
| `photo_track.py` | 경기장 사진 → `viz/track_pts.json` |
| `viz/` | 인터랙티브 JS 시뮬 (아직 4센서 고정) |

## 분석 도구 (`tools/`, 2026-10-24)

실기 파일을 그대로 실행하면서 상수만 바꿔 비교한다. 모두 `/opt/homebrew/bin/python3.13`(numpy)으로 돌고, 그림이 필요한 `--png`만 matplotlib가 필요하다.

| 파일 | 역할 |
|---|---|
| `tools/corner_eval.py` | 변형 평가: 완주·시간·최대 이탈·뱅킹·피벗·루프 속도. `--base` 공통 상수, `--dump` 월드별 JSON |
| `tools/paired.py` | `--dump` 결과를 같은 월드끼리 짝지어 비교 (시간 차, 빨라진/느려진 월드 수) |
| `tools/loop_trace.py` | 원·교차로 구간의 월드별 결과와 바퀴축·센서 궤적 그림 |
| `tools/ra_trace.py` | 직각 트랙 첫 직각: 센서가 지나친 거리, 재진입 각도 |
| `tools/stop_test.py` | 정지 버튼 시험: 정지 후 출력량, 프로그램이 붙잡는 시간, 그동안 바퀴가 더 굴러간 거리 |
| `tools/step_dump.py` | 한 월드의 스텝별 센서·오차·조향·위치 기록 (실패 직전 구간 보기) |
| `tools/make4.py`, `make_guard.py`, `make_sent.py` | 센서 4개·원 대책·앞 센티넬 실험 변형을 만들어 `tools/variants/`에 저장 (git 제외) |

```bash
B='{"RAW_MODE": false, "VALUE": [34, 6, 22, 3, 21, 3], "AUTO_CAL": false}'   # 시뮬 센서 목은 반사 모드만
python3.13 sim/tools/corner_eval.py '[["E8R9", {}], ["CPRI 끔", {"CPRI": 0}]]' --src spike/spike_lf3_rule_e8r9_170.py \
  --profile spike_3s_geo2 --fwd 170 --prof '{"gear_ratio": 3.3333}' --weff 1.4,1.6 --base "$B" \
  --tracks rightangle,main,hard,photo@1.3 --lost 450 --worlds 32 --dump /tmp/e8r9.json
python3.13 sim/tools/paired.py "/tmp/e8r9.json:CPRI 끔" "/tmp/e8r9.json:E8R9"
python3.13 sim/tools/stop_test.py '[["E8R8", "spike/spike_lf3_rule_e8r8_170.py", {}]]' --prof '{"gear_ratio": 3.3333}' --track photo --stop 5000
```

`--fwd 0`이면 프로파일의 센서 위치를 그대로 쓴다 (센서 4개 프로파일은 반드시 0).

## 변경 이력

- **2026-10-17** N센서·프로파일·호출비용·EV3 목 일반화 (기존 결과 비트 동일, regress 통과).
  - 버그 수정: `run_controller`의 주행거리가 기어비를 한 번 더 곱해 기어배로 부풀었음.
    목 `Motor.angle()`도 바퀴각을 돌려줬음(→ 모터각으로 수정).
  - `sweep_geo` 지그재그 관통 구간 기본값 해제: 좌표가 1.6배 확대 전 기준이라 실제 지그재그
    (≈5759mm~)와 안 맞았고, 위 버그 때문에 기어2·4에선 첫 직선에서 켜지고 있었음.

- **2026-10-17 (오후)** 실기 진단 첫 로그 해석, 진단 [4] 자동 복귀판, 빛값 측정 도구, 실기 파일 STAT 출력,
  RL 내보내기 오버플로 검사·GC 없는 코드·POLICY_US, sweep 정책 연산시간 주입, 검증 요약 머리말 월드 수 표시.

- **2026-10-24** 실측 치수 프로파일(`spike_3s_geo`, 센서 170mm)과 회전 보정(`spike_3s_geo2`: core `yaw_tau_s`, 실험용 `turn_load`),
  `rightangle`·`photo@배율` 트랙, EV3판 프로파일(`ev3_3s_geo`), 센서 4개 실험 프로파일, `tools/` 분석 도구.
  실기 E 계열(E7 → E8R9) 개발과 평가는 루트 README 참고.

## 한계 (정직하게)

- 슬립은 횡가속 한계(언더스티어)만, 옆미끄럼 속도는 없음. 전압 강하·바닥 마찰 편차 없음
  → 도메인 랜덤화로 흡수. **시뮬 1등 ≠ 실기 1등**, 후보 추리기 용도.
- 센서는 반사광만 재현. 컨트롤러 자체 연산 시간은 비용에 안 들어감(API 호출 비용만, RL 정책만 `_POLICY_US`로 반영).
- 센서 프레임은 공통 1개(동기)로 모델링 — 실기는 센서별 독립(비동기)·노이즈 더 작음 (위 실측표).
- EV3 프로파일의 모터·지연·호출비용은 실측 전 임시값.
- 원(작은 루프)은 1배 `photo`에서 실기보다 훨씬 비관적이다(원이 실기보다 작음). 원 관련 비교는 `photo@1.3`으로 한다.
- 실기에서 라인 하나가 두 센서 사이에 걸치면 둘 다 30~45%로 읽혀 '가짜 직각 신호'가 생겼는데, 시뮬은 이걸 재현하지 못했다.
