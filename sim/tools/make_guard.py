# E8R8 + 루프 탈출 교차로 대책 후보들 (스위치, 기본 모두 끔 = E8R8과 같음) — 시뮬 실험용 (결과는 sim/tools/variants/)
#   LOOP_DEG: 자이로로 한 방향 누적 회전을 세서 이 각도(°) 넘으면 '루프 끝(lx)' 상태. 느린 요레이트 평균(LOOP_TAU_MS)이 LOOP_RST°/s 아래면 0으로
#   CPRI·CPRI_MODE: 가운데가 진한 검정(< CPRI)이면 옆 센서 무시하고 오차 0 (1 = 루프 끝에서만, 2 = 항상)
#   LX_CLAMP: 루프 끝 상태에서 루프 방향으로 더 꺾는 조향을 이 값으로 제한 (교차하는 선으로 끌려가지 않게)
#   XHOLD_MS: 양옆이 함께 검정(교차로)을 본 뒤 이 시간 동안 조향 0 (자이로 감쇠만 = 직진 유지)
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + "/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "variants") + "/"
os.makedirs(OUT, exist_ok=True)
s = open(ROOT + "spike/spike_lf3_rule_e8r8_170.py", encoding="utf-8").read()


def rep(s, old, new):
    assert s.count(old) == 1, (old[:70], s.count(old))
    return s.replace(old, new)


s = rep(s, "\nSENSOR_GAP_MM = 23",
        "\nLOOP_DEG = 0\nLOOP_TAU_MS = 150\nLOOP_RST = 80\nCPRI = 0\nCPRI_MODE = 1\nLX_CLAMP = 0\nXHOLD_MS = 0\n"
        "SENSOR_GAP_MM = 23")
s = rep(s, "    sw = StopWatch()\n    swt = sw.time\n",
        "    turn_acc = 0\n    yl = 0\n    lx = 0\n    t_xhold = -10000\n    sw = StopWatch()\n    swt = sw.time\n")
s = rep(s, "        nF = (min(max(rF, bF), wF) - bF) * 100 // gF\n",
        "        nF = (min(max(rF, bF), wF) - bF) * 100 // gF\n"
        "        if LOOP_DEG:\n"
        "            yq = int(av(YAW_AXIS)) * YAW_SIGN\n"
        "            turn_acc += yq * dt\n"
        "            yl += (yq * 1000 - yl) * dt // LOOP_TAU_MS\n"
        "            if -LOOP_RST * 1000 < yl < LOOP_RST * 1000:\n"
        "                turn_acc = 0\n"
        "            lx = 1 if turn_acc >= LOOP_DEG * 1000 else (-1 if turn_acc <= -LOOP_DEG * 1000 else 0)\n")
s = rep(s, "            error = -ymm * 87 // 10                 # mm → E2 오차 단위 (옆 센서 하나 완전 검정 = ±200)\n",
        "            if CPRI and nF < CPRI and (CPRI_MODE == 2 or lx != 0):\n"
        "                ymm = 0\n"
        "            error = -ymm * 87 // 10                 # mm → E2 오차 단위 (옆 센서 하나 완전 검정 = ±200)\n")
s = rep(s, "            if sL < BLACK_MID and sR < BLACK_MID:\n                steer = 0                           # 교차: 직진 유지\n",
        "            if sL < BLACK_MID and sR < BLACK_MID:\n                steer = 0                           # 교차: 직진 유지\n"
        "                t_xhold = now\n"
        "            elif XHOLD_MS and now - t_xhold <= XHOLD_MS:\n"
        "                steer = 0\n")
s = rep(s, "        delta = steer - prev_steer                  # 슬루 리미터\n",
        "        if LX_CLAMP and lx != 0:\n"
        "            if lx > 0 and steer < -LX_CLAMP:\n"
        "                steer = -LX_CLAMP\n"
        "            elif lx < 0 and steer > LX_CLAMP:\n"
        "                steer = LX_CLAMP\n"
        "        delta = steer - prev_steer                  # 슬루 리미터\n")
open(OUT + "v_guard.py", "w", encoding="utf-8").write(s)
print("만듦: v_guard.py")
