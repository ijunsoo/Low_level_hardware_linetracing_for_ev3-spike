# E8R8에서 시뮬 실험용 4센서 변형 두 개 만들기 (시뮬 실험용 — 실기 배포 아님, 결과는 sim/tools/variants/)
#   v_read4.py : 3개 일렬 그대로 + 포트 D 센서를 매 스텝 읽기만 (안 씀) → '센서 하나 더 읽는 시간'만의 영향
#   v_4row.py  : 4개 일렬(바깥좌 D +34.5 · 안좌 C +11.5 · 안우 B −11.5 · 바깥우 A −34.5mm)로 바로 옮긴 판
#                오차 = 네 센서 진하기 무게중심, 아래 3센서 로직용 옆 = 바깥 센서, 가운데 = 안쪽 둘의 진하기 합
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + "/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "variants") + "/"
os.makedirs(OUT, exist_ok=True)
SRC = open(ROOT + "spike/spike_lf3_rule_e8r8_170.py", encoding="utf-8").read()


def rep(s, old, new):
    assert s.count(old) == 1, (old[:70], s.count(old))
    return s.replace(old, new)


DEV = "cF = dev(PUPDevice, Port.A)    # 가운데 (전방 90mm) — 페어+센티넬 배치면 전방 130mm 중앙\n"
RDS = "    rdL, rdR, rdF = cL.read, cR.read, cF.read     # 메서드를 지역 변수로 (루프에서 속성 조회 줄임)\n"

# ---- v_read4: 읽기만 ----
s = rep(SRC, DEV, DEV + "cX = dev(PUPDevice, Port.D)    # 4번째 센서 (읽기 비용 시험: 읽기만 하고 안 씀)\n")
s = rep(s, RDS, RDS + "    rdX = cX.read\n")
s = rep(s, "            rF = rdF(0)[0]\n        now = swt()\n",
        "            rF = rdF(0)[0]\n            rX = rdX(0)[0]\n        now = swt()\n")
open(OUT + "v_read4.py", "w", encoding="utf-8").write(s)

# ---- v_4row: 4개 일렬 ----
s = rep(SRC, DEV, DEV + "cX = dev(PUPDevice, Port.D)    # 4개 일렬: D 바깥좌 · C 안좌 · B 안우 · A 바깥우\n")
s = rep(s, RDS, RDS + "    rdX = cX.read\n")
s = rep(s, "AUTO_CAL = True", "AUTO_CAL = False")
s = rep(s, "\nSENSOR_GAP_MM = 23", "\nVALUE4 = [34, 6, 22, 3, 21, 3, 22, 3]   # [흰·검] 바깥좌 D, 안좌 C, 안우 B, 바깥우 A\nSENSOR_GAP_MM = 23")
s = rep(s, """    pL, pR, pF = rd3()
    sL = (min(max(pL, bL), wL) - bL) * 100 // gL
    sR = (min(max(pR, bR), wR) - bR) * 100 // gR
    sF = (min(max(pF, bF), wF) - bF) * 100 // gF
""", """    pL, pR, pF = rd3()
    pX = rdX(0)[0]
    wLo, bLo, wLi, bLi, wRi, bRi, wRo, bRo = VALUE4
    gLo, gLi, gRi, gRo = wLo - bLo, wLi - bLi, wRi - bRi, wRo - bRo
    sL = sR = sF = 100
    sLi = sRi = 100
""")
s = rep(s, """        if md:
            a = rdL(3)
            rL = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
            a = rdR(3)
            rR = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
            a = rdF(3)
            rF = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
        else:
            rL = rdL(0)[0]
            rR = rdR(0)[0]
            rF = rdF(0)[0]
        now = swt()
        # 프레임 동기: 센서 새 값에서만 제어 (값이 오래 안 변하면 FORCE_MS마다 한 번)
        if rL == pL and rR == pR and rF == pF and now - t_step < FORCE_MS:
            continue
        pL, pR, pF = rL, rR, rF
        STAT[0] += 1                                # 제어 스텝 수 (멈출 때 출력)
        dt = now - t_step
        t_step = now

        nL = (min(max(rL, bL), wL) - bL) * 100 // gL
        nR = (min(max(rR, bR), wR) - bR) * 100 // gR
        nF = (min(max(rF, bF), wF) - bF) * 100 // gF
""", """        rLi = rdL(0)[0]
        rRi = rdR(0)[0]
        rRo = rdF(0)[0]
        rLo = rdX(0)[0]
        now = swt()
        if rLi == pL and rRi == pR and rRo == pF and rLo == pX and now - t_step < FORCE_MS:
            continue
        pL, pR, pF, pX = rLi, rRi, rRo, rLo
        rL, rR, rF = rLo, rRo, rLi
        STAT[0] += 1
        dt = now - t_step
        t_step = now

        nLo = (min(max(rLo, bLo), wLo) - bLo) * 100 // gLo
        nLi = (min(max(rLi, bLi), wLi) - bLi) * 100 // gLi
        nRi = (min(max(rRi, bRi), wRi) - bRi) * 100 // gRi
        nRo = (min(max(rRo, bRo), wRo) - bRo) * 100 // gRo
        nL = nLo                                    # 아래 3센서 로직용: 옆 = 바깥 센서
        nR = nRo
        v = 200 - nLi - nRi                         # 가운데 = 안쪽 둘의 진하기 합 (라인이 둘 사이면 둘 다 회색)
        nF = 100 - (v if v < 100 else 100)
""")
s = rep(s, """        sF = (ALPHA * nF + A_COMP * sF) // 1000
""", """        sF = (ALPHA * nF + A_COMP * sF) // 1000
        sLi = (ALPHA * nLi + A_COMP * sLi) // 1000
        sRi = (ALPHA * nRi + A_COMP * sRi) // 1000
""")
a = s.index("        if ERR_MODE == 1:")
b = s.index("            error = -ymm * 87 // 10", a)
b = s.index("\n", b) + 1
s = s[:a] + """        if ERR_MODE == 1:                           # 4개 일렬: 네 센서 진하기 무게중심 (좌 +, mm)
            dLo = 100 - DFLOOR - sL
            dLi = 100 - DFLOOR - sLi
            dRi = 100 - DFLOOR - sRi
            dRo = 100 - DFLOOR - sR
            if dLo < 0:
                dLo = 0
            if dLi < 0:
                dLi = 0
            if dRi < 0:
                dRi = 0
            if dRo < 0:
                dRo = 0
            tot = dLo + dLi + dRi + dRo
            if tot > SIDE_SEE:
                ymm = (345 * (dLo - dRo) + 115 * (dLi - dRi)) // (10 * tot)
                if ymm > 2:
                    ysgn = 1
                elif ymm < -2:
                    ysgn = -1
            else:
                ymm = ysgn * 35
            error = -ymm * 87 // 10
""" + s[b:]
open(OUT + "v_4row.py", "w", encoding="utf-8").write(s)
print("만듦: v_read4.py, v_4row.py")
