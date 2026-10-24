# E8R8 + 앞쪽 가운데 센티넬(포트 D): 긴 직선(BRK_STR_MS 이상) 중 일렬 가운데는 라인 위인데 센티넬만 흰색이
#   SEN_N스텝 이어지면 = 앞에서 라인이 끊김(직각·예각) → 가지가 보이기 전에 브레이크를 먼저 검 (시뮬 실험용, 결과는 sim/tools/variants/)
import os
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + "/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "variants") + "/"
os.makedirs(OUT, exist_ok=True)
s = open(ROOT + "spike/spike_lf3_rule_e8r8_170.py", encoding="utf-8").read()


def rep(s, old, new):
    assert s.count(old) == 1, (old[:70], s.count(old))
    return s.replace(old, new)


DEV = "cF = dev(PUPDevice, Port.A)    # 가운데 (전방 90mm) — 페어+센티넬 배치면 전방 130mm 중앙\n"
s = rep(s, DEV, DEV + "cX = dev(PUPDevice, Port.D)    # 앞쪽 가운데 센티넬\n")
s = rep(s, "    rdL, rdR, rdF = cL.read, cR.read, cF.read     # 메서드를 지역 변수로 (루프에서 속성 조회 줄임)\n",
        "    rdL, rdR, rdF = cL.read, cR.read, cF.read     # 메서드를 지역 변수로 (루프에서 속성 조회 줄임)\n"
        "    rdX = cX.read\n    sen_n = 0\n")
s = rep(s, "\nSENSOR_GAP_MM = 23", "\nSEN_N = 3               # 센티넬 흰색 연속 스텝\nSEN_WB = [21, 3]        # 센티넬 [흰, 검]\nSENSOR_GAP_MM = 23")
s = rep(s, "            rF = rdF(0)[0]\n        now = swt()\n",
        "            rF = rdF(0)[0]\n            rX = rdX(0)[0]\n        now = swt()\n")
s = rep(s, "        nF = (min(max(rF, bF), wF) - bF) * 100 // gF\n",
        "        nF = (min(max(rF, bF), wF) - bF) * 100 // gF\n"
        "        nX = (min(max(rX, SEN_WB[1]), SEN_WB[0]) - SEN_WB[1]) * 100 // (SEN_WB[0] - SEN_WB[1])\n")
s = rep(s, "            br = 0                                  # 직진 중 가지(직각 신호): 가운데 + 한쪽 진한 검정, 반대쪽 흰색\n",
        "            if (SEN_N and straight and nX > W_NORM and nF < BLACK_MID and not brk_on and now - t_brk > 500\n"
        "                    and now - t_str0 >= BRK_STR_MS):\n"
        "                sen_n += 1\n"
        "                if sen_n >= SEN_N:              # 앞이 끊김 → 가지 보기 전에 브레이크\n"
        "                    brk_on = True\n"
        "                    brk_dir = 1\n"
        "                    t_brk = now\n"
        "            else:\n"
        "                sen_n = 0\n"
        "            br = 0                                  # 직진 중 가지(직각 신호): 가운데 + 한쪽 진한 검정, 반대쪽 흰색\n")
open(OUT + "v_sent.py", "w", encoding="utf-8").write(s)
print("만듦: v_sent.py")
