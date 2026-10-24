# E8R8 + 곡선 뱅킹 회전 속도 상한 옵션(RA_C_YAW, 기본 0 = 끔 = E8R8과 같음) — 시뮬 실험용 (결과는 sim/tools/variants/)
#   원(곡선 중)에서 라인을 놓치면 ST_BANK(85)로 강제 회전하는데 상한이 없음 → 직진 중 이탈(RA_YAW)처럼
#   곡선 뱅킹도 회전 속도가 RA_C_YAW°/s를 넘으면 넘은 만큼(RA_KQ) 조향을 줄임
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + "/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "variants") + "/"
os.makedirs(OUT, exist_ok=True)
s = open(ROOT + "spike/spike_lf3_rule_e8r8_170.py", encoding="utf-8").read()


def rep(s, old, new):
    assert s.count(old) == 1, (old[:70], s.count(old))
    return s.replace(old, new)


s = rep(s, "\nRA_KQ = 40 ", "\nRA_C_YAW = 0            # 곡선 뱅킹 회전 속도 상한 °/s (0 = 끔)\nRA_KQ = 40 ")
s = rep(s, "    ra_on = False           # 이번 뱅킹이 직진 중 이탈(직각)인가\n",
        "    ra_on = False           # 이번 뱅킹이 직진 중 이탈(직각)인가\n    ra_lim = RA_YAW\n")
s = rep(s, "                    bank_st = ST_BANK\n                    ra_on = False\n",
        "                    bank_st = ST_BANK\n                    ra_on = RA_C_YAW > 0\n                    ra_lim = RA_C_YAW\n")
s = rep(s, "                        ra_on = RA_YAW > 0\n",
        "                        ra_on = RA_YAW > 0\n                        ra_lim = RA_YAW\n")
s = rep(s, "                    if rate > RA_YAW:\n                        steer = bank_dir * (bank_st - RA_KQ * (rate - RA_YAW) // 100)\n",
        "                    if rate > ra_lim:\n                        steer = bank_dir * (bank_st - RA_KQ * (rate - ra_lim) // 100)\n")
open(OUT + "v_bank.py", "w", encoding="utf-8").write(s)
print("만듦: v_bank.py")
