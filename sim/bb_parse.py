# =====================================================================
#  블랙박스 해석: spike_lf3_rule.py / ev3_lf3_rule.py (+ *_rl.py)가 정지 버튼에서 출력하는
#  BB / BBD 줄(정지 직전 2초, 4ms 간격) → 사람이 읽는 시간표 + 이벤트 요약
#
#    python3 sim/bb_parse.py 붙여넣은로그.txt            # 전체 표
#    python3 sim/bb_parse.py 로그.txt --events           # 상태가 바뀌는 순간만
#
#  열: t(ms, 정지 시각 = 0) · 생값 좌/우/가운데 · 정규화(%) · 조향(−=좌) · 파워 · 상태 · 요레이트(°/s, +=좌회전)
#  상태: 선=라인 보임(제어) 앞=가운데가 최근 검정 흰=좌우 둘 다 흰 뱅=이탈 뱅킹 중
#        기억 L/R = 마지막으로 라인을 본 쪽, 뱅킹 L/R = 뱅킹 방향, 예=코너 예고, 코=코너 회전 고정
# =====================================================================
import argparse
import sys


HEX = set("0123456789abcdef")


def parse(text):
    """BB 머리줄 + BBD 줄들. 여러 번 정지했으면 마지막 블록. 붙여넣다 줄이 접혀
    'BBD,' 없이 이어진 16진 조각도 앞 줄에 이어 붙인다."""
    hdr, data, inside = None, "", False
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("BB,"):
            p = line.split(",")
            hdr = dict(ms=int(p[1]), n=int(p[2]), w=int(p[3]), value=[int(x) for x in p[4:10]],
                       newest_first=len(p) > 10 and p[10] == "new")
            data, inside = "", True
        elif line.startswith("BBD,") and hdr:
            data += line[4:]
        elif inside and line and set(line.lower()) <= HEX:
            data += line.lower()
        elif line:
            inside = inside and line.startswith("BBD")
    if not hdr:
        raise SystemExit("BB 줄이 없음 — 정지 버튼을 눌러 나온 출력 전체를 붙여넣을 것")
    raw = bytes.fromhex(data)
    w = hdr["w"]
    rows = [raw[i:i + w] for i in range(0, len(raw) - w + 1, w)]
    if len(rows) < hdr["n"]:
        print("※ 기록 %d개 중 %d개만 받음 (출력이 잘림) — %s 쪽만 있음" % (
            hdr["n"], len(rows), "정지 직전" if hdr.get("newest_first") else "앞"))
    if hdr.get("newest_first"):
        rows = rows[::-1]                       # 최신→과거로 출력됨 → 시간순으로
    return hdr, rows


def norm(r, w, b):
    return max(0, min(100, (min(max(r, b), w) - b) * 100 // max(1, w - b)))


def decode(hdr, rows):
    wL, bL, wR, bR, wF, bF = hdr["value"]
    out = []
    n = len(rows)
    for k, r in enumerate(rows):
        f = r[5]
        out.append(dict(t=(k - n + 1) * hdr["ms"], rL=r[0], rR=r[1], rF=r[2],
                        nL=norm(r[0], wL, bL), nR=norm(r[1], wR, bR), nF=norm(r[2], wF, bF),
                        steer=r[3] - 128, power=r[4], online=bool(f & 1), ahead=bool(f & 2),
                        white=bool(f & 4), bank=bool(f & 8), side="R" if f & 16 else "L",
                        bdir="R" if f & 32 else "L", armed=bool(f & 64), corner=bool(f & 128),
                        yaw=(r[6] - 128) * 8))
    return out


def fmt(d):
    st = "".join([("선" if d["online"] else "  "), ("앞" if d["ahead"] else "  "), ("흰" if d["white"] else "  "),
                  ("뱅" + d["bdir"] if d["bank"] else "   "), ("예" if d["armed"] else "  "), ("코" if d.get("corner") else "  ")])
    return "%6d  %3d %3d %3d | %3d%% %3d%% %3d%% | 조향 %+4d 파워 %3d | %s 기억%s | 요 %+5d" % (
        d["t"], d["rL"], d["rR"], d["rF"], d["nL"], d["nR"], d["nF"], d["steer"], d["power"], st, d["side"], d["yaw"])


def events(rows):
    """상태가 바뀌는 순간 + 핵심 진단."""
    out, prev = [], None
    for d in rows:
        key = (d["online"], d["white"], d["bank"], d["bdir"] if d["bank"] else "", d["side"])
        if key != prev:
            out.append(d)
        prev = key
    return out


def diagnose(rows):
    msgs = []
    allw = [d for d in rows if d["nL"] > 75 and d["nR"] > 75 and d["nF"] > 75]
    banks = [d for d in rows if d["bank"]]
    if allw and not banks:
        msgs.append("세 센서 모두 흰색인 기록이 %d개(%.0fms)인데 뱅킹이 한 번도 없음 → 이탈 판정이 안 됨 "
                    "(상태 '선'이 계속 켜져 있으면 앞/흰 비트 확인)" % (len(allw), len(allw) * 4))
    if banks:
        b0 = banks[0]
        msgs.append("첫 뱅킹 t=%dms 방향 %s (그때 기억 %s, 조향 %+d, 요레이트 %+d°/s)"
                    % (b0["t"], b0["bdir"], b0["side"], b0["steer"], b0["yaw"]))
        turning = [d for d in banks if (d["yaw"] > 60 if d["bdir"] == "L" else d["yaw"] < -60)]
        msgs.append("뱅킹 중 실제로 그 방향으로 도는 기록 %d/%d (요레이트 60°/s 넘음)" % (len(turning), len(banks)))
    sat = [d for d in rows if abs(d["steer"]) >= 100]
    if sat:
        msgs.append("조향 포화(±100) %d개" % len(sat))
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", help="붙여넣은 출력 (없으면 표준입력)")
    ap.add_argument("--events", action="store_true", help="상태가 바뀌는 순간만")
    args = ap.parse_args()
    text = open(args.log, encoding="utf-8").read() if args.log else sys.stdin.read()
    hdr, raw = parse(text)
    rows = decode(hdr, raw)
    print("블랙박스 %d개 × %dms (VALUE %s)" % (len(rows), hdr["ms"], hdr["value"]))
    print("     t  생값 좌 우 가 | 정규화 좌  우  가 | 조향·파워         | 상태          | 요레이트")
    for d in (events(rows) if args.events else rows):
        print(fmt(d))
    print("\n== 진단 ==")
    for m in diagnose(rows):
        print("- " + m)


if __name__ == "__main__":
    main()
