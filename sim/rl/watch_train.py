# =====================================================================
#  강화학습(ppo) 진행을 터미널에서 실시간으로 보여 줌 — 학습 출력 로그를 2초마다 다시 읽음
#
#    python3 sim/rl/watch_train.py 학습출력.log [--total 200] [--title 이름]
#
#  학습 완주율 = 학습 중 끝난 에피소드 가운데 결승까지 간 비율 (무작위 출발점·월드)
#  평가       = 트랙마다 고정 월드 12개 · 출발점 고정 · 평균 행동 (25회마다, E2 규칙과 같은 조건)
# =====================================================================
import argparse
import json
import os
import re
import time
import unicodedata

IT_RE = re.compile(r"it\s+(\d+)\s*\|\s*결정\s*([\d.]+)M\s*\|\s*에피소드 완주\s*([\d.]+)\s*도달\s*([\d.]+)"
                   r"\s*\|\s*std\s*\[([\d.]+),\s*([\d.]+)\]\s*\|\s*([\d.]+)s")
BARS = "▁▂▃▄▅▆▇█"
EVAL_EVERY = 25


def width(s):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def pad(s, w):
    return s + " " * max(0, w - width(s))


def parse(path):
    its, evals, rule, done = [], [], None, False
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            m = IT_RE.match(s)
            if m:
                its.append(dict(it=int(m.group(1)), dec=float(m.group(2)), fin=float(m.group(3)),
                                reach=float(m.group(4)), std=(float(m.group(5)), float(m.group(6))),
                                sec=float(m.group(7))))
            elif s.startswith("평가:"):
                try:
                    evals.append((its[-1]["it"] if its else 0, json.loads(s.split(":", 1)[1])))
                except ValueError:
                    pass                        # 쓰는 도중인 줄
            elif s.startswith("규칙기반 기준 평가:"):
                try:
                    rule = json.loads(s.split(":", 1)[1])
                except ValueError:
                    pass
            elif s.startswith("완료:"):
                done = True
    return its, evals, rule, done


def spark(vals, n=48):
    vals = vals[-n:]
    if not vals:
        return ""
    return "".join(BARS[min(7, int(v * 8))] for v in vals)


def frac(d, key):
    r = d.get(key) or {}
    if not r.get("n"):
        return "-"
    return "%d/%d" % (r["fin"], r["n"])


def row(label, d):
    cells = [pad(label, 14), pad(frac(d, "photo"), 10), pad(frac(d, "mrc"), 9),
             pad(frac(d, "_holdout"), 12), "%3.0f%%" % (100 * d.get("_all_finish", 0))]
    return "  ".join(cells)


def fmt_min(sec):
    return "%d분 %02d초" % (sec // 60, sec % 60) if sec >= 60 else "%d초" % sec


def cpu_rows(path):
    rows = []
    if not path or not os.path.exists(path):
        return rows
    for line in open(path, encoding="utf-8"):
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    return rows


def cell(r):
    if not r:
        return "-"
    return "%d/%d%s" % (r["fin"], r["n"], (" %.1fs" % r["time"]) if r.get("time") else "")


def draw(path, total, title, cpu=None):
    its, evals, rule, done = parse(path)
    st = os.stat(path)
    born = getattr(st, "st_birthtime", st.st_ctime)
    now = time.time()
    out = ["\033[H\033[2J", "━━ 강화학습 실시간 ━━  %s   (실측 치수 · 파워 100 · 조향만 학습)" % title, ""]
    if not its:
        out.append("  준비 중 (트랙 만들기·규칙 기준 평가) … 경과 %s" % fmt_min(int(now - born)))
    else:
        last = its[-1]
        k = last["it"]
        n = 30
        fill = int(n * k / total)
        el = now - born
        eta = el / k * (total - k) if k else 0
        out.append("  진행        %s%s  %d/%d회 (%d%%) · 경과 %s · %s" % (
            "█" * fill, "░" * (n - fill), k, total, 100 * k // total, fmt_min(int(el)),
            "끝" if done else "남은 시간 약 " + fmt_min(int(eta))))
        out.append("  학습 완주율  %.2f  %s" % (last["fin"], spark([x["fin"] for x in its])))
        out.append("  도달률      %.2f  %s   (트랙을 평균 몇 %% 갔는지)" % (last["reach"], spark([x["reach"] for x in its])))
        out.append("  탐색 폭      %.2f / %.2f   (줄수록 정책이 자기 행동을 확신)" % last["std"])
        out.append("  학습량      %.1fM 결정 · 1회 %.1f초" % (last["dec"], last["sec"]))
        if not done and EVAL_EVERY and k % EVAL_EVERY == 0 and (not evals or evals[-1][0] != k):
            out.append("  ⏳ %d회 평가 중 — 트랙 18개 × 월드 12개를 30초씩 달려 봄 (몇 분 걸림, %s째)" % (k, fmt_min(int(now - st.st_mtime))))
        elif now - st.st_mtime > 300 and not done:
            out.append("  ※ 로그가 %s째 안 바뀜 — 학습이 멈췄을 수 있음" % fmt_min(int(now - st.st_mtime)))
    if EVAL_EVERY or evals:
        out += ["", "  평가 (트랙마다 고정 월드 12개 · 출발점 고정)",
                "  " + "  ".join([pad("", 14), pad("사진 트랙", 10), pad("mrc", 9), pad("생성 16개", 12), "전체"])]
        if rule:
            out.append("  " + row("E2 (지금 규칙)", rule))
        for it, d in evals[-8:]:
            out.append("  " + row("정책 %d회" % it, d))
        if not evals:
            out.append("  (첫 평가는 %d회째)" % EVAL_EVERY)
    if cpu:
        rows = cpu_rows(cpu)
        out += ["", "  실기용 파일로 CPU 시뮬 평가 (CPU 코어 병렬 · 트랙마다 월드 12개 · 실측 치수)",
                "  " + "  ".join([pad("", 14), pad("사진 트랙", 16), pad("직각 트랙", 16)])]
        for r in rows[-9:]:
            lab = "E2 (지금 규칙)" if r["it"] == "E2" else "정책 %s회" % r["it"]
            out.append("  " + "  ".join([pad(lab, 14), pad(cell(r["res"].get("photo")), 16), pad(cell(r["res"].get("rightangle")), 16)]))
        if not rows:
            out.append("  (E2 기준 평가 중 …)")
    out += ["", "  %s · Ctrl-C로 닫기" % ("학습 끝" if done else "2초마다 새로 고침")]
    print("\n".join(out), flush=True)
    return done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--total", type=int, default=200)
    ap.add_argument("--title", default="")
    ap.add_argument("--eval-every", type=int, default=25)
    ap.add_argument("--cpu", default=None, help="cpu_eval_ckpt.py 결과 jsonl")
    args = ap.parse_args()
    global EVAL_EVERY
    EVAL_EVERY = args.eval_every
    title = args.title or os.path.basename(args.log)
    try:
        while True:
            if draw(args.log, args.total, title, args.cpu):
                break
            time.sleep(2)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
