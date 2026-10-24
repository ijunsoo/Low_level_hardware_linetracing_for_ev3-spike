# corner_eval.py --dump 결과를 같은 월드끼리 짝지어 비교: 둘 다 완주한 월드의 시간·최대 이탈 차이, 더 빠른/느린 월드 수
#   python3 paired.py dump_A.json:"A_3개(E8R8)" dump_D.json:"D_4개일렬" ...   (첫 번째가 기준)
import json
import sys


def load(spec):
    path, name = spec.split(":", 1)
    rows = [r for r in json.load(open(path)) if r[0] == name]
    return name, {(r[1], r[6]): r for r in rows}


def main():
    base_name, base = load(sys.argv[1])
    tracks = sorted({t for t, _ in base}, key=["rightangle", "main", "hard"].index if False else str)
    for spec in sys.argv[2:]:
        name, other = load(spec)
        print("== %s  vs  기준 %s" % (name, base_name))
        for t in sorted({t for t, _ in base}):
            keys = [k for k in base if k[0] == t and k in other]
            fa = sum(1 for k in keys if base[k][2] == "finish")
            fb = sum(1 for k in keys if other[k][2] == "finish")
            both = [k for k in keys if base[k][2] == "finish" and other[k][2] == "finish"]
            if both:
                dt = [other[k][3] - base[k][3] for k in both]
                dl = [other[k][5].get("latmax", 0) - base[k][5].get("latmax", 0) for k in both]
                faster = sum(1 for x in dt if x < -0.02)
                slower = sum(1 for x in dt if x > 0.02)
                print("  %-11s 완주 %2d→%2d/%d | 둘 다 완주 %2d월드: 시간 %+5.2fs (빨라짐 %2d·느려짐 %2d) | 최대 이탈 %+4.0fmm" % (
                    t, fa, fb, len(keys), len(both), sum(dt) / len(dt), faster, slower, sum(dl) / len(dl)))
            else:
                print("  %-11s 완주 %2d→%2d/%d" % (t, fa, fb, len(keys)))


if __name__ == "__main__":
    main()
