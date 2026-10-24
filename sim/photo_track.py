# =====================================================================
#  경기장 사진 → 시뮬 트랙 추출 파이프라인
#  1) 원근 보정된 워프에서 라인 마스크 추출
#  2) 라인 폭(=20mm)으로 mm 스케일 결정
#  3) 스켈레톤 → '직진 우선 걷기'로 교차를 통과하며 순서 있는 경로 추적
#  4) 마커(분리 성분)는 장식 검정점으로 보존
#  5) 원본 사진 방향(마커 직선이 위)으로 180° 회전 후 저장
# =====================================================================
import json
import math

import cv2
import numpy as np
from skimage.morphology import skeletonize

WARP = "/tmp/warp_1.8.jpg"
MAT_W_MM = 1950.0   # ★실측: 경기장 가로 길이 195cm (스케일 기준)
LINE_MM = 20.0      # 참고용(진단 출력) — 스케일 기준으로는 쓰지 않음

img = cv2.imread(WARP)
H, W = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
line = (gray < 100).astype(np.uint8)
line = cv2.morphologyEx(line, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
line = cv2.morphologyEx(line, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

# ---- 성분 분리: 최대 = 주행 라인, 나머지 = 마커 ----
n, lab, stats, _ = cv2.connectedComponentsWithStats(line)
areas = stats[1:, cv2.CC_STAT_AREA]
main_id = 1 + int(np.argmax(areas))
main = (lab == main_id).astype(np.uint8)
markers = ((lab > 0) & (lab != main_id)).astype(np.uint8)
print("성분:", n - 1, "개 / 주행라인 면적:", int(areas.max()),
      "/ 마커 성분:", sorted(areas)[:-1])

# ---- 스케일: 매트 실측 가로 195cm 기준 ----
dist = cv2.distanceTransform(main, cv2.DIST_L2, 5)
skel = skeletonize(main.astype(bool))
widths = dist[skel] * 2
mm_per_px = MAT_W_MM / W
print("스케일 %.3f mm/px (매트 %.0f x %.0f mm) / 라인폭 중앙값 %.1fpx = %.1fmm"
      % (mm_per_px, W * mm_per_px, H * mm_per_px,
         np.median(widths), np.median(widths) * mm_per_px))

# ---- 경로 추적: 스켈레톤 점집합 위 '직진 우선 걷기' ----
pts = np.column_stack(np.nonzero(skel))[:, ::-1].astype(float)  # (x, y)
from scipy.spatial import cKDTree
tree = cKDTree(pts)

# 시작점: 가장 넓게 뻗은 수평 밴드 = 마커 직선 (워프 기준 아래쪽)
best_band, best_span = None, 0
for y0 in range(0, H - 20, 10):
    band = pts[(pts[:, 1] >= y0) & (pts[:, 1] < y0 + 20)]
    if len(band) < 30:
        continue
    span = band[:, 0].max() - band[:, 0].min()
    # 마커 직선 = 넓게 뻗은 밴드 중 가장 아래(y 최대)
    if span > 800 and y0 > best_span:
        best_span, best_band = y0, (y0, band)
y0, band = best_band
sx = band[:, 0].min() + 350
sy = float(np.median(band[np.abs(band[:, 0] - sx) < 60][:, 1]))
print("마커 직선 밴드 y≈%d, 시작 (%.0f, %.0f)" % (y0, sx, sy))
_, i0 = tree.query([sx, sy])
cur = pts[i0].copy()
heading = 0.0     # 워프 기준 +x 방향 (마커 직선 방향)

visited = np.zeros(len(pts), bool)
STEP = 7.0        # px
path = [cur.copy()]
for _ in range(200000):
    idx = tree.query_ball_point(cur, STEP * 1.8)
    best = None
    best_da = 1e9
    for i in idx:
        if visited[i]:
            continue
        d = pts[i] - cur
        r = math.hypot(*d)
        if r < STEP * 0.5:
            continue
        da = abs((math.atan2(d[1], d[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
        if da < best_da:
            best_da = da
            best = pts[i]
    if best is None or best_da > math.radians(75):
        # ★예각 코너(지그재그 120°): 가까운 반경에서 큰 꺾임 허용 재탐색
        best = None; best_da = 1e9
        for i in tree.query_ball_point(cur, STEP * 1.1):
            if visited[i]:
                continue
            d = pts[i] - cur
            if math.hypot(*d) < STEP * 0.35:
                continue
            da = abs((math.atan2(d[1], d[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
            if da < best_da:
                best_da = da; best = pts[i]
        if best is None or best_da > math.radians(140):
            print("추적 중단: 막다른 곳")
            break
    heading = math.atan2(best[1] - cur[1], best[0] - cur[0])
    cur = best
    # 지나온 자리 방문 처리 (교차 반대 가닥은 남기기 위해 반경 작게)
    for i in tree.query_ball_point(cur, STEP * 0.6):
        visited[i] = True
    path.append(cur.copy())
    if len(path) > 50 and math.hypot(*(cur - path[0])) < STEP * 1.5:
        print("루프 폐합!")
        break
path = np.array(path)
length_px = np.sum(np.hypot(*np.diff(path, axis=0).T))
print("추적 점:", len(path), "/ 경로 길이: %.0f mm" % (length_px * mm_per_px))

# ---- mm 변환 + 180° 회전 (원본 사진 방향: 마커 직선이 위) ----
def to_mm(p):
    x = (W - p[..., 0]) * mm_per_px
    y = (H - (H - p[..., 1])) * mm_per_px * -1 + H * mm_per_px
    # 180° 회전: (x,y)px → ((W-x), (H-y))px → y축 뒤집기(mm, 위=+)
    return np.stack([(W - p[..., 0]) * mm_per_px,
                     (p[..., 1] - H) * -1 * mm_per_px * -1], -1)

# px → mm (180° 회전: 원본 사진 방향, 위=+y)
pmm = np.stack([(W - path[:, 0]) * mm_per_px,
                (path[:, 1] - H) * mm_per_px], -1)
mpx = np.column_stack(np.nonzero(cv2.resize(markers, (W // 3, H // 3)) > 0))[:, ::-1] * 3.0
mmm = np.stack([(W - mpx[:, 0]) * mm_per_px, (mpx[:, 1] - H) * mm_per_px], -1)

# 진행방향: 실기와 동일하게 시작 직후 -x(서쪽, 마커 방향) → 필요시 반전
if pmm[min(30, len(pmm) - 1), 0] > pmm[0, 0]:
    pmm = pmm[::-1]
    print("경로 방향 반전(서쪽 출발)")
origin = pmm[0].copy()
pmm -= origin
mmm -= origin

# ---- 저장 ----
seg = np.hypot(*np.diff(pmm, axis=0).T)
s = np.concatenate([[0], np.cumsum(seg)])
black = []
# 경로점 2mm 간격 재샘플
t = np.arange(0, s[-1], 2.0)
rx = np.interp(t, s, pmm[:, 0]); ry = np.interp(t, s, pmm[:, 1])
for x, y in zip(rx, ry):
    black += [round(float(x), 1), round(float(y), 1)]
for x, y in mmm:
    black += [round(float(x), 1), round(float(y), 1)]
# ★실선 마커: 주행선과 붙은 성분이라 추적에서 빠짐 → 경로에서 먼 스켈레톤 점 복원
smm = np.stack([(W - pts[:, 0]) * mm_per_px, (pts[:, 1] - H) * mm_per_px], -1) - origin
ptree = cKDTree(np.stack([rx, ry], -1))
d_to_path, _ = ptree.query(smm)
extra = smm[d_to_path > 15.0]
print("실선 마커 복원 점:", len(extra))
for x, y in extra:
    black += [round(float(x), 1), round(float(y), 1)]
coarse_i = np.arange(0, len(t), 5)
coarse = {"px": [round(float(rx[i]), 1) for i in coarse_i],
          "py": [round(float(ry[i]), 1) for i in coarse_i],
          "s":  [round(float(t[i]), 1) for i in coarse_i],
          "length": round(float(s[-1]), 1)}
json.dump({"black": black, "coarse": coarse},
          open("sim/viz/track_pts.json", "w"))
print("저장 완료 / 코스 길이 %.2f m" % (s[-1] / 1000))

# ---- 배경: 워프 사진(180° 회전) + mm 좌표 메타 ----
import base64, io
bg = cv2.rotate(img, cv2.ROTATE_180)
bg_small = cv2.resize(bg, (1200, int(1200 * H / W)))
ok, buf = cv2.imencode(".jpg", bg_small, [cv2.IMWRITE_JPEG_QUALITY, 70])
meta = {"png": base64.b64encode(buf).decode(), "jpg": True,
        "x0": float(-origin[0]), "y0": float(-H * mm_per_px - origin[1]),
        "w_mm": float(W * mm_per_px), "h_mm": float(H * mm_per_px)}
json.dump(meta, open("sim/viz/track_bg.json", "w"))
print("배경 저장")

# ---- 미리보기 ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(13, 7))
ax.plot(rx, ry, 'k-', lw=3)
ax.plot(mmm[:, 0], mmm[:, 1], 'k.', ms=2)
ax.plot(rx[0], ry[0], 'g^', ms=14)
ax.plot(rx[-1], ry[-1], 'rx', ms=14)
ax.set_aspect("equal"); ax.set_title("photo-traced / %.2fm" % (s[-1] / 1000))
fig.tight_layout(); fig.savefig("sim/track_preview.png", dpi=90)
EOF_MARKER = True
