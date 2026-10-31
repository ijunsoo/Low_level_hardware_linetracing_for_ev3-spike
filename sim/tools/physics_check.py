# 물리 엔진 검토: 인터넷에 공개된 SPIKE 라지 모터 값으로 직진·회전 반응을 계산해 시뮬 프로파일·실기와 비교 (docs/PHYSICS_ENGINE.md)
#   python3 physics_check.py            모터값 풀기(미디엄 모터로 검산) + 식으로 낸 시정수 vs 시뮬 프로파일
#   python3 physics_check.py --mujoco   MuJoCo 로봇으로 직진 출발·제자리 회전 vs 실기 블랙박스 (pip install mujoco)
#   python3 physics_check.py --final    SPIKE 완성판(웹 프리셋)을 지금 2배 모형 vs 물리식 2배 모형으로 주행
# 모터 = Pybricks 펌웨어 lib/pbio/src/motor/servo_settings.c 의 모델 정수 (observer.c PRESCALE, 단위 µN·m·mV·mdeg)
import argparse
import contextlib
import io
import itertools
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np      # noqa: E402
import core             # noqa: E402

PRESCALE = dict(voltage=178956, speed=858, acceleration=85)      # observer.c
PYBRICKS = {   # servo_settings.c: d_torque_d_voltage, d_torque_d_speed, d_torque_d_acceleration, torque_friction
    "technic_l_angular": (2872, 1919, 3997, 23239),      # SPIKE 라지 모터 (45602)
    "technic_m_angular": (8071, 5903, 16163, 21413),     # SPIKE 미디엄 모터 (45603) — 검산용
}
C_TH = 180000 / math.pi                                   # mdeg / rad
G1, G2 = 1.6667, 3.3333                                   # 기어 1배(20:12)·2배
# 우리 로봇 무게 어림 (kg): 허브 0.19(판매처 170~190g) · 라지 모터 2×0.071(Philo) · 센서 바 0.08 · 볼 캐스터 0.012 · 바퀴 2×0.025
#   + 프레임·축·기어·케이블 0.10~0.25 → 0.57~0.72kg, 바퀴축 중심 회전 관성 0.005~0.008 kg·m² (센서 바가 170mm 앞)
ROBOTS = ((0.57, 0.005), (0.72, 0.008))
MID = (0.65, 0.0065)


def decode(name):
    """펌웨어 정수 → SI: 전압당 토크 ktr(N·m/V), 역기전력 kkr(N·m·s/rad), 출력축 관성 J(kg·m²), 마찰 tf(N·m)."""
    dv, dw, da, tf = PYBRICKS[name]
    return dict(ktr=PRESCALE["voltage"] / dv * 1e-3, kkr=PRESCALE["speed"] / dw * C_TH * 1e-6,
                J=PRESCALE["acceleration"] / da * C_TH * 1e-6, tf=tf * 1e-6)


def from_experiment(V, tau_x, i_x, w_x, i_0, w_0, a):
    """Pybricks lib/pbio/doc/control/motor_model.py 의 make_model()과 같은 계산: 모터 곡선 실험값 → 같은 SI 값."""
    kt = tau_x / (i_x - i_0)
    tau_s = kt * i_0
    r_over_ke = kt * (w_x - w_0) / -tau_x
    ke = V / (w_0 + r_over_ke / kt * tau_s)
    R = r_over_ke * ke
    return dict(ktr=kt / R, kkr=kt * ke / R, J=(kt * V / R - tau_s) / a, tf=tau_s)


def motors():
    """모터 A = Pybricks 모델, B = LEGO 공식 사양서(7.2V: 무부하 175rpm, 멈춤 25 N·cm, 마찰은 A와 같게).
    배터리 전압은 시뮬 프로파일의 실측 dc100 모터 속도가 나오게 (Pybricks dc(duty) = 배터리 전압 × duty)."""
    A = decode("technic_l_angular")
    B = dict(A, ktr=(0.25 + A["tf"]) / 7.2, kkr=0.25 / (175 * 2 * math.pi / 60))
    w = math.radians(core.load_profile("spike_3s_geo2")["dc_table"][-1][1])
    for M in (A, B):
        M["V"] = (w * M["kkr"] + M["tf"]) / M["ktr"]
    return A, B


def taus(M, g, mass, iz, r, b):
    """바퀴가 미끄러지지 않을 때 시정수(s): 직진 = (J + M(rg)²/2)/kkr, 제자리 회전 = (J + 2·Iz(rg)²/b²)/kkr."""
    rg = r * g
    return (M["J"] + mass * rg * rg / 2) / M["kkr"], (M["J"] + 2 * iz * rg * rg / (b * b)) / M["kkr"]


def physics_profile():
    """1배 프로파일(spike_3s_geo2)의 시정수를 물리식 비율(2배 ÷ 1배, 중간 무게)로 늘린 2배 값."""
    A, _ = motors()
    p = core.load_profile("spike_3s_geo2")
    r, b = p["wheel_diameter_mm"] / 2000, p["axle_track_mm"] / 1000
    f1, t1 = taus(A, G1, *MID, r, b)
    f2, t2 = taus(A, G2, *MID, r, b)
    return dict(motor_tau_s=round(p["motor_tau_s"] * f2 / f1, 3), yaw_tau_s=round(p["yaw_tau_s"] * t2 / t1, 3))


def show_formula():
    m_exp = from_experiment(V=7.2, tau_x=1018.64 / 1000 * 9.81 / 100, i_x=0.51, w_x=4.6, i_0=0.09, w_0=16.6,
                            a=math.radians(920 / 0.035))
    m_fw = decode("technic_m_angular")
    print("검산 (미디엄 모터): 펌웨어 정수를 푼 값 vs motor_model.py 실험값으로 계산한 값")
    for k in ("ktr", "kkr", "J", "tf"):
        print("  %-3s %.6g vs %.6g" % (k, m_fw[k], m_exp[k]))
    A, B = motors()
    for name, M in (("A Pybricks 모델", A), ("B LEGO 사양서", B)):
        print("모터 %s: 멈춤 토크 %.2f N·m @ %.2fV · 무부하 %.0f°/s · 관성 %.5f kg·m² · 마찰 %.3f N·m · 모터 혼자 시정수 %.0fms" % (
            name, M["ktr"] * M["V"] - M["tf"], M["V"], math.degrees((M["ktr"] * M["V"] - M["tf"]) / M["kkr"]),
            M["J"], M["tf"], 1000 * M["J"] / M["kkr"]))
    p1, p2 = core.load_profile("spike_3s_geo2"), core.load_profile("spike_3s_geo2x")
    r, b = p1["wheel_diameter_mm"] / 2000, p1["axle_track_mm"] / 1000
    print("바퀴가 미끄러지지 않을 때 시정수 (바퀴 %.0fmm · 바퀴 사이 %.0fmm · 무게 %.2f~%.2fkg)" % (2000 * r, 1000 * b, ROBOTS[0][0], ROBOTS[1][0]))
    for name, M in (("A", A), ("B", B)):
        rows = [[taus(M, g, mass, iz, r, b) for mass, iz in ROBOTS] for g in (G1, G2)]
        print("  모터 %s | 1배 직진 %.2f~%.2fs 회전 %.2f~%.2fs | 2배 직진 %.2f~%.2fs 회전 %.2f~%.2fs" % (
            name, rows[0][0][0], rows[0][1][0], rows[0][0][1], rows[0][1][1], rows[1][0][0], rows[1][1][0], rows[1][0][1], rows[1][1][1]))
    print("  시뮬 프로파일 | 1배 spike_3s_geo2 직진 %.2fs 회전 %.2fs | 2배 spike_3s_geo2x 직진 %.2fs 회전 %.2fs (1배는 실기 기록, 2배 회전은 실기 기록 7.3초에 맞춘 값 · 직진은 못 잼)" % (
        p1["motor_tau_s"], p1["yaw_tau_s"], p2["motor_tau_s"], p2["yaw_tau_s"]))
    print("  → 실기 회전 %.2fs ≈ 모터 혼자 %.3fs: 회전할 때 바퀴가 많이 미끄러져 모터가 로봇 무게를 거의 못 느낌" % (
        p1["yaw_tau_s"], A["J"] / A["kkr"]))
    print("물리식 2배 (1배 값 × 물리식 비율): %s" % physics_profile())


def mujoco_robot(M, g, m_frame, hub_x, mu, r=0.04):
    k = M["ktr"] * M["V"] / g
    joint = 'type="hinge" axis="0 1 0" armature="%.7f" frictionloss="%.5f"' % (M["J"] / g / g, M["tf"] / g)
    wheel = '<geom type="cylinder" size="%g 0.008" euler="90 0 0" mass="0.025" condim="4" friction="%g 0.005 0.0001"/>' % (r, mu)
    nc = 'contype="0" conaffinity="0"'
    act = '<general joint="%s" ctrlrange="-1 1" gainprm="%g" biastype="affine" biasprm="0 0 %g"/>'
    return f"""<mujoco>
  <option timestep="0.0005" integrator="implicitfast"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" friction="0.001 0.001 0.0001"/>
    <body name="robot" pos="0 0 {r}">
      <freejoint/>
      <geom type="box" size="0.044 0.028 0.016" pos="{hub_x} 0 0.05" mass="0.19" {nc}/>
      <geom type="box" size="0.03 0.012 0.02" pos="0 0.06 0" mass="0.071" {nc}/>
      <geom type="box" size="0.03 0.012 0.02" pos="0 -0.06 0" mass="0.071" {nc}/>
      <geom type="box" size="0.095 0.08 0.006" pos="0.075 0 0.02" mass="{m_frame}" {nc}/>
      <geom type="box" size="0.012 0.05 0.012" pos="0.17 0 -0.012" mass="0.08" {nc}/>
      <geom type="sphere" size="0.008" pos="0.165 0 {-(r - 0.008)}" mass="0.012" friction="0.2 0.001 0.0001"/>
      <body pos="0 0.1025 0"><joint name="jl" {joint}/>{wheel}</body>
      <body pos="0 -0.1025 0"><joint name="jr" {joint}/>{wheel}</body>
    </body>
  </worldbody>
  <actuator>{act % ("jl", k, -M["kkr"] / g / g)}{act % ("jr", k, -M["kkr"] / g / g)}</actuator>
</mujoco>"""


def run_mujoco():
    try:
        import mujoco
    except ImportError:
        sys.exit("MuJoCo가 없습니다: pip install mujoco")
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import pivot_resp

    def make(*a):
        m = mujoco.MjModel.from_xml_string(mujoco_robot(*a))
        return m, mujoco.MjData(m)

    def straight(M, g, *a):
        """정지에서 dc(100): 모터축 정상 속도(°/s)와 63% 도달 시간(ms)."""
        m, d = make(M, g, *a)
        for _ in range(int(0.3 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        w = []
        for _ in range(int(1.2 / m.opt.timestep)):
            d.ctrl[:] = (1.0, 1.0)
            mujoco.mj_step(m, d)
            w.append(math.degrees(d.qvel[6]) / g)
        w = np.array(w)
        return w[-1], 1000 * m.opt.timestep * int(np.argmax(w >= 0.632 * w[-1]))

    def pivot(M, g, *a, lag=0.010):
        """곧게 dc(100)으로 달리다 조향 92(안쪽 −84%, 바깥 100%), 명령 지연 10ms → 20/40/60ms 회전 속도 증가(°/s)."""
        m, d = make(M, g, *a)
        dt = m.opt.timestep
        q = [(0.0, 0.0)] * int(round(lag / dt))

        def step(c):
            q.append(c)
            d.ctrl[:] = q.pop(0)
            mujoco.mj_step(m, d)

        for _ in range(int(0.8 / dt)):
            step((1.0, 1.0))
        y0, out, t = math.degrees(d.qvel[5]), [], 0.0
        for h in (20, 40, 60):
            while t < h - 1e-9:
                step((-0.84, 1.0))
                t += dt * 1000
            out.append(math.degrees(d.qvel[5]) - y0)
        return out

    for name, log in (("1배", "bb_2026-10-24_E7_130.txt"), ("2배", "bb_2026-10-24_E8R_2x_short.txt")):
        E = pivot_resp.real(os.path.join(ROOT, "diag_logs", log))
        print("실기 %s %s: 회전 %d번, 20/40/60ms %s°/s" % (name, log, len(E), "/".join("%.0f" % np.nanmean(E[:, i]) for i in range(3))))
    A, B = motors()
    for (mn, M), mf, hx, mu in itertools.product((("A", A), ("B", B)), (0.10, 0.25), (0.0, 0.08), (0.8, 1.2)):
        a = (mf, hx, mu)
        s1, t1 = straight(M, G1, *a)
        s2, t2 = straight(M, G2, *a)
        p1, p2 = pivot(M, G1, *a), pivot(M, G2, *a)
        print("모터%s 무게 %.2fkg 허브 %+3.0fmm 타이어 %.1f | 직진 1배 %4.0f°/s %3.0fms · 2배 %4.0f°/s %3.0fms | 회전 1배 %s · 2배 %s · 2배÷1배 %.2f" % (
            mn, 0.474 + mf, 1000 * hx, mu, s1, t1, s2, t2, "/".join("%.0f" % v for v in p1),
            "/".join("%.0f" % v for v in p2), p2[2] / max(p1[2], 1)))


def run_final():
    import eval_lf3 as EV
    sys.path.insert(0, os.path.join(ROOT, "web"))
    import build
    path, edits = [(p[1], p[6]) for p in build.PROGRAMS if p[0] == "spike_final"][0]
    src = open(os.path.join(ROOT, path), encoding="utf-8").read()
    for a, b in edits:
        src = src.replace(a, b)
    # 2배 모형에는 2배 상수로 (완성판 마지막 줄은 1.5배 기준 — 설명 주석의 GEAR=2 줄로 바꿈)
    two = re.search(r"#\s+2배:\s+(GEAR=2, .*?BRK_MS=\d+)", src).group(1)
    src, n = re.subn(r"GEAR=1\.5, .*?BRK_MS=\d+(?=, AUTO_STOP)", two, src)
    assert n == 1, "완성판 마지막 줄에서 1.5배 상수를 못 찾음"
    rand = dict(noise=[0.3, 2.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=[1.4, 1.6], wob=[0.005, 0.035])
    old = dict(cmd_lag_ms=32.0, yaw_tau_s=0.1, w_eff_scale=1.5)    # 10-31 이전 2배 모형 (물리식 줄도 이 미끄러짐 기준)

    def one(params, track, off=0.0, seed=0):
        sim = core.Sim(params, core.make_track(track, params["line_width_mm"]), start_lateral_mm=off, time_limit_s=40, seed=seed)
        sim.LOST_MM = 450
        core.SIM = sim
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                exec(compile(src, "spike_final", "exec"), {"__name__": "__main__"})
            except (core.SimDone, SystemExit):
                pass
        return sim.result

    phys = physics_profile()
    variants = (("지금 2배 (spike_3s_geo2x)", {}),
                ("예전 2배 0.2/0.1 · w_eff 1.5", old),
                ("물리식 2배 %.3f/%.3f" % (phys["motor_tau_s"], phys["yaw_tau_s"]),
                 dict(old, motor_tau_s=phys["motor_tau_s"], motor_tau_dn_s=phys["motor_tau_s"], yaw_tau_s=phys["yaw_tau_s"])),
                ("물리식 직진만 %.3f/0.1" % phys["motor_tau_s"],
                 dict(old, motor_tau_s=phys["motor_tau_s"], motor_tau_dn_s=phys["motor_tau_s"])))
    for name, over in variants:
        base = dict(core.load_profile("spike_3s_geo2x"), **over)
        w = base["w_eff_scale"]
        rand_w = dict(rand, weff=[w * 1.4 / 1.5, w * 1.6 / 1.5])     # 무작위 월드 미끄러짐은 모형 값 ±7% (웹과 같음)
        cells = []
        for track in ("main", "hard", "rightangle", "photo@1.3"):
            r = one(base, track)
            done = []
            for k in range(8):
                p, off = EV.world(base, k, rand_w)
                rr = one(p, track, off, k)
                if rr["reason"] == "finish":
                    done.append(rr["time_s"])
            cells.append("%s 기본 %s %.1fs · 무작위 %d/8%s" % (track, r["reason"], r["time_s"], len(done),
                                                        (" %.1fs" % np.mean(done)) if done else ""))
        print("%-26s %s" % (name, " | ".join(cells)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mujoco", action="store_true", help="MuJoCo 로봇으로 직진 출발·제자리 회전 (pip install mujoco)")
    ap.add_argument("--final", action="store_true", help="SPIKE 완성판을 지금 2배 모형 vs 물리식 2배 모형으로 주행")
    args = ap.parse_args()
    show_formula()
    if args.mujoco:
        print()
        run_mujoco()
    if args.final:
        print()
        run_final()


if __name__ == "__main__":
    import warnings
    warnings.filterwarnings("ignore")
    main()
