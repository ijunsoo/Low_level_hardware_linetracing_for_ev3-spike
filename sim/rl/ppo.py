# =====================================================================
#  4단계: PPO 학습 (잔차 정책, 비대칭 액터-크리틱, GPU 병렬 환경)
#
#  정책(액터)은 실기에서 잴 수 있는 관측 22개만 보고 Δ조향·파워를 낸다.
#  크리틱은 + 특권 관측 12개(진짜 횡편차·헤딩오차·전방 곡률·월드 파라미터).
#  규칙기반(TorchRule = 실기 3센서 파일 로직)이 바탕 조향을 하고 정책은 그 위에 더한다.
#
#    ~/.venvs/linefollowing/bin/python -m rl.ppo --layout row3 --iters 150 --envs 16384
#  결과: sim/rl/runs/<이름>/ (policy.pt, log.json, eval.json)
#  평가: 홀드아웃 생성 트랙 + 사진 트랙 + 1.6배 트랙, 고정 월드, 출발점 고정 —
#        같은 조건의 규칙기반(mode=rule)과 비교
# =====================================================================
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn

from . import _SIM_DIR
import core
from .env import OBS_DIM
from .fastsim import TrackPool
from .parity_test import world
from .torchsim import TorchEnv, TorchPool, pick_device, REASONS
from . import tracks as gen
from . import sweep_gpu as SG

CRIT_DIM = 12


class RunningNorm(nn.Module):
    def __init__(self, dim, eps=1e-4):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.tensor(eps))

    @torch.no_grad()
    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        d = bm - self.mean
        tot = self.count + bc
        self.mean += d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot

    def forward(self, x):
        return ((x - self.mean) / (self.var + 1e-8).sqrt()).clamp(-8, 8)


def mlp(i, o, h=128, out_std=1.0, act="tanh"):
    l1, l2, l3 = nn.Linear(i, h), nn.Linear(h, h), nn.Linear(h, o)
    for l in (l1, l2):
        nn.init.orthogonal_(l.weight, 2 ** 0.5)
        nn.init.zeros_(l.bias)
    nn.init.orthogonal_(l3.weight, out_std)
    nn.init.zeros_(l3.bias)
    A = nn.ReLU if act == "relu" else nn.Tanh
    return nn.Sequential(l1, A(), l2, A(), l3)


class Agent(nn.Module):
    def __init__(self, hid=128, power_bias=0.6, actor_hid=None, act="tanh"):
        super().__init__()
        self.onorm, self.cnorm = RunningNorm(OBS_DIM), RunningNorm(CRIT_DIM)
        self.actor_hid, self.act = actor_hid or hid, act
        self.actor = mlp(OBS_DIM, 2, self.actor_hid, out_std=0.01, act=act)
        with torch.no_grad():
            self.actor[-1].bias[1] = power_bias            # 처음엔 파워 ≈ 84 (Δ조향 0)
        self.log_std = nn.Parameter(torch.full((2,), -0.7))
        self.critic = mlp(OBS_DIM + CRIT_DIM, 1, hid, out_std=1.0)

    def dist(self, on):
        return torch.distributions.Normal(self.actor(on), self.log_std.exp())

    def value(self, on, cn):
        return self.critic(torch.cat([on, cn], 1)).squeeze(1)


def layout_and_consts(layout, cfg_path=None):
    """레이아웃 센서 위치 + 규칙기반 상수(core 검증 최고값, 없으면 실기 파일 기본값)."""
    import sweep_lf3 as SL
    from .rule_lf3 import load_constants
    pos, fixed, _, _ = SG.LAYOUTS[layout]
    consts = load_constants()
    consts.update(fixed)
    vpath = os.path.join(_SIM_DIR, "sweep_lf3_validate.json")
    if os.path.exists(vpath):
        summ = SL.summarize(json.load(open(vpath)))
        sub = sorted([s for s in summ if s["cell"] == layout and s["cfg"]], key=lambda s: s["score"])
        if sub:
            consts.update({k: (float(v) if k in ("KP", "KD") else int(float(v))) for k, v in sub[0]["cfg"].items()})
    if cfg_path:                                          # 예: GPU 스윕 최고 설정
        cfg = json.load(open(cfg_path))
        consts.update({k: (float(v) if k in ("KP", "KD") else int(float(v))) for k, v in cfg.items()})
    return pos, consts


def train_pool():
    trs, names = [], []
    for i in range(48):                                   # 학습 생성 트랙 (홀드아웃 시드 아님)
        lv = ("medium", "hard")[i % 2]
        trs.append(gen.make_track(gen.generate(1000 + i, lv)))
        names.append("%s%d" % (lv, 1000 + i))
    for nm in ("photo", "mrc_real", "mrc"):
        trs.append(core.make_track(nm, 20.0))
        names.append(nm)
    return TrackPool(trs, names)


def eval_pool(n_hold=16):
    trs, names = [], []
    for i in range(n_hold):
        trs.append(gen.make_track(gen.generate(gen.HOLDOUT_SEED0 + i, "hard")))
        names.append("hold%d" % i)
    for nm in ("photo", "mrc"):
        trs.append(core.make_track(nm, 20.0))
        names.append(nm)
    return TrackPool(trs, names)


@torch.no_grad()
def evaluate(agent, pool, pos, consts, device, n_worlds=12, mode="residual2", max_s=30.0,
             lag=(10, 25), noise=(0.5, 2.0), profile="spike_3s"):
    """트랙마다 고정 월드 n_worlds개, 출발점 고정, 에피소드 1번씩 — 정책(평균 행동) vs 규칙기반.
    월드 = sweep_lf3.make_world와 같은 난수열 (지연·노이즈 범위만 학습 조건에 맞춤)."""
    from .sweep_gpu import world_r
    base = core.load_profile(profile)
    ws = [world_r(base, k, lag, noise) for k in range(n_worlds)]
    if profile == "spike_3s_geo":                      # 실측 모델: 지연·회전 권한·바퀴 흔들림도 월드마다 고정값
        for k, w in enumerate(ws):
            r = np.random.default_rng(95000 + k); wb = r.uniform(0.005, 0.035)
            w.update(cmd_lag_ms=float(np.random.default_rng(91000 + k).uniform(0, 32)),
                     imu_lag_ms=float(np.random.default_rng(93000 + k).uniform(0, 20)),
                     w_eff=float(np.random.default_rng(92000 + k).uniform(1.15, 1.5)),
                     wobble=wb * r.uniform(0.5, 1.5, 2), wobble_ph=r.uniform(0, 6.283, 2))
    n = pool.n * n_worlds
    env = TorchEnv(n, pool, device=device, profile=profile, mode=mode, rule_consts=consts, layout_pos=pos,
                   fixed_start=True, worlds=ws, p_torn=(2 / 3, 2 / 3), max_s=max_s, seed=11)
    o = env.reset_all()
    first = {}
    for _ in range(int((max_s + 1) / env.dec_dt) + 5):
        if agent is not None:
            a = agent.actor(agent.onorm(o))
        else:
            a = torch.zeros((n, 2), device=env.d)
        o, _, _, info = env.step(a)
        if info:
            for i, c, t, f in zip(info["idx"].tolist(), info["code"].tolist(),
                                  info["time_s"].tolist(), info["frac"].tolist()):
                if i not in first:
                    first[i] = (REASONS[c], t, f)
        if len(first) >= n:
            break
    out = {}
    for ti, nm in enumerate(pool.names):
        rs = [first[i] for i in range(ti * n_worlds, (ti + 1) * n_worlds) if i in first]
        fin = [r for r in rs if r[0] == "finish"]
        out[nm] = dict(fin=len(fin), n=len(rs), time=round(float(np.mean([r[1] for r in fin])), 2) if fin else None,
                       frac=round(float(np.mean([r[2] for r in rs])), 3) if rs else None)
    allr = list(first.values())
    hold = [first[i] for i in first if pool.names[i // n_worlds].startswith("hold")]
    hf = [r for r in hold if r[0] == "finish"]
    out["_holdout"] = dict(fin=len(hf), n=len(hold), time=round(float(np.mean([r[1] for r in hf])), 2) if hf else None,
                           frac=round(float(np.mean([r[2] for r in hold])), 3) if hold else None)
    out["_all_finish"] = round(sum(r[0] == "finish" for r in allr) / max(1, len(allr)), 3)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="row3")
    ap.add_argument("--envs", type=int, default=16384)
    ap.add_argument("--steps", type=int, default=32, help="env당 롤아웃 길이 (결정 수)")
    ap.add_argument("--iters", type=int, default=150)
    ap.add_argument("--mode", default="residual2", choices=["residual", "residual2", "steer_only"])
    ap.add_argument("--actor-hid", type=int, default=16, help="배포용 작은 액터 (실기 MicroPython)")
    ap.add_argument("--act", default="relu", choices=["relu", "tanh"])
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.995)
    ap.add_argument("--lam", type=float, default=0.95)
    ap.add_argument("--epochs", type=int, default=4)
    ap.add_argument("--minibatch", type=int, default=65536)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--eval-every", type=int, default=25)
    ap.add_argument("--name", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cfg", default=None, help="규칙기반 상수 JSON (GPU 스윕 최고값 등)")
    ap.add_argument("--eval-s", type=float, default=30.0)
    ap.add_argument("--known-weight", type=float, default=0.0, help="알려진 트랙(photo·mrc_real·mrc) 표본 비중 합")
    ap.add_argument("--r-lost", type=float, default=-50.0)
    ap.add_argument("--lag", default=None, help="학습·평가 센서 지연 범위 ms (기본: rl 프리셋 5~35, 평가 10~25)")
    ap.add_argument("--noise", default=None, help="센서 노이즈 σ 범위 (기본 0.5~2.0)")
    ap.add_argument("--profile", default="spike_3s", help="하드웨어 프로파일 (실측 치수: spike_3s_geo + --layout geo --dr geo)")
    ap.add_argument("--dr", default=None, help="도메인 랜덤화 프리셋 (rl, geo ...)")
    ap.add_argument("--init", default=None, help="이어 학습: 이 정책(policy_itN.pt 또는 policy.pt)에서 시작")
    ap.add_argument("--save-every", type=int, default=0, help="평가 없이 N회마다 내보내기용 체크포인트 저장")
    ap.add_argument("--skip-rule-eval", action="store_true", help="시작 때 규칙기반 기준 평가 생략 (몇 분 걸림)")
    args = ap.parse_args()
    dev = pick_device(args.device)
    torch.manual_seed(args.seed)
    name = args.name or "%s_%s_%s" % (args.layout, args.mode, time.strftime("%m%d_%H%M"))
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", name)
    os.makedirs(out_dir, exist_ok=True)

    pos, consts = layout_and_consts(args.layout, args.cfg)
    t0 = time.time()
    tpool = TorchPool(train_pool(), dev)
    epool = eval_pool()
    print("트랙 풀 준비 %.0fs · 장치 %s · 레이아웃 %s · 규칙상수 %s" % (time.time() - t0, dev, args.layout, consts), flush=True)
    tp = None
    if args.known_weight > 0:
        known = [i for i, nm in enumerate(tpool.np.names) if nm in ("photo", "mrc_real", "mrc")]
        tp = np.full(tpool.n, (1 - args.known_weight) / (tpool.n - len(known)))
        tp[known] = args.known_weight / len(known)
    dr = args.dr or "rl"
    ev_lag, ev_noise = ((3, 10), (0.3, 1.5)) if args.profile == "spike_3s_geo" else ((10, 25), (0.5, 2.0))
    if args.lag or args.noise:                       # 실측에 맞춘 범위로 rl 프리셋을 덮어씀
        from . import fastsim
        over = {}
        if args.lag:
            ev_lag = tuple(float(x) for x in args.lag.split(","))
            over["lag_ms"] = ev_lag
        if args.noise:
            ev_noise = tuple(float(x) for x in args.noise.split(","))
            over["noise"] = ev_noise
        fastsim.DR_PRESETS["custom"] = dict(fastsim.DR_PRESETS[dr], **over)
        dr = "custom"
    env = TorchEnv(args.envs, tpool, device=dev, profile=args.profile, mode=args.mode, rule_consts=consts, layout_pos=pos,
                   dr=dr, seed=args.seed, track_p=tp, r_lost=args.r_lost)
    agent = Agent(power_bias=(0.0 if args.mode == "residual2" else 0.6), actor_hid=args.actor_hid,
                  act=args.act).to(dev)
    if args.init:
        sd = torch.load(args.init, map_location=dev)
        agent.load_state_dict(sd["state"] if isinstance(sd, dict) and "state" in sd else sd)
        print("이어 학습:", args.init, flush=True)
    opt = torch.optim.Adam(agent.parameters(), lr=args.lr, eps=1e-5)

    rule_eval = {} if args.skip_rule_eval else evaluate(None, epool, pos, consts, dev, mode="rule", max_s=args.eval_s, lag=ev_lag, noise=ev_noise, profile=args.profile)
    print("규칙기반 기준 평가:", json.dumps(rule_eval, ensure_ascii=False), flush=True)
    log = dict(args=vars(args), consts=consts, rule_eval=rule_eval, iters=[])

    N, T = args.envs, args.steps
    obs_b = torch.zeros((T, N, OBS_DIM), device=dev)
    crit_b = torch.zeros((T, N, CRIT_DIM), device=dev)
    act_b = torch.zeros((T, N, 2), device=dev)
    logp_b = torch.zeros((T, N), device=dev)
    rew_b = torch.zeros((T, N), device=dev)
    done_b = torch.zeros((T, N), device=dev)
    val_b = torch.zeros((T, N), device=dev)
    ret_rms = RunningNorm(1).to(dev)
    disc_ret = torch.zeros(N, device=dev)
    o = env.reset_all()
    c = env.critic_obs()
    ep_stats = []
    best_score = -1e9
    for it in range(1, args.iters + 1):
        t_it = time.time()
        for t in range(T):
            with torch.no_grad():
                agent.onorm.update(o)
                agent.cnorm.update(c)
                on, cn = agent.onorm(o), agent.cnorm(c)
                dist = agent.dist(on)
                a = dist.sample()
                logp = dist.log_prob(a).sum(1)
                v = agent.value(on, cn)
            o2, r, d, info = env.step(a)
            disc_ret = disc_ret * args.gamma + r
            ret_rms.update(disc_ret[:, None])
            disc_ret = disc_ret * (~d).float()
            obs_b[t], crit_b[t], act_b[t], logp_b[t] = on, cn, a, logp
            rew_b[t] = r / (ret_rms.var.sqrt() + 1e-8)
            done_b[t], val_b[t] = d.float(), v
            if info:
                ep_stats += [(c_, f_, t_) for c_, f_, t_ in zip(info["code"].tolist(), info["frac"].tolist(),
                                                                info["time_s"].tolist())]
            o = o2
            c = env.critic_obs()
        with torch.no_grad():
            nv = agent.value(agent.onorm(o), agent.cnorm(c))
            adv = torch.zeros_like(rew_b)
            last = torch.zeros(N, device=dev)
            for t in reversed(range(T)):
                nxt = nv if t == T - 1 else val_b[t + 1]
                nonterm = 1.0 - done_b[t]
                delta = rew_b[t] + args.gamma * nxt * nonterm - val_b[t]
                last = delta + args.gamma * args.lam * nonterm * last
                adv[t] = last
            ret = adv + val_b
        B = T * N
        fo, fc, fa = obs_b.reshape(B, -1), crit_b.reshape(B, -1), act_b.reshape(B, 2)
        flp, fadv, fret = logp_b.reshape(B), adv.reshape(B), ret.reshape(B)
        fadv = (fadv - fadv.mean()) / (fadv.std() + 1e-8)
        for _ in range(args.epochs):
            perm = torch.randperm(B, device=dev)
            for s0 in range(0, B, args.minibatch):
                mb = perm[s0:s0 + args.minibatch]
                dist = agent.dist(fo[mb])
                lp = dist.log_prob(fa[mb]).sum(1)
                ratio = (lp - flp[mb]).exp()
                pg = -torch.min(ratio * fadv[mb], ratio.clamp(0.8, 1.2) * fadv[mb]).mean()
                vl = 0.5 * ((agent.value(fo[mb], fc[mb]) - fret[mb]) ** 2).mean()
                ent = dist.entropy().sum(1).mean()
                loss = pg + 0.5 * vl - 0.001 * ent
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), 0.5)
                opt.step()
        recent = ep_stats[-2000:]
        fin = sum(1 for e in recent if e[0] == 2) / max(1, len(recent))
        frac = float(np.mean([e[1] for e in recent])) if recent else 0.0
        row = dict(it=it, decisions=it * B, eps=len(ep_stats), finish=round(fin, 3), frac=round(frac, 3),
                   std=[round(float(x), 3) for x in agent.log_std.exp().tolist()], sec=round(time.time() - t_it, 2))
        log["iters"].append(row)
        print("it %3d | 결정 %.1fM | 에피소드 완주 %.2f 도달 %.2f | std %s | %.1fs" % (
            it, it * B / 1e6, fin, frac, row["std"], row["sec"]), flush=True)
        def ckpt():
            return dict(state=agent.state_dict(), actor_hid=agent.actor_hid, act=agent.act,
                        mode=args.mode, layout=args.layout, pos=pos, consts=consts, profile=args.profile,
                        dsteer_max=env.dsteer_max, dpower_max=env.dpower_max,
                        decision_ticks=env.K, v_ref=env.v_ref)
        if args.save_every and (it % args.save_every == 0 or it == args.iters):
            torch.save(ckpt(), os.path.join(out_dir, "policy_it%d.pt" % it))
            if it == args.iters:
                torch.save(ckpt(), os.path.join(out_dir, "policy.pt"))
        if args.eval_every and (it % args.eval_every == 0 or it == args.iters):
            ev = evaluate(agent, epool, pos, consts, dev, mode=args.mode, max_s=args.eval_s, lag=ev_lag, noise=ev_noise, profile=args.profile)
            row["eval"] = ev
            print("   평가:", json.dumps(ev, ensure_ascii=False), flush=True)
            score = ev["_all_finish"]
            torch.save(agent.state_dict(), os.path.join(out_dir, "policy_it%d.pt" % it))
            if score >= best_score:
                best_score = score
                torch.save(ckpt(), os.path.join(out_dir, "policy.pt"))
                json.dump(dict(it=it, eval=ev, rule_eval=rule_eval), open(os.path.join(out_dir, "eval.json"), "w"),
                          ensure_ascii=False, indent=1)
        json.dump(log, open(os.path.join(out_dir, "log.json"), "w"), ensure_ascii=False)
    print("완료:", out_dir, flush=True)


if __name__ == "__main__":
    main()
