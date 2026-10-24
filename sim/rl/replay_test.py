# 실기 파일(CPU)과 TorchRule에 같은 입력(센서 생값·시각·자이로)을 넣어 조향 출력이 같은지 확인
#   ~/.venvs/linefollowing/bin/python sim/rl/replay_test.py '{"ERR_MODE": 1, "KP2": 0.4}'
import sys, os, io, re, json, contextlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np, torch
import core, sweep_lf3
from rl.torchsim import TorchRule
from rl.rule_lf3 import load_constants
OVR = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
track = sys.argv[2] if len(sys.argv) > 2 else 'rightangle'
src = open(os.path.join(os.path.dirname(os.path.dirname(HERE)), 'spike/spike_lf3_rule.py'), encoding='utf-8').read()
src = src.replace('AUTO_STOP = True', 'AUTO_STOP = False').replace('AUTO_CAL = True', 'AUTO_CAL = False')
for k, v in OVR.items():
    src = re.sub(r'(?m)^(%s = )([^#\n]*?)(\s*#.*)?$' % k, lambda m: m.group(1) + repr(v) + (m.group(3) or ''), src, count=1)
a = '        steering_dc(steer, power)\n\n\n'
src = src.replace(a, '        REC.append((now, rL, rR, rF, steer, error, GY[0]))\n' + a)
b = '        if md:\n            a = rdL(3)\n            rL = a[0]'
assert src.count(b) == 1
src = src.replace(b, '        GY[0] = int(av(YAW_AXIS)) * YAW_SIGN\n' + b)
p, off = sweep_lf3.make_world(core.load_profile('spike_3s_geo'), 0, {"_ASYNC": True, "_NOISE": [0.3, 0.8], "_LAG_MS": [3, 8]})
sim = core.Sim(p, core.make_track(track, 19.0), start_lateral_mm=off, time_limit_s=4.0, seed=0); core.SIM = sim
g = {'__name__': '__main__', 'REC': [], 'GY': [0]}
with contextlib.redirect_stdout(io.StringIO()):
    try: exec(compile(src, 'r', 'exec'), g)
    except (core.SimDone, SystemExit): pass
REC = g['REC']
c = load_constants(); c.update(OVR)
tr = TorchRule(1, torch.device('cpu'), c)
idx = torch.arange(1)
tr.set_value(idx, torch.tensor([g['VALUE'][0::2]], dtype=torch.float32), torch.tensor([g['VALUE'][1::2]], dtype=torch.float32))
tr.reset(idx, torch.tensor([[REC[0][1], REC[0][2], REC[0][3]]], dtype=torch.int32))
bad = big = 0
for (now, rL, rR, rF, steer, error, gy) in REC:
    tr.step(torch.tensor([[rL, rR, rF]], dtype=torch.int32), torch.tensor([now]), torch.tensor([gy]), gate=torch.tensor([True]))
    d = abs(int(tr.steer_out[0]) - steer)
    bad += d > 0
    big += d > 3
print('CPU 결과 %s %.2fs · 스텝 %d · 조향 불일치 %d (3 넘게 차이 %d)' % (sim.result['reason'], sim.result['time_s'], len(REC), bad, big))
