// =====================================================================
// MRC 라인트레이싱 JS 시뮬 엔진 — 파이썬 sim/core.py의 이식본
//  물리: 실측 모터곡선(dc100=1625.8°/s, tau 115ms) 1차지연 + 차동구동
//  센서: 라인 점군 거리판정 발자국(6.4mm) 평균 + 노이즈, 1457Hz 갱신
//  컨트롤러: 사용자의 실기 pid_control(v1) 그대로 이식
// =====================================================================
'use strict';

const CFG = {
  wheel: 56.0, axle: 72.0, fwd: 70.0, spacing: 25.0,
  fpRadius: 3.2, noise: 1.0, sensorHz: 1457, loopHz: 5302,
  tau: 0.115,        // 가속 시정수 (실측)
  sensorLag: 0.02,   // ★센서 지연(s) — EV3 UART 캐시 경유 지연 (실기 흔들림의 주범)
  tauDn: 0.035,      // ★감속 시정수 — 역기전력 제동은 가속보다 훨씬 빠름
  dcP: [0, 17.4, 100], dcV: [0, 0, 1625.8],
  white: [45, 42, 31, 25], black: [5, 5, 3, 2],
  lineHalf: 10.0, lostMM: 200, finishMM: 550, timeLimit: 90,
};

// ---- 시드 난수 (mulberry32 + Box-Muller) ----
function makeRng(seed) {
  let a = seed >>> 0;
  const u = () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return { gauss() {
    let u1 = u(); if (u1 < 1e-12) u1 = 1e-12;
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u());
  } };
}

// ---- 트랙: 검정 점군(2mm 간격) 공간 해시 ----
function makeTrack(blackPts, coarse) {
  const CELL = 40;
  const grid = new Map();
  const key = (cx, cy) => cx * 100000 + cy;
  for (let i = 0; i < blackPts.length; i += 2) {
    const k = key(Math.floor(blackPts[i] / CELL), Math.floor(blackPts[i + 1] / CELL));
    if (!grid.has(k)) grid.set(k, []);
    grid.get(k).push(blackPts[i], blackPts[i + 1]);
  }
  function isBlack(x, y) {
    const cx = Math.floor(x / CELL), cy = Math.floor(y / CELL);
    const lim = CFG.lineHalf * CFG.lineHalf;
    for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
      const cell = grid.get(key(cx + dx, cy + dy));
      if (!cell) continue;
      for (let i = 0; i < cell.length; i += 2) {
        const ddx = cell[i] - x, ddy = cell[i + 1] - y;
        if (ddx * ddx + ddy * ddy < lim) return 1;
      }
    }
    return 0;
  }
  return { isBlack, px: coarse.px, py: coarse.py, s: coarse.s, length: coarse.length };
}

// ---- 발자국 샘플 오프셋 (파이썬과 동일: 중심1 + 내륜5 + 외륜10) ----
const FP = (() => {
  const pts = [[0, 0]];
  for (let i = 0; i < 5; i++) {
    const a = 2 * Math.PI * i / 5;
    pts.push([0.5 * CFG.fpRadius * Math.cos(a), 0.5 * CFG.fpRadius * Math.sin(a)]);
  }
  for (let i = 0; i < 10; i++) {
    const a = 2 * Math.PI * i / 10;
    pts.push([0.95 * CFG.fpRadius * Math.cos(a), 0.95 * CFG.fpRadius * Math.sin(a)]);
  }
  return pts;
})();

// ---- 시뮬 본체 ----
function makeSim(track, gear, seed, white, black) {
  white = white || CFG.white;
  black = black || CFG.black;
  const rng = makeRng(seed);
  const readDt = 1 / (CFG.loopHz * 4);
  const sensorDt = 1 / CFG.sensorHz;
  const alpha = Math.exp(-readDt / CFG.tau);
  const k = Math.PI * CFG.wheel / 360;          // deg/s → mm/s
  const lat = [1.5, 0.5, -0.5, -1.5].map(v => v * CFG.spacing);

  const sim = {
    t: 0, x: track.px[0], y: track.py[0],
    h: Math.atan2(track.py[1] - track.py[0], track.px[1] - track.px[0]),
    wl: 0, wr: 0, dl: 0, dr: 0, angL: 0, angR: 0,
    raw: [50, 50, 50, 50], nextSensor: 0, pi: 0, maxS: 0, latSum: 0, latSq: 0, latN: 0,
    lagBuf: [],
    trace: [], nextTrace: 0, result: null,
  };

  function target(duty) {
    const a = Math.abs(duty);
    let v;
    if (a <= CFG.dcP[1]) v = 0;
    else if (a >= 100) v = CFG.dcV[2];
    else v = (a - CFG.dcP[1]) / (100 - CFG.dcP[1]) * CFG.dcV[2];
    return (duty < 0 ? -v : v) * gear;
  }

  function refreshSensors() {
    const ch = Math.cos(sim.h), sh = Math.sin(sim.h);
    const fresh = [0, 0, 0, 0];
    for (let i = 0; i < 4; i++) {
      const sx = sim.x + ch * CFG.fwd - sh * lat[i];
      const sy = sim.y + sh * CFG.fwd + ch * lat[i];
      let b = 0;
      for (const [ox, oy] of FP) b += track.isBlack(sx + ox, sy + oy);
      b /= FP.length;
      let r = white[i] - (white[i] - black[i]) * b + rng.gauss() * CFG.noise;
      fresh[i] = Math.max(0, Math.min(100, Math.round(r)));
    }
    // ★센서 지연: t-sensorLag 시점의 값을 컨트롤러에 전달
    sim.lagBuf.push([sim.t, fresh]);
    while (sim.lagBuf.length > 1 && sim.lagBuf[1][0] <= sim.t - CFG.sensorLag)
      sim.lagBuf.shift();
    sim.raw = sim.lagBuf[0][1];
    // 진행도 / 이탈 판정 (국소 최근접)
    const lo = Math.max(0, sim.pi - 6);
    const hi = Math.min(track.px.length - 1, sim.pi + 40);
    let best = 1e18, bi = sim.pi;
    for (let i = lo; i <= hi; i++) {
      const dx = track.px[i] - sim.x, dy = track.py[i] - sim.y;
      const d = dx * dx + dy * dy;
      if (d < best) { best = d; bi = i; }
    }
    sim.pi = bi;
    if (track.s[bi] > sim.maxS) sim.maxS = track.s[bi];
    const latNow = Math.sqrt(best);
    sim.latSum += latNow; sim.latSq += latNow * latNow; sim.latN++;
    if (latNow > CFG.lostMM) finish('lost');
    if (track.s[bi] > track.length - CFG.finishMM) finish('finish');
  }

  function finish(reason) {
    sim.result = {
      reason, time_s: +sim.t.toFixed(2),
      progress_mm: Math.round(sim.maxS),
      track_mm: Math.round(track.length),
      mean_lat: +(sim.latSum / Math.max(1, sim.latN)).toFixed(1),
      // 흔들림 = 편차의 표준편차 (일정한 엣지 오프셋은 제외)
      sway: +Math.sqrt(Math.max(0, sim.latSq / Math.max(1, sim.latN)
            - Math.pow(sim.latSum / Math.max(1, sim.latN), 2))).toFixed(1),
    };
    throw { simDone: true };
  }

  const alphaDn = Math.exp(-1 / (CFG.loopHz * 4) / CFG.tauDn);
  function decel(w, t) {   // 목표가 현재보다 '느려지는' 방향인가
    return (w > 0 && t < w) || (w < 0 && t > w);
  }
  function advance(dt) {
    const tl = target(sim.dl), tr2 = target(sim.dr);
    const std = dt === readDt;
    const aUp = std ? alpha : Math.exp(-dt / CFG.tau);
    const aDn = std ? alphaDn : Math.exp(-dt / CFG.tauDn);
    sim.wl = tl + (sim.wl - tl) * (decel(sim.wl, tl) ? aDn : aUp);
    sim.wr = tr2 + (sim.wr - tr2) * (decel(sim.wr, tr2) ? aDn : aUp);
    const vl = sim.wl * k, vr = sim.wr * k;
    const v = (vl + vr) / 2, w = (vr - vl) / CFG.axle;
    sim.x += v * Math.cos(sim.h + w * dt / 2) * dt;
    sim.y += v * Math.sin(sim.h + w * dt / 2) * dt;
    sim.h += w * dt;
    sim.angL += sim.wl * dt; sim.angR += sim.wr * dt;
    sim.t += dt;
    if (sim.t >= sim.nextSensor) { sim.nextSensor += sensorDt; refreshSensors(); }
    if (sim.t >= sim.nextTrace) {
      sim.nextTrace += 0.01;
      sim.trace.push([+sim.t.toFixed(2), +sim.x.toFixed(1), +sim.y.toFixed(1),
                      +sim.h.toFixed(4), Math.round(sim.dl), Math.round(sim.dr),
                      sim.raw[0], sim.raw[1], sim.raw[2], sim.raw[3]]);
    }
    if (sim.t > CFG.timeLimit) finish('timeout');
  }

  sim.read = i => { advance(readDt); return sim.raw[i]; };
  sim.gyro = () => {   // deg/s (IMU — 지연 ≈0)
    const vl = sim.wl * k, vr = sim.wr * k;
    return (vr - vl) / CFG.axle * 57.2958 + rng.gauss() * 1.5;
  };
  sim.waitMs = ms => {
    const end = sim.t + ms / 1000;
    while (sim.t < end) advance(Math.min(readDt, end - sim.t));
  };
  sim.setDuty = (l, r) => { sim.dl = Math.max(-100, Math.min(100, l));
                            sim.dr = Math.max(-100, Math.min(100, r)); };
  refreshSensors();
  return sim;
}

// ---- 사용자 실기 코드(pid_control) 이식 — 파이썬과 1:1 대응 ----
function runUserCode(sim, P) {
  // P: {power, dec_v, st, kp, ki, kd, value(8), w_offset}
  const fl = Math.floor;
  function steeringDc(steering, power) {
    steering = Math.max(-100, Math.min(100, steering));
    const slope = power * 2 / 100;
    let stB, stC;
    if (steering < 0) { stB = slope * steering + power; stC = power; }
    else { stB = power; stC = -slope * steering + power; }
    sim.setDuty(stB, stC);
  }
  try {
    // calibrate_error_offset(20)
    let total = 0;
    for (let i = 0; i < 20; i++) {
      const r1 = sim.read(0), r2 = sim.read(1), r3 = sim.read(2), r4 = sim.read(3);
      total += (r1 + r2) - (r3 + r4);
      sim.waitMs(50);
    }
    const error_offset = Math.round(total / 20);

    const S = 100000;
    const KP = Math.round(P.kp * S), KI = Math.round(P.ki * S), KD = Math.round(P.kd * S);
    const ALPHA = 700, A_COMP = 300, E_S = 1000;
    const [wh1, b1, wh2, b2, wh3, b3, wh4, b4] =
      [P.value[0], P.value[1], P.value[2], P.value[3],
       P.value[4], P.value[5], P.value[6], P.value[7]];
    let integral = 0;
    const max_error = fl(((wh1 - b1) + (wh2 - b2) + (wh3 - b3) + (wh4 - b4)) / 8);
    let last_error = 0, repower = P.power, direction = 0, on_count = 0;
    const ON_CONFIRM = 2;
    let line_off = false, prev_steering = 0;
    const MAX_STEER_DELTA = 15;
    const l_black = fl((wh1 + b1) / 2), r_black = fl((wh4 + b4) / 2);
    const cross = fl((b1 + b2 + b3 + b4) / 4) + 20;
    const w1 = wh1 - P.w_offset, w2 = wh2 - P.w_offset,
          w3 = wh3 - P.w_offset, w4 = wh4 - P.w_offset;
    let ps1 = sim.read(0), ps2 = sim.read(1), ps3 = sim.read(2), ps4 = sim.read(3);
    const clamp = (r, b, w) => Math.min(Math.max(r, b), w);

    let fsPrev = null;
    // ★코너 사전 피벗 상태 (P.corner=true일 때, v6 검증 로직)
    let turning = false, turnDir = 0, memDir = -1, memAge = 0,
        whiteCnt = 0, turnFrames = 0;
    const CB1 = b1 + (wh1 - b1) * 0.3, CB4 = b4 + (wh4 - b4) * 0.3;
    const WHITE_CONF = P.c_wc || 120, MEM_FR = P.c_mem || 500,
          TURN_MIN = P.c_tmin || 150, TURN_MAX = 8000, TURN_ST = P.c_st || 95;
    for (;;) {
      const r1 = sim.read(0), r2 = sim.read(1), r3 = sim.read(2), r4 = sim.read(3);
      if (P.corner) {
        if (turning) {
          turnFrames++;
          const reacq = (r2 < (b2 + wh2) / 2) || (r3 < (b3 + wh3) / 2);
          if (turnFrames >= TURN_MIN && reacq) {
            turning = false; integral = 0; last_error = 0;
            prev_steering = 0; line_off = false; on_count = 0; memDir = -1;
            steeringDc(0, P.power);
            continue;
          }
          if (turnFrames > TURN_MAX) { turning = false; memDir = -1; }
          else { steeringDc(turnDir === 0 ? -TURN_ST : TURN_ST, P.power); continue; }
        }
        if (r1 < CB1 && r4 > w4) { memDir = 0; memAge = 0; }
        else if (r4 < CB4 && r1 > w1) { memDir = 1; memAge = 0; }
        else if (memDir >= 0 && ++memAge > MEM_FR) memDir = -1;
        const allWhite = (r1 > w1 && r2 > w2 && r3 > w3 && r4 > w4);
        if (allWhite && memDir >= 0) whiteCnt++; else whiteCnt = 0;
        if (whiteCnt >= WHITE_CONF) {
          whiteCnt = 0; turning = true; turnDir = memDir; memDir = -1; turnFrames = 0;
          steeringDc(turnDir === 0 ? -TURN_ST : TURN_ST, P.power);
          continue;
        }
      }
      if (P.framesync) {           // ★v4식 프레임 동기: 새 센서 프레임에서만 1스텝
        const key = r1*1e6 + r2*1e4 + r3*100 + r4;
        if (key === fsPrev) continue;
        fsPrev = key;
      }
      const s1 = fl((ALPHA * clamp(r1, b1, wh1) + A_COMP * ps1) / E_S);
      const s2 = fl((ALPHA * clamp(r2, b2, wh2) + A_COMP * ps2) / E_S);
      const s3 = fl((ALPHA * clamp(r3, b3, wh3) + A_COMP * ps3) / E_S);
      const s4 = fl((ALPHA * clamp(r4, b4, wh4) + A_COMP * ps4) / E_S);
      ps1 = s1; ps2 = s2; ps3 = s3; ps4 = s4;

      const raw_off = (r1 > w1 && r2 > w2 && r3 > w3 && r4 > w4);   // 생값 판정(v4)
      if (raw_off) { on_count = 0; line_off = true; }
      else if (line_off) {
        on_count += 1;
        if (on_count >= ON_CONFIRM) {
          line_off = false; on_count = 0; integral = 0;
          last_error = (s1 + s2) - (s3 + s4) - error_offset;
        }
      }
      const error = (s1 + s2) - (s3 + s4) - error_offset;
      if (!line_off) {
        integral += error;
        if (KI !== 0) {
          const mi = fl((S * 100 - Math.abs(max_error * KP)) / KI);
          integral = Math.max(-mi, Math.min(mi, integral));
        }
      }
      const diff = error - last_error;
      let steering = fl((KP * error + KI * integral + KD * diff) / S);
      last_error = error;

      if (s1 < l_black) direction = 0;
      else if (s4 < r_black) direction = 1;

      if (line_off) {
        repower = Math.max(50, repower - P.dec_v);
        if (on_count === 0) steering = direction === 0 ? -P.st : P.st;
      } else {
        repower = P.power;
        if (s1 + s2 + s3 + s4 < cross) steering = 0;
      }
      // ★자이로 댐핑 — 라인이 보일 때만 (이탈 뱅킹 회전을 방해하지 않도록)
      if (P.kg && !line_off) steering += Math.round(P.kg * sim.gyro());
      const delta = steering - prev_steering;
      if (delta > MAX_STEER_DELTA) steering = prev_steering + MAX_STEER_DELTA;
      else if (delta < -MAX_STEER_DELTA) steering = prev_steering - MAX_STEER_DELTA;
      prev_steering = steering;

      steeringDc(steering, repower);
    }
  } catch (e) {
    if (!e || !e.simDone) throw e;
  }
  return { result: sim.result, trace: sim.trace };
}

if (typeof module !== 'undefined') {
  module.exports = { CFG, makeTrack, makeSim, runUserCode };
}
