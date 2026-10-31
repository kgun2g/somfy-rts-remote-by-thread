# -*- coding: utf-8 -*-
"""★2026-10-29 배터리 % 방전 하한이 회복 **도중에 다시 잠기는** 문제 — 수정 전 가상검증 (사용자 규칙 "빌드 전 가상검증").

증상(sim/tools/bat_adc_period_sim.py C 에서 부수 발견, 옛 펌웨어부터 있던 거동):
  측정 오류로 전압이 크게 낮게 읽힌 뒤(8/20형) 회복하면, 하한 해제 조건(+100mV × 5표본 연속)이 **EMA 가 오르는 도중**에 차서
  하한이 풀리고 — 바로 다음 표본에서 그 순간의(아직 덜 오른) 값으로 **다시 잠긴다**. 그 뒤로는 +100mV 가 다시 안 차서
  참값 80% 대신 69~73% 에 굳는다(다음 USB 연결까지).

수정(somfy_app.c 배터리 블록):
  하한을 푼 뒤에는 **오름이 멈출 때까지** 다시 잠그지 않는다. 푼 순간부터 평활 전압의 최고값을 따라가다가
  BAT_FLOOR_SETTLE_MV(3mV) 넘게 새로 오르지 않는 표본이 BAT_FLOOR_RELEASE_N(5) 개 이어지면 그때 그 값으로 잠근다.
  해제가 없는 평소 방전에서는 아무것도 바뀌지 않는다(정착 상태에 들어가지 않는다).

검증
  C  오류 2시간 뒤 회복 → k초 뒤 화면 ON, 200초 뒤 표시 — 시드 40개: 새 방식은 참값 ±2%p, 옛 방식보다 낮지 않다
  R  실측 8/20 세션(logs/nvs_c6_bat0.bin, 120초 기록) — 최종 표시가 참값에 더 가깝다(없으면 건너뜀)
  A  평소 대기 24시간(오류 없음) — 옛 방식과 표시가 **완전히 같다**(정착 상태 0회)
  N  풀린 뒤 평탄부 잡음(31mV 두 무리)에서도 정착이 끝난다 — 끝날 때까지 걸린 표본 수, 잠긴 값과 참값 차
  D  정착 중 실제 방전이 이어져도(풀린 직후 수명 끝 급강하) 잠금이 늦게 걸려 표시가 오르내리지 않는다
"""
import os
import random
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bat_adc_period_sim import (Pipe, mv_to_pct, period_s, FLOOR_MIN_N, REL_MV, REL_N,   # noqa: E402
                                SPIKE_MV, SPIKE_N, DROP, IDLE_OFF_S, wakes_per_day)

SETTLE_MV = 3          # 펌웨어 BAT_FLOOR_SETTLE_MV
SETTLE_N = REL_N       # 펌웨어 BAT_FLOOR_RELEASE_N 재사용


class PipeSettle(Pipe):
    """Pipe 와 같고, 하한 해제 뒤 '오름이 멈출 때까지' 다시 잠그지 않는다(펌웨어 새 방식)."""

    def __init__(self):
        Pipe.__init__(self)
        self.settling = False
        self.settle_mv = 0
        self.settle_n = 0
        self.settle_runs = 0     # 정착 상태에 들어간 횟수(계측)
        self.settle_len = []     # 정착이 끝날 때까지 표본 수

    def feed(self, mv):
        self.hist.append(mv)
        if len(self.hist) > 9:
            self.hist.pop(0)
        med = sorted(self.hist)[len(self.hist) // 2]
        self.ema = med * 16 if self.ema == 0 else self.ema + int((med * 16 - self.ema) / 4)
        sm = self.ema // 16
        if self.ok_mv > 0 and sm < self.ok_mv - SPIKE_MV and self.rej < SPIKE_N:
            self.rej += 1
            sm = self.ok_mv
        else:
            self.rej = 0
            self.ok_mv = sm
        pct = mv_to_pct(sm)
        if len(self.hist) >= FLOOR_MIN_N:
            if self.floor is not None and sm >= self.floor_mv + REL_MV:
                self.up_run += 1
                if self.up_run >= REL_N:
                    self.floor = None
                    self.up_run = 0
                    self.settling = True               # ★새 방식: 풀린 순간부터 정착 대기
                    self.settle_mv = sm
                    self.settle_n = 0
                    self.settle_runs += 1
                    self._settle_cnt = 0
            else:
                self.up_run = 0
            if self.settling:
                self._settle_cnt += 1
                if sm > self.settle_mv + SETTLE_MV:
                    self.settle_mv = sm
                    self.settle_n = 0
                else:
                    self.settle_n += 1
                    if self.settle_n >= SETTLE_N:
                        self.settling = False
                        self.settle_len.append(self._settle_cnt)
            if self.floor is not None and pct > self.floor:
                pct = self.floor
            elif not self.settling:
                self.floor = pct
                self.floor_mv = sm
        if self.reseed:
            self.reseed = False
        elif self.disp is not None and pct < self.disp - DROP:
            pct = self.disp - DROP
        self.disp = pct
        return sm


def mk(design):
    return PipeSettle() if design == 'new' else Pipe()


def c_final(design, seed, k, err_mv=-1400, err_len=7200, base=4000):
    """bat_adc_period_sim C 와 같은 조건 — 오류 err_len 초 뒤 회복, k초 뒤 화면 ON, 200초 뒤 표시."""
    err = lambda t: err_mv if 1000 <= t < 1000 + err_len else 0
    on = 1000 + err_len + k
    pp = mk(design)
    rng = random.Random(seed)
    last = -10 ** 9
    for t in range(on + 201):
        if t - last >= period_s('new', on <= t, len(pp.hist), t):
            last = t
            clus = -31 if rng.random() < 0.45 else 0
            pp.feed(int(base + clus + rng.gauss(0, 3) + err(t)))
    return pp.disp, pp


def load_measured(path):
    env = dict(os.environ, PYTHONIOENCODING='utf-8')      # ★파이프로 받으면 자식이 CP949 로 찍다가 '—' 에서 죽는다
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'batlog_decode.py'), path],
                         capture_output=True, text=True, encoding='utf-8', errors='replace', env=env).stdout
    rows = []
    for l in out.splitlines():
        m = re.match(r'\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s', l)
        if m:
            rows.append((int(m.group(2)), int(m.group(3))))   # (t_s, mv)
    return rows


def floor_only(rows, settle):
    """기록(이미 평활된 120초 표본)을 하한 단계에만 통과 — bat_floor_release_sim.run 과 같은 근사."""
    floor = floor_mv = None
    up = 0
    st, st_mv, st_n = False, 0, 0
    disp = []
    for _, mv in rows:
        pct = mv_to_pct(mv)
        if floor is not None and mv >= floor_mv + REL_MV:
            up += 1
            if up >= REL_N:
                floor = floor_mv = None
                up = 0
                if settle:
                    st, st_mv, st_n = True, mv, 0
        else:
            up = 0
        if st:
            if mv > st_mv + SETTLE_MV:
                st_mv, st_n = mv, 0
            else:
                st_n += 1
                if st_n >= SETTLE_N:
                    st = False
        if floor is not None and pct > floor:
            pct = floor
        elif not st:
            floor, floor_mv = pct, mv
        disp.append(pct)
    return disp


def main():
    ok = True
    truth = mv_to_pct(4000 - 15)
    # ── C 오류 2시간 뒤 회복 ─────────────────────────────────────────
    print('[C] 오류(-1400mV) 2시간 뒤 회복 → k초 뒤 화면 ON, 200초 뒤 표시 (참값 %d%%, 시드 40개)' % truth)
    for k in (10, 300, 900, 3600):
        fo = sorted(c_final('old', s, k)[0] for s in range(40))
        fn = sorted(c_final('new', s, k)[0] for s in range(40))
        mo, mn = sum(fo) / 40.0, sum(fn) / 40.0
        print('   회복 %4d초 뒤 켬   옛 평균 %.1f%% [%d~%d]   새 평균 %.1f%% [%d~%d]' % (k, mo, fo[0], fo[-1], mn, fn[0], fn[-1]))
        if abs(mn - truth) > 2 or fn[0] < truth - 3:
            print('   ✗ 새 방식이 참값 ±2%%p(최저 -3%%p) 안에 못 든다'); ok = False
        if mn < mo:
            print('   ✗ 새 방식이 옛 방식보다 낮다'); ok = False
    # 오류 크기·길이를 바꿔도(작은 오류 -300·-600, 짧은 10분) 같은가
    print('[C2] 오류 크기·길이 바꿈 (회복 900초 뒤 켬, 시드 40개)')
    for em, el in ((-300, 7200), (-600, 7200), (-1400, 600), (-2000, 20000)):
        fo = [c_final('old', s, 900, em, el)[0] for s in range(40)]
        fn = [c_final('new', s, 900, em, el)[0] for s in range(40)]
        mo, mn = sum(fo) / 40.0, sum(fn) / 40.0
        print('   오류 %5dmV × %5d초   옛 %.1f%% [%d~%d]   새 %.1f%% [%d~%d]' % (em, el, mo, min(fo), max(fo), mn, min(fn), max(fn)))
        if abs(mn - truth) > 2:
            print('   ✗ 새 방식이 참값에서 2%%p 넘게 벗어난다'); ok = False
    # 정착 길이(표본 수)
    lens = []
    for s in range(40):
        lens += c_final('new', s, 900)[1].settle_len
    lens.sort()
    print('   정착이 끝날 때까지 표본 수(풀린 순간부터): 중앙 %d, 최대 %d  — 화면 OFF 30초 주기면 최대 %.1f분'
          % (lens[len(lens) // 2], lens[-1], lens[-1] * 30 / 60.0))
    if lens[-1] > 40:
        print('   ✗ 정착이 너무 오래 걸린다(40표본 초과)'); ok = False

    # ── R 실측 8/20 세션 ──────────────────────────────────────────────
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    p = os.path.join(root, 'logs', 'nvs_c6_bat0.bin')
    rows = load_measured(p) if os.path.exists(p) else []
    if len(rows) > 50:
        do, dn = floor_only(rows, False), floor_only(rows, True)
        tr = [mv_to_pct(mv) for _, mv in rows]
        eo = sum(abs(a - b) for a, b in zip(do[150:], tr[150:])) / len(tr[150:])
        en = sum(abs(a - b) for a, b in zip(dn[150:], tr[150:])) / len(tr[150:])
        print('[R] 실측 8/20 세션 %d건 — 마지막 표시 옛 %d%% / 새 %d%% / 전압환산 %d%%   회복 구간(#150~) 평균 오차 옛 %.1f / 새 %.1f %%p'
              % (len(rows), do[-1], dn[-1], tr[-1], eo, en))
        if en > eo + 0.5:
            print('   ✗ 실측 회복 구간에서 새 방식이 더 틀린다'); ok = False
        rises = sum(1 for i in range(1, len(dn)) if dn[i] > dn[i - 1])
        print('   새 방식 표시 오름 횟수 %d (회복이니 오르는 게 맞다 — 옛 %d)' % (rises, sum(1 for i in range(1, len(do)) if do[i] > do[i - 1])))
    else:
        print('[R] 실측 파일(logs/nvs_c6_bat0.bin) 없음 — 건너뜀')

    # ── A 평소 대기 24시간 — 옛 방식과 완전히 같아야 ───────────────────
    print('[A] 평소 대기 24시간(오류 없음, 화면 하루 40회) — 옛/새 표시 비교')
    for seed in range(10):
        rng = random.Random(100 + seed)
        dur = 86400
        wk = set(wakes_per_day(rng, dur, 40))
        ocv = lambda t: 4060 - 70.0 * t / dur
        outs = {}
        for d in ('old', 'new'):
            pp = mk(d)
            r2 = random.Random(seed)
            last = -10 ** 9
            on_until = -1
            trace = []
            for t in range(dur):
                if t in wk:
                    on_until = t + 10 + IDLE_OFF_S
                scr = t < on_until
                if t - last >= period_s('new', scr, len(pp.hist), t):
                    last = t
                    clus = -31 if r2.random() < 0.45 else 0
                    dip = -100 if r2.random() < 0.005 else 0
                    pp.feed(int(ocv(t) + clus + dip + r2.gauss(0, 3)))
                    trace.append(pp.disp)
            outs[d] = (trace, pp)
        same = outs['old'][0] == outs['new'][0]
        if not same or outs['new'][1].settle_runs:
            print('   ✗ 시드 %d: 오류 없는 평소 대기에서 표시가 달라졌다(정착 %d회)' % (seed, outs['new'][1].settle_runs)); ok = False
    print('   10시드 모두 표시 동일·정착 진입 0회' if ok else '   (위 실패 참조)')

    # ── N 풀린 뒤 평탄부 잡음 ─────────────────────────────────────────
    print('[N] 풀린 뒤 평탄부(4000mV) 잡음 σ 3/6/10mV + 31mV 두 무리 — 잠긴 값과 참값 차(시드 40개)')
    for sg in (3, 6, 10):
        diffs, lens = [], []
        for s in range(40):
            pp = PipeSettle()
            rng = random.Random(s)
            for t in range(0, 30000, 30):
                e = -1400 if 1000 <= t < 8200 else 0
                clus = -31 if rng.random() < 0.45 else 0
                pp.feed(int(4000 + clus + rng.gauss(0, sg) + e))
            diffs.append(pp.disp - truth)
            lens += pp.settle_len
            if pp.settling:
                lens.append(10 ** 6)
        print('   σ=%2dmV  표시-참값 [%+d ~ %+d]  정착 표본 최대 %d' % (sg, min(diffs), max(diffs), max(lens)))
        if max(lens) > 60 or min(diffs) < -3 or max(diffs) > 3:
            print('   ✗ 잡음에서 정착이 안 끝나거나 잠긴 값이 참값에서 3%%p 넘게 벗어난다'); ok = False

    # ── D 풀린 직후 수명 끝 급강하 ─────────────────────────────────────
    print('[D] 풀린 직후 실제 급강하(4000→3500mV / 60분) — 표시가 오르내리지 않아야')
    for d in ('old', 'new'):
        pp = mk(d)
        rng = random.Random(7)
        seq = []
        for t in range(0, 20000, 30):
            e = -1400 if 1000 <= t < 8200 else 0
            ocv = 4000 if t < 8800 else 4000 - 500.0 * min(1.0, (t - 8800) / 3600.0)
            clus = -31 if rng.random() < 0.45 else 0
            pp.feed(int(ocv + clus + rng.gauss(0, 3) + e))
            if t >= 9000:
                seq.append(pp.disp)
        ups = sum(1 for i in range(1, len(seq)) if seq[i] > seq[i - 1])
        print('   %-3s 급강하 중 표시 오름 %d번  끝 %d%% (참값 %d%%)' % (d, ups, seq[-1], mv_to_pct(3500 - 15)))
        if d == 'new' and ups > 2:
            print('   ✗ 정착 대기 중 표시가 잡음을 따라 오르내린다'); ok = False

    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
