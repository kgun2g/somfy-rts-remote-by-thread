# -*- coding: utf-8 -*-
"""★2026-10-20 배터리 ADC 주기 — 화면 OFF(배터리)일 때 5초 → 30초 (GPT 재감사 N08, 사용자 결정 "30초").

빌드 전 검증(사용자 규칙 "빌드 전 가상검증"). 표시 % 파이프라인은 5초 주기를 전제로 다듬어 왔다:
  중앙값 9 + EMA(α=1/4)  →  급락 표본 거부(300mV, 연속 3회까지)  →  %  →  방전 하한(창 5개 뒤, 해제 100mV×5표본)
  →  하락 속도 제한(1%p/측정, USB 전환 1회 면제)
주기를 늘리면 이 장치들이 "몇 표본"으로 세는 시간이 6배가 된다. 그래서 다음 조건이면 5초를 유지한다(펌웨어와 같다):
  · USB 이거나 화면이 켜져 있을 때(사용자가 보는 동안은 그대로)
  · 평활 창이 다 안 찼을 때(부팅·USB 전환 직후 — 첫 45초)
  · batlog 방전 세션이 없거나 시작 120초 안(기준점 수렴·빠른 기록 구간 BATLOG_FAST_S)
  · BOARD_BAT_SWAPPED 보드(ADC 로 VBUS 를 재 USB 판정에 쓴다 — 이 시뮬 대상 아님)

★판정 기준: 사용자는 **화면이 켜져 있을 때만** 숫자를 본다. 화면 OFF 동안 표시가 늦게 따라가는 것은 아무도 못 본다.
  그래서 '보이는 오차'는 화면 ON 구간에서만 잰다. 기록(batlog 120초 표본)은 전압을 그대로 쓰므로 따로 본다.

시나리오(각각 옛 5초 고정 vs 새 규칙)
  A 정지 대기 24시간, 하루 40회 화면 10초 — 보이는 오차·오르내림·ADC 횟수
  B 측정 오류 급락(8/23형) 1표본 / 15초 / 60초 / 5분 지속 — 화면에 보이는 최저
  C 오류 급락이 2시간 이어진 뒤 회복(8/20형) — 하한 해제까지(화면 ON 기준)
  D 수명 끝 급강하(3700→3300mV / 60분) — 화면 ON 순간 지연
  E 배터리 재부팅(세션 이어받기) — 첫 45초는 5초여야 한다
"""
import random
import sys

V = [3200, 3450, 3580, 3680, 3750, 3850, 3980, 4038, 4078, 4108]   # somfy_app.c _bat_mv_to_pct (BAT_FULL_MV 4108)
P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]
WIN = 9
FLOOR_MIN_N = WIN // 2 + 1
REL_MV, REL_N = 100, 5
SPIKE_MV, SPIKE_N = 300, 3
DROP = 1
FAST_S = 120
IDLE_OFF_S = 10          # 배터리 화면 OFF 10초


def mv_to_pct(mv):
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i - 1] + (P[i] - P[i - 1]) * (mv - V[i - 1]) // (V[i] - V[i - 1])
    return 100


class Pipe(object):
    """somfy_app.c 배터리 블록(정상 배선, 배터리 구동 중) 그대로."""

    def __init__(self):
        self.hist = []
        self.ema = 0
        self.ok_mv = 0
        self.rej = 0
        self.floor = None
        self.floor_mv = 0
        self.up_run = 0
        self.disp = None
        self.reseed = False

    def reset(self):            # _bat_smooth_reset + s_disp_reseed (USB 전환)
        self.hist = []
        self.ema = 0
        self.reseed = True

    def feed(self, mv):
        self.hist.append(mv)
        if len(self.hist) > WIN:
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
            else:
                self.up_run = 0
            if self.floor is not None and pct > self.floor:
                pct = self.floor
            else:
                self.floor = pct
                self.floor_mv = sm
        if self.reseed:
            self.reseed = False
        elif self.disp is not None and pct < self.disp - DROP:
            pct = self.disp - DROP
        self.disp = pct
        return sm


def period_s(rule, screen_on, hist_n, sess_el):
    if rule == 'old':
        return 5
    if screen_on or hist_n < WIN or sess_el is None or sess_el < FAST_S:
        return 5
    return 30


def run(rule, ocv_fn, err_fn, dur_s, wakes, seed, resume=False, sess0=None):
    """1초 메인 루프 근사(화면 OFF 중 메인 루프 1초, ON 중 100ms — ADC 판정은 1초 해상도로 충분).
    wakes: 화면 ON 시작 시각 목록(각 10초 + 유휴 10초 뒤 OFF). 반환: 측정."""
    rng = random.Random(seed)
    pp = Pipe()
    last = -10 ** 9
    on_until = -1
    wi = 0
    sess_t0 = sess0 if sess0 is not None else 0     # 세션 시작(초). None 이면 세션 없음
    m = dict(reads=0, vis_err=0, vis_rise=0, vis_min=101, lag_on=[], first45_max_iv=0)
    prev_vis = None
    prev_read = None
    for t in range(dur_s):
        while wi < len(wakes) and wakes[wi] <= t:
            on_until = wakes[wi] + 10 + IDLE_OFF_S
            wi += 1
        screen = t < on_until
        sess_el = None if sess_t0 is None else t - sess_t0
        if t - last >= period_s(rule, screen, len(pp.hist), sess_el):
            if prev_read is not None and t < 45 + 5:
                m['first45_max_iv'] = max(m['first45_max_iv'], t - prev_read)
            prev_read = t
            last = t
            ocv = ocv_fn(t)
            clus = -31 if rng.random() < 0.45 else 0          # 배터리 구동 중 31mV 두 무리(실측 #16)
            dip = -100 if rng.random() < 0.005 else 0          # 송신 순간 걸린 표본
            pp.feed(int(ocv + clus + dip + rng.gauss(0, 3) + err_fn(t)))
            m['reads'] += 1
        if screen and pp.disp is not None:
            truth = mv_to_pct(ocv_fn(t) - 15)                  # 두 무리 평균 쪽 참값
            e = abs(pp.disp - truth)
            m['vis_err'] = max(m['vis_err'], e)
            m['vis_min'] = min(m['vis_min'], pp.disp)
            if prev_vis is not None and pp.disp > prev_vis:
                m['vis_rise'] += 1
            prev_vis = pp.disp
            if t in wakes:
                m['lag_on'].append(pp.disp - truth)
    return m


def wakes_per_day(rng, dur_s, n_per_day):
    n = int(n_per_day * dur_s / 86400.0)
    return sorted(rng.randrange(60, dur_s - 30) for _ in range(n))


def main():
    ok = True
    day = 86400
    rng = random.Random(7)

    # ── A 정지 대기 24시간 ─────────────────────────────────────
    ocvA = lambda t: 4060 - 60.0 * t / day                    # 하루 ~60mV (15일 목표 근처 기울기)
    W = wakes_per_day(rng, day, 40)
    print('[A] 정지 대기 24시간, 화면 ON %d회(10초)' % len(W))
    rA = {r: run(r, ocvA, lambda t: 0, day, W, 1) for r in ('old', 'new')}
    for r in ('old', 'new'):
        x = rA[r]
        print('   %-3s ADC %6d회/일  보이는 오차 최대 %2d%%p  화면 ON 중 오름 %3d번  ON 순간 오차 %s'
              % (r, x['reads'], x['vis_err'], x['vis_rise'],
                 '%+d~%+d' % (min(x['lag_on']), max(x['lag_on'])) if x['lag_on'] else '-'))
    cut = 100.0 * (1 - rA['new']['reads'] / float(rA['old']['reads']))
    print('   → ADC 측정 %.1f%% 감소' % cut)
    if cut < 80:
        print('   ✗ 감소가 80%% 미만'); ok = False
    if rA['new']['vis_err'] > rA['old']['vis_err'] + 1:
        print('   ✗ 보이는 오차가 1%%p 넘게 커졌다'); ok = False
    if rA['new']['vis_rise'] > rA['old']['vis_rise']:
        print('   ✗ 화면 ON 중 오르내림이 늘었다'); ok = False

    # ── B 측정 오류 급락(8/23형: 4055→3002mV) — **화면 OFF 중에** 일어나고, 그 뒤 화면을 켠다 ──
    #  화면 ON 중에 일어나면 두 규칙 다 5초라 같다(첫 판에서 그렇게 짜 놓고 '같다'를 확인한 건 의미가 없었다).
    print('[B] 측정 오류 급락 -1050mV 가 화면 OFF 중 발생 → 오류 끝 0~120초 뒤 화면 ON, 그때 보이는 최저')
    for lab, dur in (('1표본', 1), ('15초', 15), ('60초', 60), ('5분', 300)):
        res = {}
        for r in ('old', 'new'):
            worst = 101
            for start in range(3000, 3000 + 60, 3):            # 측정 위상을 바꿔 가며
                for k in (0, 30, 120):
                    err = (lambda s, d: (lambda t: -1050 if s <= t < s + d else 0))(start, dur)
                    on = start + dur + k
                    x = run(r, lambda t: 4000.0, err, on + 200, [on + i for i in range(0, 200, 20)], start)
                    worst = min(worst, x['vis_min'])
            res[r] = worst
        print('   %-5s 화면 최저  옛 %3d%%  새 %3d%%   (참값 %d%%)' % (lab, res['old'], res['new'], mv_to_pct(3985)))
        if res['new'] < res['old'] - 1:
            print('   ✗ 새 규칙이 급락을 더 많이 보인다'); ok = False

    # ── C 오류 2시간 뒤 회복(8/20형) — 회복 k초 뒤 화면을 켜고 200초 켜 둔다 ──
    #  ★부수 발견(두 규칙 공통, 이번 변경과 무관): 회복 중 EMA 가 오르는 도중 하한이 풀렸다가 **곧바로 다시 잠겨**
    #   (3935mV=73%) 참값(78~80%)까지 못 올라간다. 해제 조건(+100mV)이 다시 안 차서다. 옛 펌웨어 그대로의 거동.
    print('[C] 오류 급락 2시간 뒤 회복 → k초 뒤 화면 ON: 켠 순간 / 30초 / 60초 / 200초 표시 (참값 %d%%)' % mv_to_pct(3985))
    fin = {}
    for r in ('old', 'new'):
        err = lambda t: -1400 if 1000 <= t < 1000 + 7200 else 0
        rec = 1000 + 7200
        line = []
        for k in (10, 60, 300, 900):
            on = rec + k
            trace = {}
            pp = Pipe(); rng2 = random.Random(3); last = -10 ** 9
            for t in range(on + 201):
                scr = on <= t
                if t - last >= period_s(r, scr, len(pp.hist), t):
                    last = t
                    clus = -31 if rng2.random() < 0.45 else 0
                    pp.feed(int(4000 + clus + rng2.gauss(0, 3) + err(t)))
                if t - on in (0, 30, 60, 200):
                    trace[t - on] = pp.disp
            line.append('%4d초: %3d/%3d/%3d/%3d' % (k, trace[0], trace[30], trace[60], trace[200]))
            fin[(r, k)] = (trace[60], trace[200])
        print('   %-3s ' % r + '  '.join(line))
    for k in (10, 60, 300, 900):
        if fin[('new', k)][0] < fin[('new', k)][1] - 2:
            print('   ※회복 %d초 뒤 켬: 새 규칙은 켠 뒤 60초 안에 못 맞춘다' % k)
    #  다시 잠기는 값(위 73 vs 69)은 해제 순간 EMA 가 어디쯤인지 = 측정 위상에 달린 운이다. 한 시드로 가르면 안 된다 →
    #  시드 40개로 '켜고 200초 뒤 표시'의 분포를 비교한다.
    def c_final(r, seed, k):
        err = lambda t: -1400 if 1000 <= t < 8200 else 0
        on = 8200 + k
        pp = Pipe(); rng2 = random.Random(seed); last = -10 ** 9
        for t in range(on + 201):
            if t - last >= period_s(r, on <= t, len(pp.hist), t):
                last = t
                clus = -31 if rng2.random() < 0.45 else 0
                pp.feed(int(4000 + clus + rng2.gauss(0, 3) + err(t)))
        return pp.disp
    for k in (10, 300, 900):
        fo = sorted(c_final('old', s, k) for s in range(40))
        fn = sorted(c_final('new', s, k) for s in range(40))
        mo, mn = sum(fo) / 40.0, sum(fn) / 40.0
        print('   회복 %3d초 뒤 켬, 200초 뒤 표시 (시드 40개)  옛 평균 %.1f%% [%d~%d]   새 평균 %.1f%% [%d~%d]'
              % (k, mo, fo[0], fo[-1], mn, fn[0], fn[-1]))
        if mn < mo - 2:
            print('   ✗ 새 규칙이 평균 2%%p 넘게 낮게 다시 잠긴다'); ok = False

    # ── D 수명 끝 급강하 ───────────────────────────────────────
    print('[D] 수명 끝 3700→3300mV / 60분 — 화면을 켠 순간의 표시 - 참값')
    ocvD = lambda t: 3700 - 400.0 * max(0, t - 600) / 3600.0
    WD = list(range(700, 600 + 3600, 300))
    for r in ('old', 'new'):
        x = run(r, ocvD, lambda t: 0, 600 + 3600, WD, 5)
        print('   %-3s 켠 순간 오차 %s  (최대 %d%%p)' % (r, ' '.join('%+d' % v for v in x['lag_on']),
                                                max(abs(v) for v in x['lag_on'])))
        if r == 'new' and max(abs(v) for v in x['lag_on']) > 3:
            print('   ✗ 수명 끝에서 켠 순간 3%%p 넘게 늦다'); ok = False

    # ── E 배터리 재부팅(세션 이어받기, 세션 경과 큼) ───────────
    print('[E] 배터리 재부팅 — 세션은 이어받아 경과가 크지만 평활 창이 비어 있다')
    x = run('new', lambda t: 3950.0, lambda t: 0, 600, [], 9, sess0=-5000)
    print('   첫 45초 측정 간격 최대 %d초 (5초여야 한다)' % x['first45_max_iv'])
    if x['first45_max_iv'] > 5:
        print('   ✗ 창이 차기 전에 30초로 넘어갔다'); ok = False

    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
