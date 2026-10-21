#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-20 진동 경로 '레벨 기준' 재설계 시뮬 — GPT 재감사(doc/POWER_SAVING_REAUDIT_2026-10-19.md) N01·N02.

사실(IDF 5.5.5 소스 확인): gpio_wakeup_enable(VIBE, HIGH_LEVEL) 이 gpio_hal_set_intr_type() 으로 **일반 인터럽트도
HIGH 레벨**로 바꾼다(gpio_wakeup_disable 은 안 되돌림). 그래서 진동 ISR 은 엣지가 아니라 "켜져 있는 동안 HIGH 면 걸림"이다.
  → 이 시뮬은 그 레벨 의미를 그대로 모델링한다(이전 vibe_stuck_edge_sim 은 엣지로 가정해 이 반례를 못 봤다).

옛(현재 e9ed0b5 코드)
  · ISR: 끄고 세기만(통지 없음). 버튼 태스크가 30ms 뒤 **핀 레벨과 무관하게** 다시 켬 → HIGH 가 이어지면 30ms 마다 재발화.
  · 웨이크(HIGH 레벨): '고장'(HIGH 표본만 2초·5개)일 때만 끄고, LOW 보이면 다시 켬 → HIGH 동안 절전 즉시 깨어남(폭주).
  · 진동 인정: ISR 증가 / HIGH 표본 2개 300ms 안. 고장이면 판정 뒤 60초 동안 ISR 증가만.
새(레벨 기준)
  · 인터럽트·웨이크는 **핀이 LOW 일 때만** 켠다. ISR 이 걸리면 끄고 버튼 태스크에 **통지**(한 번 켜면 한 번만 걸려 폭주 없음).
  · 30ms 뒤 확인: LOW 면 다시 켬, HIGH 면 꺼 둔 채 LOW 가 보일 때 켬(30ms 시한 반복 없음). HIGH 를 보면 웨이크 즉시 끔.
  · 진동 인정: ISR 증가 · 절전에서 GPIO 로 깸 · HIGH 표본(HIGH 가 이어진 지 2초+60초 안). 부팅 때 HIGH 여도 첫 LOW 에서 켬.
공통: 버튼 태스크 유휴 1000ms, 메인 루프 화면 ON 100ms / OFF 1000ms(+진동 kick: 잠듦 즉시·ON 1초 1회),
      배터리 화면 OFF 10초·holdoff 5초. CPU 한 번 깨면 1ms 깨어 있음, 절전 즉시 깨어남 1회 = 3.4ms(전환 2.5 + 깨어남 0.9).
"""
import bisect
import random
import statistics as st
import sys

REARM, IDLE_BTN, MIN_WAIT = 30.0, 1000.0, 2.0
MAIN_ON, MAIN_OFF = 100.0, 1000.0
IDLE_OFF, HOLDOFF, RECENT = 10000.0, 5000.0, 500.0
STUCK_MS, STUCK_N, RECOG = 2000.0, 5, 60000.0
DUTY = 300.0
STORM_DT, AWAKE = 3.4, 1.0


class Pin:
    def __init__(self, tr):
        self.t = [a for a, _ in tr]
        self.v = [b for _, b in tr]

    def level(self, t):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else 0

    def next_high(self, t):
        """t 이후(포함) 처음 HIGH 인 시각."""
        if self.level(t):
            return t
        i = bisect.bisect_right(self.t, t)
        while i < len(self.t):
            if self.v[i]:
                return self.t[i]
            i += 1
        return 1e18


def build(scen, rng, total):
    tr = [(0.0, 0)]
    taps, moves, opens = [], [], []

    def tap(t0):
        d = rng.uniform(2, 20)
        tr.extend([(t0, 1), (t0 + d, 0)])
        taps.append(t0)

    def handle(t0, dur, mode):
        t = t0
        if mode == 'open':          # 대부분 열림(HIGH), 짧게 닫힘
            lv = 1
            tr.append((t, 1))
            while t < t0 + dur:
                t += rng.expovariate(1 / 150.0) if lv else rng.uniform(0.5, 8.0)
                lv ^= 1
                tr.append((min(t, t0 + dur), lv))
        else:                       # 대부분 닫힘(LOW), 짧게 열림
            while t < t0 + dur:
                t += rng.expovariate(1 / 80.0)
                d = rng.uniform(2, 20)
                tr.extend([(t, 1), (t + d, 0)])
                t += d
        tr.append((t0 + dur, 0))
        moves.append((t0, t0 + dur))

    t = 3000.0
    if scen == 'rest_tap':
        while t < total:
            tap(t); t += rng.uniform(20000, 120000)
    elif scen in ('handle_open', 'handle_closed'):
        while t < total:
            d = rng.uniform(3000, 40000)
            handle(t, d, 'open' if scen == 'handle_open' else 'closed')
            t += d + rng.uniform(15000, 90000)
    elif scen == 'rest_open':        # 흔들다 내려놓았는데 열린 채(HIGH) 10분, 다시 닫힘
        while t < total:
            handle(t, 5000, 'open')
            t += 5000
            tr.extend([(t, 1)]); opens.append((t, t + 600000)); t += 600000
            tr.append((t, 0)); t += rng.uniform(30000, 90000)
    elif scen == 'stuck':
        tr = [(0.0, 1)]
    elif scen == 'boot_high':        # 부팅 1.5초 동안 HIGH → LOW, 이후 두드림
        tr = [(0.0, 1), (1500.0, 0)]
        t = 20000.0
        while t < total:
            tap(t); t += rng.uniform(20000, 120000)
    elif scen == 'tap_awake':        # 두드림이 CPU 가 다른 일로 깨어 있을 때 오는 경우(N02)
        while t < total:
            tap(t); t += rng.uniform(12000, 30000)
    tr.sort(key=lambda x: x[0])
    return Pin(tr), taps, moves, opens


def run(design, scen, seed=1, total=3600_000.0):
    rng = random.Random(seed)
    pin, taps, moves, opens = build(scen, rng, total)
    bg = []                                   # 배경 깨어남(Thread 폴 등) — tap_awake 에서는 두드림 직전에 둔다
    if scen == 'tap_awake':
        bg = sorted(tt - rng.uniform(0.2, 4.0) for tt in taps)   # 두드림 순간 CPU 가 이미 깨어 있음(최대 ~5ms 일)
    else:
        x = 0.0
        while x < total:
            x += rng.uniform(6500, 7500); bg.append(x)
    s = dict(intr=False, wake=(pin.level(0) == 0), fired=False, off_t=0.0, isr=0, isr_prev=0,
             gpio_woke=False, low_ms=0.0, high_n=0, run0=None, stuck=False, stuck_t=0.0, hi1=False, hi1_t=0.0)
    if design == 'old':
        s['intr'] = True                      # ANYEDGE 설정 + 등록(레벨로 바뀜) — 처음부터 켜져 있다
    m = dict(wakes=0, storm=0, btn=0, main=0, isr=0, lat=[], off_moving=0, open_on=0.0, open_wakes=0, boot_ok=0, boot_n=0)
    last_vib = None; last_vib_seen = None
    sleeping = False; last_act = 0.0
    btn_next = rng.uniform(0, IDLE_BTN); main_next = 0.0
    awake_until = -1.0
    last_kick = -1e9
    pend_tap = None
    bgi = 0

    def wake_at(t, storm=False):
        nonlocal awake_until
        if t > awake_until:
            m['wakes'] += 1
            if storm:
                m['storm'] += 1
            if opens and any(a <= t < b for a, b in opens):
                m['open_wakes'] += 1
        awake_until = max(awake_until, t + AWAKE)

    def activity(t):
        nonlocal last_vib, main_next, last_kick
        last_vib = t
        if sleeping or (t - last_kick) >= 1000.0:     # somfy_app_vibe_kick
            last_kick = t
            main_next = min(main_next, t + 0.05)

    def isr(t):
        nonlocal btn_next
        s['intr'] = False; s['fired'] = True; s['off_t'] = t; s['isr'] += 1; m['isr'] += 1
        if design == 'new':
            btn_next = min(btn_next, t + 0.05)         # ISR 통지

    def on_wake(t, storm=False):
        """CPU 가 깨는 모든 사건: 깨어남을 세고, 인터럽트가 켜져 있고 HIGH 면 그 자리에서 ISR(레벨)."""
        wake_at(t, storm)
        if s['intr'] and pin.level(t):
            isr(t)

    ti = 0
    t = 0.0
    while t < total:
        cands = [btn_next, main_next, bg[bgi] if bgi < len(bg) else 1e18]
        # 깨어 있는 동안 HIGH 가 되면 ISR(레벨) — 깨어 있는 구간 안에서만
        t_isr = 1e18
        if s['intr']:
            th = pin.next_high(t)
            if th <= awake_until:
                t_isr = th
        # 자는 중 GPIO 웨이크: 자려는 순간(awake_until) HIGH 면 즉시 깨어남(폭주), 아니면 다음 HIGH 순간
        t_gw = 1e18
        storm_next = False
        if s['wake']:
            ts = max(t, awake_until)
            if pin.level(ts):
                t_gw = ts + STORM_DT; storm_next = True
            else:
                t_gw = pin.next_high(ts)
        nt = min(cands + [t_isr, t_gw])
        if nt >= total:
            break
        t = max(nt, t)
        # 두드림 시작이 화면 OFF 중이면 켜짐 지연 측정 시작
        while ti < len(taps) and taps[ti] <= t:
            if sleeping and pend_tap is None:
                pend_tap = taps[ti]
            ti += 1
        if nt == t_gw:
            on_wake(t, storm_next)
            s['gpio_woke'] = True                       # _ls_exit_cb: GPIO 원인 → 버튼 태스크 통지
            btn_next = min(btn_next, t + 0.05)
            continue
        if nt == t_isr:
            isr(t)
            continue
        if bgi < len(bg) and nt == bg[bgi]:
            bgi += 1
            on_wake(t)
            awake_until = max(awake_until, t + 5.0)     # Thread 폴 ~5ms 깨어 있음
            continue
        if nt == btn_next:
            on_wake(t); m['btn'] += 1
            hi = pin.level(t) == 1
            dlt = s['isr'] - s['isr_prev']; s['isr_prev'] = s['isr']
            if design == 'old':
                if not hi:
                    s['low_ms'] = t; s['high_n'] = 0
                else:
                    s['high_n'] += 1
                now_stuck = hi and (t - s['low_ms']) >= STUCK_MS and s['high_n'] >= STUCK_N
                if now_stuck != s['stuck']:
                    s['stuck'] = now_stuck
                    s['wake'] = not now_stuck
                    if now_stuck:
                        s['stuck_t'] = t
                if s['stuck']:
                    if (t - s['stuck_t']) < RECOG and dlt >= 1:
                        activity(t)
                elif dlt >= 1:
                    activity(t); s['hi1'] = False
                elif hi:
                    if s['hi1'] and t - s['hi1_t'] <= DUTY:
                        activity(t); s['hi1'] = False
                    else:
                        s['hi1'] = True; s['hi1_t'] = t
                s['gpio_woke'] = False
                if s['fired'] and t - s['off_t'] >= REARM:
                    s['fired'] = False; s['intr'] = True        # 레벨 무관 재활성(HIGH 면 곧바로 재발화)
                    if pin.level(t):
                        isr(t)                                   # gpio_intr_enable 순간 ISR → 다음 시한 30ms
            else:
                if not hi:
                    s['run0'] = None; s['high_n'] = 0
                else:
                    if s['run0'] is None:
                        s['run0'] = t
                    s['high_n'] += 1
                s['stuck'] = hi and (t - s['run0']) >= STUCK_MS and s['high_n'] >= STUCK_N
                if dlt >= 1 or s['gpio_woke']:
                    activity(t)
                s['gpio_woke'] = False
                if hi and (t - s['run0']) < STUCK_MS + RECOG:
                    activity(t)
                if hi:
                    s['wake'] = False; s['intr'] = False
                    if s['fired'] and t - s['off_t'] >= REARM:
                        s['fired'] = False                       # LOW 를 볼 때까지 꺼 둠(30ms 시한 끝)
                elif not (s['fired'] and t - s['off_t'] < REARM):
                    s['fired'] = False; s['intr'] = True; s['wake'] = True
            tmo = IDLE_BTN
            if s['fired']:
                tmo = min(tmo, max(REARM - (t - s['off_t']), 0) + 1)
            btn_next = t + max(tmo, MIN_WAIT)
            continue
        if nt == main_next:
            on_wake(t); m['main'] += 1
            va = last_vib is not None and (t - last_vib) < RECENT
            ve = last_vib is not None and last_vib != last_vib_seen
            if va or ve:
                last_act = t
                if sleeping:
                    sleeping = False
                    if pend_tap is not None:
                        m['lat'].append(t - pend_tap); pend_tap = None
            last_vib_seen = last_vib
            hold = last_vib is not None and (t - last_vib) < HOLDOFF
            if not (va or hold) and (t - last_act) >= IDLE_OFF and not sleeping:
                sleeping = True
                if any(a <= t < b for a, b in moves):
                    m['off_moving'] += 1
            if not sleeping and opens and any(a <= t < b for a, b in opens):
                m['open_on'] += (MAIN_ON if not sleeping else MAIN_OFF)
            main_next = t + (MAIN_OFF if sleeping else MAIN_ON)
            continue
    m['open_s'] = sum(min(b, total) - a for a, b in opens if a < total) / 1000.0 if opens else 1.0
    m['n_open'] = max(1, len([1 for a, b in opens if a < total]))
    return m, taps


def tap_latencies(design, scen, seed=1):
    """두드림 → 화면 켜짐 지연(화면 OFF 상태에서)만 따로 잰다."""
    rng = random.Random(seed)
    m, taps = run(design, scen, seed)
    return m


def main():
    ok = True
    H = 3600.0
    rows = [('rest_tap', '정지 LOW + 가끔 두드림'), ('handle_closed', '들고 움직임(접점 대부분 닫힘)'),
            ('handle_open', '들고 움직임(접점 대부분 열림)'), ('rest_open', '흔들다 내려놓으니 열린 채 10분'),
            ('stuck', '센서 HIGH 고정(고장)'), ('boot_high', '부팅 때 HIGH → LOW 뒤 두드림'),
            ('tap_awake', 'CPU 가 깨어 있을 때 두드림(N02)')]
    res = {}
    for scen, name in rows:
        res[scen] = {}
        print(f"\n[{name}] 1시간")
        for d in ('old', 'new'):
            m, taps = run(d, scen, 3)
            res[scen][d] = m
            lat = m['lat']
            lat_s = f"두드림→켜짐 중앙 {st.median(lat):5.1f} / 최대 {max(lat):7.1f}ms ({len(lat)}회)" if lat else "두드림→켜짐 없음"
            extra = ''
            if scen == 'rest_open':
                extra = f" | 열린 채 구간 깨어남 {m['open_wakes'] / m['open_s']:.2f}/s, 그 구간 화면 켜짐 {m['open_on']/1000/m['n_open']:.0f}초/회"
            print(f"  {d:3s} 깨어남 {m['wakes']/H:6.2f}/s (폭주 {m['storm']/H:6.2f}, 버튼 {m['btn']/H:6.2f}, ISR {m['isr']/H:6.2f})"
                  f" | 움직이는데 화면 꺼짐 {m['off_moving']} | {lat_s}{extra}")
    o, n = res, res
    # 판정
    def w(sc, d): return res[sc][d]['wakes'] / H
    if w('rest_tap', 'new') > w('rest_tap', 'old') * 1.05 + 0.05:
        print('  ✗ 새: 정지 대기 깨어남이 늘었다'); ok = False
    for sc in ('handle_closed', 'handle_open'):
        if res[sc]['new']['off_moving'] != 0:
            print(f'  ✗ 새: {sc} 움직이는데 화면 꺼짐'); ok = False
    if w('stuck', 'new') > 2.6:
        print(f'  ✗ 새: 고장 깨어남 {w("stuck", "new"):.2f}/s > 2.6(유휴 버튼 1 + 메인 1 + Thread)'); ok = False
    # 열린 채 구간에는 화면이 켜져 있는 60초(메인 100ms)가 섞인다 — 옛보다 확실히 적고 3.5/s 이하면 된다
    ro, rold = res['rest_open']['new'], res['rest_open']['old']
    if ro['open_wakes'] / ro['open_s'] > 3.5 or ro['open_wakes'] > rold['open_wakes'] / 5:
        print('  ✗ 새: 열린 채 구간 깨어남이 충분히 줄지 않았다'); ok = False
    for sc in ('rest_tap', 'boot_high', 'tap_awake'):
        ln = res[sc]['new']['lat']
        if not ln:
            print(f'  ✗ 새: {sc} 두드림으로 화면이 안 켜진다'); ok = False
        elif max(ln) > 15.0:
            print(f'  ✗ 새: {sc} 켜짐 최대 {max(ln):.1f}ms > 15ms'); ok = False
    print('\n판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
