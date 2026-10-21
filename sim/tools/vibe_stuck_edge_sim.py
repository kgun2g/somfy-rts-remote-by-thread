#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★★2026-10-20 이 시뮬은 **폐기됨 — sim/tools/vibe_level_sim.py 를 볼 것.** ISR 을 엣지로 가정했지만 실제로는
  gpio_wakeup_enable 이 인터럽트 유형을 HIGH 레벨로 덮어써(GPT 재감사 N01), HIGH 동안 30ms 마다 재발화했다.
  아래 '잡음 고장(엣지 33회/초)' 시나리오는 그 재발화를 잡음으로 오해한 것이다. 기록으로 남겨 둔다.

★2026-10-18 진동 센서 '고장(HIGH 고정)' 오판 — 사용자 조건 "화면이 켜져 있어도 움직일때 화면이 꺼지지 않아야함".

발견(COM7 실기 10/18 22:1x): 손에 들고 움직이는 동안
    [VIBE] 센서 고장 판정(핀 HIGH 고정) (마지막 LOW 뒤 2023ms, HIGH 표본 34개) — 진동 무시함
  → 26초 뒤 정상 복귀. 고장 상태에서는 _vibration_track 이 진동을 기록하지 않아(s_last_vibration_us 그대로),
    배터리에서는 마지막으로 인정된 진동 10초 뒤 **움직이는 중에도 화면이 꺼진다**.
  원인: 3차(F09)에서 고장 판정을 "마지막 LOW 표본 뒤 2초 + HIGH 표본 5개"로 바꿨다(그 전엔 표본 200개 = 사실상 수십 초).
    X160 은 평상시 닫힘(LOW)·진동 시 열림(HIGH)인데, 흔드는 동안은 대부분 열려 있고 닫히는 순간이 짧아
    표본(30~60ms 간격)에 LOW 가 2초 넘게 안 걸릴 수 있다.
★함정: "엣지가 있으면 고장 아님"(안 A)은 쓸 수 없다 — button_handler.c 주석의 8/12 실측에 **고장 난 C6 센서도
  HIGH 고정 + 잡음 엣지 초당 33회(ISR 재활성 상한)** 가 있다. A 로는 그 고장을 영영 못 잡아 웨이크 폭주(9/02 340회/초)·
  화면 상시 켜짐이 돌아온다.
안 B: 고장 판정(2초, 그대로) 때 **웨이크만 끈다**(전력 보호 즉시). 고장 상태여도 엣지는 판정 뒤 CAP 초 동안
  진동으로 인정한다(움직이는 동안 화면 유지). CAP 이 지나도 LOW 가 한 번도 안 보이면 진짜 고장 → 예전처럼 무시.

모델
  센서: 움직임 = 열림(HIGH) 평균 MEAN_OPEN, 닫힘 0.5~8ms / 깨끗한 고장 = HIGH 고정·엣지 없음 /
        잡음 고장 = HIGH 고정 + 짧은 글리치(재활성되자마자 엣지 — 8/12 실측 33회/초).
  ISR: 엣지 1회 세고 스스로 꺼짐 → 버튼 태스크가 30ms 뒤 다시 켬. 버튼 태스크 표본: ISR 꺼져 있으면 재활성 시한,
       아니면 HIGH 레벨 웨이크가 켜져 있으면 ~60ms(웨이크 폭주 근사), 꺼져 있으면 유휴 1000ms.
  메인 루프·화면: 배터리 화면 OFF 10초, holdoff 5초, 화면 ON 100ms 회전.
측정: 움직이는 동안 화면 꺼짐, 고장인데 화면이 켜져 있던 시간, 웨이크 폭주(HIGH 레벨 웨이크가 켜진 채 HIGH) 시간.
"""
import random
import sys

REARM = 30.0
STUCK_MS = 2000.0
STUCK_N = 5
IDLE_POLL = 1000.0
IDLE_OFF = 10000.0
HOLDOFF = 5000.0
CAP_MS = 60000.0


def sensor_moving(rng, t0, dur, mean_open):
    tr = [(t0, 1)]
    t, lv = t0, 1
    while t < t0 + dur:
        if lv == 1:
            t += rng.expovariate(1.0 / mean_open); lv = 0
        else:
            t += rng.uniform(0.5, 8.0); lv = 1
        tr.append((min(t, t0 + dur), lv))
    tr.append((t0 + dur, 0))
    return tr


def level_at(tr, t, default=0):
    if not tr or t < tr[0][0]:
        return default
    lo, hi = 0, len(tr) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if tr[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return tr[lo][1]


def run(design, scen, seed, mean_open=150.0, total=3600_000.0):
    rng = random.Random(seed)
    tr, moves = [], []
    if scen == 'handle':
        t = 2000.0
        while t < total:
            d = rng.uniform(3000, 40000)
            moves.append((t, t + d))
            tr += sensor_moving(rng, t, d, mean_open)
            t += d + rng.uniform(5000, 60000)
        edges = [tr[i][0] for i in range(1, len(tr)) if tr[i][1] != tr[i - 1][1]]
    else:
        tr = [(0.0, 1)]
        edges = []
    noisy = (scen == 'noisy_stuck')
    ei = 0
    isr_en, isr_off_t, isr_cnt, isr_prev = True, 0.0, 0, 0
    quiet_ms, high_n, isr_quiet = 0.0, 0, 0
    stuck, stuck_t = False, None
    wake_en = True
    last_vib = None
    sleeping, last_act, last_vib_seen = False, 0.0, None
    next_btn, next_main = 0.0, 0.0
    m = dict(off_moving=0, stuck_screen_on=0.0, storm=0.0, stuck_at=None)
    prev_t = 0.0

    def moving(t):
        return any(a <= t < b for a, b in moves)

    while True:
        ne = edges[ei] if ei < len(edges) else 1e18
        nn = (isr_off_t + REARM + 0.5) if (noisy and not isr_en) else 1e18   # 잡음 글리치: 켜지자마자 엣지
        t = min(ne, next_btn, next_main, nn if noisy and isr_en is False else 1e18)
        if noisy and isr_en:
            t = min(t, prev_t + 0.5)                                       # 켜져 있으면 곧 글리치
        if t >= total:
            break
        # 적분: 고장인데 화면 켜짐 / 웨이크 폭주
        dt = t - prev_t
        if scen != 'handle':
            if not sleeping and m['stuck_at'] is not None:
                m['stuck_screen_on'] += dt
            if wake_en:
                m['storm'] += dt
        prev_t = t
        if noisy and isr_en and t >= prev_t:
            isr_en = False; isr_off_t = t; isr_cnt += 1
            next_btn = min(next_btn, t + REARM)
            continue
        if t == ne:
            ei += 1
            if isr_en:
                isr_en = False; isr_off_t = t; isr_cnt += 1
                next_btn = min(next_btn, t + REARM)
            continue
        if t == next_btn:
            if not isr_en and t - isr_off_t >= REARM:
                isr_en = True
            hi = level_at(tr, t) == 1
            if not hi:
                quiet_ms, high_n, isr_quiet = t, 0, isr_cnt
            else:
                high_n += 1
                if design == 'A' and isr_cnt != isr_quiet:
                    isr_quiet, quiet_ms, high_n = isr_cnt, t, 1
            now_stuck = hi and (t - quiet_ms) >= STUCK_MS and high_n >= STUCK_N
            if now_stuck != stuck:
                stuck = now_stuck
                wake_en = not stuck
                if stuck:
                    stuck_t = t
                    if m['stuck_at'] is None:
                        m['stuck_at'] = t
            dlt = isr_cnt - isr_prev
            isr_prev = isr_cnt
            recog = (not stuck) or (design == 'B' and stuck_t is not None and (t - stuck_t) < CAP_MS)
            if recog and dlt >= 1:
                last_vib = t
            if not isr_en:
                next_btn = isr_off_t + REARM
            elif hi and wake_en:
                next_btn = t + 60.0
            else:
                next_btn = t + IDLE_POLL
            continue
        # 메인 루프
        vib_active = last_vib is not None and (t - last_vib) < 500.0
        vib_event = last_vib is not None and last_vib != last_vib_seen
        if vib_active or vib_event:
            last_act = t; sleeping = False
        last_vib_seen = last_vib
        hold = last_vib is not None and (t - last_vib) < HOLDOFF
        if not (vib_active or hold) and (t - last_act) >= IDLE_OFF and not sleeping:
            sleeping = True
            if moving(t):
                m['off_moving'] += 1
        next_main = t + (1000.0 if sleeping else 100.0)
    return m


def main():
    ok = True
    print('[움직임 1시간 — 움직이는데 화면 꺼진 횟수]  ※실측 vibelog 는 열림 평균 50~150ms 쪽')
    for mo in (50.0, 150.0, 300.0, 600.0):
        r = {d: run(d, 'handle', 3, mo) for d in ('old', 'A', 'B')}
        print(f"  열림 평균 {mo:4.0f}ms   옛 {r['old']['off_moving']:3d}   A {r['A']['off_moving']:3d}   B {r['B']['off_moving']:3d}")
        if r['B']['off_moving'] > (0 if mo <= 300 else r['old']['off_moving'] // 10):
            print('  ✗ B: 움직이는데 화면이 꺼진다'); ok = False
    for scen, name in (('stuck', '깨끗한 고장(HIGH 고정·엣지 없음)'), ('noisy_stuck', '잡음 고장(HIGH 고정 + 엣지 33회/초, 8/12 실측형)')):
        print(f'[{name} — 1시간]')
        for d in ('old', 'A', 'B'):
            r = run(d, scen, 1)
            sa = r['stuck_at']
            print(f"  {d:3s} 고장 판정 {('%.2f초' % (sa / 1000)) if sa else '못 함':>8s} | 고장 뒤 화면 켜짐 {r['stuck_screen_on']/1000:7.1f}초"
                  f" | 웨이크 폭주 {r['storm']/1000:7.1f}초")
            if d == 'B':
                if sa is None or sa > 2200:
                    print('  ✗ B: 2.2초 안에 고장 판정 못 함'); ok = False
                if r['storm'] > 2500:
                    print('  ✗ B: 웨이크 폭주가 판정 시간보다 길다'); ok = False
                if r['stuck_screen_on'] > CAP_MS + IDLE_OFF + 2000:
                    print('  ✗ B: 고장인데 화면이 CAP+10초보다 오래 켜져 있다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
