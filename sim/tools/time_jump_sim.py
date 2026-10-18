#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 '외부 시각 동기 감지' 오탐 수정 시뮬 (사용자 결정 "시각 점프 오탐 고치기").

옛(somfy_app.c _time_tick): 직전 호출 때의 벽시계와 비교해 `now - prev > 120` 이면 "외부 시각 동기 감지" + NVS 저장.
  주석은 "본 루프는 1초 주기"라고 가정했지만 _time_tick_sched 는 **화면 OFF 면 300초**마다 부른다.
  → 화면이 2분 넘게 꺼졌다 켜지면 그동안 흐른 시간을 점프로 잘못 본다(10/18 실측 "+248s 점프", 실제 보정 아님).
  또 앞으로만 봐서, 시계가 빨랐다가 뒤로 맞춰지는 보정은 못 잡는다.
새: 벽시계 차이에서 **실제로 흐른 시간(esp_timer)** 을 뺀 값이 점프. |점프| > 120초면 감지(앞·뒤 모두).

모델: 화면 ON 1초 / OFF 300초 주기로 _time_tick 호출(화면이 켜지는 순간 즉시 1번 — 메인 루프 100ms 안).
  화면 사용: 하루 20~50회 켜짐, 켜짐 10~60초(배터리 OFF 10초 기준 + 조작), 꺼짐 1분~3시간.
  외부 보정(SNTP·Matter SetUTCTime): 몇 시간에 한 번, 크기 ±(5~900)초.
측정: 오탐(보정 없는데 감지) / 놓침(|보정| > 120초인데 감지 없음) / 작은 보정(≤120초) 감지(둘 다 안 잡는 게 정상).
"""
import random
import sys

TH = 120


def run(design, seed, days=7):
    rng = random.Random(seed)
    T = days * 86400.0
    t = 0.0            # 실제 경과(단조) 초
    offset = 0.0       # 벽시계 = t + offset
    panel_on = True
    next_toggle = rng.uniform(10, 60)
    next_set = rng.uniform(1800, 6 * 3600)
    next_tick = 0.0
    prev_wall = None
    prev_mono = None
    m = dict(false_pos=0, miss=0, caught=0, small_caught=0, sets_big=0, sets_small=0)
    pending = []       # 아직 감지 여부가 정해지지 않은 큰 보정(다음 틱에서 판정)
    while t < T:
        t_next = min(next_toggle, next_set, next_tick)
        t = t_next
        if t == next_set:
            d = rng.choice([-1, 1]) * rng.uniform(5, 900)
            offset += d
            big = abs(d) > TH
            m['sets_big' if big else 'sets_small'] += 1
            pending.append(big)
            next_set = t + rng.uniform(1800, 6 * 3600)
            continue
        if t == next_toggle:
            panel_on = not panel_on
            next_toggle = t + (rng.uniform(10, 60) if panel_on else rng.uniform(60, 3 * 3600))
            if panel_on:
                next_tick = t + 0.1           # 켜지는 순간 메인 루프가 바로 부른다(주기 1초로 바뀜)
            continue
        # _time_tick
        wall = t + offset
        if prev_wall is not None:
            if design == 'old':
                hit = (wall - prev_wall) > TH
            else:
                jump = (wall - prev_wall) - (t - prev_mono)
                hit = abs(jump) > TH
            had_big = any(pending)
            had_small = (not had_big) and len(pending) > 0
            if hit:
                if had_big:
                    m['caught'] += 1
                elif had_small:
                    m['small_caught'] += 1
                else:
                    m['false_pos'] += 1
            else:
                if had_big:
                    m['miss'] += 1
            pending = []
        prev_wall, prev_mono = wall, t
        next_tick = t + (1.0 if panel_on else 300.0)
    return m


def main():
    ok = True
    for d in ('old', 'new'):
        m = run(d, seed=5)
        print(f"  {d:3s} 오탐 {m['false_pos']:4d}회 | 큰 보정(>{TH}s) {m['sets_big']}건 중 감지 {m['caught']} / 놓침 {m['miss']}"
              f" | 작은 보정 {m['sets_small']}건 중 감지 {m['small_caught']}   (7일)")
        if d == 'new':
            if m['false_pos'] != 0:
                print('  ✗ 새: 오탐'); ok = False
            if m['miss'] != 0:
                print('  ✗ 새: 큰 보정을 놓쳤다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
