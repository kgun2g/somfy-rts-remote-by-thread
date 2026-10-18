#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 진동 감지 → 메인 루프 깨우기 빈도 제한 시뮬 (사용자 결정:
  "화면이 켜져 있을 때 진동 감지로 메인 루프를 깨우는 것은 1초 변경(화면이 켜져 있어도 움직일때 화면이 꺼지지 않아야함)").

옛(3차 F10): 버튼 태스크가 진동을 판정할 **때마다** somfy_app_main_kick() — 손에 들고 있으면 ISR 재활성(30ms)마다
  판정이 나서 메인 루프가 초당 ~30번 더 돈다(10/18 2분 배터리 시험: 메인 890회, 화면 켜짐만으로는 ~580회).
새: somfy_app_vibe_kick() — 화면 OFF(s_is_sleeping, 메인 루프 1초 대기)면 **즉시**, 화면 ON 이면 **1초에 한 번만**.
  화면 ON 이면 메인 루프는 어차피 100ms 마다 돌며 진동 시각을 읽으므로(_mark_activity·5초 holdoff) 깨우기가 없어도
  화면이 안 꺼진다 — 이 시뮬로 확인한다.

메인 루프 모델(somfy_app.c): 화면 ON 100ms / 화면 OFF(배터리, 잠듦) 1000ms, 통지(kick)면 즉시 한 회전.
  회전마다: vib_event(마지막 진동 시각이 바뀜) 또는 진동 중(최근 500ms) → _mark_activity, 잠들어 있으면 _exit_sleep(화면 ON)
           → 진동 중·holdoff(마지막 진동 뒤 5초) 아니면 유휴 10초(CFG_SCREEN_OFF_SEC)에 화면 OFF(잠듦).
진동 판정 모델(button_handler.c _vibration_track): 움직이는 동안 판정이 25~40ms 간격(ISR 재활성 30ms 근처)으로 난다.
  잠깐 두드림은 판정 1~3번. 판정 시각 → last_vib 갱신 + 깨우기 호출.
검증
  ① 움직이는 동안 화면이 꺼진 적 0 (옛·새 모두)
  ② 화면 OFF 상태에서 두드림 → 화면 켜짐 지연: 새 = 옛 (즉시 깨움 유지)
  ③ 손에 든 동안 메인 루프 회전/초: 옛 ≈ 30+ → 새 ≈ 11 (100ms 회전 + 1초 1회)
  ④ 마지막 움직임 뒤 화면이 꺼지기까지 시간: 옛 = 새 (holdoff 5초·유휴 10초 그대로)
"""
import random
import statistics as st
import sys

ON_MS, OFF_MS = 100.0, 1000.0
IDLE_OFF_MS = 10000.0
HOLDOFF_MS = 5000.0
RECENT_MS = 500.0
LOOP_BODY_MS = 1.5


def make_events(rng, total_ms):
    """움직임 사건열: (종류, 시작, 길이). handle=손에 들고 움직임(1~20초), tap=두드림, rest=정지."""
    evs, t = [], 2000.0
    while t < total_ms:
        r = rng.random()
        if r < 0.45:
            d = rng.uniform(1000, 20000)
            evs.append(('handle', t, d))
            t += d + rng.uniform(3000, 40000)
        else:
            evs.append(('tap', t, rng.uniform(20, 120)))
            t += rng.uniform(5000, 60000)
    return evs


def detections(rng, evs):
    out = []
    for kind, t0, d in evs:
        if kind == 'handle':
            t = t0
            while t < t0 + d:
                out.append(t)
                t += rng.uniform(25, 40)          # ISR 재활성 30ms 근처마다 판정
        else:
            n = rng.randint(1, 3)
            t = t0
            for _ in range(n):
                out.append(t)
                t += rng.uniform(25, 40)
    return out


def run(policy, seed, total_ms=6 * 3600 * 1000.0):
    rng = random.Random(seed)
    evs = make_events(rng, total_ms)
    dets = detections(rng, evs)
    # 상태
    sleeping = True               # 화면 OFF 로 시작
    last_act = -1e12
    last_vib = None
    last_vib_seen = None
    next_loop = 0.0
    pending_kick = False
    last_kick = -1e12
    loops = 0
    m = dict(off_while_moving=0, wake_lat=[], loops_handle=[], off_after_last=[])
    # 움직임 구간 인덱스(판정이 이어지는 동안 = 움직이는 중)
    moving_until = -1.0
    # 손에 든 구간의 회전 수 측정
    handle_windows = [(t0, t0 + d) for k, t0, d in evs if k == 'handle']
    loop_times = []
    di = 0
    pending_tap_wake = None       # 화면 OFF 중 첫 판정 시각(켜짐 지연 측정)
    last_motion_end = None
    t = 0.0
    while t < total_ms:
        nd = dets[di] if di < len(dets) else 1e18
        if nd <= next_loop:
            # 진동 판정(버튼 태스크)
            t = nd
            di += 1
            last_vib = t
            moving_until = t + 60.0
            if sleeping and pending_tap_wake is None:
                pending_tap_wake = t
            if policy == 'old':
                pending_kick = True
            else:
                if sleeping or (t - last_kick) >= 1000.0:
                    last_kick = t
                    pending_kick = True
            if pending_kick:
                next_loop = min(next_loop, t + 0.05)   # 통지 → 메인 루프가 곧 돈다
            continue
        # 메인 루프 한 회전
        t = next_loop
        pending_kick = False
        loops += 1
        loop_times.append(t)
        vib_active = last_vib is not None and (t - last_vib) < RECENT_MS
        vib_event = last_vib is not None and last_vib != last_vib_seen
        if vib_active or vib_event:
            last_act = t
            if sleeping:
                sleeping = False
                if pending_tap_wake is not None:
                    m['wake_lat'].append(t - pending_tap_wake)
                    pending_tap_wake = None
        last_vib_seen = last_vib
        holdoff = last_vib is not None and (t - last_vib) < HOLDOFF_MS
        if not (vib_active or holdoff) and (t - last_act) >= IDLE_OFF_MS and not sleeping:
            sleeping = True
            if t < moving_until:
                m['off_while_moving'] += 1
            if last_vib is not None:
                m['off_after_last'].append(t - last_vib)
        next_loop = t + LOOP_BODY_MS + (OFF_MS if sleeping else ON_MS)
    # 손에 든 구간의 메인 루프 회전/초
    import bisect
    for a, b in handle_windows:
        i = bisect.bisect_left(loop_times, a)
        j = bisect.bisect_left(loop_times, b)
        if b - a > 2000:
            m['loops_handle'].append((j - i) / ((b - a) / 1000.0))
    m['loops_total'] = loops / (total_ms / 1000.0)
    return m


def main():
    ok = True
    res = {p: run(p, seed=11) for p in ('old', 'new')}
    for p in ('old', 'new'):
        m = res[p]
        wl = m['wake_lat']
        print(f"  {p:3s} 움직이는데 화면 꺼짐 {m['off_while_moving']}회 | 두드림→화면 켜짐 중앙 {st.median(wl):.2f} / 최대 {max(wl):.2f}ms"
              f" | 손에 든 동안 메인 회전 {st.median(m['loops_handle']):.1f}/초(최대 {max(m['loops_handle']):.1f})"
              f" | 마지막 움직임 → 화면 OFF 중앙 {st.median(m['off_after_last'])/1000:.2f}초 | 전체 평균 {m['loops_total']:.2f}회/초")
    o, n = res['old'], res['new']
    if n['off_while_moving'] != 0:
        print('  ✗ 새: 움직이는 동안 화면이 꺼졌다'); ok = False
    if max(n['wake_lat']) > max(o['wake_lat']) + 0.5:
        print('  ✗ 새: 화면 OFF 에서 켜짐이 늦어졌다'); ok = False
    if st.median(n['loops_handle']) > 12.0:
        print('  ✗ 새: 손에 든 동안 메인 회전이 12/초를 넘는다'); ok = False
    if abs(st.median(n['off_after_last']) - st.median(o['off_after_last'])) > 150:
        print('  ✗ 새: 마지막 움직임 뒤 꺼지는 시간이 달라졌다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
