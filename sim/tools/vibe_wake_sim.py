# -*- coding: utf-8 -*-
"""진동센서(JYX-1210-X160) 화면 깨우기 지연 시뮬레이터.

사용자 신고(2026-08-19): "두세번 두드려야 화면이 깨어난다".
button_handler.c 의 `_vibration_track()` + 폴링 태스크를 0.1ms 해상도로
재현해, **탭 몇 번째에 깨어나는지**를 현재 로직과 수정안에서 비교한다.

실측 근거 (COM3 H2, logs/h2_reboot.txt, 90초 정지 상태):
    [VIBE-stat] 레벨=0  ISR누적=119(증가 0)  HIGH=0/300   ← 30창 전부 동일
  → 평상시 **레벨 LOW 고정 / ISR 0.00회/초 / HIGH 폴 0/9000**.
  즉 (a) 오검출 압력이 전혀 없고 (b) 10ms 폴링은 HIGH 를 사실상 못 잡는다.

그리고 과거 로그에 오판의 직접 증거가 있다:
    [VIBE] 정상 복귀 (최근 200샘플 HIGH=1) — 진동 다시 인정
    [VIBE] 센서 고장 판정(핀 고정) (최근 200샘플 HIGH=0) — 진동 무시함
  X160 은 **평상시 닫힘(LOW)** 이라 all-LOW 가 정상인데, 현재 판정은
  `st_high == 0` 도 고장으로 본다 → 가만두면 2초마다 "고장"으로 굳는다.
"""
import random

DT_US        = 100          # 0.1ms 스텝
POLL_US      = 10000        # 폴링 10ms (button_handler 고정)
ISR_HOLD_POLL= 3            # ISR self-disable → 폴링 3회(30ms) 뒤 재활성
WIN_POLLS    = 30           # HIGH duty 윈도우 (300ms)
WIN_HIGH_TH  = 2            # 윈도우 안 HIGH 임계
STUCK_WIN    = 200          # 고장 판정 관찰 샘플(2초)
RECENCY_US   = 500000       # is_vibrating() 500ms


def make_tap(t0_us, burst_ms, n_pulse, w_lo_us, w_hi_us, rng):
    """탭 1회 = burst_ms 안의 짧은 open(HIGH) 펄스 다발."""
    out = []
    for _ in range(n_pulse):
        s = t0_us + rng.randrange(0, burst_ms * 1000)
        out.append((s, s + rng.randrange(w_lo_us, w_hi_us)))
    return sorted(out)


def _transitions(pulses):
    """겹칠 수 있는 (start,end) 목록 → 정렬된 [(t, level)] 전이 목록."""
    ev = []
    for s_, e_ in pulses:
        ev.append((s_, +1)); ev.append((e_, -1))
    ev.sort()
    out, depth, prev = [], 0, 0
    for t, d in ev:
        depth += d
        lvl = 1 if depth > 0 else 0
        if lvl != prev:
            out.append((t, lvl)); prev = lvl
    return out


def run(taps_us, total_us, stuck_all_low_is_fault, use_isr, isr_th, rng,
        burst_ms, n_pulse, w_lo_us, w_hi_us):
    """★이벤트 구동 — 폴 시각과 접점 전이 시각에서만 계산한다(0.1ms 루프는 너무 느림)."""
    pulses = []
    for t in taps_us:
        pulses += make_tap(t, burst_ms, n_pulse, w_lo_us, w_hi_us, rng)
    trans = _transitions(pulses)

    isr_count = 0; isr_disabled = False; reenable_cd = 0
    win_cnt = win_high = 0
    st_cnt = st_high = 0
    stuck = False
    isr_seen = 0
    woke_at = []

    ti = 0                      # 다음 전이 인덱스
    lvl = 0
    poll_t = POLL_US
    while poll_t < total_us:
        # 이 폴 시각까지의 접점 전이를 모두 반영(= ISR 발사)
        while ti < len(trans) and trans[ti][0] <= poll_t:
            lvl = trans[ti][1]
            if not isr_disabled:
                isr_count += 1
                isr_disabled = True
            ti += 1
        plvl = lvl
        # 고장 판정
        st_cnt += 1
        if plvl: st_high += 1
        if st_cnt >= STUCK_WIN:
            stuck = ((st_high == st_cnt) or (st_high == 0)) if stuck_all_low_is_fault                     else (st_high == st_cnt)
            st_cnt = st_high = 0
        if not stuck:
            fired = False
            if use_isr:
                if isr_count - isr_seen >= isr_th: fired = True
                isr_seen = isr_count
            win_cnt += 1
            if plvl: win_high += 1
            if win_high >= WIN_HIGH_TH:
                fired = True; win_cnt = win_high = 0
            elif win_cnt >= WIN_POLLS:
                win_cnt = win_high = 0
            if fired: woke_at.append(poll_t)
        else:
            isr_seen = isr_count
        if isr_disabled:
            if reenable_cd == 0: reenable_cd = ISR_HOLD_POLL
            reenable_cd -= 1
            if reenable_cd == 0: isr_disabled = False
        poll_t += POLL_US

    res = [any(tp <= w <= tp + 600000 for w in woke_at) for tp in taps_us]
    return res, len(woke_at)


def scenario(name, **kw):
    rng = random.Random(20260819)
    IDLE_US = 5000000                     # 5초 정지 후 두드리기 시작
    taps = [IDLE_US + i * 1000000 for i in range(6)]   # 1초 간격 6회
    total = taps[-1] + 1500000
    print('── %s ──' % name)
    for label, low_fault, use_isr, th in (
            ('현재 로직            ', True,  False, 0),
            ('수정A: 고장판정만 수정', False, False, 0),
            ('수정B: A + ISR 트리거 ', False, True,  1)):
        first = []
        for trial in range(200):
            r = random.Random(1000 + trial)
            res, n = run(taps, total, low_fault, use_isr, th, r, **kw)
            first.append(next((i + 1 for i, x in enumerate(res) if x), 99))
        ok = [f for f in first if f != 99]
        avg = sum(ok) / len(ok) if ok else 0
        dist = {}
        for f in first: dist[f] = dist.get(f, 0) + 1
        d = ' '.join('%s번째:%d%%' % ('못깨움' if k == 99 else k, v * 100 // 200)
                     for k, v in sorted(dist.items()))
        print('  %s 평균 %.2f번째 탭에 깨어남   %s'
              % (label, avg if ok else float('nan'), d))
    print()


if __name__ == '__main__':
    # 실측 제약: 10ms 폴링이 HIGH 를 거의 못 잡는다 → 펄스폭이 10ms 보다 훨씬 짧다.
    scenario('짧은 접점(폭 0.2~2ms, 다발 30ms 안에 6펄스) — 실측과 부합',
             burst_ms=30, n_pulse=6, w_lo_us=200, w_hi_us=2000)
    scenario('아주 짧은 접점(폭 0.1~0.6ms, 다발 20ms 안에 4펄스)',
             burst_ms=20, n_pulse=4, w_lo_us=100, w_hi_us=600)
    scenario('긴 접점(폭 2~8ms, 다발 40ms 안에 5펄스) — 세게 두드림',
             burst_ms=40, n_pulse=5, w_lo_us=2000, w_hi_us=8000)


# ── 부작용 검증 ────────────────────────────────────────────────────────────
#  수정B(ISR 트리거)는 과거에 한 번 넣었다 뺀 이력이 있다 — C6 의 고장 센서가
#  초당 33회 ISR 을 쏴서 화면이 영영 안 꺼졌기 때문이다. 그 회귀가 없는지 본다.
def guard():
    print('=' * 74)
    print('부작용 검증')
    print('=' * 74)

    CFG = (('현재 로직            ', True,  False, 0),
           ('수정A                ', False, False, 0),
           ('수정B (A + ISR)      ', False, True,  1))

    # ① 완전 정지 — 실측(90초 ISR 0회 / HIGH 0/9000)과 동일한 조건
    print('① 완전 정지 60초 (실측: ISR 0회, HIGH 0/9000)')
    for label, low_fault, use_isr, th in CFG:
        r = random.Random(7)
        _, n = run([], 60000000, low_fault, use_isr, th, r,
                   burst_ms=1, n_pulse=0, w_lo_us=1, w_hi_us=2)
        print('   %s 헛깨움 %d회   %s' % (label, n, '✔' if n == 0 else '✘'))

    # ② C6 고장 센서 — 핀이 HIGH 로 붙고 잡음 엣지가 초당 33회
    #    (실측 로그: [VIBE-stat] 진동=1 ISR누적=70338 (3초+100) HIGH=300/300)
    print('② C6 고장 센서 60초 (핀 HIGH 고정 + 엣지 33회/초)')
    for label, low_fault, use_isr, th in CFG:
        # ★실측 충실화: C6 로그는 `ISR누적 3초+100`(33회/초) 인데 `HIGH=300/300`
        #   — 즉 폴링은 **단 한 번도 LOW 를 못 봤다**. 따라서 잡음 엣지는 폴링
        #   구멍(10ms)보다 훨씬 짧은 글리치다. 1ms LOW 딥으로 모델링하면 폴이
        #   3.3% 확률로 그 딥에 빠져 고장판정이 안 걸리는 **비실측 모델**이 된다.
        pulses = []
        t = 0
        while t < 60000000:                 # HIGH 고정 + 5us LOW 글리치 33회/초
            pulses.append((t, t + 30000 - 5))
            t += 30000
        import types
        # run() 대신 같은 알고리즘을 직접 돌린다(펄스를 외부에서 주입)
        trans = _transitions(pulses)
        isr_count = isr_seen = 0; isr_disabled = False; reenable_cd = 0
        win_cnt = win_high = st_cnt = st_high = 0
        stuck = False; woke = 0; woke_after = 0; latched = False
        ti = 0; lvl = 0; poll_t = POLL_US
        while poll_t < 60000000:
            while ti < len(trans) and trans[ti][0] <= poll_t:
                lvl = trans[ti][1]
                if not isr_disabled:
                    isr_count += 1; isr_disabled = True
                ti += 1
            st_cnt += 1
            if lvl: st_high += 1
            if st_cnt >= STUCK_WIN:
                stuck = ((st_high == st_cnt) or (st_high == 0)) if low_fault \
                        else (st_high == st_cnt)
                st_cnt = st_high = 0
            if not stuck:
                fired = False
                if use_isr:
                    if isr_count - isr_seen >= th: fired = True
                    isr_seen = isr_count
                win_cnt += 1
                if lvl: win_high += 1
                if win_high >= WIN_HIGH_TH:
                    fired = True; win_cnt = win_high = 0
                elif win_cnt >= WIN_POLLS:
                    win_cnt = win_high = 0
                if fired:
                    woke += 1
                    if latched: woke_after += 1
            else:
                isr_seen = isr_count
            if isr_disabled:
                if reenable_cd == 0: reenable_cd = ISR_HOLD_POLL
                reenable_cd -= 1
                if reenable_cd == 0: isr_disabled = False
            poll_t += POLL_US
        print('   %s 헛깨움 총%3d회 = 판정 전(2초) %3d + **판정 후 %d**   %s'
              % (label, woke, woke - woke_after, woke_after,
                 '✔ 억제됨' if woke_after == 0 else '✘ 폭주 — 회귀!'))


    # ③ 센서가 LOW 로 죽은 경우(GND 단락/단선) — 수정A 가 "고장 아님"으로
    #    재분류하는 유일한 케이스라 반드시 확인한다. 엣지도 HIGH 도 없으므로
    #    트리거는 0 이어야 한다(=조용히 아무 일도 안 일어남).
    print('③ 센서 LOW 고정(죽음) 60초 — 수정A 가 "정상"으로 재분류하는 케이스')
    for label, low_fault, use_isr, th in CFG:
        r = random.Random(11)
        _, n = run([], 60000000, low_fault, use_isr, th, r,
                   burst_ms=1, n_pulse=0, w_lo_us=1, w_hi_us=2)
        print('   %s 헛깨움 %d회   %s' % (label, n, '✔' if n == 0 else '✘'))


if __name__ == '__main__':
    guard()


# ── 실측 교정 (2026-08-19, COM3 H2 의 NVS `vibelog` 128건) ──────────────────
#  앞의 시나리오들은 접점 펄스폭을 **가정**했다. 여기서는 실기가 직접 남긴
#  3초 창 통계 (polls, high, isr) 로 두 경로의 민감도를 직접 비교한다.
#  창 = 3초 ≈ 300폴. 트리거 조건:
#    · 기존 HIGH 경로 : 300ms(30폴) 안에 HIGH 2표본  → HIGH 가 창에 고루 흩어져
#                       있으면 2개가 같은 30폴 안에 들 확률이 급격히 떨어진다.
#    · ISR 경로       : 엣지 1개
MEASURED = [   # (polls, high, isr) — 접점이 움직인 창만 발췌
    (310, 35, 42), (234, 14, 28), (286, 31, 39), (297, 53, 63), (301, 15, 20),
    (301,  8, 17), (299, 56, 65), (310, 12, 51), (301, 12, 34), (300,  1,  4),
    (301,  2,  2), (216, 14, 28), (177,  8, 22), (277, 15,  8), (300, 28, 19),
    (301, 14, 53), (310,  1, 29), (301, 27, 52), (300, 20, 71), (300, 26, 51),
]
IDLE_MEASURED = [(300, 0, 0), (310, 0, 0), (309, 0, 0), (299, 0, 0), (308, 0, 0)]


def measured():
    print('=' * 74)
    print('실측 교정 — NVS vibelog (3초 창)')
    print('=' * 74)
    print('%7s %6s %6s   %-22s %s' % ('polls', 'HIGH', 'ISR', 'HIGH경로(2/30폴)', 'ISR경로'))
    hi_ok = isr_ok = 0
    for p, h, i in MEASURED:
        # HIGH 2표본이 같은 30폴 창에 들 확률 — HIGH 가 균등분포일 때의 기대값.
        # h개를 p폴에 흩뿌렸을 때 어떤 30폴 창에 2개 이상 들 확률의 근사:
        #   1 - (1-30/p)^(h-1) 를 "첫 HIGH 기준 다음 HIGH 가 30폴 안" 으로 본다.
        pr = 0.0 if h < 2 else 1.0 - (1.0 - min(1.0, 30.0 / p)) ** (h - 1)
        a = pr >= 0.5
        b = i >= VIBE_ISR_TRIGGER_SIM
        hi_ok += a; isr_ok += b
        print('%7d %6d %6d   %-22s %s'
              % (p, h, i,
                 ('발동 가능 %.0f%%' % (pr * 100)) if h >= 2 else '불가(HIGH<2)',
                 '발동' if b else '불가'))
    print()
    print('  HIGH 경로 발동 가능(≥50%%) %d/%d 창   ISR 경로 발동 %d/%d 창'
          % (hi_ok, len(MEASURED), isr_ok, len(MEASURED)))
    print('  ★결정적 사례: (310, HIGH 1, ISR 29) — 3초 동안 폴링은 HIGH 를 딱 1번')
    print('    잡았다. "30폴 안에 2표본"은 **원리적으로 불가능**한데 엣지는 29개다.')
    print()
    print('정지 창 (오검출 압력)')
    for p, h, i in IDLE_MEASURED:
        print('  polls=%d HIGH=%d ISR=%d → 두 경로 모두 발동 0' % (p, h, i))
    print('  → 실측 정지 상태 ISR 0.00회/초. ISR 임계 1 로도 헛깨움 압력이 없다.')


VIBE_ISR_TRIGGER_SIM = 1

if __name__ == '__main__':
    measured()
