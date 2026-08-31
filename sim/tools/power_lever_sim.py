# -*- coding: utf-8 -*-
"""C6 절전 지렛대 두 개의 **가능한 상한**과 **안전 한계**를 미리 계산한다.

배경(2026-08-27 실측):
    C6  4187→3549mV  94.5h  =  6.68 mA   잠 96.3%  깨어남 25.3/초  회당 깨어있음 1.47ms
    H2  4170→3963mV  51.9h  =  2.24 mA   잠 96.1%  깨어남 33.1/초  회당 깨어있음 1.16ms
    → C6 가 **3.0배**. 잠 비율은 같으니 duty 가 아니라 **전류 자체**의 차이다.

측정 못 하는 것: 수면 전류와 깨어있는 전류를 분리할 수 없다(미지수 2, 식 1).
배터리 선을 못 끊어 직렬 전류계도 불가하다.
→ 그래서 **민감도 분석**을 한다. 수면 전류를 여러 값으로 가정했을 때 각 지렛대가
   최대 얼마나 줄일 수 있는지 상한을 보면, 재보기 전에 기대치를 정할 수 있다.
"""

TOTAL_MA   = 6.68     # C6 실측 평균
DUTY_AWAKE = 0.037    # 3.7% (잠 96.3%)
LP_POLL_US = 2000     # 현재 LP 폴 주기


def decompose(i_sleep):
    """I_total = duty*I_awake + (1-duty)*I_sleep  →  I_awake"""
    return (TOTAL_MA - (1 - DUTY_AWAKE) * i_sleep) / DUTY_AWAKE


def lever_cpu(i_sleep, ratio=0.5):
    """CPU 클럭을 ratio 로 낮췄을 때. **깨어있는 구간에만** 작용한다.
       보수적으로: 전력 ∝ 주파수, 일은 그만큼 느려져 깨어있는 시간이 1/ratio 배.
       → 에너지는 그대로다(최선의 경우에만 이득). 낙관/비관 두 경우를 본다."""
    ia = decompose(i_sleep)
    opt = TOTAL_MA - DUTY_AWAKE * ia * (1 - ratio)      # 낙관: 시간 그대로, 전력만 감소
    pes = TOTAL_MA                                      # 비관: 시간이 늘어 완전 상쇄
    return opt, pes


def lever_lp(i_sleep, new_us=10000, lp_ma_guess=0.3):
    """LP 코어 폴링을 늦췄을 때. LP 는 **수면 중에도** 돈다 = 수면 전류에 포함.
       LP 소비를 lp_ma_guess 로 가정하고 트래픽 비율만큼 준다고 본다."""
    saved = lp_ma_guess * (1 - LP_POLL_US / float(new_us))
    return TOTAL_MA - saved


if __name__ == '__main__':
    print('=== 지렛대 상한 (수면 전류를 모르므로 민감도 분석) ===')
    print('%10s %12s %14s %14s' % ('가정 I_sleep', '역산 I_awake', 'CPU 절반(낙관)', 'CPU 절반(비관)'))
    for isl in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
        ia = decompose(isl)
        if ia < 0:
            print('%9.1fmA  %12s  (수면만으로 총량 초과 — 불가능)' % (isl, '-'))
            continue
        o, p = lever_cpu(isl)
        print('%9.1fmA %11.1fmA %13.2fmA %13.2fmA  (%.0f%% ~ 0%% 절감)'
              % (isl, ia, o, p, 100 * (TOTAL_MA - o) / TOTAL_MA))
    print()
    print('★읽는 법: 수면 전류가 클수록 깨어있는 구간의 비중이 작아 **CPU 로 얻을 게 없다.**')
    print('  H2 가 2.24mA 인데 C6 가 6.68mA 라는 것은, 차이가 주로 **수면 구간**에')
    print('  있다는 뜻이다(잠 비율이 같으므로). 즉 수면 전류 가정은 높은 쪽이 현실적이고,')
    print('  그러면 CPU 클럭의 상한은 매우 작다.')
    print()
    print('=== LP 폴 주기 (수면 중에도 도는 부하) ===')
    print('%10s %14s %12s' % ('새 주기', 'I2C 트래픽', '총 전류(가정)'))
    for us in (2000, 4000, 5000, 10000):
        print('%9dus %11.0f회/초 %11.2fmA' % (us, 1e6 / us / 1.0,
              lever_lp(3.0, us) if us > 2000 else TOTAL_MA))
    print('  ※LP 소비를 0.3mA 로 가정한 값 — **실측된 적 없다.** 상한 감각용.')


# ── LP 폴 주기의 **안전 한계** — 로터리 디텐트를 놓치지 않는가 ──────────────
#  LP 프로그램(lp_core/pcf_poll.c)은 POLL_US 마다 PCF 를 읽어 A/B 쿼드러처를
#  디코딩한다. 주기를 늘리면 **전이를 건너뛰어** 방향을 틀리거나 디텐트를 잃는다.
#  코드 주석: "2ms — 디텐트(수 ms) 를 놓치지 않는 최소선".
#  버튼은 press_latch 가 OR 로 모으므로 주기와 무관하지만, **로터리는 래치가 없다**
#  (LP 가 그때그때 디코딩해 rot_delta 에 누적한다) → 여기가 진짜 제약이다.
KQUAD = {  # (prev_ab<<2)|ab  →  증분
    (0b00 << 2) | 0b01: +1, (0b01 << 2) | 0b11: +1,
    (0b11 << 2) | 0b10: +1, (0b10 << 2) | 0b00: +1,
    (0b00 << 2) | 0b10: -1, (0b10 << 2) | 0b11: -1,
    (0b11 << 2) | 0b01: -1, (0b01 << 2) | 0b00: -1,
}


def rotary_test(poll_us, detent_ms, n_detent=20):
    """EC05 반스텝: 한 디텐트가 A/B 전이 2회. 전이 간격 = detent_ms/2.
       LP 가 poll_us 마다 샘플링할 때 누적값이 맞는지 본다."""
    step_us = detent_ms * 1000 // 2
    seq = [0b11]                                  # 디텐트 기본 위치
    order = [0b11, 0b10, 0b00, 0b01]              # CW 한 바퀴
    t, states, k = 0, [], 0
    for _ in range(n_detent * 2):
        k = (k + 1) % 4
        t += step_us
        states.append((t, order[k]))
    # 샘플링
    prev_ab, accum, delta, si = 0b11, 0, 0, 0
    tp = 0
    cur = 0b11
    while tp <= states[-1][0]:
        while si < len(states) and states[si][0] <= tp:
            cur = states[si][1]; si += 1
        if cur != prev_ab:
            accum += KQUAD.get((prev_ab << 2) | cur, 0)
            prev_ab = cur
            if accum >= 2:   delta += 1; accum = 0
            elif accum <= -2: delta -= 1; accum = 0
        tp += poll_us
    return delta


if __name__ == '__main__':
    print()
    print('=== LP 폴 주기 안전 한계 (로터리 20디텐트 CW, 정답 +20) ===')
    print('%9s' % '폴주기' + ''.join('%12s' % ('디텐트 %dms' % d) for d in (10, 20, 40, 80)))
    for us in (2000, 4000, 5000, 8000, 10000):
        row = ''
        for d in (10, 20, 40, 80):
            got = rotary_test(us, d)
            row += '%11s%s' % (('%+d' % got), '' if got == 20 else '★')
        print('%8dus' % us + row)
    print('  ★ = 정답(+20)과 다름 = 디텐트 손실/오방향')
    print()
    print('판정: 빠른 회전(디텐트 10~20ms)에서 주기를 늘리면 바로 깨진다.')
    print('      버튼은 press_latch 가 지켜주지만 **로터리는 래치가 없다.**')
