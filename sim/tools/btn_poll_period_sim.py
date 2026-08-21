# -*- coding: utf-8 -*-
"""버튼 HP 폴링 주기를 늘렸을 때 **짧은 누름을 놓칠 확률**.

배경(2026-08-20 실측): 깨어남 88.07회/초 중 btn_handler 가 46.81회/초로 **53%**.
유휴 폴 주기를 25ms → 100ms 로 늘리면 전체가 88 → 53회/초(-40%)가 된다.

근거로 든 것: LP 코어가 2ms 마다 PCF 를 읽고 변화 시 HP 를 깨운다
(`ulp_lp_core_wakeup_main_processor`). 그런데 **주의**: 그 호출은 HP **CPU** 를
light sleep 에서 깨울 뿐, `vTaskDelay(100ms)` 로 블록된 btn_handler **태스크를
조기 해제하지는 않는다**. 즉 태스크는 여전히 100ms 를 다 잔다.
→ 누름이 폴 사이에 통째로 들어가면 HP 는 그 누름을 **못 본다**.
   (LP 는 `pcf_state = rx` 로 현재 상태만 쓰고 눌림을 래치하지 않는다.)

코드에도 경고가 남아 있다: "버튼은 눌림이 100ms 이상이라 25ms 주기로 충분하다
(**150ms 는 예전에 실패**)".

여기서는 누름 길이 분포별로 놓칠 확률을 계산해 판단 근거를 만든다.
폴 시각이 누름과 무관(균등)하므로, 길이 D 인 누름을 주기 T 로 놓칠 확률은
    P_miss = max(0, 1 - D/T)
"""


def miss(d_ms, t_ms):
    return max(0.0, 1.0 - float(d_ms) / t_ms)


if __name__ == '__main__':
    periods = (25, 50, 100, 150)
    # 사람의 버튼 누름 길이 — 아주 빠른 탭부터 보통 누름까지
    durations = (40, 60, 80, 100, 120, 150, 200, 300)
    print('놓칠 확률 (P_miss = max(0, 1 - 누름길이/폴주기))')
    print('%10s' % '누름길이' + ''.join('%12s' % ('폴 %dms' % t) for t in periods))
    for d in durations:
        row = ''.join('%11.0f%%' % (100 * miss(d, t)) for t in periods)
        print('%9dms' % d + row)
    print()
    print('깨어남 절감 (btn 46.81회/초 = 전체 88.07 의 53%)')
    for t in periods:
        b = 1000.0 / t
        tot = 88.07 - 46.81 + b
        print('  폴 %3dms → btn %5.1f회/초, 전체 %5.1f회/초 (%+.0f%%), '
              '깨어있는 시간 %.1f%%'
              % (t, b, tot, 100.0 * (tot - 88.07) / 88.07, tot * 1.52 / 10.0))
    print()
    print('판정: 100ms 는 **80ms 이하의 짧은 탭을 20% 이상 놓친다**.')
    print('      보통 누름(120ms+)은 안전하다. 실사용에서 탭이 짧으면 체감된다.')
    print('      → 놓침이 보이면 두 갈래:')
    print('        (a) 50ms 로 후퇴 (-26% 절감, 60ms 탭까지 안전)')
    print('        (b) LP 가 **눌림을 래치**하게 고친다(pressed 마스크 OR 누적).')
    print('            그러면 폴 주기와 무관하게 누락 0 — 100ms 를 그대로 쓸 수 있다.')
