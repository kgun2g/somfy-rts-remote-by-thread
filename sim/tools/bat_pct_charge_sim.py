# -*- coding: utf-8 -*-
"""충전 연결 시 배터리 표시%가 튀는 문제와 그 수정(상승률 제한)을 검증한다.

사용자 실측 (2026-08-16, xiao-c6 배터리 단독):
    84%  →  USB 연결  →  즉시 100%  →  1분 내 분리  →  96%

원인 (main/somfy_app.c BAT_FULL_MV 주석에 이미 기록돼 있던 것):
  충전 전류가 셀 내부저항을 지나며 **단자전압을 100mV 남짓 들뜨게** 한다.
  그런데 코드는 그 전압을 그대로 %로 환산하고, USB 중에는 방전용 하한
  (s_pct_floor)마저 풀어버려 **상한이 전혀 없다** → 표시가 잔량이 아니라
  단자전압을 따라간다.

수정 (하한의 대칭):
  충전 중에는 **천장**을 둔다. USB 연결 순간의 표시값에서 출발해 실제 충전
  속도(700mAh 를 0.5C 로 ≈ 0.85%p/분)보다 빨리 오르지 못하게 하고,
  표시 = min(전압환산%, 천장).
"""
import sys

BAT_FULL_MV = 4108
TOP_BASE = 3980
RISE_MPCT_PER_MIN = 850     # 0.85 %p/분


def top(v):
    return TOP_BASE + (v - TOP_BASE) * (BAT_FULL_MV - TOP_BASE) // (4200 - TOP_BASE)


V = [3200, 3450, 3580, 3680, 3750, 3850, TOP_BASE, top(4080), top(4150), BAT_FULL_MV]
P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]


def mv_to_pct(mv):
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) // (V[i]-V[i-1])
    return 100


def pct_to_mv(p):
    """표시%에 해당하는 전압(검증용 역함수)."""
    for i in range(1, len(P)):
        if p <= P[i]:
            return V[i-1] + (V[i]-V[i-1]) * (p - P[i-1]) // (P[i]-P[i-1])
    return V[-1]


def run(fix, minutes_on_usb, bump_mv=108, start_pct=84):
    """USB 연결 → minutes_on_usb 분 → 분리. 표시% 궤적을 돌려준다."""
    v_cell = pct_to_mv(start_pct)          # 셀 실제(개방) 전압
    shown = start_pct
    ceil_mpct = -1
    traj = [('배터리', 0, v_cell, shown)]

    # ── USB 연결 (단자전압이 bump 만큼 들뜬다) ──
    for m in range(1, minutes_on_usb + 1):
        v_cell += 4                        # 실제 충전으로 셀도 조금 오른다(≈0.85%p/분)
        v_term = v_cell + bump_mv
        pct = mv_to_pct(v_term)
        if fix:
            if ceil_mpct < 0:
                ceil_mpct = shown * 1000
            else:
                ceil_mpct += RISE_MPCT_PER_MIN
            if ceil_mpct > 100000:
                ceil_mpct = 100000
            pct = min(pct, ceil_mpct // 1000)
        shown = pct
        traj.append(('USB', m, v_term, shown))

    # ── 분리 (단자전압이 셀 전압으로 돌아온다) ──
    pct = mv_to_pct(v_cell)
    if fix:
        ceil_mpct = -1                     # 방전 복귀 → 천장 해제
    traj.append(('분리', minutes_on_usb, v_cell, pct))
    return traj


def show(tag, fix, mins):
    traj = run(fix, mins)
    s = ' → '.join('%s%d%%' % ('' if w == '배터리' else '', p) for w, _, _, p in traj)
    jump = max(traj[i+1][3] - traj[i][3] for i in range(len(traj)-1))
    print('  %-8s %s   (한 스텝 최대 상승 %+d%%p)' % (tag, s, jump))
    return traj, jump


if __name__ == '__main__':
    print('전압-퍼센트 앵커: ' + str(list(zip(V, P))))
    print('84%% 에 해당하는 셀 전압 = %dmV, 충전 시 단자전압 = %dmV → %d%%'
          % (pct_to_mv(84), pct_to_mv(84) + 108, mv_to_pct(pct_to_mv(84) + 108)))
    print()
    print('■ USB 1분 연결 후 분리 (사용자가 겪은 시나리오)')
    _, j_old = show('옛 코드', False, 1)
    _, j_new = show('수정본', True, 1)
    print()
    print('■ USB 30분 연결 후 분리 (정상 충전)')
    show('옛 코드', False, 30)
    t_new, _ = show('수정본', True, 30)
    print()
    ok = (j_old >= 10) and (j_new <= 1)
    print('=== 판정 ===')
    print('  옛 코드 한 스텝 상승 %+d%%p  → %s' % (j_old, '튐 재현됨' if j_old >= 10 else '재현 실패'))
    print('  수정본 한 스텝 상승 %+d%%p  → %s' % (j_new, '억제됨' if j_new <= 1 else '억제 실패'))
    print('\n%s' % ('통과 — 버그 재현 + 수정 확인' if ok else '실패 — 모델 재검토 필요'))
    sys.exit(0 if ok else 1)
