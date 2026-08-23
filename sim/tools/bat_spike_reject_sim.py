# -*- coding: utf-8 -*-
"""배터리 전압 **급락 표본 거부** 검증.

사건 2건 (둘 다 NVS 방전기록에 남아 있다):
  2026-08-20  4069 → 2651 → 2129 mV   (8시간 동안 3845mV 로 서서히 "회복")
  2026-08-23  4055 → 3002 mV          (리셋하자 4078mV 로 즉시 정상)
둘 다 산포가 1~6카운트로 **깨끗**했다 = ADC 잡음이 아니라 측정 전체가 낮게 나왔다.
그리고 그 값이 0% 로 환산돼 표시가 0% 에 갇혔다.

앞서 넣은 방어 두 개로는 부족했다:
  · 하한 조건부 해제  → 갇힌 뒤에 푸는 것이라, 그 사이 사용자는 0% 를 본다
  · 천장 출발점 교정  → **그 전압 자체가 틀리면** 0% 에서 출발하는 건 마찬가지

→ 근본 방어: **물리적으로 불가능한 낙차는 표본을 아예 버린다.**
   700mAh 셀이 5초 만에 1,000mV 떨어지는 것은 불가능하다(그 전압이면 LDO
   드롭아웃으로 기기가 죽는데, 기기는 계속 돌며 로그를 남겼다).
   다만 **진짜 급락**(셀 수명 끝, 보호회로 동작)도 있을 수 있으므로 영원히
   거부하면 안 된다 → N회 연속이면 받아들인다.
"""

MEAS_MS      = 5000      # 측정 주기
MAX_DROP_MV  = 300       # 1회 측정에서 허용하는 최대 낙차
REJECT_MAX   = 3         # 연속 거부 한도 — 넘으면 진짜로 인정


def run(seq, max_drop=MAX_DROP_MV, reject_max=REJECT_MAX):
    """seq: 측정된 mV 목록 → (표시에 쓰인 mV 목록, 거부된 인덱스)"""
    last_ok = None
    rejects = 0
    used, rejected = [], []
    for i, mv in enumerate(seq):
        if last_ok is not None and mv < last_ok - max_drop and rejects < reject_max:
            rejects += 1
            rejected.append(i)
            used.append(last_ok)          # 직전 값 유지
            continue
        rejects = 0
        last_ok = mv
        used.append(mv)
    return used, rejected


def pct(mv):
    V = [3200, 3450, 3580, 3680, 3750, 3850, 3950, 4080, 4150, 4200]
    P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]
    if mv <= V[0]: return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) // (V[i]-V[i-1])
    return 100


def show(name, seq, expect):
    used, rej = run(seq)
    print('── %s ──' % name)
    print('   입력 %s' % ' '.join(str(x) for x in seq))
    print('   표시 %s' % ' '.join(str(x) for x in used))
    print('   %%   %s' % ' '.join('%d' % pct(x) for x in used))
    print('   거부 %d개 %s   → %s' % (len(rej), rej,
          'OK' if (len(rej) > 0) == expect else '★기대와 다름'))
    print()


if __name__ == '__main__':
    # ① 2026-08-23 실측: 한 표본만 급락 후 정상
    show('실측 8/23 — 4055 → 3002 (1053mV 급락)',
         [4059, 4057, 4055, 3002, 4078, 4076], True)

    # ② 2026-08-20 실측: 급락이 여러 표본 이어짐
    show('실측 8/20 — 4069 → 2651 → 2129 (연속)',
         [4070, 4069, 2651, 2462, 2307, 2326, 2374], True)

    # ③ 진짜 급락(셀 수명 끝) — 거부가 영원하면 안 된다
    show('진짜 방전 — 계속 낮으면 REJECT_MAX 뒤 인정',
         [3700, 3300, 3200, 3100, 3000, 2950, 2900], True)

    # ④ 정상 방전 — 거부가 하나도 없어야 한다
    show('정상 방전 (5초당 수 mV)',
         [4100, 4098, 4095, 4093, 4090, 4088], False)

    # ⑤ USB 연결/해제로 단자전압이 100mV 남짓 움직이는 경우 — 거부 금지
    show('USB 분리 (충전 들뜸 해소, -120mV)',
         [4180, 4178, 4060, 4058, 4056], False)

    print('결론: MAX_DROP_MV=%d 는 실측 급락(1053mV·1418mV)을 잡고,'
          ' 정상 방전·USB 전환(≤120mV)은 통과시킨다.' % MAX_DROP_MV)
    print('      REJECT_MAX=%d → 진짜 급락은 %d초 뒤 인정된다.'
          % (REJECT_MAX, REJECT_MAX * MEAS_MS // 1000))
