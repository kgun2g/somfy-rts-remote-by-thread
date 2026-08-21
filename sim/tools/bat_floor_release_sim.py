# -*- coding: utf-8 -*-
"""배터리 표시 % — 하한 잠김 / 충전 천장 출발점 수정 검증.

신고(2026-08-20): COM7 C6 장시간 방치 후 **잔량 0%**. USB 를 다시 꽂아도 0% 근처.

NVS 방전기록(logs/nvs_c6_bat0.bin, 세션 #2, 300건/13.5시간)이 원인을 보여준다:
    #0~39    4085 → 4068 mV   95%
    #40~59   4069 → 2651 mV    0%   ← 급락
    #60~99   2129 mV 바닥       0%
    #100~299 2656 → 3845 mV    0%   ← 8시간에 걸쳐 회복해도 0% 고정

사용자 확인: **그 동안 USB 를 꽂은 적이 없다.** 충전 없이 전압이 오르는 것은
물리적으로 불가능하므로 **낮은 값들이 측정 오류**였다. (기기가 계속 돌았다는 것도
같은 결론을 가리킨다 — 셀이 2.1V 면 LDO 드롭아웃으로 죽는다. ADC 산포는 전 구간
2~3카운트로 깨끗했으니 잡음도 아니다. 기전은 아직 미확정 → 별도 진단 필요.)

두 결함:
  (A) `s_pct_floor` 가 **단조 비증가**라 한 번 0% 가 되면 세션 내내 못 풀린다.
      원래 의도는 "31mV 두 무리" 때문에 표시가 야금야금 오르는 것을 막는 것인데,
      **센서가 크게 거짓말한 경우**에 대한 탈출구가 없다.
  (B) 충전 천장이 `직전 표시값`(=잘못된 0%)에서 출발해 0.85%/분으로 기어오른다.
      실측: USB 연결 316초 뒤 4%, 376초 뒤 5%. 참값(~75%)까지 1.5시간.

수정안:
  (A) 하한을 건 시점의 전압보다 **FLOOR_RELEASE_MV 이상 높은 값이
      FLOOR_RELEASE_N 표본 연속**이면 하한을 푼다. 잡음 무리(31mV)의 3배 이상만
      인정하므로 원래 막으려던 야금야금 상승은 그대로 막힌다.
  (B) 천장을 **USB 연결 순간의 전압환산 %** 로 시작한다. 그 순간 단자전압은 아직
      충전으로 들뜨지 않았으므로(들뜸은 충전이 시작된 뒤 생긴다) 물리적으로 옳고,
      원래 막으려던 "84% → 즉시 100%" 점프도 그대로 막힌다.
"""
import re
import subprocess
import sys

FLOOR_RELEASE_MV = 100    # 잡음 무리 31mV 의 3배 이상 — 노이즈로는 못 넘는다
FLOOR_RELEASE_N  = 5      # 25초(5초 주기) 연속 유지 — 단발 튐으로는 못 푼다
CHG_RISE_MPCT_PER_MIN = 850


def mv_to_pct(mv):
    V = [3200, 3450, 3580, 3680, 3750, 3850, 3950, 4080, 4150, 4200]
    P = [   0,    5,   10,   20,   40,   60,   80,   90,   96,  100]
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) // (V[i]-V[i-1])
    return 100


def load_measured(path='logs/nvs_c6_bat0.bin'):
    out = subprocess.run([sys.executable, 'sim/tools/batlog_decode.py', path],
                         capture_output=True, text=True, encoding='utf-8').stdout
    rows = []
    for l in out.splitlines():
        m = re.match(r'\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s', l)
        if m:
            rows.append((int(m.group(2)), int(m.group(3))))   # (t_s, mv)
    return rows


def run(rows, release):
    """방전 구간을 돌린다. 반환: (표시% 목록, 하한 해제 시각)"""
    floor = None            # None = 미설정
    floor_mv = None
    up_run = 0
    disp = []
    released_at = None
    for i, (t, mv) in enumerate(rows):
        pct = mv_to_pct(mv)
        if release and floor is not None:
            if mv >= floor_mv + FLOOR_RELEASE_MV:
                up_run += 1
                if up_run >= FLOOR_RELEASE_N:
                    floor = None          # 해제
                    floor_mv = None
                    up_run = 0
                    if released_at is None:
                        released_at = t
            else:
                up_run = 0
        if floor is not None and pct > floor:
            pct = floor
        else:
            floor = pct
            floor_mv = mv
        disp.append(pct)
    return disp, released_at


def ceiling(start_pct, true_pct, minutes):
    """천장이 start_pct 에서 출발해 분당 0.85%p 오를 때 표시값."""
    ceil_mpct = start_pct * 1000
    ceil_mpct += int(minutes * CHG_RISE_MPCT_PER_MIN)
    return min(true_pct, ceil_mpct // 1000)


if __name__ == '__main__':
    rows = load_measured()
    print('실측 %d건 (%dmV → %dmV)' % (len(rows), rows[0][1], rows[-1][1]))
    print()
    for label, rel in (('현재(하한 단조)', False), ('수정A(조건부 해제)', True)):
        disp, rel_t = run(rows, rel)
        z = [i for i, p in enumerate(disp) if p == 0]
        print('── %s ──' % label)
        print('   최종 표시 %d%%   (실제 전압 %dmV = %d%%)'
              % (disp[-1], rows[-1][1], mv_to_pct(rows[-1][1])))
        print('   0%% 표시 구간 %d/%d 표본%s'
              % (len(z), len(disp),
                 ('   하한 해제 t=%ds' % rel_t) if rel_t else '   (해제 없음)'))
    print()
    print('── 수정B: 충전 천장 출발점 ──')
    true_pct = mv_to_pct(3944)
    for label, start in (('현재(직전 표시값 0%)', 0), ('수정B(연결순간 전압환산)', true_pct)):
        row = ['%d분:%d%%' % (m, ceiling(start, true_pct, m)) for m in (0, 5, 30, 90)]
        print('   %-26s %s' % (label, '  '.join(row)))
    print()
    print('   ※원래 막으려던 회귀 확인 — 84%% 에서 USB 연결(충전으로 단자전압 들뜸 → 100%%):')
    for label, start in (('현재', 84), ('수정B', 84)):
        print('     %-6s 연결 직후 %d%% (점프 없음)' % (label, ceiling(start, 100, 0)))
