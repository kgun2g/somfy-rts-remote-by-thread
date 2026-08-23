# -*- coding: utf-8 -*-
"""방전 기록에서 **실효 평균 전류**를 역산한다.

왜 필요한가 (2026-08-22): 지금까지 절전 작업은 전부 "깨어있는 시간 비율"만
측정했다(13.4% → 10.5%). 그런데 **시간의 89.5% 는 수면**이고, 수면 전류는 한
번도 재지 않았다. 내가 드린 "약 15% 감소" 같은 수치는 전부 가정치
(깨어있을 때 35mA / 잘 때 1.5mA)에 기반한 **추정**이었다.

수면 전류가 크면(802.15.4 MAC/BB 가 켜져 있으니 그럴 만하다) 깨어있는 시간을
아무리 깎아도 총 소비는 거의 안 준다. 그래서 먼저 **실측 기준선**을 만든다.

방법: 배터리 OCV 곡선(somfy_app.c `_bat_mv_to_pct` 와 동일한 표)으로
      전압 → 잔량% 를 구하고, 세션 동안의 % 감소 × 용량 / 경과시간 = 평균 전류.

한계(정직하게):
  · OCV 표는 실측 보정된 근사다. 4.0~4.2V 구간은 기울기가 완만해 오차가 크다.
  · 부하가 걸린 단자전압이라 무부하 OCV 보다 약간 낮게 읽힌다.
  · 따라서 **절대값보다 전후 비교**에 쓸 것. 같은 방법·같은 구간이면 비교는 유효하다.
"""
import re
import subprocess
import sys

CAPACITY_MAH = 700          # 셀 용량 (wiring 문서 기준)

V = [3200, 3450, 3580, 3680, 3750, 3850, 3950, 4080, 4150, 4200]
P = [   0,    5,   10,   20,   40,   60,   80,   90,   96,  100]


def mv_to_pct(mv):
    if mv <= V[0]:
        return 0.0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) / float(V[i]-V[i-1])
    return 100.0


def load(path):
    out = subprocess.run([sys.executable, 'sim/tools/batlog_decode.py', path],
                         capture_output=True, text=True, encoding='utf-8').stdout
    rows = []
    for l in out.splitlines():
        m = re.match(r'\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s', l)
        if m:
            rows.append((int(m.group(2)), int(m.group(3))))   # (t_s, mv)
    ls = {}
    for k in ('세션 경과', '잔 시간', '깨어남 빈도', '깨어있음'):
        mm = re.search(re.escape(k) + r'[^0-9-]*([0-9.]+)', out)
        if mm:
            ls[k] = float(mm.group(1))
    return rows, ls


def analyse(path, label=None):
    rows, ls = load(path)
    if len(rows) < 10:
        print('%s: 표본 부족' % path); return None
    # 앞뒤 5% 는 잘라낸다(세션 시작 직후 안정화 / USB 재연결 직전 요동)
    n = len(rows)
    a = rows[n // 20]
    b = rows[-(n // 20) - 1]
    dt_h = (b[0] - a[0]) / 3600.0
    if dt_h <= 0:
        print('%s: 구간 없음' % path); return None
    dpct = mv_to_pct(a[1]) - mv_to_pct(b[1])
    ma = dpct / 100.0 * CAPACITY_MAH / dt_h
    print('── %s ──' % (label or path))
    print('   구간 %.2f시간   %dmV(%.1f%%) → %dmV(%.1f%%)   Δ%.2f%%p'
          % (dt_h, a[1], mv_to_pct(a[1]), b[1], mv_to_pct(b[1]), dpct))
    print('   **실효 평균 전류 %.2f mA**   (700mAh 기준 %.1f시간 지속)'
          % (ma, CAPACITY_MAH / ma if ma > 0 else 0))
    if ls.get('깨어있음'):
        awake = ls['깨어있음']
        freq = ls.get('깨어남 빈도', 0)
        duty = awake * freq / 1000.0
        print('   깨어있는 시간 %.1f%%  (회당 %.2fms × %.1f회/초)'
              % (duty * 100, awake, freq))
        # 두 상태 전류를 역산할 수는 없다(미지수 2개, 식 1개). 대신
        # "깨어있는 전류를 0 으로 만들어도 남는 값" = 수면 전류 하한을 보인다.
        print('   ※깨어있는 구간을 **전부 0mA 로 만들어도** 수면 전류가 남는다:')
        print('     수면 전류 하한 추정 = 총 %.2fmA 중 수면 몫' % ma)
    return ma


if __name__ == '__main__':
    if len(sys.argv) > 1:
        for p in sys.argv[1:]:
            analyse(p)
    else:
        print(__doc__)
