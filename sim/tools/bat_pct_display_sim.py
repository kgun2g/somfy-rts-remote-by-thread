#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
배터리 % 표시 상태머신 시뮬레이터 (2026-08-11)
=============================================

"배터리를 안 꽂았는데 78% 로 표시된다" 를 고친 로직을 **플래시 전에** 검증한다.

배경
----
배터리를 빼도 충전 IC(MCP73831)가 BAT+ 를 무부하로 ~3970mV 로 띄우는데, 이 값이
OCV-SoC 곡선상 정확히 **78%** 라 "배터리 없는데 78%" 로 보인다. 전압만으로는 실제
배터리 78% 와 구분이 **불가능**하다.

유일한 단서는 "전압이 오르는가"(무배터리 float = 안 오름 / 충전 중인 셀 = 오름).
그런데 이 판정은 5분 창이 차야 나온다 → 그 전 5분간 78% 가 보였다.

수정
----
첫 판정 전에는 % 를 숨기고 "--%" 를 표시한다. 단 **애매한 경우에만** 숨긴다:
무배터리 float 창(3940~4010mV) **밖**이면 충전 IC 가 그 값을 만들 수 없으므로
배터리가 있다는 게 전압만으로 확정 → 즉시 % 를 보여준다.

    ambiguous = (BAT_NOBAT_LO <= mv <= BAT_NOBAT_HI) and usb_powered
    표시 = "--%"           if (not judged) and ambiguous
           0%              elif nobat
           _bat_mv_to_pct(mv)  else

왜 창을 못 줄이나
-----------------
충전 상승률이 ≈9.3mV/5분 이라, 창이 4mV/9.3mV*5 = **2분 10초**보다 짧으면 진짜
충전 중인 배터리도 "안 오름" 으로 오판해 0% 로 표시된다. 5분은 그 2배 여유다.

실행: python sim/tools/bat_pct_display_sim.py
"""

import sys

# ── 펌웨어와 동일한 상수 (somfy_app.c) ──────────────────────────────────────
# ★2026-08-11 만충 전압 보정 — 실기 만충 실측이 4,128~4,132 mV 라(이론 4,200 mV,
#   분압·ADC 오차 약 1.7%) 그대로 두면 만충인데 94% 에서 멈춘다.
#   상단 3개 앵커만 원곡선 모양을 유지한 채 [3980..4200] → [3980..4128] 로 압축했다.
#   중·저 구간은 그대로라 무배터리 float 판정(3,970 mV = 78%)은 영향 없다.
#   펌웨어 somfy_app.c 의 BAT_FULL_MV / BAT_TOP_SCALE 과 동일해야 한다.
V = [3200, 3450, 3580, 3680, 3750, 3850, 3980, 4047, 4094, 4128]
P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]

BAT_NOBAT_LO = 3940
BAT_NOBAT_HI = 4010
BAT_NOBAT_RISE_MV = 4
BAT_NOBAT_WINDOW_MS = 300000      # 5분
BAT_NOBAT_MIN_N = 3
BAT_PCT_UNKNOWN = 255

BAT_PERIOD_MS = 5000              # 배터리 측정 주기


def bat_mv_to_pct(mv):
    """_bat_mv_to_pct() 와 동일."""
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i - 1] + (P[i] - P[i - 1]) * (mv - V[i - 1]) // (V[i] - V[i - 1])
    return 100


class BatModel(object):
    """_nobat_track() + 표시 결정 로직을 그대로 옮긴 것."""

    def __init__(self):
        self.nobat = False
        self.judged = False
        self.t0 = None
        self.sum1 = self.sum2 = 0
        self.n1 = self.n2 = 0
        self.judgements = []

    def track(self, t_ms, mv, usb):
        if mv <= 0:
            return
        if self.t0 is None:
            self.t0 = t_ms
        elapsed = t_ms - self.t0
        win = BAT_NOBAT_WINDOW_MS
        if elapsed < win // 2:
            self.sum1 += mv; self.n1 += 1; return
        if elapsed < win:
            self.sum2 += mv; self.n2 += 1; return
        if self.n1 >= BAT_NOBAT_MIN_N and self.n2 >= BAT_NOBAT_MIN_N:
            m1 = self.sum1 // self.n1
            m2 = self.sum2 // self.n2
            rise = m2 - m1
            in_window = (BAT_NOBAT_LO <= m1 <= BAT_NOBAT_HI and
                         BAT_NOBAT_LO <= m2 <= BAT_NOBAT_HI)
            flat = rise < BAT_NOBAT_RISE_MV
            self.nobat = in_window and flat and usb
            self.judged = True
            self.judgements.append((t_ms, self.nobat, m1, m2, rise))
        self.t0 = t_ms
        self.sum1 = self.sum2 = 0
        self.n1 = self.n2 = 0

    def display(self, mv, usb):
        """수정된 표시 결정."""
        ambiguous = (BAT_NOBAT_LO <= mv <= BAT_NOBAT_HI) and usb
        if not self.judged and ambiguous:
            return BAT_PCT_UNKNOWN
        return 0 if self.nobat else bat_mv_to_pct(mv)

    def display_old(self, mv, usb):
        """수정 전(항상 % 표시)."""
        return 0 if self.nobat else bat_mv_to_pct(mv)


def render(p):
    return "--%" if p > 100 else "%d%%" % p


def run(name, mv_fn, usb, minutes=12):
    m = BatModel()
    rows = []
    t = 0
    while t <= minutes * 60000:
        mv = mv_fn(t)
        m.track(t, mv, usb)
        rows.append((t, mv, m.display_old(mv, usb), m.display(mv, usb)))
        t += BAT_PERIOD_MS
    return name, m, rows


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print("=" * 78)
    print(" 배터리 % 표시 상태머신 검증")
    print("=" * 78)
    print("  OCV 곡선 확인: 3970mV -> %d%%   (사용자가 본 값)" % bat_mv_to_pct(3970))
    print("                 3972mV -> %d%%" % bat_mv_to_pct(3972))
    print("  -> 무배터리 float 전압이 우연히 78%% 구간에 떨어진다.")
    print()

    # 시나리오: (이름, 전압함수, USB연결)
    scenarios = [
        ("1. 무배터리 + USB (문제 상황)",
         lambda t: 3970 + (1 if (t // 5000) % 3 == 0 else 0),      # float, 노이즈 ±1
         True),
        ("2. 실배터리 78% + USB (충전중)",
         lambda t: 3965 + int(9.3 * (t / 300000.0)),               # 9.3mV/5분 상승
         True),
        ("3. 실배터리 40% + USB",
         lambda t: 3750 + int(9.3 * (t / 300000.0)),               # 창 밖 -> 즉시 표시
         True),
        ("4. 배터리 구동(USB 없음), 3970mV",
         lambda t: 3970, False),
    ]

    print("%-32s%14s%14s%14s" % ("시나리오", "0분", "3분", "10분"))
    print("-" * 78)
    results = []
    for name, fn, usb in scenarios:
        nm, m, rows = run(name, fn, usb)
        pick = {}
        for t, mv, old, new in rows:
            for mark, tt in (("0분", 0), ("3분", 180000), ("10분", 600000)):
                if t == tt:
                    pick[mark] = (old, new)
        print("%-32s%14s%14s%14s" % (
            name,
            "%s -> %s" % (render(pick["0분"][0]), render(pick["0분"][1])),
            "%s -> %s" % (render(pick["3분"][0]), render(pick["3분"][1])),
            "%s -> %s" % (render(pick["10분"][0]), render(pick["10분"][1]))))
        results.append((name, m, rows))
    print()
    print("  각 칸은 '수정전 -> 수정후'")
    print()

    print("판정 로그 (창이 찰 때마다)")
    print("-" * 78)
    for name, m, rows in results:
        if m.judgements:
            for t, nb, m1, m2, rise in m.judgements[:2]:
                print("  %-32s %5.1f분  %s (전반%d 후반%d 상승%+d)"
                      % (name, t / 60000.0,
                         "미연결->0%" if nb else "연결됨->실측%", m1, m2, rise))
        else:
            print("  %-32s  판정 없음 (창 미완성)" % name)
    print()

    # ── 검증 ────────────────────────────────────────────────────────────────
    print("검증")
    print("-" * 78)
    ok = True

    nm, m1_, rows1 = results[0]
    first = rows1[0]
    late = [r for r in rows1 if r[0] >= 400000][0]
    c1 = first[3] > 100 and late[3] == 0
    print("  [%s] 무배터리: 초기 '--%%' -> 판정 후 '0%%'   (실제 %s -> %s)"
          % ("OK" if c1 else "X", render(first[3]), render(late[3])))
    ok &= c1
    c1b = first[2] == 78
    print("  [%s] 수정 전이었다면 초기에 '%s' 로 보였다 (사용자 신고와 일치)"
          % ("OK" if c1b else "X", render(first[2])))
    ok &= c1b

    nm, m2_, rows2 = results[1]
    late2 = [r for r in rows2 if r[0] >= 400000][0]
    c2 = rows2[0][3] > 100 and late2[3] == bat_mv_to_pct(late2[1])
    print("  [%s] 실배터리 78%%+USB: 초기 '--%%' -> 판정 후 실측 %s (0%% 오판 없음)"
          % ("OK" if c2 else "X", render(late2[3])))
    ok &= c2

    nm, m3_, rows3 = results[2]
    c3 = rows3[0][3] == bat_mv_to_pct(rows3[0][1]) and rows3[0][3] <= 100
    print("  [%s] 실배터리 40%%: float 창 밖이라 **대기 없이 즉시** %s"
          % ("OK" if c3 else "X", render(rows3[0][3])))
    ok &= c3

    nm, m4_, rows4 = results[3]
    c4 = rows4[0][3] == 78
    print("  [%s] USB 없음(배터리 구동): 배터리가 있는 게 자명 -> 즉시 %s"
          % ("OK" if c4 else "X", render(rows4[0][3])))
    ok &= c4

    # 창 단축 위험 검증
    print()
    print("창을 줄이면 왜 안 되는가 (충전 중 배터리 오판 검사)")
    print("-" * 78)
    RISE_PER_5MIN = 9.3
    for win_min in (1, 2, 3, 5):
        rise = RISE_PER_5MIN * win_min / 5.0 / 2.0 * 2.0   # 전반->후반 평균차 ≈ 창의 절반만큼
        rise_half = RISE_PER_5MIN * (win_min / 5.0) / 2.0 * 2.0
        # 전반 평균 시점 = 창의 1/4, 후반 = 3/4 -> 차이는 창의 1/2 만큼의 상승
        delta = RISE_PER_5MIN * (win_min / 5.0) * 0.5
        verdict = "오판(충전중을 미연결로)" if delta < BAT_NOBAT_RISE_MV else "안전"
        print("  창 %d분 -> 전반/후반 평균차 %.1fmV  vs 문턱 %dmV   %s"
              % (win_min, delta, BAT_NOBAT_RISE_MV, verdict))
    print("  -> 현재 5분 유지가 맞다. (2분 이하는 확실히 위험)")

    print()
    print("결과: %s" % ("전부 통과" if ok else "실패 항목 있음"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
