#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
배터리 % 표시 평활 시뮬레이터 (2026-08-11)
==========================================

증상
----
배터리 구동으로 바꾸자 표시가 81 → 82 → **74** → 84 → 81 → 83 → 81 → 82 % 로 튄다.

왜 지금 생겼나
--------------
USB 전원일 땐 레일이 단단해 전압이 거의 안 변했다. 배터리로 바꾸면 BLE 광고·RF 송신
순간 **실제로** 전압이 떨어진다. 5초 주기 측정이 그 순간에 걸리면 한 점이 크게 낮게
찍힌다(74%). OCV 곡선상 이 구간은 1% ≈ 10mV 라 100mV 만 흔들려도 10%p 가 움직인다.

현재 코드의 한계
----------------
`_read_bat_mv()` 는 8회를 **연속 루프**로 읽어 평균한다. 8샘플이 수십 us 안에 끝나므로
**같은 순간을 8번 재는 것**과 같다 → ADC 노이즈는 줄지만 **부하 변동은 전혀 못 거른다**.
게다가 평균은 이상치 하나에 통째로 끌려간다.

설계
----
측정 자체는 그대로 두고(주기 5초·8회 평균 — `_nobat_track` 가정 보존), **표시용으로만**
후처리한다:

  1) **중앙값(median) 9주기** : 송신 순간에 걸린 한 점을 **통째로 버린다**.
     평균과 달리 이상치가 결과를 못 끌어당긴다.
  2) **EMA(α=1/4)** : 남은 흔들림을 시간축으로 눌러 표시가 안 튀게 한다.

`_nobat_track` 에는 **원본 값**을 그대로 준다 — 그쪽은 5분 창 30표본 통계를 쓰므로
평활하면 오히려 가정이 깨진다.

실행: python sim/tools/bat_pct_smooth_sim.py
"""

import sys

# ── 펌웨어와 동일한 OCV 곡선 ────────────────────────────────────────────────
# ★2026-08-11 만충 전압 보정 — 실기 만충 실측이 4,128~4,132 mV 라(이론 4,200 mV,
#   분압·ADC 오차 약 1.7%) 그대로 두면 만충인데 94% 에서 멈춘다.
#   상단 3개 앵커만 원곡선 모양을 유지한 채 [3980..4200] → [3980..4128] 로 압축했다.
#   중·저 구간은 그대로라 무배터리 float 판정(3,970 mV = 78%)은 영향 없다.
#   펌웨어 somfy_app.c 의 BAT_FULL_MV / BAT_TOP_SCALE 과 동일해야 한다.
V = [3200, 3450, 3580, 3680, 3750, 3850, 3980, 4047, 4094, 4128]
P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]


def bat_mv_to_pct(mv):
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i - 1] + (P[i] - P[i - 1]) * (mv - V[i - 1]) // (V[i] - V[i - 1])
    return 100


class Rng(object):
    """재현 가능한 선형합동 난수(외부 의존성 없이 매번 같은 결과)."""

    def __init__(self, seed=20260811):
        self.s = seed

    def next(self, n):
        self.s = (self.s * 1103515245 + 12345) & 0x7FFFFFFF
        return self.s % n if n else 0

    def noise(self, span):
        return self.next(span) - span // 2


# ── 표시용 평활기 (펌웨어에 넣을 로직과 동일) ───────────────────────────────
# ★만충 보정으로 곡선 상단이 압축돼 1% ≈ 7.4mV 로 가팔라졌다 → 창을 9 로.
#   (창5 진폭 11%p / 창9 6%p, 추종 지연은 둘 다 1%p 로 동일)
WIN = 9


class Smoother(object):
    """★EMA 를 **1/16 mV 단위**로 누적한다.

    왜: C 의 정수 나눗셈은 0 쪽으로 절단한다. 1mV 단위로 `ema += (med-ema)/4` 를
    쓰면 차이가 1~3mV 일 때 몫이 0 이 되어 **EMA 가 영영 안 움직인다**(고착).
    파이썬의 // 는 내림이라 이 버그가 안 보이므로, 여기서도 int() 로 C 와 같은
    절단을 흉내내 검증한다. 16배 해상도면 1mV 차이도 4/16mV 씩 확실히 수렴한다."""

    def __init__(self):
        self.hist = []
        self.ema_q4 = 0          # 1/16 mV 단위

    def feed(self, mv):
        self.hist.append(mv)
        if len(self.hist) > WIN:
            self.hist.pop(0)
        med = sorted(self.hist)[len(self.hist) // 2]
        if self.ema_q4 == 0:
            self.ema_q4 = med * 16
        else:
            self.ema_q4 += int((med * 16 - self.ema_q4) / 4)   # C 와 같은 절단
        return self.ema_q4 // 16


def run(cycles=40, seed=20260811):
    """실측 패턴 재현: 완만한 방전 + 가끔 송신 순간의 전압 강하 + ADC 노이즈."""
    rng = Rng(seed)
    true_mv = 4030          # 약 82% 지점
    raw_list, sm_list = [], []
    sm = Smoother()
    for k in range(cycles):
        true_mv -= 1                       # 완만한 자연 방전
        mv = true_mv + rng.noise(24)       # ADC 노이즈 ±12mV
        # 부하 버스트(BLE 광고/RF 송신)에 걸리는 확률 ~20% → 순간 강하
        if rng.next(100) < 20:
            mv -= 60 + rng.next(60)        # -60~-120mV
        raw_list.append(mv)
        sm_list.append(sm.feed(mv))
    return raw_list, sm_list


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print("=" * 74)
    print(" 배터리 % 표시 평활 검증")
    print("=" * 74)
    print("  OCV 감도: 3980mV=80%%, 4080mV=90%%  ->  이 구간 1%% = 10mV")
    print("  모델: 완만한 방전 + ADC 노이즈 +-12mV + 20%% 확률로 송신 강하 -60~-120mV")
    print()

    raw, sm = run()
    rp_all = [bat_mv_to_pct(v) for v in raw]
    sp_all = [bat_mv_to_pct(v) for v in sm]
    # ★워밍업(창이 차기 전) 구간은 평가에서 제외 — 부팅 직후 몇 주기는 어차피 과도기
    rp, sp = rp_all[WIN:], sp_all[WIN:]

    print("  주기   원본mV   원본%   평활mV   평활%")
    print("  " + "-" * 44)
    for i in range(len(raw)):
        mark = "  <= 송신 강하" if i > 0 and rp_all[i] <= rp_all[i-1] - 4 else ""
        if i < 20:
            print("  %3d  %7d %6d%%  %7d %6d%%%s" % (i, raw[i], rp_all[i], sm[i], sp_all[i], mark))
    print("  ... (총 %d주기)" % len(raw))
    print()

    def swing(a):
        return max(a) - min(a)

    def jitter(a):
        return sum(abs(a[i] - a[i - 1]) for i in range(1, len(a))) / float(len(a) - 1)

    print("판정")
    print("-" * 74)
    print("  %-22s %-12s %-12s" % ("", "원본", "평활"))
    print("  %-22s %-12s %-12s" % ("최대-최소(%p)", swing(rp), swing(sp)))
    print("  %-22s %-12.2f %-12.2f" % ("주기간 평균 변동(%p)", jitter(rp), jitter(sp)))
    print("  %-22s %-12s %-12s" % ("최저값(%)", min(rp), min(sp)))
    print()

    ok = True
    c1 = swing(sp) < swing(rp) / 2
    print("  [%s] 진폭이 절반 미만으로 감소 (%d%%p -> %d%%p)"
          % ("OK" if c1 else "X", swing(rp), swing(sp)))
    ok &= c1
    c2 = jitter(sp) <= 1.0
    print("  [%s] 주기간 변동 1%%p 이하 (표시가 안 튄다): %.2f"
          % ("OK" if c2 else "X", jitter(sp)))
    ok &= c2
    c3 = min(sp) >= min(rp) + 3
    print("  [%s] 송신 강하 이상치가 표시에 안 나타남 (원본 최저 %d%% -> 평활 최저 %d%%)"
          % ("OK" if c3 else "X", min(rp), min(sp)))
    ok &= c3

    # 추종성: 진짜로 전압이 떨어지면 따라가야 한다(멈추면 안 됨)
    print()
    print("추종성 검사 — 실제 방전을 따라가는가 (평활이 값을 붙잡아두면 안 된다)")
    print("-" * 74)
    sm2 = Smoother()
    disp = []
    mv = 4030
    for k in range(120):
        mv -= 1                      # 현실적 방전(주기 5초당 1mV ≈ 시간당 720mV 상당의
                                     #  가속 조건 — 실제보다 빠르게 잡아 보수적으로 본다)
        disp.append(bat_mv_to_pct(sm2.feed(mv)))
    drop_true = bat_mv_to_pct(4030) - bat_mv_to_pct(mv)
    drop_disp = disp[0] - disp[-1]
    c4 = abs(drop_true - drop_disp) <= 2
    print("  실제 하락 %d%%p, 표시 하락 %d%%p  → 지연 %d%%p"
          % (drop_true, drop_disp, abs(drop_true - drop_disp)))
    print("  [%s] 방전을 정상 추종 (지연 2%%p 이내)" % ("OK" if c4 else "X"))
    ok &= c4

    print()
    print("결과: %s" % ("전부 통과" if ok else "실패 항목 있음"))
    print()
    print("설계 메모")
    print("-" * 74)
    print("  · 중앙값을 쓰는 이유: 평균은 -100mV 이상치 하나에 20%/5 = 2%p 끌려간다.")
    print("    중앙값은 그 점을 **통째로 버린다**(5개 중 1~2개까지 무해).")
    print("  · _nobat_track 에는 **원본**을 준다 — 그쪽은 5분 창 30표본 통계라")
    print("    평활하면 '전압이 오르는가' 판정 가정이 깨진다.")
    print("  · 측정 주기(5초)·표본(8회)은 건드리지 않는다(같은 이유).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
