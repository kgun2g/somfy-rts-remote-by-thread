#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BAT_ADC 8표본 집계 방식 비교 — 평균 vs 절사평균 (2026-08-12)
============================================================

배경 (실측, NVS 방전기록 세션 #8 / 107건)
------------------------------------------
`_read_bat_mv()` 는 ADC 를 8회 연속 읽어 **평균**한다. 8표본은 수십 us 안에 끝나므로
**같은 순간**을 재는 것이고, 따라서 그 안의 min/max 산포는 순수 ADC 잡음이다.

    USB 구동   : 산포 3~14카운트 (4~21mV)
    배터리 구동: 산포 중앙값 36카운트(54mV), 64%가 30카운트 이상, 최대 115(174mV)

배터리가 수십 us 만에 54mV 를 움직일 수는 없다 → **ADC 교란 확정**.
(원인 추정: 분압 100k/100k = 소스 임피던스 50kΩ 으로 높은데, 배터리 레일은 USB LDO
 처럼 단단하지 않아 잡음이 그대로 실린다. HW 로는 분압 하단 100nF 가 정석 해법.)

문제
----
**평균은 이상치 하나에 통째로 끌려간다.** 115카운트짜리 튐이 한 번 끼면 8표본 평균이
14카운트(≈21mV ≈ 3%p)나 밀린다. 이게 측정 간 값이 두 무리로 갈라져 보이던 원인이다.

수정
----
8표본을 정렬해 **가운데 4개만 평균**한다(25% 절사평균). 양쪽 끝 2개씩을 버리므로
한쪽에 이상치가 2개까지 끼어도 결과가 안 흔들린다.

    · 표본 수 8 은 그대로 둔다 — `_nobat_track` 의 5분 창 통계 가정을 유지하기 위해.
    · 산포 진단값(s_bat_last_spread)은 **전체 8표본의 min/max** 를 그대로 쓴다
      (절사한 값으로 재면 잡음 크기를 못 본다).

실행: python sim/tools/bat_adc_trim_sim.py
"""

import sys

# ── 실측 산포 분포 (세션 #8 의 산포 카운트, 그대로 옮김) ────────────────────
OBSERVED_SPREAD = [
    32, 41, 45, 45, 29, 22, 57, 39, 43, 45, 46, 59, 26, 23, 24, 27, 57, 52,
    55, 49, 44, 11, 48, 21, 33, 27, 15, 44, 7, 37, 3, 4, 34, 32, 35, 29,
    32, 8, 32, 44, 30, 115, 72, 69, 67, 10, 13, 36, 47, 28, 28, 44, 20, 21,
    16, 37, 25, 27, 25, 8, 32, 21, 26, 6, 31, 19, 26, 34, 42, 41, 43, 40,
    45, 39, 11, 48, 61, 56,
]

ADC_TO_MV = 3100.0 / 4095.0 * 2.0      # 1카운트 → 배터리 기준 mV (분압 100k/100k)


class Rng(object):
    """재현 가능한 선형합동 난수 (외부 의존성 없이 매번 같은 결과)."""

    def __init__(self, seed=20260812):
        self.s = seed

    def next(self, n):
        self.s = (self.s * 1103515245 + 12345) & 0x7FFFFFFF
        return self.s % n if n else 0


TRUE = 2050          # 시뮬레이터만 아는 참값 — 평가는 이것과의 오차로 한다


def make_samples(rng, body, exc, exc_pct):
    """8표본 생성 — **좁은 본체 + 드문 큰 튐**(heavy-tail).

    ★첫 판에서 틀렸던 점: 튐을 −s/2, +s/2 로 **대칭 쌍**으로 넣었더니 평균에서
      정확히 상쇄돼 "평균이 더 안정" 이라는 가짜 결과가 나왔다. 실제 ADC 교란에
      그런 대칭성은 보장되지 않는다 — 보장됐다면 애초에 측정값이 두 무리로
      갈라지지도 않았을 것이다(갈라졌다는 게 편향이 남는다는 증거다).
    → 튐의 **개수와 방향을 독립적으로** 뽑는다. 대칭도 한쪽 쏠림도 자연히 나온다.

    평가도 바꿨다: 인접 측정 간 변동(jitter)이 아니라 **참값과의 오차**로 잰다.
    jitter 는 값이 엉뚱한 곳에 눌러앉아도 작게 나와 판정 기준이 못 된다."""
    s = []
    for _ in range(8):
        if rng.next(100) < exc_pct:                       # 큰 튐
            s.append(TRUE + rng.next(2 * exc + 1) - exc)
        else:                                             # 좁은 본체
            s.append(TRUE + rng.next(2 * body + 1) - body)
    return s


def agg_mean(s):
    """현재 코드: 단순 평균."""
    return sum(s) // len(s)


def agg_trimmed(s):
    """수정안: 정렬 후 가운데 4개만 평균 (25% 절사평균)."""
    t = sorted(s)
    mid = t[2:6]
    return sum(mid) // len(mid)


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print("=" * 76)
    print(" BAT_ADC 8표본 집계 — 평균 vs 절사평균 (실측 산포 %d건으로 검증)"
          % len(OBSERVED_SPREAD))
    print("=" * 76)
    sp = sorted(OBSERVED_SPREAD)
    print("  실측 산포: 중앙값 %d카운트(%.0fmV)  평균 %.1f  최대 %d(%.0fmV)"
          % (sp[len(sp) // 2], sp[len(sp) // 2] * ADC_TO_MV,
             sum(sp) / float(len(sp)), sp[-1], sp[-1] * ADC_TO_MV))
    print()

    # ── 잡음 모델 보정: 만들어낸 산포 분포가 실측과 맞는지 먼저 확인한다 ────
    #    (모델이 실측을 못 맞추면 그 위의 비교는 의미가 없다)
    #    실측(중앙값 33 / 평균 34.8 / 최대 115)에 가장 근접한 조합.
    #    ★결론은 이 튜닝에 민감하지 않다 — body 4~6, exc 50~70, 10~15% 를 훑어봐도
    #      절사평균의 RMS 오차는 늘 평균의 **약 절반**이었다(5.7→2.7, 6.6→3.0,
    #      6.5→2.5, 7.2→3.4 …). 모델을 맞춰서 얻은 결과가 아니다.
    BODY, EXC, EXC_PCT = 6, 60, 15
    rng = Rng()
    N = 4000
    me, tr, gen_sp = [], [], []
    for _ in range(N):
        s = make_samples(rng, BODY, EXC, EXC_PCT)
        gen_sp.append(max(s) - min(s))
        me.append(agg_mean(s))
        tr.append(agg_trimmed(s))
    g = sorted(gen_sp)
    print("  모델 산포: 중앙값 %d카운트  평균 %.1f  최대 %d   ← 실측과 대조용"
          % (g[len(g) // 2], sum(g) / float(len(g)), g[-1]))
    print()

    def rms_mv(a):
        return (sum((x - TRUE) ** 2 for x in a) / float(len(a))) ** 0.5 * ADC_TO_MV

    def worst_mv(a):
        return max(abs(x - TRUE) for x in a) * ADC_TO_MV

    print("  참값 대비 오차 (%d회 측정)" % N)
    print("  %-26s %-14s %-14s" % ("", "평균(현재)", "절사평균(수정)"))
    print("  " + "-" * 58)
    print("  %-26s %-14.1f %-14.1f" % ("RMS 오차 (mV)", rms_mv(me), rms_mv(tr)))
    print("  %-26s %-14.1f %-14.1f" % ("최악 오차 (mV)", worst_mv(me), worst_mv(tr)))
    print("  %-26s %-14.1f %-14.1f"
          % ("최악 → %p (6.5mV/%p)", worst_mv(me) / 6.5, worst_mv(tr) / 6.5))
    print()

    ok = True
    c1 = rms_mv(tr) < rms_mv(me)
    print("  [%s] RMS 오차 감소 (%.1fmV → %.1fmV, %.0f%% 축소)"
          % ("OK" if c1 else "X", rms_mv(me), rms_mv(tr),
             100.0 * (1 - rms_mv(tr) / max(0.01, rms_mv(me)))))
    ok &= c1
    c2 = worst_mv(tr) < worst_mv(me)
    print("  [%s] 최악 오차 감소 (%.1fmV → %.1fmV)"
          % ("OK" if c2 else "X", worst_mv(me), worst_mv(tr)))
    ok &= c2

    # 최악 케이스: 115카운트 이상치 하나가 끼었을 때 얼마나 끌려가는가
    print()
    print("  최악 케이스 — 조용한 표본 7개 + 115카운트 튐 1개")
    print("  " + "-" * 58)
    quiet = [2050, 2051, 2049, 2050, 2052, 2049, 2051]
    worst = quiet + [2050 + 115]
    dm = (agg_mean(worst) - 2050) * ADC_TO_MV
    dt = (agg_trimmed(worst) - 2050) * ADC_TO_MV
    print("    평균     : %+.1fmV 밀림 (= %.1f%%p)" % (dm, dm / 6.5))
    print("    절사평균 : %+.1fmV 밀림 (= %.1f%%p)" % (dt, dt / 6.5))
    c3 = abs(dt) < abs(dm) / 4
    print("  [%s] 이상치 영향이 1/4 미만으로 감소" % ("OK" if c3 else "X"))
    ok &= c3

    print()
    print("=" * 76)
    print("결과: %s" % ("전부 통과" if ok else "★실패 항목 있음"))
    print()
    print("주의")
    print("-" * 76)
    print("  · 표본 수 8 은 **그대로** 둔다 — _nobat_track 의 5분 창 통계 가정 유지.")
    print("  · 산포 진단값은 전체 8표본의 min/max 를 계속 쓴다(잡음 크기를 봐야 하므로).")
    print("  · 이건 **표시 안정화**일 뿐 잡음 자체를 없애지 못한다. 근본 해법은 HW:")
    print("    BAT_ADC 분압(100k/100k, 소스 임피던스 50kΩ) 하단에 100nF.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
