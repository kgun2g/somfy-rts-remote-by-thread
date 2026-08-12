#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
배터리 % 상승 버그 — 수정 C+B 검증 (2026-08-12)
================================================

증상
----
USB 분리 후 좌/우 버튼을 누르는 동안 표시가 68 → 69 → 70 → 71 % 로 **올라간다**.
방전 중에 잔량이 늘 수는 없다.

실측 (NVS 방전기록 세션 #16, 290건)
-----------------------------------
`bl` 기록에서 원본 전압을 뽑으면 **두 무리로 갈라진다**:

    3902 3904 3906 3906 3916 3918 3918 3922 3924        ← 9개, 평균 3913
    3932 3938 3942 3942 3944 3946 3946 3954 3956 3958   ← 11개, 평균 3944

간격 **31mV**, 중간값이 거의 없다. 가우시안 노이즈가 아니라 **두 개의 상태**다.
이 구간 OCV 기울기가 6.5mV/%p 이므로 31mV = **약 5%p**.
같은 기기가 USB 연결 중일 땐 4060~4068mV(±4mV = ADC 2카운트)로 미동도 없다
→ ADC 자체는 멀쩡하고 **배터리 구동일 때만** 갈라진다.

왜 "상승"으로 보이나
--------------------
중앙값-9 는 최근 9개 중 **어느 무리가 5개를 넘느냐**로 출력이 결정된다. 비율이
서서히 바뀌면 출력이 31mV 씩 **툭 튀고**, EMA(α=1/4)가 그 계단을 부드럽게 따라가
**단조 상승 구간**을 만든다. 아래 [1] 에서 신고와 같은 형태가 재현된다.

수정
----
* **C. USB→배터리 전환 시 평활 상태 초기화 + 기준점 지연**
  분리 순간의 평활값은 **충전 중 단자전압**(4056mV)이지 배터리 OCV 가 아니다.
  그대로 두면 중앙값 창 9개가 다 빠져나갈 때까지(45초) + EMA 지연까지 약 90초
  동안 92% → 73% 로 흘러내린다. 이 정착 기울기를 방전 로거가 진짜 방전으로
  착각해 **평균 302~819mA**(700mAh 셀에서 물리적으로 불가능)를 찍었다.
  → 전환 시 창·EMA 를 비우고, **첫 배터리 실측이 들어온 뒤** 세션 기준점을 잡는다.
  (USB 재연결 때도 대칭으로 초기화한다 — 계단 크기가 같으므로 같은 지연이 생긴다)

* **B. 배터리 구동 중 표시 % 단조 비증가**
  방전 중 잔량은 물리적으로 늘 수 없다. 세션 중 최저값을 하한으로 잡아 위로는
  안 가게 고정한다. USB 연결 중에는 충전이므로 상승을 허용한다.

  ★단, **중앙값 창이 5개는 차야 하한을 잡는다**(BAT_FLOOR_MIN_N). 이 시뮬레이터가
    잡아낸 결함이다: 그냥 첫 표본부터 하한을 걸면, 그 표본이 저전압 무리에 걸렸을 때
    **세션 내내 5%p 낮게 고정**된다([2][4] 실패로 드러남). 중앙값이 이상치 2개를
    버틸 수 있는 최소 표본수가 5이므로(=창 절반+1) 거기서 잡는다. 대가는 분리 후
    25초 동안 하한이 없어 ±2~3%p 흔들릴 수 있다는 것 — 기존의 90초/20%p 미끄러짐과
    비교하면 훨씬 작다.

실행: python sim/tools/bat_pct_monotone_sim.py
"""

import sys

# ── 펌웨어와 동일한 OCV 곡선 (somfy_app.c _bat_mv_to_pct) ────────────────────
V = [3200, 3450, 3580, 3680, 3750, 3850, 3980, 4038, 4078, 4108]
P = [0, 5, 10, 20, 40, 60, 80, 90, 96, 100]

BAT_DISP_WIN = 9
BAT_FLOOR_MIN_N = BAT_DISP_WIN // 2 + 1    # ★하한을 잡기 전 필요한 최소 표본수 = 5
BAT_PCT_UNKNOWN = 255


def bat_mv_to_pct(mv):
    if mv <= V[0]:
        return 0
    for i in range(1, len(V)):
        if mv < V[i]:
            return P[i - 1] + (P[i] - P[i - 1]) * (mv - V[i - 1]) // (V[i] - V[i - 1])
    return 100


class Smoother(object):
    """_bat_smooth_mv() 와 동일. EMA 는 1/16 mV 로 누적(C 정수절단 대비)."""

    def __init__(self):
        self.hist = []
        self.ema_q4 = 0
        self.seq = 0

    def reset(self):
        """★C: _bat_smooth_reset() — 창과 EMA 를 비운다."""
        self.hist = []
        self.ema_q4 = 0

    def feed(self, mv):
        self.seq += 1
        self.hist.append(mv)
        if len(self.hist) > BAT_DISP_WIN:
            self.hist.pop(0)
        med = sorted(self.hist)[len(self.hist) // 2]
        if self.ema_q4 == 0:
            self.ema_q4 = med * 16
        else:
            self.ema_q4 += int((med * 16 - self.ema_q4) / 4)   # C 와 같은 절단
        return self.ema_q4 // 16


class Gauge(object):
    """표시 % 결정. fix=True 면 C+B 적용."""

    def __init__(self, fix):
        self.fix = fix
        self.sm = Smoother()
        self.floor = BAT_PCT_UNKNOWN          # ★B: 배터리 구동 중 표시 하한
        self.was_usb = True
        # 방전 세션 상태
        self.pending = False
        self.seq0 = 0
        self.t0 = None
        self.mv0 = None
        self.pct0 = None

    def step(self, t_s, mv, usb):
        # ── 전원 전환 검출 (펌웨어의 was_usb_pwr 블록) ─────────────────────
        if self.was_usb and not usb:
            if self.fix:
                self.sm.reset()                # ★C
                self.pending = True
                self.seq0 = self.sm.seq
            else:
                self.t0 = t_s                  # 기존: 즉시 기준점(오염된 평활값)
                self.mv0 = self.sm.ema_q4 // 16
                self.pct0 = self.last_pct
        elif not self.was_usb and usb:
            if self.fix:
                self.sm.reset()                # ★C 대칭
            self.t0 = None
            self.pending = False
        self.was_usb = usb

        sm = self.sm.feed(mv)
        pct = bat_mv_to_pct(sm)

        # ── ★B: 배터리 구동 중 단조 비증가 ────────────────────────────────
        #   창이 BAT_FLOOR_MIN_N 개는 차야 하한을 잡는다 — 첫 표본이 저전압 무리에
        #   걸리면 세션 내내 그 값에 고정되기 때문(시뮬레이터가 잡은 결함).
        if self.fix:
            if not usb and len(self.sm.hist) >= BAT_FLOOR_MIN_N:
                if self.floor <= 100 and pct > self.floor:
                    pct = self.floor
                else:
                    self.floor = pct
            elif usb:
                self.floor = BAT_PCT_UNKNOWN   # 충전 중엔 상승 허용

        # ── ★C: 기준점은 **하한이 걸리는 시점과 같이** 확정한다 ───────────
        #   왜 첫 표본이 아닌가: 첫 표본은 아직 중앙값이 안 걸린 날값이라, 그걸
        #   기준으로 삼으면 워밍업 중 표시가 올라갈 때 누적 전류가 **음수**로 나온다.
        #   하한과 같은 시점(=수렴값)에 잡으면 기준점·하한이 같은 값이라 모순이 없다.
        if (self.fix and self.pending and not usb and
                self.sm.seq > self.seq0 and len(self.sm.hist) >= BAT_FLOOR_MIN_N):
            self.pending = False
            self.t0, self.mv0, self.pct0 = t_s, sm, pct

        self.last_pct = pct
        return sm, pct

    def avg_ma(self, t_s, pct, cap_mah=700):
        """방전 로거의 누적 평균 전류 계산과 동일."""
        if self.t0 is None or t_s <= self.t0:
            return None
        el = t_s - self.t0
        return (self.pct0 - pct) * cap_mah * 36 // (el * 10)


# ── 실측 재현용 입력 ────────────────────────────────────────────────────────
LO, HI = 3913, 3944          # 실측 두 무리의 평균
USB_MV = 4056                # 분리 직전 충전 단자전압(실측)


def run(seq, fix, usb_prefix=4):
    """seq = (usb, mv) 리스트. usb_prefix 주기만큼 USB 상태로 워밍업."""
    g = Gauge(fix)
    g.last_pct = 0
    out = []
    t = 0
    for _ in range(usb_prefix):
        g.step(t, USB_MV, True)
        t += 5
    for usb, mv in seq:
        sm, pct = g.step(t, mv, usb)
        out.append((t, mv, sm, pct, g.avg_ma(t, pct)))
        t += 5
    return g, out


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    ok = True
    print("=" * 78)
    print(" 배터리 %% 상승 버그 — 수정 C+B 검증")
    print("=" * 78)
    print("  실측 두 무리: %dmV(%d%%) / %dmV(%d%%)  → 격차 %d%%p"
          % (LO, bat_mv_to_pct(LO), HI, bat_mv_to_pct(HI),
             bat_mv_to_pct(HI) - bat_mv_to_pct(LO)))
    print()

    # ── [1] 증상 재현: 두 무리의 비율이 저→고로 이동 ────────────────────────
    print("[1] 증상 재현 — 두 무리 비율이 저전압 우세 → 고전압 우세로 이동")
    print("-" * 78)
    mix = ([LO] * 7 + [HI, LO, HI, LO, HI, HI, LO] +
           [HI, HI, HI, LO, HI, HI, HI, HI, HI, HI, HI])
    seq = [(False, v) for v in mix]

    for label, fix in (("수정 전", False), ("수정 후(C+B)", True)):
        g, out = run(seq, fix)
        pcts = [r[3] for r in out]
        # 수정 후에는 하한이 BAT_FLOOR_MIN_N 표본부터 걸리므로 그 구간만 본다
        lo_i = BAT_FLOOR_MIN_N if fix else 1
        rises = sum(1 for i in range(max(1, lo_i), len(pcts)) if pcts[i] > pcts[i - 1])
        best = 0
        run_len = 0
        for i in range(1, len(pcts)):
            run_len = run_len + 1 if pcts[i] > pcts[i - 1] else 0
            best = max(best, run_len)
        print("  %-13s %s" % (label, ">".join(str(p) for p in pcts)))
        print("  %-13s 상승 %d회, 최장 연속상승 %d단계, 총변화 %+d%%p"
              % ("", rises, best, pcts[-1] - pcts[0]))
        if not fix:
            c = rises >= 4
            print("  %-13s [%s] 신고 증상(단조 상승) 재현됨"
                  % ("", "OK" if c else "X"))
            ok &= c
        else:
            c = rises == 0
            print("  %-13s [%s] 하한 발동(%d표본=%d초) 이후 상승 0회"
                  % ("", "OK" if c else "X", BAT_FLOOR_MIN_N, BAT_FLOOR_MIN_N * 5))
            ok &= c
        print()

    # ── [2] USB 분리 직후 정착 (충전전압 잔재) ──────────────────────────────
    print("[2] USB 분리 직후 — 충전 단자전압 4056mV 가 얼마나 오래 남는가")
    print("-" * 78)
    # 분리 후 실제 배터리는 두 무리를 오간다(평균 3928)
    tail = [(False, HI if i % 2 else LO) for i in range(24)]
    target = bat_mv_to_pct((LO + HI) // 2)
    for label, fix in (("수정 전", False), ("수정 후(C+B)", True)):
        g, out = run(tail, fix)
        settle = None
        for t, mv, sm, pct, _ in out:
            if abs(pct - target) <= 3:
                settle = t - out[0][0]
                break
        print("  %-13s 첫 표시 %d%% (참값 %d%%, 오차 %+d%%p) → 오차 3%%p 이내까지 %s"
              % (label, out[0][3], target, out[0][3] - target,
                 "%d초" % settle if settle is not None else "미정착"))
        head = "  %-13s " % ""
        print(head + " ".join("%d%%" % r[3] for r in out[:14]))
        if fix:
            # C 의 약속은 "정착이 빠르다"가 아니라 **처음부터 배터리 대역에서 시작한다**
            c = abs(out[0][3] - target) <= 3 and settle == 0
            print("  %-13s [%s] 첫 표시부터 배터리 대역 (기존은 충전전압 92%%에서 출발)"
                  % ("", "OK" if c else "X"))
            ok &= c
        print()

    # ── [3] 방전 로거의 평균 전류 ───────────────────────────────────────────
    print("[3] 방전 로거 평균 전류 — 700mAh 셀에서 물리적으로 가능한 값인가")
    print("-" * 78)
    for label, fix in (("수정 전", False), ("수정 후(C+B)", True)):
        g, out = run(tail, fix)
        mas = [r[4] for r in out if r[4] is not None]
        peak = max(abs(m) for m in mas) if mas else 0
        neg = sum(1 for m in mas if m < 0)
        print("  %-13s 최대 |%dmA|, 음수 %d회   (계열: %s ...)"
              % (label, peak, neg, " ".join(str(m) for m in mas[:6])))
        if fix:
            c = peak <= 200 and neg == 0
            print("  %-13s [%s] 200mA 이하 + 음수 없음 (기존 800mA대는 불가능한 값)"
                  % ("", "OK" if c else "X"))
            ok &= c
    print()

    # ── [4] 회귀 검사: 진짜 방전은 여전히 따라가는가 ────────────────────────
    print("[4] 회귀 — 진짜 방전 추종 (하한 고정이 값을 얼려버리면 안 된다)")
    print("-" * 78)
    #   ※워밍업 구간(하한 발동 전)은 아직 중앙값이 안 걸려 비교 대상이 아니다.
    #     하한이 걸린 시점 ~ 끝 구간에서, **중앙값이 따라가는 참값**과 비교한다.
    slow, truth = [], []
    mv = 3980
    for i in range(120):
        mv -= 1 if i % 2 else 0                     # 10초당 1mV 방전
        slow.append((False, mv + (18 if i % 3 else -13)))   # 두 무리 노이즈 유지
        truth.append(mv + 18)     # 9개 중 2/3 가 +18 이므로 중앙값은 이쪽을 따라간다
    g, out = run(slow, True)
    k0 = BAT_FLOOR_MIN_N - 1                        # 하한이 걸린 첫 표본
    drop_true = bat_mv_to_pct(truth[k0]) - bat_mv_to_pct(truth[-1])
    drop_disp = out[k0][3] - out[-1][3]
    c = abs(drop_true - drop_disp) <= 3
    print("  하한 발동(%d번째 표본) ~ 끝: 실제 하락 %d%%p, 표시 하락 %d%%p → 지연 %d%%p"
          % (BAT_FLOOR_MIN_N, drop_true, drop_disp, abs(drop_true - drop_disp)))
    print("  [%s] 방전을 정상 추종 (지연 3%%p 이내 — 하한이 값을 얼리지 않는다)"
          % ("OK" if c else "X"))
    ok &= c

    # ── [5] 회귀 검사: USB 재연결 시 충전 상승은 보여야 한다 ────────────────
    print()
    print("[5] 회귀 — USB 재연결 후 충전 상승이 하한에 막히면 안 된다")
    print("-" * 78)
    chg = [(False, LO)] * 6 + [(True, 3990 + i * 8) for i in range(14)]
    g, out = run(chg, True)
    pre = out[5][3]
    post = out[-1][3]
    c = post > pre
    print("  배터리 구동 %d%% → USB 연결 후 %d%%  (%+d%%p)" % (pre, post, post - pre))
    print("  [%s] 충전 중에는 %% 가 올라간다 (하한 해제됨)" % ("OK" if c else "X"))
    ok &= c

    print()
    print("=" * 78)
    print("결과: %s" % ("전부 통과" if ok else "★실패 항목 있음"))
    print()
    print("남은 것 (A) — 두 무리의 정체")
    print("-" * 78)
    print("  C+B 는 표시를 바로잡을 뿐 31mV 격차 자체를 없애지 못한다.")
    print("  판별법: _read_bat_mv() 의 8회 표본 min/max 편차를 찍는다.")
    print("    · 편차 좁음(2~4카운트) → 그 순간 ADC 는 조용 → 31mV 는 **진짜 전압차**")
    print("                              (깨어남/light sleep 부하 전환 또는 셀 임피던스)")
    print("    · 편차 넓음(15카운트≈31mV) → **ADC 교란** (ADC1 채널 간섭 잔존)")
    print("  8표본은 수십 us 안에 끝나 같은 순간을 재므로, 편차는 순수 ADC 잡음이다.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
