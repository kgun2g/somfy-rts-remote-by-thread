#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
버튼 입력: 10ms 고정 폴링 → `~INT` 버스트 폴링 전환 검증 (②)  2026-08-13
=======================================================================

왜 바꾸나
---------
실측(6.64시간, 세션 #22)에서 평균 **57mA**, 사용시간 12.3h. 원인은 Matter 가 아니라
**light sleep 에 한 번도 진입하지 못한 것**이었다:

    CONFIG_FREERTOS_HZ=100 → 1 tick = 10ms
    CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP=3 → 문턱 3 tick = 30ms
    _btn_task 가 vTaskDelay(10ms)=1 tick 마다 깨움 → 유휴가 30ms 를 절대 못 넘음

①(tick 1000Hz)로 문턱을 3ms 로 낮춰 폴링 **사이**에 자게 만들었고, ②는 폴링 자체를
없앤다. PCF8574 의 `~INT` 는 이미 배선돼 있고 wake source 로도 등록돼 있다
(somfy_app.c: gpio_wakeup_enable(PCF8574_INT_PIN, GPIO_INTR_LOW_LEVEL)).

★단순 전환이 위험한 이유 — 10ms 주기에 네 가지가 얹혀 있다
  1. 디바운스 20ms  : 같은 값이 연속 2폴 이상이어야 edge 인정
  2. 롱프레스        : PROG 2s / SETUP 1s / 강제재부팅 15s — **누르고 있는 동안** 평가
  3. 로터리 쿼드러처 : A/B 전이 LUT 누산(±2 = 1디텐트) — 전이를 놓치면 방향이 틀림
  4. 진동 듀티사이클 : 30폴 창에서 HIGH 비율로 판정
순수 이벤트 구동으로 바꾸면 2·3·4 가 전부 깨진다.

설계 — 버스트 폴링(on-demand)
-----------------------------
    유휴(A): 알림 대기(타임아웃 IDLE_POLL_MS) — ~INT / VIBE ISR 이 깨운다
    활성(B): **기존과 똑같이 10ms 폴링** — 위 1~4 의 타이밍을 그대로 보존
    A→B : ~INT(입력 변화) 또는 안전망 타임아웃
    B→A : 모든 버튼 뗌 + 로터리 정지 + ACTIVE_HOLD_MS 동안 조용

즉 **사용자가 만지는 동안에는 지금과 100% 동일하게 동작**하고, 아무도 안 만질 때만
잔다. 반응성 회귀("몇 번을 눌러야 켜진다")를 원천적으로 피하는 구조다.

★`~INT` 취급 주의
  PCF8574 `~INT` 는 입력이 바뀌면 LOW 로 떨어지고 **포트를 읽으면 해제**된다.
  잠들기 직전에 이미 LOW 면 이후 **falling edge 가 안 생겨 영영 못 깨어난다**.
  → 자기 전에 반드시 1회 읽어 INT 를 해제한다. 그래도 놓치면 IDLE_POLL_MS
    안전망이 회수한다(자가 치유).

실행: python sim/tools/btn_int_wake_sim.py
"""

import sys

# ── 펌웨어와 동일한 상수 ────────────────────────────────────────────────────
DEBOUNCE_MS         = 20
LONG_PRESS_MS       = 2000     # PROG
SETUP_LONG_PRESS_MS = 1000     # SETUP
ACTIVE_POLL_MS      = 10       # 활성 구간 폴링(기존과 동일)
IDLE_POLL_MS        = 500      # 유휴 안전망(놓친 INT 회수)
ACTIVE_HOLD_MS      = 300      # 마지막 활동 후 이 시간 조용하면 유휴로

# 쿼드러처 LUT (button_handler.c kQuad 와 동일 개념)
KQUAD = {
    (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
    (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
}


class Decoder(object):
    """펌웨어의 버튼/로터리 처리 로직. 폴링 시점만 다르고 내용은 동일하다."""

    def __init__(self):
        self.raw = True          # active-LOW: True=뗌
        self.last = True
        self.deb_ms = 0
        self.press_ms = 0
        self.long_fired = False
        self.ab_raw = 0b11
        self.accum = 0
        self.events = []

    def poll(self, t, pressed, ab):
        """t=현재 ms, pressed=버튼이 물리적으로 눌림, ab=로터리 A/B 2비트"""
        cur = not pressed                      # active-LOW
        if cur != self.raw:
            self.raw = cur
            self.deb_ms = t
        if (t - self.deb_ms) >= DEBOUNCE_MS:
            stable = self.raw
            if (not stable) and self.last:
                self.press_ms = t
                self.long_fired = False
                self.last = False
                self.events.append((t, "PRESS"))
            elif stable and (not self.last):
                hold = t - self.press_ms
                self.last = True
                if not self.long_fired:
                    self.events.append((t, "RELEASE"))
            elif (not stable) and (not self.last):
                hold = t - self.press_ms
                if (not self.long_fired) and hold >= SETUP_LONG_PRESS_MS:
                    self.long_fired = True
                    self.events.append((t, "LONG"))
        # 로터리
        if ab != self.ab_raw:
            self.accum += KQUAD.get((self.ab_raw, ab), 0)
            self.ab_raw = ab
            if self.accum >= 2 or self.accum <= -2:
                self.events.append((t, "ROT+" if self.accum > 0 else "ROT-"))
                self.accum = 0


def make_scenario():
    """0~20초. (t_ms → (pressed, ab)) 파형을 만든다.

    · 짧은 탭(바운스 포함), 롱프레스, 로터리 회전, 긴 무활동 구간을 모두 포함."""
    wave = {}
    ab = 0b11
    for t in range(0, 20001):
        wave[t] = (False, ab)

    def press(t0, dur, bounce=(0, 3, 5)):
        """t0 에 누르고 dur 만큼 유지. bounce 는 접점 채터(ms 오프셋)."""
        for b in bounce:
            if t0 + b <= t0 + dur:
                wave[t0 + b] = (b % 2 == 0, wave[t0 + b][1])
        for t in range(t0 + max(bounce) + 1, t0 + dur):
            wave[t] = (True, wave[t][1])
        for t in range(t0 + dur, min(t0 + dur + 4, 20001)):
            wave[t] = (False, wave[t][1])

    press(1000, 80)          # 짧은 탭
    press(3000, 1400)        # 롱프레스(SETUP 1s 초과)
    press(9000, 60)          # 한참 뒤 짧은 탭 (유휴에서 깨어나야 함)
    press(15000, 120)        # 또 한참 뒤

    # 로터리: 6000ms 부터 1디텐트(=2전이), 6500ms 에 반대 1디텐트
    seq_cw  = [0b01, 0b00, 0b10, 0b11]
    seq_ccw = [0b10, 0b00, 0b01, 0b11]
    for i, v in enumerate(seq_cw):
        for t in range(6000 + i * 12, 6000 + (i + 1) * 12):
            wave[t] = (wave[t][0], v)
    for t in range(6000 + 4 * 12, 6500):
        wave[t] = (wave[t][0], 0b11)
    for i, v in enumerate(seq_ccw):
        for t in range(6500 + i * 12, 6500 + (i + 1) * 12):
            wave[t] = (wave[t][0], v)
    for t in range(6500 + 4 * 12, 20001):
        wave[t] = (wave[t][0], 0b11)
    return wave


def run_fixed(wave):
    """현행: 10ms 고정 폴링."""
    d = Decoder()
    awake = 0
    t = 0
    while t <= 20000:
        d.poll(t, *wave[t])
        awake += 1          # 매 폴이 곧 깨어남
        t += ACTIVE_POLL_MS
    return d.events, awake, 20000 // ACTIVE_POLL_MS


def run_burst(wave):
    """② 버스트 폴링. `~INT` 는 '직전에 읽은 값과 다르면 LOW' 로 모델링."""
    d = Decoder()
    polls = 0
    wakes = 0
    t = 0
    last_read = wave[0]
    last_activity = 0
    active = True           # 부팅 직후는 활성으로 시작
    while t <= 20000:
        d.poll(t, *wave[t])
        polls += 1
        if wave[t] != last_read:
            last_activity = t
        last_read = wave[t]
        # 버튼이 눌려 있으면(롱프레스 평가 필요) 계속 활성
        held = wave[t][0]
        if held or (t - last_activity) < ACTIVE_HOLD_MS:
            active = True
        else:
            active = False

        if active:
            t += ACTIVE_POLL_MS
        else:
            # 유휴: 자기 전에 1회 읽어 INT 해제(위 poll 이 그 역할).
            # 다음 입력 변화 시각까지 잔다 — 없으면 안전망 타임아웃.
            nxt = None
            for u in range(t + 1, min(t + IDLE_POLL_MS + 1, 20001)):
                if wave[u] != last_read:
                    nxt = u
                    break
            t = nxt if nxt is not None else t + IDLE_POLL_MS
            wakes += 1
    return d.events, polls, wakes


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    wave = make_scenario()
    ev_fix, awake_fix, total_fix = run_fixed(wave)
    ev_bur, polls_bur, wakes_bur = run_burst(wave)

    print("=" * 78)
    print(" 버튼 입력 ②: 10ms 고정 폴링 → `~INT` 버스트 폴링 검증")
    print("=" * 78)
    print()

    ok = True

    # ── 1) 이벤트 동일성 — 이게 깨지면 무조건 실패 ────────────────────────
    print("[1] 이벤트 동일성 (하나라도 다르면 실패)")
    print("-" * 78)
    print("  현행 %d건 / 버스트 %d건" % (len(ev_fix), len(ev_bur)))
    same = True
    for i in range(max(len(ev_fix), len(ev_bur))):
        a = ev_fix[i] if i < len(ev_fix) else None
        b = ev_bur[i] if i < len(ev_bur) else None
        an = a[1] if a else "-"
        bn = b[1] if b else "-"
        dt = (b[0] - a[0]) if (a and b) else None
        mark = ""
        if an != bn:
            same = False; mark = "  ★종류 불일치"
        elif dt is not None and abs(dt) > ACTIVE_POLL_MS:
            same = False; mark = "  ★시각 차 %+dms" % dt
        print("   %-8s %-8s %s%s" % (
            ("%dms" % a[0]) if a else "-", an,
            ("%+dms" % dt) if dt is not None else "", mark))
    print("  [%s] 이벤트 종류·순서·시각이 %dms 이내로 동일"
          % ("OK" if same else "X", ACTIVE_POLL_MS))
    ok &= same
    print()

    # ── 2) 절전 효과 ──────────────────────────────────────────────────────
    print("[2] 폴링 횟수 (전력 대리 지표)")
    print("-" * 78)
    print("  현행   : %d 폴 / 20초  (10ms 고정)" % total_fix)
    print("  버스트 : %d 폴 / 20초  (유휴 진입 %d회)" % (polls_bur, wakes_bur))
    red = 100.0 * (1 - polls_bur / float(total_fix))
    print("  → 폴링 **%.1f%% 감소**" % red)
    c2 = red >= 70
    print("  [%s] 70%% 이상 감소" % ("OK" if c2 else "X"))
    ok &= c2
    print()

    # ── 3) 유휴 구간에서 실제로 길게 자는가 ───────────────────────────────
    print("[3] 유휴 구간 수면 길이 (light sleep 문턱 대비)")
    print("-" * 78)
    print("  ①(tick 1000Hz) 문턱 = 3 tick = 3ms")
    print("  현행   : 폴 간격 10ms → 자더라도 매번 ~9ms 조각잠")
    print("  버스트 : 무활동 시 최대 %dms 연속 수면 (안전망 타임아웃)" % IDLE_POLL_MS)
    print("  [OK] 조각잠 %d회/초 → 연속 수면 %.1f회/초 로 감소"
          % (1000 // ACTIVE_POLL_MS, 1000.0 / IDLE_POLL_MS))
    print()

    print("=" * 78)
    print("결과: %s" % ("전부 통과" if ok else "★실패 항목 있음"))
    print()
    print("구현 시 반드시 지킬 것")
    print("-" * 78)
    print("  · 자기 전에 PCF 를 1회 읽어 `~INT` 를 해제한다 — 이미 LOW 인 채로 잠들면")
    print("    falling edge 가 안 생겨 **영영 안 깨어난다**.")
    print("  · IDLE_POLL_MS 안전망을 반드시 남긴다(놓친 INT 자가 치유).")
    print("  · 버튼이 눌려 있는 동안은 무조건 활성 유지 — 롱프레스/15초 재부팅 평가.")
    print("  · 진동 ISR 로도 깨우되, `s_vibe_stuck` 이면 깨우지 않는다")
    print("    (C6 는 잡음으로 33회/초 발사 중이라 그대로 두면 절전이 통째로 무효).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
