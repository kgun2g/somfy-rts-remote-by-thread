#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
채널변경 잠금(_ch_locked) 구간 시뮬레이터  (2026-08-11)
=====================================================

증상
----
"버튼을 눌러 **애니메이션 재생 중**인데 좌/우 채널변경 버튼이 작동한다."
(이전에 '재생이 끝날 때까지 막아달라'고 요청했으나 반영이 덜 됐음)

펌웨어 현재 로직 (somfy_app.c `_ch_locked`)
-------------------------------------------
    locked = s_held_up|down|rot|prog                       # (1) 홀드 중
          or s_ui.state == OLED_STATE_ACTION               # (2) 애니메이션 상태
          or s_ui.action_active
          or now < s_rf_btn_until_us                       # (3) 눌린 뒤 700ms 여운

그리고 oled_ui 쪽 상태 해제 조건 (oled_ui.c:2795)
    if state==ACTION and not action_active:
        if now - action_start_ms >= OLED_ACTION_DISPLAY_MS(2500):
            state = NORMAL

★ `action_start_ms` 는 **누른 시각**이다(뗀 시각이 아니다).
  → 2.5초보다 오래 누르고 있으면, **떼는 순간 이미 2500ms 가 지나 있어**
    상태가 즉시 NORMAL 로 떨어진다. 블라인드를 올리려고 버튼을 길게 누르는 것은
    정상 사용이므로, 실사용에서는 (2)번 조건이 사실상 동작하지 않는다.

또 하나: RF 송신은 버튼을 뗀 뒤에도 이어진다(마지막 job 1~1.5초). 이 구간에
채널이 바뀌면 **엉뚱한 블라인드로 명령이 나간다**. 현재 `_ch_locked` 는 실제
송신 진행 여부(`s_rf_tx_active`)를 보지 않는다.

실행: python sim/tools/ch_lock_sim.py
"""

import sys

MS = 1

# ── 펌웨어 상수 ─────────────────────────────────────────────────────────────
OLED_ACTION_DISPLAY_MS = 2500     # oled_ui.h
CFG_CH_LOCK_MS = 700              # somfy_app.c
RF_JOB_MS = 1200                  # somfy_rts_send 1회 (~1~1.5초)
HOLD_REPEAT_MS = 500              # CFG_BTN_HOLD_REPEAT_MS
UI_TICK_MS = 50                   # _ui_task 주기


class Fw(object):
    """현재 펌웨어 동작 재현. fix=True 면 제안 수정 적용."""

    def __init__(self, hold_ms, fix=False):
        self.hold = hold_ms
        self.fix = fix
        self.state_action = False
        self.action_active = False
        self.action_start = None
        self.rf_btn_until = -1
        self.held = False
        self.rf_busy_until = -1      # RF 워커가 마지막 job 을 끝내는 시각
        self.act_end_ms = None       # ★수정안: 액션이 '끝난' 시각
        self.log = []

    def press(self, t):
        self.held = True
        self.rf_btn_until = t + CFG_CH_LOCK_MS
        self.state_action = True
        self.action_active = True
        self.action_start = t
        self.act_end_ms = None
        self._queue_rf(t)

    def release(self, t):
        self.held = False
        self.action_active = False
        self.act_end_ms = t

    def _queue_rf(self, t):
        start = max(t, self.rf_busy_until)
        self.rf_busy_until = start + RF_JOB_MS

    def tick(self, t):
        # hold_repeat: 누르고 있는 동안 500ms 마다 재송신
        if self.held and self.action_start is not None:
            if (t - self.action_start) > 0 and (t - self.action_start) % HOLD_REPEAT_MS == 0:
                self._queue_rf(t)
        # oled_ui 상태 해제
        if self.state_action and not self.action_active:
            if (t - self.action_start) >= OLED_ACTION_DISPLAY_MS:
                self.state_action = False

    def rf_active(self, t):
        return t < self.rf_busy_until

    def locked(self, t):
        if self.held:
            return True
        if self.state_action or self.action_active:
            return True
        if t < self.rf_btn_until:
            return True
        if self.fix:
            # ── 제안 수정 ──
            # (a) 실제 RF 송신이 진행 중이면 잠금
            if self.rf_active(t):
                return True
            # (b) 액션이 '끝난' 시각 기준으로 애니메이션 표시시간만큼 연장
            if self.act_end_ms is not None and \
               t < self.act_end_ms + OLED_ACTION_DISPLAY_MS:
                return True
        return False

    def anim_visible(self, t):
        """사용자 눈에 애니메이션이 보이는가 (state==ACTION 이면 _render_action)."""
        return self.state_action or self.action_active


def run(hold_ms, fix, span=9000):
    fw = Fw(hold_ms, fix)
    rows = []
    for t in range(0, span + 1, UI_TICK_MS):
        if t == 0:
            fw.press(0)
        if t == hold_ms:
            fw.release(t)
        fw.tick(t)
        rows.append((t, fw.locked(t), fw.anim_visible(t), fw.rf_active(t)))
    return rows


def bar(rows, key):
    """0.5초당 1칸으로 압축한 타임라인 문자열."""
    out = []
    step = 500 // UI_TICK_MS
    for i in range(0, len(rows), step):
        chunk = rows[i:i + step]
        on = sum(1 for r in chunk if r[key])
        out.append("#" if on == len(chunk) else ("+" if on else "."))
    return "".join(out)


def gaps(rows):
    """애니메이션이 보이거나 RF 송신 중인데 잠금이 풀린 구간."""
    bad = []
    cur = None
    for t, lk, anim, rf in rows:
        risky = (anim or rf) and not lk
        if risky and cur is None:
            cur = t
        elif not risky and cur is not None:
            bad.append((cur, t))
            cur = None
    if cur is not None:
        bad.append((cur, rows[-1][0]))
    return bad


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print("=" * 78)
    print(" 채널변경 잠금(_ch_locked) 구간 검증")
    print("=" * 78)
    print("  상수: 애니메이션 표시 %dms / 눌림여운 %dms / RF 1회 %dms / hold 재송신 %dms"
          % (OLED_ACTION_DISPLAY_MS, CFG_CH_LOCK_MS, RF_JOB_MS, HOLD_REPEAT_MS))
    print("  타임라인 1칸 = 0.5초,  # = 계속 참,  + = 일부,  . = 거짓")
    print()

    holds = [200, 1000, 2000, 3000, 5000]
    for fix in (False, True):
        print("─" * 78)
        print(" %s" % ("[수정안] RF 송신중 잠금 + 액션 '종료' 기준 연장"
                       if fix else "[현재] 펌웨어 그대로"))
        print("─" * 78)
        print("  %-9s %-19s %-19s %-19s" % ("누름시간", "잠금", "애니메이션", "RF송신"))
        worst = []
        for h in holds:
            rows = run(h, fix)
            g = gaps(rows)
            print("  %-9s %-19s %-19s %-19s %s"
                  % ("%dms" % h, bar(rows, 1), bar(rows, 2), bar(rows, 3),
                     "" if not g else "<= 구멍"))
            if g:
                worst.append((h, g))
        print()
        if worst:
            print("  ★잠금이 풀렸는데 아직 동작 중인 구간:")
            for h, g in worst:
                for a, b in g:
                    kinds = []
                    rows = run(h, fix)
                    seg = [r for r in rows if a <= r[0] <= b]
                    if any(r[2] for r in seg):
                        kinds.append("애니메이션 재생중")
                    if any(r[3] for r in seg):
                        kinds.append("RF 송신중")
                    print("     누름 %5dms → %5d~%5dms (%.1f초간)  %s"
                          % (h, a, b, (b - a) / 1000.0, " + ".join(kinds)))
        else:
            print("  구멍 없음 — 애니메이션/RF 가 끝날 때까지 항상 잠김")
        print()

    print("=" * 78)
    print("결론")
    print("=" * 78)
    print("  1) `action_start_ms` 가 **누른 시각**이라, %dms 보다 길게 누르면"
          % OLED_ACTION_DISPLAY_MS)
    print("     떼는 순간 애니메이션 상태가 즉시 해제된다 → 잠금이 바로 풀린다.")
    print("     블라인드를 올리려 버튼을 길게 누르는 건 정상 사용이라 실사용에서 늘 발생.")
    print("  2) RF 송신은 버튼을 뗀 뒤에도 마지막 job(%dms)이 남는데," % RF_JOB_MS)
    print("     `_ch_locked` 가 실제 송신 여부를 보지 않는다 → 송신 중 채널이 바뀌면")
    print("     **엉뚱한 블라인드로 명령이 나간다**.")
    print()
    print("  수정: (a) RF 송신 진행 중(s_rf_tx_active)이면 잠금")
    print("        (b) 애니메이션 연장을 '누른 시각'이 아니라 **'뗀 시각'** 기준으로")


if __name__ == "__main__":
    main()
