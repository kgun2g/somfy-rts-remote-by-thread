# -*- coding: utf-8 -*-
"""버튼 길게 누름 → RF 송신 연속성 시뮬레이터.

목적 (2026-08-13 사용자 요청):
  "위/아래/정지(MY) 버튼 1초 이상 누르면 첫 신호 발생 후 0.5초 정도 텀이 생기고
   다음부터 연속으로 나온다" → 텀을 없앤다.

정품 실측 근거 (D:\\RTL_SDR\\sdrsharp-x64\\somfy_rts_447, scratchpad/hold_pattern.py):
  ON 길이 330ms(탭) → 400 → 700 → 900 → 1360ms 가 **끊김 없이 한 덩어리**.
  간격이 보이는 캡처(340[1390]340)는 뗐다 다시 누른 별개 누름.
  ⇒ 정품은 누르는 동안 계속 쏜다. 목표: hold 중 gap 0.

모델링 대상 (main/somfy_app.c, main/somfy_rts.c 실제 구조):
  * _btn_event_cb   : s_held_*, s_action_press_us, s_last_sent_cmd, somfy_rts_abort
  * _hold_repeat_task : CFG_BTN_HOLD_REPEAT_MS(500ms) 주기, HOLD_REPEAT_START_MS(500ms) 게이트
  * _rf_worker_task : **직렬** — job 하나가 끝나야 다음 job 시작 (여기서 유출이 생긴다)
  * somfy_rts_send  : min_loops=SOMFY_REPEAT_COUNT(2) 는 abort 무시하고 무조건 송신,
                      이후 **프레임마다 1번만** somfy_rts_abort 검사 (프레임 ≈ 143ms)
                      → abort 를 짧게 pulse 하면 **놓친다**. 이게 설계의 핵심 함정.
"""
import sys

FRAME_MS = 143            # somfy_rts.c: max_loops = min + hold_ms/143
MIN_LOOPS = 2             # SOMFY_REPEAT_COUNT — abort 무시 구간 (2026-08-14 3→2, 정품 실측)
MAX_HOLD_MS = 15000       # CFG_BTN_MAX_HOLD_MS
HOLD_REPEAT_MS = 500      # CFG_BTN_HOLD_REPEAT_MS
HOLD_START_MS = 500       # HOLD_REPEAT_START_MS
COMBO_POLL_MS = 20        # COMBO_SWITCH_POLL_MS — 전환 대기 중에만 쓰는 주기
IFG_OK_MS     = 30        # 프레임간 정상 무신호(≈30ms) 이내면 끊김 아님

UP, DOWN, MY, UP_DOWN, MY_UP, MY_DOWN, NONE = 'UP', 'DOWN', 'MY', 'UP_DOWN', 'MY_UP', 'MY_DOWN', 0


class Sim:
    def __init__(self, design):
        self.design = design          # 'old' | 'new'
        self.t = 0
        self.held = {'up': False, 'down': False, 'rot': False}
        self.press_us = None
        self.last_sent = NONE
        self.abort = False
        self.queue = []               # [(cmd, hold_ms, abortable)]
        self.busy = None              # 진행 중 job
        self.busy_left = 0            # 남은 프레임 수
        self.busy_done = 0            # 이미 보낸 프레임 수
        self.tx = []                  # (t_start, t_end, cmd) 실제 전파 구간
        self.next_hold_tick = HOLD_REPEAT_MS
        self.combo_wait = None        # new 설계의 '조합변경 대기' 상태
        self.next_combo_poll = 0

    # ── 헬퍼 ────────────────────────────────────────────────
    def combo(self):
        h = self.held
        if h['up'] and h['down']:   return UP_DOWN
        if h['up'] and h['rot']:    return MY_UP
        if h['down'] and h['rot']:  return MY_DOWN
        if h['up']:                 return UP
        if h['down']:               return DOWN
        if h['rot']:                return MY
        return NONE

    def enqueue(self, cmd, hold_ms, abortable):
        self.queue.append([cmd, hold_ms, abortable])

    def rf_inflight(self):
        return self.busy is not None or len(self.queue) > 0

    # ── 버튼 이벤트 ──────────────────────────────────────────
    def press(self, which):
        first = not any(self.held.values())
        self.held[which] = True
        self.abort = False                       # 실제 코드: 새 누름 → 중단요청 해제
        cmd = {'up': UP, 'down': DOWN, 'rot': MY}[which]
        if first:
            self.press_us = self.t
            self.last_sent = cmd
            if self.design == 'old':
                self.enqueue(cmd, 0, False)              # 3프레임만, 중단불가
            else:
                # 이미 같은 cmd 가 진행/대기 중이면 새 job 을 만들지 않는다
                # (abort 해제만으로 그 job 이 이어진다 — 중복 job 유출 방지)
                if not (self.rf_inflight() and self.busy_cmd() == cmd):
                    self.enqueue(cmd, MAX_HOLD_MS, True)  # 뗄 때까지 연속

    def busy_cmd(self):
        if self.busy: return self.busy[0]
        if self.queue: return self.queue[-1][0]
        return NONE

    def release(self, which):
        self.held[which] = False
        if not any(self.held.values()):
            self.press_us = None
            self.last_sent = NONE
            self.abort = True

    # ── _hold_repeat_task (500ms 주기) ───────────────────────
    def hold_tick(self):
        if self.press_us is None or not any(self.held.values()):
            return
        if self.t - self.press_us < HOLD_START_MS:
            return
        cmd = self.combo()
        if self.design == 'old':
            keep = (cmd == self.last_sent)
            self.last_sent = cmd
            self.abort = False
            self.enqueue(cmd, 0, True)           # 500ms 마다 3프레임 job
            return
        # ── new: 조합이 바뀔 때만 재송신 ──
        if cmd == self.last_sent:
            return                                # PRESS 송신이 계속 진행 중
        self.last_sent = cmd
        self.abort = True                         # 진행 중 송신 종료 요청 (level 유지!)
        self.combo_wait = cmd                     # idle 될 때까지 기다렸다 재송신

    def combo_wait_tick(self):
        """new 설계: abort 를 pulse 하지 않고, 진행 중 job 이 **실제로 끝날 때까지**
        기다린 뒤 새 조합을 송신한다. (프레임 143ms 단위로만 abort 를 보므로
        20ms 짜리 pulse 는 놓친다 — 타이밍 추측 금지)"""
        if self.combo_wait is None:
            return
        if self.rf_inflight():
            return                                # 아직 진행 중 — abort 유지한 채 대기
        if self.press_us is None:                 # 기다리는 사이 다 뗐다 → 송신 안 함
            self.combo_wait = None
            return
        cmd = self.combo()
        self.combo_wait = None
        if cmd == NONE:
            return
        self.last_sent = cmd
        self.abort = False
        self.enqueue(cmd, MAX_HOLD_MS, True)

    # ── 직렬 RF worker ───────────────────────────────────────
    #  프레임 회계: t 에 job 을 시작하면 첫 프레임이 [t, t+143] 을 점유한다.
    #  다음 프레임 경계(t+143)에서만 abort 를 본다 — 짧은 pulse 는 놓친다.
    def worker_start(self):
        if self.busy is not None or not self.queue:
            return False
        cmd, hold, abortable = self.queue.pop(0)
        hold = min(hold, MAX_HOLD_MS)
        self.busy = (cmd, hold, abortable)
        self.busy_left = MIN_LOOPS + hold // FRAME_MS
        self.busy_done = 1                       # 첫 프레임 송신 중
        self.tx.append([self.t, self.t + FRAME_MS, cmd])
        return True

    def worker_frame_edge(self):
        """프레임 경계 — 계속할지 판정. True 면 프레임 1개 더 송신."""
        cmd, hold, abortable = self.busy
        if self.busy_done >= self.busy_left:
            self.busy = None; return False
        if self.busy_done >= MIN_LOOPS and abortable and self.abort:
            self.busy = None; return False
        self.busy_done += 1
        self.tx[-1][1] = self.t + FRAME_MS
        return True

    # ── 실행 ────────────────────────────────────────────────
    def run(self, events, total_ms):
        """events = [(t_ms, 'press'/'release', 'up'/'down'/'rot')]"""
        evs = sorted(events)
        ei = 0
        next_frame = None
        self.held_log = []                       # (t, 눌림여부) — 공백 판정용
        while self.t <= total_ms:
            while ei < len(evs) and evs[ei][0] <= self.t:
                _, kind, which = evs[ei]
                (self.press if kind == 'press' else self.release)(which)
                ei += 1
            if self.t >= self.next_hold_tick:
                self.hold_tick()
                self.next_hold_tick += HOLD_REPEAT_MS
            if self.t >= self.next_combo_poll:
                self.combo_wait_tick()
                self.next_combo_poll = self.t + COMBO_POLL_MS
            if self.busy is not None and self.t >= next_frame:
                if self.worker_frame_edge():
                    next_frame = self.t + FRAME_MS
            if self.busy is None and self.worker_start():
                next_frame = self.t + FRAME_MS
            self.held_log.append(any(self.held.values()))
            self.t += 1
        # 인접 구간 병합 (틈 없이 이어지면 연속 송신)
        merged = []
        for s, e, c in self.tx:
            if merged and s - merged[-1][1] <= 1:
                merged[-1][1] = max(merged[-1][1], e)
                if c != merged[-1][2]:
                    merged[-1][2] = '%s>%s' % (merged[-1][2], c)
            else:
                merged.append([s, e, c])
        return merged


def report(name, design, events, total, hold_span):
    sim = Sim(design)
    runs = sim.run(events, total)
    press_t, rel_t = hold_span
    held = sim.held_log
    gaps = []
    for i in range(1, len(runs)):
        g0, g1 = runs[i - 1][1], runs[i][0]
        if g1 <= g0:
            continue
        # **버튼이 실제로 눌려 있던 동안**의 공백만 결함이다.
        # (탭과 탭 사이 공백은 정상 — 정품도 뗐다 누르면 끊긴다)
        if g1 - g0 <= IFG_OK_MS:
            continue          # 프레임간 정상 무신호 수준
        if all(held[t] for t in range(g0, min(g1, len(held)))):
            gaps.append((g0, g1 - g0))
    leak = max(0, (runs[-1][1] - rel_t)) if runs else 0
    ontot = sum(e - s for s, e, _ in runs)
    print('  %-5s ON구간 %d개  총 %4dms  | %s' %
          (design, len(runs), ontot,
           ' '.join('%d~%d(%s)' % (s, e, c) for s, e, c in runs[:6])))
    if gaps:
        print('        ✗ hold 중 공백 %d개: %s' %
              (len(gaps), ', '.join('t=%dms %dms' % g for g in gaps)))
    else:
        print('        ✓ hold 중 공백 없음')
    print('        뗀 뒤 유출 %dms %s' % (leak, '✓' if leak <= FRAME_MS * MIN_LOOPS + 50 else '✗'))
    return gaps, leak


CASES = [
    ('① UP 1초 누름',        [(0, 'press', 'up'), (1000, 'release', 'up')], 3000, (0, 1000)),
    ('② UP 3초 누름',        [(0, 'press', 'up'), (3000, 'release', 'up')], 5000, (0, 3000)),
    ('③ UP 짧은 탭(120ms)',  [(0, 'press', 'up'), (120, 'release', 'up')], 2000, (0, 120)),
    ('④ MY 2초 누름',        [(0, 'press', 'rot'), (2000, 'release', 'rot')], 4000, (0, 2000)),
    ('⑤ UP+DOWN 동시 2초',   [(0, 'press', 'up'), (60, 'press', 'down'),
                              (2000, 'release', 'up'), (2040, 'release', 'down')], 5000, (0, 2000)),
    ('⑥ UP 누르고 도중 DOWN 추가', [(0, 'press', 'up'), (800, 'press', 'down'),
                              (2500, 'release', 'down'), (2560, 'release', 'up')], 5000, (0, 2500)),
    ('⑦ 연타(탭 3회)',       [(0, 'press', 'up'), (120, 'release', 'up'),
                              (300, 'press', 'up'), (420, 'release', 'up'),
                              (600, 'press', 'up'), (720, 'release', 'up')], 3000, (0, 720)),
]

def phase_sweep():
    """_hold_repeat_task 는 누름과 **위상이 맞지 않는** 자유주행 500ms 태스크다.
    위상 0~499ms 를 전부 훑어 최악 공백을 구한다 —
    사용자가 말한 '0.5초 텀'이 여기서 나온다."""
    print('── _hold_repeat_task 위상 0~499ms 스윕 (UP 3초 누름) ──')
    evs = [(0, 'press', 'up'), (3000, 'release', 'up')]
    out = {}
    for design in ('old', 'new'):
        worst, worst_ph, allgaps = 0, 0, 0
        for ph in range(0, 500, 10):
            sim = Sim(design)
            sim.next_hold_tick = ph            # 위상 어긋남
            runs = sim.run(evs, 5000)
            held = sim.held_log
            for i in range(1, len(runs)):
                g0, g1 = runs[i - 1][1], runs[i][0]
                if g1 - g0 > IFG_OK_MS and all(held[t] for t in range(g0, min(g1, len(held)))):
                    allgaps += 1
                    if g1 - g0 > worst:
                        worst, worst_ph = g1 - g0, ph
        out[design] = (worst, allgaps)
        print('  %-5s 최악 공백 %3dms (위상 %dms)   공백 총 %d개' %
              (design, worst, worst_ph, allgaps))
    return out


ADVERSARIAL = [
    # 조합 변경 대기 중에 전부 떼면 → 뒤늦은 유출 송신이 있으면 안 된다
    ('⑧ 조합변경 직후 즉시 뗌',
     [(0, 'press', 'up'), (600, 'press', 'down'),
      (1010, 'release', 'up'), (1015, 'release', 'down')], 4000, (0, 1015)),
    # 조합 진입/이탈 반복 — job 이 쌓여 뗀 뒤 오래 새는지
    ('⑨ 조합 넣었다 뺐다 반복',
     [(0, 'press', 'up'), (600, 'press', 'down'), (1200, 'release', 'down'),
      (1800, 'press', 'down'), (2400, 'release', 'down'),
      (3000, 'release', 'up')], 6000, (0, 3000)),
    # MY+UP 조합 (동시작동 — 모터 limit 설정용)
    ('⑩ MY+UP 조합 2초',
     [(0, 'press', 'rot'), (80, 'press', 'up'),
      (2000, 'release', 'up'), (2050, 'release', 'rot')], 5000, (0, 2000)),
]

if __name__ == '__main__':
    bad = 0
    for name, evs, total, span in CASES + ADVERSARIAL:
        print(name)
        for design in ('old', 'new'):
            gaps, leak = report(name, design, evs, total, span)
            if design == 'new' and (gaps or leak > FRAME_MS * MIN_LOOPS + 50):
                bad += 1
        print()
    sweep = phase_sweep()
    if sweep['new'][1] != 0:
        bad += 1
    print()
    print('=== new 설계 실패 케이스: %d개 ===' % bad)
    sys.exit(1 if bad else 0)
