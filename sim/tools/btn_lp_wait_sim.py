# -*- coding: utf-8 -*-
"""버튼 태스크(LP 경로) 대기 구조 — 기존(이중 대기) vs 새(단일 대기 + 디바운스 시한) 비교.

왜 (2026-10-18, 감사 보고서 POWER_SAVING_FULL_CODE_AUDIT_2026-10-16 F04·F03)
────────────────────────────────────────────────────────────────────
button_handler.c 의 LP 경로는 통지 대기(유휴 100ms / 눌림 10ms)에서 돌아온 뒤 **공통
vTaskDelay(10ms)** 를 한 번 더 한다. 그래서 유휴 중 110ms 마다 **두 번** 깨어난다.
batlog 해석기가 "남의 몫(스택) 8.4회/초"로 표시하던 것의 대부분이 이 두 번째 깨어남이다.
그리고 첫 변화를 본 뒤(디바운스 미확정)에도 any_held 가 확정 상태만 보므로 100ms 를 또
기다린다 → 누름 확정이 ~120ms 늦다.

새 구조: 통지 대기 **한 곳**에서, 타임아웃 = min(유휴 100 / 눌림 10, 미확정 디바운스 남은 시간+1).
뒤의 10ms 지연은 LP 경로에서 건너뛴다(HP 폴링 경로는 그대로).

★첫 실행이 잡은 것 — 래치를 **읽는 순간** 지우면(기존 코드) 짧은 탭이 사라진다.
  HP 가 막 깨어 있는 창(일·절전 진입)에 누름이 오면 LP 통지가 유실되고(F05), 다음 읽기에서
  래치로 누름을 봐도 그 자리에서 래치를 지운다(F06). 그 사이 손을 떼면 디바운스(20ms 뒤 재확인)
  때 이미 뗀 상태라 **누름이 확정되지 않는다**. 기존 구조도 20~50ms 탭의 3~8% 를 이렇게 놓쳤다.
  → 새 구조는 래치로 본 누름을 **확정될 때까지 붙잡는다**(s_lat_hold). 이미 확정된 눌림에는
    붙잡지 않는다(안 그러면 영영 안 떼어진다).

★2026-10-18 2차 — F05(HP 가 깨어 있을 때 사라지던 LP 통지)를 **유휴 훅**으로 막고 유휴 대기를 1000ms 로.
  유휴 훅: HP 가 할 일을 마치고 잠들기 직전마다 LP 의 seq 를 보고, 태스크가 본 적 없는 변화면 통지한다.
  여기서는 "HP 바쁨 창이 끝나는 순간"에 그 검사를 한다. seq 는 태스크가 PCF 를 읽을 때(상태보다 먼저)도 갱신.
  LP 정지 판정은 "연속 2회 읽기 + 마지막 변화 뒤 500ms" — 유휴 1초면 ~2초, 눌림 중이면 0.5초.

★이 시뮬레이터가 재현하는 것 (펌웨어와 같게)
  · LP: 5ms 마다 PCF 를 읽는다. 바뀌면 pcf_state 갱신 + HP 깨움 요청. 2연속 눌림이면 래치.
  · HP 가 **자고 있을 때만** 깨움 요청이 절전 복귀 콜백 → 버튼 태스크 통지로 이어진다.
    깨어 있으면(다른 일 처리·절전 진입 중) 요청이 사라진다(보고서 F05 — 이번엔 안 고침).
  · 다른 태스크(메인 10Hz·OLED 2Hz·hold 2Hz)의 깨어남이 그 창을 만든다.
  · 읽기: pcf_state 에 래치를 합치고 본 래치를 지운다. 디바운스 20ms(두 번 읽어 같아야 확정).
  · LP 정지 감지: poll_cnt 가 연속 8회 읽기 동안 그대로면 정지로 본다 — 읽기가 촘촘해져도
    거짓 판정이 없어야 한다(새 구조는 시간 조건 200ms 도 같이 건다).
못 하는 것: 실제 전하(µC) — 깨어남 수만 센다. 회당 전하는 lsfloor wake 실측(~50µC)을 쓴다.

    python sim/tools/btn_lp_wait_sim.py
"""
import random
import sys

DT = 0.25                    # ms
LP_POLL = 5.0                # LP_POLL_US 5000
DEBOUNCE = 20.0              # CFG_BTN_DEBOUNCE_MS
ACTIVE = 10.0                # BTN_ACTIVE_POLL_MS
IDLE_OLD = 100.0             # 기존 BTN_LP_IDLE_POLL_MS
IDLE_NEW = 1000.0            # ★2026-10-18 2차: F05(유휴 훅) 해결 뒤 1000ms
MIN_WAIT = 2.0               # 새 구조 최소 대기(BTN_LP_MIN_WAIT_MS)
STALE_POLLS = 8              # 기존 LP_STALE_POLLS(연속 8회)
STALE_MIN_MS = 500.0         # ★2차: 새 구조는 "연속 2회 + 마지막 변화 뒤 500ms"(LP_STALE_MIN_MS/READS)
STALE_MIN_READS = 2
HP_BUSY = 2.0                # HP 가 한 번 깨면 이만큼 깨어 있다(일 + 절전 진입) — 그동안 LP 요청 유실
BG = ((100.0, 'main'), (500.0, 'oled'), (500.0, 'hold'))   # 다른 태스크 주기(ms)


ROT_MIN = 30.0               # ROT_MIN_INTERVAL_MS
ROT_PEND_MAX = 8             # 새 구조: 남겨 둘 디텐트 상한(기존 '한 번에 8' 과 같은 폭주 방어)


class Sim:
    def __init__(self, variant, phys, seed=1, t_end=1000.0, lp_dead_at=None, rot_fn=None):
        self.v = variant
        self.phys = phys                     # t(ms) -> 0 눌림 / 1 뗌
        # ★2026-10-18 로터리(F07): rot_fn(t) = 그 시각까지의 누적 디텐트(+CW / −CCW)
        self.rot_fn = rot_fn
        self.rot_lp_seen = 0
        self.rot_delta = 0                   # LP 공유 rot_delta
        self.rot_pend = 0                    # 새 구조: 아직 전달 안 한 디텐트
        self.rot_last = -1e9
        self.rot_events = []                 # (t, +1/-1)
        self.rng = random.Random(seed)
        self.t = 0.0
        self.t_end = t_end
        self.lp_dead_at = lp_dead_at
        # LP
        self.lp_next = self.rng.uniform(0, LP_POLL)
        self.pcf_state, self.last, self.latch, self.prev_low, self.poll_cnt = 1, 1, 0, False, 0
        self.seq = 0                         # LP seq — 상태 변화·로터리 디텐트마다 +1
        self.seq_seen = 0                    # 새 구조: 유휴 훅·태스크가 본 seq
        self.idle_hook_notify = 0
        # HP
        self.hp_busy_until = 0.0
        self.was_busy = False
        self.bg_next = [self.rng.uniform(0, p) for p, _ in BG]
        # 버튼 태스크
        self.state = 'body'
        self.deadline = 0.0
        self.notified = False
        self.raw, self.last_state, self.deb_ms = 1, 1, -1e9
        self.press_t = None
        self.lat_hold = 0                    # 새 구조: 래치로 본 누름을 확정까지 붙잡는다
        self.events = []                     # (t, 'press'|'release')
        self.btn_wakes = 0                   # 버튼 태스크 자신의 타임아웃으로 HP 를 깨운 횟수
        self.lp_wakes = 0
        self.lost_notify = 0
        # LP 정지 감지
        self.cnt_prev, self.stale, self.chg_t = -1, 0, 0.0
        self.lp_declared_dead = False
        self.dead_t = None

    def hp_asleep(self):
        return self.t >= self.hp_busy_until

    def _wake_hp(self, why):
        self.hp_busy_until = max(self.hp_busy_until, self.t + HP_BUSY)

    def _lp_poll(self):
        if self.lp_dead_at is not None and self.t >= self.lp_dead_at:
            return                            # LP 정지(시험용) — poll_cnt 가 얼어붙는다
        rx = self.phys(self.t)
        self.poll_cnt += 1
        low = (rx == 0)
        if low and self.prev_low:
            self.latch = 1
        self.prev_low = low
        rot_chg = False
        if self.rot_fn:
            c = self.rot_fn(self.t)
            if c != self.rot_lp_seen:
                self.rot_delta += c - self.rot_lp_seen
                self.rot_lp_seen = c
                rot_chg = True                 # A/B 비트가 바뀌었으니 rx != last 와 같다(HP 깨움 요청)
        if rx != self.last or rot_chg:
            self.last = rx
            self.pcf_state = rx
            self.seq += 1
            if self.hp_asleep():              # 절전 복귀 콜백(ULP) → 통지
                self.lp_wakes += 1
                self._wake_hp('ulp')
                self.notified = True
            else:
                self.lost_notify += 1         # HP 가 깨어 있어 콜백이 안 돈다(F05)

    def _read(self):
        """_pcf8574_read() — LP 공유값 + 래치 소비 + 정지 감지."""
        if self.v == 'new':
            self.seq_seen = self.seq          # ★상태보다 먼저 seq 를 적는다(F05)
        cnt = self.poll_cnt
        if cnt != self.cnt_prev:
            self.cnt_prev, self.stale, self.chg_t = cnt, 0, self.t
        elif self.stale < STALE_POLLS:
            self.stale += 1
        if self.v == 'new':
            dead = self.stale >= STALE_MIN_READS and (self.t - self.chg_t) >= STALE_MIN_MS
        else:
            dead = self.stale >= STALE_POLLS
        if dead and not self.lp_declared_dead:
            self.lp_declared_dead = True
            self.dead_t = self.t
        st = self.pcf_state
        if self.latch:
            self.latch = 0
            st = 0
            if self.v == 'new' and self.last_state == 1:   # 아직 확정 안 된 눌림만 붙잡는다
                self.lat_hold = 1
        if self.v == 'new' and self.lat_hold:
            st = 0
        return st

    def _rotary(self):
        """button_handler.c LP 경로 로터리 — 기존: 읽은 만큼 차감하고 30ms 제한으로 1개만 / 새: 남겨 뒀다 차례로."""
        d = self.rot_delta
        if self.v == 'old':
            if d:
                self.rot_delta -= d
                n = min(abs(d), 8)
                for _ in range(n):                       # 같은 now → 첫 1개만 통과
                    if self.t - self.rot_last >= ROT_MIN:
                        self.rot_last = self.t
                        self.rot_events.append((self.t, 1 if d > 0 else -1))
            return
        if d:
            self.rot_delta -= d
            self.rot_pend = max(-ROT_PEND_MAX, min(ROT_PEND_MAX, self.rot_pend + d))
        if self.rot_pend and self.t - self.rot_last >= ROT_MIN:
            sgn = 1 if self.rot_pend > 0 else -1
            self.rot_pend -= sgn
            self.rot_last = self.t
            self.rot_events.append((self.t, sgn))

    def _body(self):
        if self.rot_fn:
            self._rotary()
        cur = self._read()
        if cur != self.raw:
            self.raw = cur
            self.deb_ms = self.t
        if self.t - self.deb_ms >= DEBOUNCE:
            stable = self.raw
            if stable == 0 and self.last_state == 1:
                self.last_state = 0
                self.press_t = self.t
                self.lat_hold = 0                       # 확정 — 붙잡기 해제
                self.events.append((self.t, 'press'))
            elif stable == 1 and self.last_state == 0:
                self.last_state = 1
                self.events.append((self.t, 'release'))

    def _timeout(self):
        held = (self.last_state == 0)
        tmo = ACTIVE if held else (IDLE_NEW if self.v == 'new' else IDLE_OLD)
        if self.v == 'new' and self.raw != self.last_state:      # 미확정 변화 — 디바운스 시한까지만
            left = max(0.0, DEBOUNCE - (self.t - self.deb_ms))
            tmo = min(tmo, left + 1.0)
        if self.v == 'new' and self.rot_pend:                    # 남은 디텐트 — 다음 30ms 창까지만
            left = max(0.0, ROT_MIN - (self.t - self.rot_last))
            tmo = min(tmo, left + 1.0)
        if self.v == 'new':
            tmo = max(tmo, MIN_WAIT)
        return tmo

    def run(self):
        while self.t < self.t_end:
            if self.t >= self.lp_next:
                self._lp_poll()
                self.lp_next += LP_POLL
            for i, (per, _) in enumerate(BG):
                if self.t >= self.bg_next[i]:
                    self._wake_hp('bg')
                    self.bg_next[i] += per
            # ★F05 유휴 훅 — HP 바쁨 창이 끝나 잠들려는 순간, 태스크가 못 본 LP 변화가 있으면 통지
            busy = not self.hp_asleep()
            if self.v == 'new' and self.was_busy and not busy and self.seq != self.seq_seen:
                self.seq_seen = self.seq
                self.idle_hook_notify += 1
                self.notified = True
                self._wake_hp('idlehook')
                busy = True
            self.was_busy = busy
            # 버튼 태스크 상태기계
            if self.state == 'body':
                self._wake_hp('task')
                self._body()
                self.state = 'nwait'
                self.deadline = self.t + self._timeout()
            elif self.state == 'nwait':
                if self.notified or self.t >= self.deadline:
                    if not self.notified and self.hp_asleep():
                        self.btn_wakes += 1
                    self.notified = False
                    if self.v == 'old':
                        self.state = 'delay'
                        self.deadline = self.t + ACTIVE
                    else:
                        self.state = 'body'
                        continue                      # 같은 시각에 바로 본문
            elif self.state == 'delay':
                if self.t >= self.deadline:
                    if self.hp_asleep():
                        self.btn_wakes += 1
                    self.state = 'body'
                    continue
            self.t += DT
        return self


def tap(t0, dur, bounce=0.0, rng=None):
    """t0 에 눌러 dur 동안 유지. bounce>0 이면 누름·뗌 가장자리에서 0.3ms 간격으로 흔들린다."""
    edges = []
    if bounce and rng:
        for edge in (t0, t0 + dur):
            k = rng.randint(2, 6)
            for j in range(k):
                edges.append(edge + j * bounce / k)

    def f(t):
        base = 0 if t0 <= t < t0 + dur else 1
        for e in edges:
            if e <= t < e + 0.3:
                return 1 - base
        return base
    return f


def main():
    ok = True
    print('== ① 유휴 60초 — 버튼 태스크가 스스로 HP 를 깨운 횟수 ==')
    idle = {}
    for v in ('old', 'new'):
        s = Sim(v, lambda t: 1, seed=3, t_end=60000.0).run()
        idle[v] = s.btn_wakes / 60.0
        print('  %-4s  %.2f 회/초   (LP 정지 오판: %s)' % (v, idle[v], s.lp_declared_dead))
        ok &= not s.lp_declared_dead
    ok &= abs(idle['old'] - 18.2) < 1.0 and abs(idle['new'] - 1.0) < 0.2   # ★2차: 1000ms 유휴
    print('  → 줄어든 깨어남 %.2f 회/초' % (idle['old'] - idle['new']))

    print('== ② 짧은 탭 검출률(무작위 위상 400회) · 누름 확정 지연 ==')
    print('  탭(ms)   old 검출  new 검출   old 지연(중앙/최대)   new 지연(중앙/최대)')
    for dur in (6, 8, 10, 12, 15, 20, 30, 50, 100, 300):
        res = {}
        for v in ('old', 'new'):
            det, lat = 0, []
            for k in range(400):
                rng = random.Random(1000 * dur + k)
                t0 = 200.0 + rng.uniform(0, 110.0)
                s = Sim(v, tap(t0, dur), seed=k, t_end=t0 + dur + 400.0).run()
                pr = [e for e in s.events if e[1] == 'press']
                rl = [e for e in s.events if e[1] == 'release']
                ok &= not s.lp_declared_dead
                ok &= len(pr) <= 1 and len(rl) <= 1      # 한 번 누름이 두 번이 되면 안 된다
                if pr and rl:
                    det += 1
                    lat.append(pr[0][0] - t0)
            lat.sort()
            res[v] = (det / 400.0, lat[len(lat) // 2] if lat else float('nan'), lat[-1] if lat else float('nan'))
        print('  %5d   %6.1f%%   %6.1f%%       %5.1f / %5.1f          %5.1f / %5.1f'
              % (dur, 100 * res['old'][0], 100 * res['new'][0], res['old'][1], res['old'][2],
                 res['new'][1], res['new'][2]))
        ok &= res['new'][0] >= res['old'][0] - 0.02           # 새 구조가 더 놓치면 안 된다
        if dur >= 30:
            ok &= res['new'][0] == 1.0
            ok &= res['new'][1] < res['old'][1]               # 누름 확정이 빨라져야 한다
            ok &= res['new'][2] < 45.0                        # ★2차: F05 — 통지 유실로 타임아웃까지 가는 최악이 없어야

    print('== ③ 단발 글리치(LP 한 표본만 눌림) — 누름이 생기면 안 된다 ==')
    for v in ('old', 'new'):
        bad = 0
        for k in range(400):
            rng = random.Random(77 + k)
            t0 = 200.0 + rng.uniform(0, 110.0)
            s = Sim(v, tap(t0, 3.0), seed=k, t_end=800.0).run()
            bad += any(e[1] == 'press' for e in s.events)
        print('  %-4s  글리치 3ms → 누름 발생 %d/400' % (v, bad))
        # 3ms 글리치는 LP 한 표본에만 걸린다 — 래치(2연속)는 막지만 raw 는 한 번 보일 수 있다.
        # 디바운스가 20ms 안정을 요구하므로 누름으로 확정되면 안 된다.
        ok &= bad == 0

    print('== ④ 채터링(누름·뗌 가장자리 2~6회 흔들림) — 정확히 누름 1·뗌 1 ==')
    for v in ('old', 'new'):
        badc = 0
        for k in range(300):
            rng = random.Random(500 + k)
            t0 = 200.0 + rng.uniform(0, 110.0)
            s = Sim(v, tap(t0, 120.0, bounce=4.0, rng=rng), seed=k, t_end=900.0).run()
            pr = sum(1 for e in s.events if e[1] == 'press')
            rl = sum(1 for e in s.events if e[1] == 'release')
            badc += not (pr == 1 and rl == 1)
            ok &= not s.lp_declared_dead
        print('  %-4s  어긋난 경우 %d/300' % (v, badc))
        ok &= badc == 0 if v == 'new' else True

    print('== ⑤ 연타(80ms 누름, 80ms 간격 × 5) — 5번 모두 ==')
    for v in ('old', 'new'):
        miss = 0
        for k in range(200):
            rng = random.Random(900 + k)
            t0 = 200.0 + rng.uniform(0, 110.0)
            taps = [tap(t0 + i * 160.0, 80.0) for i in range(5)]
            s = Sim(v, lambda t, taps=taps: min(f(t) for f in taps), seed=k, t_end=t0 + 1400.0).run()
            miss += 5 - sum(1 for e in s.events if e[1] == 'press')
        print('  %-4s  놓친 누름 %d/1000' % (v, miss))
        ok &= miss == 0 if v == 'new' else True

    print('== ⑥ 롱프레스 2초 — 뗌 확정 시각 오차(실제 뗌 대비) ==')
    for v in ('old', 'new'):
        errs = []
        for k in range(100):
            rng = random.Random(1300 + k)
            t0 = 200.0 + rng.uniform(0, 110.0)
            s = Sim(v, tap(t0, 2000.0), seed=k, t_end=t0 + 2600.0).run()
            rl = [e for e in s.events if e[1] == 'release']
            if rl:
                errs.append(rl[0][0] - (t0 + 2000.0))
        errs.sort()
        print('  %-4s  뗌 확정 지연 중앙 %.1f / 최대 %.1f ms' % (v, errs[len(errs) // 2], errs[-1]))

    print('== ⑧ 로터리(F07) — 빠르게 돌린 디텐트가 전부 오는가, 간격 30ms 이상, 되돌림은 상쇄 ==')

    def spin(t0, n, gap, sign=1):
        return lambda t: sign * max(0, min(n, int((t - t0) // gap) + 1)) if t >= t0 else 0
    # 기대: 느림·빠름 5칸은 전부. 아주 빠른 12칸(36ms)은 상한이 **쌓아 둔 칸 수**(8)에 걸리므로
    #  먼저 내보낸 1~2칸 + 남긴 8칸 = 9~10칸이 맞다(전부는 아니지만 기존 1~2칸보다 훨씬 많다).
    for name, mk, want in (('느리게 5칸(150ms 간격)', lambda t0: spin(t0, 5, 150.0), (5, 5)),
                           ('빠르게 5칸(8ms 간격)', lambda t0: spin(t0, 5, 8.0), (5, 5)),
                           ('아주 빠르게 12칸(3ms 간격)', lambda t0: spin(t0, 12, 3.0), (8, 12))):
        res = {}
        for v in ('old', 'new'):
            got, gaps_ok = [], True
            for k in range(200):
                rng = random.Random(4000 + k)
                t0 = 200.0 + rng.uniform(0, 110.0)
                s = Sim(v, lambda t: 1, seed=k, t_end=t0 + 1500.0, rot_fn=mk(t0)).run()
                got.append(sum(e[1] for e in s.rot_events))
                ts = [e[0] for e in s.rot_events]
                gaps_ok &= all(b - a >= ROT_MIN - 1e-6 for a, b in zip(ts, ts[1:]))
                ok &= not s.lp_declared_dead
            got.sort()
            res[v] = (got[0], got[len(got) // 2], got[-1], gaps_ok)
        print('  %-24s old 전달 최소/중앙/최대 %d/%d/%d   new %d/%d/%d   (기대 %d~%d, new 간격≥30ms %s)'
              % (name, *res['old'][:3], *res['new'][:3], want[0], want[1], res['new'][3]))
        ok &= want[0] <= res['new'][0] and res['new'][2] <= want[1] and res['new'][3]

    def back_forth(t0):
        def f(t):
            if t < t0:
                return 0
            k = int((t - t0) // 8.0) + 1                  # 8ms 마다 1칸: CW 4칸 뒤 CCW 4칸
            return k if k <= 4 else max(0, 8 - k)
        return f
    nets = []
    for k in range(200):
        rng = random.Random(6000 + k)
        t0 = 200.0 + rng.uniform(0, 110.0)
        s = Sim('new', lambda t: 1, seed=k, t_end=t0 + 1500.0, rot_fn=back_forth(t0)).run()
        nets.append(sum(e[1] for e in s.rot_events))
    print('  CW 4칸 → CCW 4칸(빠르게)   new 순이동 최소/최대 %d/%d  (기대 0)' % (min(nets), max(nets)))
    ok &= min(nets) == 0 and max(nets) == 0

    print('== ⑦ 진짜 LP 정지(1초 뒤) — 새 구조도 정지를 잡아야 한다 ==')
    for v in ('old', 'new'):
        s = Sim(v, lambda t: 1, seed=5, t_end=6000.0, lp_dead_at=1000.0).run()
        dt = (s.dead_t - 1000.0) if s.lp_declared_dead else float('nan')
        print('  %-4s  정지 판정: %s  (정지 뒤 %.0fms)' % (v, s.lp_declared_dead, dt))
        ok &= s.lp_declared_dead and dt < 3500.0             # ★2차: 유휴 1초 간격이면 ~2~3초 안
    # 눌림 중(10ms 간격 읽기)에 LP 가 멈추면 0.5초 남짓에 잡아야 한다
    s = Sim('new', tap(500.0, 3000.0), seed=6, t_end=4000.0, lp_dead_at=1500.0).run()
    dt = (s.dead_t - 1500.0) if s.lp_declared_dead else float('nan')
    print('  new   눌림 중 정지: %s  (정지 뒤 %.0fms)' % (s.lp_declared_dead, dt))
    ok &= s.lp_declared_dead and dt < 700.0

    print('결과: %s' % ('통과' if ok else '★실패'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
