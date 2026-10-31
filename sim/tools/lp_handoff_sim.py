# -*- coding: utf-8 -*-
"""★2026-10-29 LP 코어 정지 → HP 인계 (GPT 재감사 N07) — 수정 전 가상검증 (사용자 규칙 "빌드 전 가상검증").

옛 코드(button_handler.c _pcf8574_read 의 'LP 정지 확정', _lp_core_try_start 의 실패 경로):
  · LP 가 멈추면 ulp_enabled=0·s_lp_active=false 만 하고 HP 비트뱅 읽기로 넘어간다. 그런데 GPIO6/7 은
    lp_core_i2c_master_init 의 rtc_gpio_init 으로 **LP_I2C 기능에 묶인 채**다 → HP 의 gpio_set_direction/gpio_get_level 이
    선에 안 닿아 NACK → s_pcf_last(LP 경로에선 갱신된 적 없음 = 0xFFFF)만 돌려준다 = 버튼·로터리 전부 먹통(재부팅까지).
  · 부팅 때 LP_I2C init 은 됐는데 적재·기동·폴링 확인이 실패한 경로도 같다.
새 코드:
  · _lp_pins_to_hp(): LP 정지 → rtc_gpio_deinit(6/7) → HP 입력+풀업 → SDA 가 LOW 면 9클럭+STOP → PCF 재검출(3회).
    두 실패 경로 모두 이걸 부른다.
  · 인계 뒤 LP_RESTART_DELAY_MS(30초) 지나 버튼·로터리가 쉬고 있으면 LP 를 다시 올린다(부팅당 LP_RESTART_MAX=2회).
    다시 올라오면 절전 경로(LP 폴링)로 복귀. ★재기동은 LP RAM 을 새로 적재하므로 rot_delta·press_gen·poll_cnt 가 0 이 된다 —
    HP 의 로터리 기준선(s_rot_taken)과 정지 판정 기록도 같이 다시 잡아야 한다(안 그러면 헛회전이 쏟아진다).

모델(1ms 해상도): 사용자 누름(30~400ms)·로터리 디텐트, LP 코어(5ms 폴, 세대 카운터·로터리 누적·poll_cnt),
HP 버튼 태스크(LP 경로: 통지 또는 시한 / HP 경로: 10ms), 디바운스 20ms, 정지 판정 500ms·2회.
불변식: 헛누름 0 · 헛회전 0 · 버스 충돌(LP 가 돌 때 HP 비트뱅) 0 · 재기동 ≤ 2 · 인계 뒤 누름 놓침은 재기동 창(~60ms)뿐.
"""
import random
import sys

POLL_LP_MS = 5
DEBOUNCE_MS = 20
STALE_MS, STALE_READS = 500, 2
IDLE_TMO_MS, ACTIVE_MS = 1000, 10
HP_POLL_MS = 10
RESTART_DELAY_MS, RESTART_MAX = 30000, 2
START_COST_MS = 60          # _lp_core_try_start: stop 10 + init + 적재 + 폴링 확인 50
PROG = 3                    # 버튼 번호(PROG — 눌림 고정이면 모터가 프로그래밍 모드로 간다)
NBTN = 7


class World(object):
    def __init__(self, seed, design, dur_ms, faults):
        self.rng = random.Random(seed)
        self.design, self.dur, self.f = design, dur_ms, faults
        # 진짜 입력
        self.presses = []                      # (t0, t1, btn)
        t = 500
        while True:
            t += self.rng.randint(800, 6000)
            if t > dur_ms - 3000:            # 끝을 넘는 누름은 만들지 않는다(판정 창이 잘린다 — 첫 판 인공물)
                break
            self.presses.append((t, t + self.rng.randint(30, 400), self.rng.randrange(NBTN)))
        self.dets = []                         # (t, +1/-1)
        t = 700
        while t < dur_ms - 2000:
            t += self.rng.randint(1500, 9000)
            for _ in range(self.rng.randint(1, 4)):
                t += self.rng.randint(40, 200)
                self.dets.append((t, self.rng.choice((1, -1))))
        # 핀·LP 상태
        self.pins = 'gpio'
        self.lp_run = False
        self.lp_stuck = False
        self.lp_ram = None
        self.conflicts = 0
        self.lp_starts = 0
        self.stuck_since = None
        self.now = 0

    def gone(self, t):
        at = self.f.get('pcf_gone_at')
        return at is not None and t >= at

    # 진짜 PCF 값(버튼 비트만 — 0=눌림)
    def btn_low(self, t):
        m = 0
        for a, b, k in self.presses:
            if a <= t < b:
                m |= 1 << k
        return m

    def det_sum(self, t):
        return sum(d for tt, d in self.dets if tt <= t)

    # ── LP 코어 ──
    def lp_load(self):
        self.lp_ram = dict(state=None, gen=[0] * NBTN, rot=0, cnt=0, prev_low=0, held=0, rot_base=None)

    def lp_poll(self, t):
        r = self.lp_ram
        if not self.lp_run or self.lp_stuck or self.pins != 'rtc':
            return False
        if self.gone(t):                     # PCF 무응답 — LP 도 못 읽는다(폴 횟수만 는다)
            r['cnt'] += 1
            return False
        low = self.btn_low(t)
        now_held = low & r['prev_low']
        starts = now_held & ~r['held']
        r['held'] = now_held
        r['prev_low'] = low
        for k in range(NBTN):
            if starts >> k & 1:
                r['gen'][k] = (r['gen'][k] + 1) & 0xF
        ds = self.det_sum(t)
        if r['rot_base'] is None:
            r['rot_base'] = ds
        newrot = ds - r['rot_base']
        changed = (r['state'] != low) or (newrot != r['rot'])
        r['state'] = low
        r['rot'] = newrot
        r['cnt'] += 1
        return changed


class Hp(object):
    """button_handler 버튼 태스크 — 디자인별(옛/새)."""

    def __init__(self, w):
        self.w = w
        self.lp_active = False
        self.pcf_last = 0          # 0 = 아무것도 안 눌림(비트 1=눌림 으로 단순화: btn_low 와 같은 극성)
        self.raw = [False] * NBTN  # True=눌림
        self.last = [False] * NBTN
        self.deb = [0] * NBTN
        self.lat_hold = 0
        self.gen_seen = [0] * NBTN
        self.cnt_prev, self.stale, self.chg_ms = -1, 0, 0
        self.rot_sync, self.rot_taken = False, 0
        self.rot_events = 0        # +CW −CCW 순합
        self.rot_phantom = 0
        self.events = []           # (t, btn) PRESS
        self.restarts = 0
        self.restart_at = None
        self.handoffs = 0
        self.dead_at = None
        self.recovered_at = []
        self.attempts = []         # 재기동 시도 시각(그 창의 짧은 누름은 놓쳐도 된다)
        self.stalls = []           # (LP 가 멈춘 시각, 정지 확정 시각) — 그 사이 입력은 누구도 못 본다(어느 설계든)
        self.hp_det_base = None    # HP 직접 디코딩 경로의 로터리 기준(모델 단순화: 읽을 때 순합)

    # _lp_core_try_start
    def lp_try_start(self, t, fail_after_init=False):
        w = self.w
        w.lp_run = False
        w.pins = 'rtc'                                  # lp_core_i2c_master_init → rtc_gpio_init
        w.lp_starts += 1
        if fail_after_init or w.f.get('start_fail'):
            if self.w.design == 'new':
                self.pins_to_hp()
            return False
        w.lp_load()
        w.lp_stuck = False
        w.lp_run = True
        self.gen_seen = list(w.lp_ram['gen'])
        self.lp_active = True
        if self.w.design == 'new':
            self.rot_sync = False                       # ★재기동 = LP RAM 새로 적재 — 기준선 다시
            self.cnt_prev, self.stale, self.chg_ms = -1, 0, t
        return True

    def pins_to_hp(self):
        w = self.w
        w.lp_run = False                                # ulp_lp_core_stop
        w.pins = 'gpio'                                 # rtc_gpio_deinit + gpio_config
        return not w.gone(w.now)

    def hp_bitbang(self, t):
        w = self.w
        if w.lp_run and not w.lp_stuck and w.pins == 'gpio':
            w.conflicts += 1                            # ★LP 가 도는데 선이 HP 쪽 — LP 는 선을 잃고 HP 와 소유가 엇갈린다
        # (선이 LP_I2C 에 묶인 채 HP 가 비트뱅하는 것은 패드에 안 닿아 무해 — 대신 읽기가 실패한다. 부팅 첫 5ms 가 원래 그렇다)
        if w.pins != 'gpio' or w.gone(t):
            return self.pcf_last                        # NACK — 직전 값(없던 눌림을 만들지 않는다)
        self.pcf_last = w.btn_low(t)
        return self.pcf_last

    def read(self, t):
        w = self.w
        if self.lp_active and w.lp_ram['state'] is not None:
            r = w.lp_ram
            if r['cnt'] != self.cnt_prev:
                self.cnt_prev, self.stale, self.chg_ms = r['cnt'], 0, t
            elif self.stale < 8:
                self.stale += 1
            if self.stale < STALE_READS or t - self.chg_ms < STALE_MS:
                st = r['state']
                lat = 0
                for k in range(NBTN):
                    if r['gen'][k] != self.gen_seen[k]:
                        lat |= 1 << k
                self.gen_seen = list(r['gen'])
                unconf = sum(1 << k for k in range(NBTN) if not self.last[k])
                st |= lat
                self.lat_hold |= lat & unconf
                st |= self.lat_hold
                # 로터리(LP 누적 − HP 소비)
                if not self.rot_sync:
                    self.rot_sync, self.rot_taken = True, r['rot']
                d = r['rot'] - self.rot_taken
                self.rot_taken += d
                self.rot_events += d
                return st, True
            # ── LP 정지 확정 ──
            self.lp_active = False
            self.lat_hold = 0
            self.handoffs += 1
            self.stalls.append((w.stuck_since if w.stuck_since is not None else t, t))
            if self.dead_at is None:
                self.dead_at = t
            if w.design == 'new':
                self.pins_to_hp()
                if self.restarts < RESTART_MAX:
                    self.restart_at = t + RESTART_DELAY_MS
        st = self.hp_bitbang(t)
        if self.hp_det_base is None:
            self.hp_det_base = (w.det_sum(t), self.rot_events)
        if w.pins == 'gpio' and not w.gone(t):
            # HP 직접 디코딩(하프스텝) — 10ms 폴이면 디텐트를 놓치지 않는다고 본다(기존 경로, 이 시뮬 대상 아님)
            self.rot_events = self.hp_det_base[1] + (w.det_sum(t) - self.hp_det_base[0])
        return st, False

    def step(self, t, st):
        for k in range(NBTN):
            cur = bool(st >> k & 1)
            if cur != self.raw[k]:
                self.raw[k] = cur
                self.deb[k] = t
            if t - self.deb[k] >= DEBOUNCE_MS:
                if self.raw[k] and not self.last[k]:
                    self.last[k] = True
                    self.lat_hold &= ~(1 << k)
                    self.events.append((t, k))
                elif not self.raw[k] and self.last[k]:
                    self.last[k] = False


def run(design, seed, faults, dur_ms=200000):
    w = World(seed, design, dur_ms, faults)
    hp = Hp(w)
    w.lp_load()
    ok = hp.lp_try_start(0, fail_after_init=faults.get('boot_fail', False))
    die_at = faults.get('die_at')
    redie_after = faults.get('redie_after')     # 재기동 뒤 이만큼 지나 다시 죽음(영구 고장)
    next_lp, next_hp = 0, 0
    notify = False
    last_start = 0
    pending_hp_det = None
    for t in range(dur_ms):
        w.now = t
        if die_at is not None and t == die_at:
            w.lp_stuck, w.stuck_since = True, t
        if redie_after and w.lp_run and not w.lp_stuck and hp.restarts and t - last_start == redie_after:
            w.lp_stuck, w.stuck_since = True, t
        if t >= next_lp:
            next_lp = t + POLL_LP_MS
            if w.lp_poll(t):
                notify = True                           # ulp_lp_core_wakeup_main_processor → 태스크 통지
        if t >= next_hp or (notify and hp.lp_active):
            notify = False
            st, lp_path = hp.read(t)
            hp.step(t, st)
            if lp_path:
                held = any(hp.last) or any(hp.raw[k] != hp.last[k] for k in range(NBTN))
                tmo = ACTIVE_MS if held else IDLE_TMO_MS
                for k in range(NBTN):
                    if hp.raw[k] != hp.last[k]:
                        tmo = min(tmo, max(2, DEBOUNCE_MS - (t - hp.deb[k]) + 1))
                next_hp = t + tmo
            else:
                next_hp = t + HP_POLL_MS
                # ★새: 재기동 시도(버튼·로터리 쉬는 중에만)
                if design == 'new' and hp.restart_at is not None and t >= hp.restart_at and not any(hp.last):
                    hp.restart_at = None
                    hp.restarts += 1
                    hp.attempts.append(t)
                    last_start = t + START_COST_MS
                    good = hp.lp_try_start(t + START_COST_MS)
                    next_hp = t + START_COST_MS
                    if good:
                        hp.recovered_at.append(t)
                        hp.hp_det_base = None
                    elif hp.restarts < RESTART_MAX:
                        hp.restart_at = t + RESTART_DELAY_MS
            if not hp.lp_active and pending_hp_det is None:
                pending_hp_det = t
    return w, hp


def score(w, hp, design):
    """누름 놓침·헛누름·헛회전 집계. 놓침 허용: 재기동 창 동안의 짧은 누름."""
    miss, phantom = 0, 0
    used = set()
    # 아무도 못 보는 창: 재기동 중(버튼 태스크가 _lp_core_try_start 에 묶임 ~60ms) / LP 가 멈춘 뒤 정지 확정까지(어느 설계든)
    windows = [(a, a + START_COST_MS + DEBOUNCE_MS * 2) for a in hp.attempts] + \
              [(a, b + DEBOUNCE_MS * 2) for a, b in hp.stalls]
    for (a, b, k) in w.presses:
        hit = [i for i, (te, kk) in enumerate(hp.events) if kk == k and a <= te <= b + 3 * DEBOUNCE_MS + IDLE_TMO_MS and i not in used]
        if hit:
            used.add(hit[0])
        else:
            if not any(x <= a <= y or x <= b <= y for x, y in windows):
                miss += 1
    phantom = len(hp.events) - len(used)
    lost = [(a, b) for a, b in hp.stalls] + [(a, a + START_COST_MS + 10) for a in hp.attempts]
    true_net = w.det_sum(w.dur) - sum(d for tt, d in w.dets if any(a <= tt <= b for a, b in lost))
    rot_err = hp.rot_events - true_net
    return miss, phantom, rot_err


def main():
    ok = True
    print('[1] 정상(고장 없음) — 두 설계 같은가')
    for d in ('old', 'new'):
        m = p = r = c = 0
        for s in range(10):
            w, hp = run(d, s, {})
            a, b, e = score(w, hp, d)
            m, p, r, c = m + a, p + b, r + abs(e), c + w.conflicts
        print('   %-3s 놓침 %d  헛누름 %d  로터리 오차 %d  버스 충돌 %d' % (d, m, p, r, c))
        if d == 'new' and (m or p or r or c):
            ok = False
    scen = [
        ('LP 일시 정지(재기동하면 산다)',       dict(die_at=60000)),
        ('LP 영구 고장(재기동 1초 뒤 또 죽음)',  dict(die_at=60000, redie_after=1000)),
        ('부팅 때 LP_I2C init 뒤 기동 실패',    dict(boot_fail=True)),
        ('인계 뒤 PCF 무응답(선 고착)',          dict(die_at=60000, pcf_gone_at=60000)),
    ]
    for name, f in scen:
        print('[%s]' % name)
        for d in ('old', 'new'):
            m = p = r = c = 0
            rs, ho, lp_end = [], [], 0
            for s in range(10):
                w, hp = run(d, s, f)
                a, b, e = score(w, hp, d)
                m, p, r, c = m + a, p + b, r + abs(e), c + w.conflicts
                rs.append(hp.restarts)
                ho.append(hp.handoffs)
                lp_end += 1 if hp.lp_active else 0
            print('   %-3s 놓침 %3d  헛누름 %d  로터리 오차 %4d  버스 충돌 %d  재기동 %s  인계 %s  끝에 LP 경로 %d/10'
                  % (d, m, p, r, c, max(rs), max(ho), lp_end))
            if d == 'new':
                pcf_gone = 'pcf_gone_at' in f
                if p or c or max(rs) > RESTART_MAX:
                    print('   ✗ 새 설계: 헛누름·버스 충돌·재기동 한도 위반'); ok = False
                if not pcf_gone and (m > 10 or r):
                    print('   ✗ 새 설계: 인계 뒤에도 놓치거나 로터리가 틀린다'); ok = False
                if 'redie_after' not in f and not pcf_gone and not f.get('boot_fail') and lp_end != 10:
                    print('   ✗ 일시 정지인데 LP 경로(절전)로 복귀하지 못했다'); ok = False
    # 옛 설계의 재기동 기준선 버그를 따로 보인다: 재기동만 '새'처럼 하고 기준선을 안 잡으면?
    print('[기준선] 재기동 뒤 로터리 기준선을 안 다시 잡으면(옛 s_rot_sync 정적 변수 그대로)')
    bad = 0
    for s in range(10):
        w = World(s, 'new', 200000, dict(die_at=60000))
        hp = Hp(w)
        orig = hp.lp_try_start

        def no_resync(t, fail_after_init=False, _o=orig, _hp=hp):
            keep = (_hp.rot_sync, _hp.rot_taken)
            r_ = _o(t, fail_after_init)
            _hp.rot_sync, _hp.rot_taken = keep        # 옛 코드: 기준선 유지
            return r_
        hp.lp_try_start = no_resync
        # run() 을 그대로 쓰기 위해 run 의 내부를 다시 돌린다
        w2, hp2 = w, hp
        w2.lp_load()
        hp2.lp_try_start(0)
        # (간단히) run 과 같은 루프
        next_lp = next_hp = 0
        notify = False
        for t in range(200000):
            if t == 60000:
                w2.lp_stuck = True
            if t >= next_lp:
                next_lp = t + POLL_LP_MS
                if w2.lp_poll(t):
                    notify = True
            if t >= next_hp or (notify and hp2.lp_active):
                notify = False
                st, lp_path = hp2.read(t)
                hp2.step(t, st)
                next_hp = t + (IDLE_TMO_MS if lp_path else HP_POLL_MS)
                if not lp_path and hp2.restart_at is not None and t >= hp2.restart_at:
                    hp2.restart_at = None
                    hp2.restarts += 1
                    if hp2.lp_try_start(t):
                        hp2.hp_det_base = None
        bad += abs(hp2.rot_events - w2.det_sum(200000))
    print('   로터리 순합 오차 합 %d (0 이 아니면 재기동 때 헛회전이 쏟아진다 — 새 코드가 기준선을 다시 잡는 이유)' % bad)
    if bad == 0:
        print('   ※기준선 버그가 재현되지 않았다 — 모델이 실제 위험을 못 담는다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
