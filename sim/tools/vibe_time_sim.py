# -*- coding: utf-8 -*-
"""진동 처리 — 기존(폴 횟수 기준, 버튼 태스크 110ms 주기) vs 새(시간 기준 + GPIO 웨이크 통지 + 유휴 1000ms).

왜 (2026-10-18, 감사 보고서 POWER_SAVING_FULL_CODE_AUDIT_2026-10-16 F09 · 사용자 "나머지 진행")
────────────────────────────────────────────────────────────────────
button_handler.c 의 진동 처리는 **폴 횟수**로 시간을 쟀다(10ms 폴링 전제):
  ISR 재활성 3폴=30ms · HIGH 창 30폴=300ms · 고장(HIGH 고정) 판정 200폴=2초.
버튼 태스크 유휴 주기가 110ms 가 되며 이미 0.33/3.3/22초였고, 1000ms 로 늘리면 3/30/200초가 된다.
그리고 진동으로 화면을 켜는 일은 버튼 태스크(판정) → 메인 루프(100ms, 화면 켜기) 순서라, 버튼 태스크가
1초를 자면 화면이 최대 1초 늦게 켜진다.

새 구조:
  · 시간 기준: ISR 재활성 30ms(대기 시한으로 깨움), HIGH 2표본 300ms 창, 고장 = 마지막 LOW 뒤 2초 + HIGH 5표본.
  · 진동 GPIO 웨이크(레벨 HIGH)로 깨면 절전 복귀 콜백이 버튼 태스크에 통지(somfy_app.c _ls_exit_cb).
  · 버튼 태스크 유휴 1000ms.
이 시뮬레이터가 보는 것:
  ① 화면 OFF 중 두드림 → 화면 켜짐까지 지연  ② HIGH 고정 고장 센서 → 고장 판정까지 시간·그동안의 깨어남
  ③ 평상시(LOW) 60초 — 진동 오검출 0, 버튼 태스크 깨어남  ④ 잡음 센서(짧은 HIGH 스파이크 33회/초) — 깨어남 상한
못 하는 것: 실제 센서 접점 파형(모델), PMU 레벨 웨이크의 정확한 검출 폭.

    python sim/tools/vibe_time_sim.py
"""
import random
import sys

DT = 0.25
HP_BUSY = 2.0                # 한 번 깨면 이만큼 깨어 있다
MAIN_PER = 100.0             # 메인 루프(화면 켜기 담당) 주기
MAIN_SLOW = 1000.0           # ★2026-10-18 새: 화면 OFF·배터리면 1초 + 진동 판정 시 즉시 깨움(somfy_app_main_kick)
BG_PER_OLD = (500.0, 500.0)  # OLED OFF·hold 유휴(기존)
BG_PER_NEW = (5000.0, 5000.0)
BTN_IDLE_OLD = 100.0         # + 공통 10ms(이중 대기)
BTN_IDLE_NEW = 1000.0
REARM_MS, DUTY_MS, STUCK_MS, STUCK_N = 30.0, 300.0, 2000.0, 5
OLD_REARM_N, OLD_DUTY_N, OLD_STUCK_N = 3, 30, 200
RECENT_MS = 500.0            # btn_handler_is_vibrating() 최근성


class Sim:
    def __init__(self, v, level, t_end, seed=1):
        self.v, self.level, self.t_end = v, level, t_end
        self.rng = random.Random(seed)
        self.t = 0.0
        self.prev_lv = level(0.0)
        # HP
        self.busy_until = 0.0
        self.hp_wakes = 0
        # 웨이크·ISR
        self.wake_en = True
        self.isr_en = True
        self.isr_cnt = 0
        self.isr_off_t = 0.0
        # 버튼 태스크
        self.notified = False
        self.btn_next = self.rng.uniform(0, 100.0)
        self.btn_phase = 'nwait'
        self.btn_runs = 0
        self.isr_prev = 0
        self.last_vib = None
        self.stuck = False
        self.stuck_t = None
        # old 상태
        self.o_rearm_cd = 0
        self.o_win_cnt = self.o_win_high = 0
        self.o_st_cnt = self.o_st_high = 0
        # new 상태
        self.n_low_t = 0.0
        self.n_high_n = 0
        self.n_hi1 = None
        # 메인 루프
        self.main_next = self.rng.uniform(0, MAIN_SLOW if v == 'new' else MAIN_PER)
        self.main_seen = None
        self.main_kick = False
        self.main_runs = 0
        self.screen_on_t = None
        per = BG_PER_NEW if v == 'new' else BG_PER_OLD
        self.bg = [[self.rng.uniform(0, p), p] for p in per]

    def asleep(self):
        return self.t >= self.busy_until

    def wake(self):
        if self.asleep():
            self.hp_wakes += 1
        self.busy_until = max(self.busy_until, self.t + HP_BUSY)

    # ── 버튼 태스크 본문의 진동 부분 ──
    def _track(self, high):
        if self.v == 'old':
            self.o_st_cnt += 1
            self.o_st_high += 1 if high else 0
            if self.o_st_cnt >= OLD_STUCK_N:
                st = (self.o_st_high == self.o_st_cnt)
                if st != self.stuck:
                    self.stuck = st
                    self.wake_en = not st
                    if st and self.stuck_t is None:
                        self.stuck_t = self.t
                self.o_st_cnt = self.o_st_high = 0
        else:
            if not high:
                self.n_low_t = self.t
                self.n_high_n = 0
            else:
                self.n_high_n += 1
            st = high and (self.t - self.n_low_t) >= STUCK_MS and self.n_high_n >= STUCK_N
            if st != self.stuck:
                self.stuck = st
                self.wake_en = not st
                if st and self.stuck_t is None:
                    self.stuck_t = self.t
        d = self.isr_cnt - self.isr_prev
        self.isr_prev = self.isr_cnt
        if self.stuck:
            return
        if d >= 1:
            self.last_vib = self.t
            self.n_hi1 = None
            self.o_win_cnt = self.o_win_high = 0
            if self.v == 'new':
                self.main_kick = True                 # somfy_app_main_kick()
            return
        if self.v == 'old':
            self.o_win_cnt += 1
            self.o_win_high += 1 if high else 0
            if self.o_win_high >= 2:
                self.last_vib = self.t
                self.o_win_cnt = self.o_win_high = 0
            elif self.o_win_cnt >= OLD_DUTY_N:
                self.o_win_cnt = self.o_win_high = 0
        elif high:
            if self.n_hi1 is not None and self.t - self.n_hi1 <= DUTY_MS:
                self.last_vib = self.t
                self.n_hi1 = None
                self.main_kick = True
            else:
                self.n_hi1 = self.t

    def _btn_body(self):
        self.btn_runs += 1
        self.wake()
        self._track(self.level(self.t) == 1)
        # ISR 재활성
        if not self.isr_en:
            if self.v == 'old':
                if self.o_rearm_cd == 0:
                    self.o_rearm_cd = OLD_REARM_N
                self.o_rearm_cd -= 1
                if self.o_rearm_cd == 0:
                    self.isr_en = True
            elif self.t - self.isr_off_t >= REARM_MS:
                self.isr_en = True

    def run(self):
        while self.t < self.t_end:
            lv = self.level(self.t)
            if lv != self.prev_lv:                        # any-edge ISR
                if self.isr_en:
                    self.isr_cnt += 1
                    self.isr_en = False
                    self.isr_off_t = self.t
                self.prev_lv = lv
            if lv == 1 and self.wake_en and self.asleep():  # 레벨 HIGH 웨이크
                self.wake()
                if self.v == 'new':
                    self.notified = True                    # 절전 복귀 콜백(GPIO) → 버튼 태스크 통지
            for b in self.bg:
                if self.t >= b[0]:
                    self.wake()
                    b[0] += b[1]
            # 버튼 태스크
            if self.v == 'old':
                if self.t >= self.btn_next:
                    if self.asleep():
                        self.hp_wakes += 0                  # 아래 wake() 가 센다
                    if self.btn_phase == 'nwait':           # 통지 대기(100ms) 끝 → 공통 10ms
                        self.wake()
                        self.btn_phase = 'delay'
                        self.btn_next = self.t + 10.0
                    else:
                        self._btn_body()
                        self.btn_phase = 'nwait'
                        self.btn_next = self.t + BTN_IDLE_OLD
            else:
                if self.notified or self.t >= self.btn_next:
                    self.notified = False
                    self._btn_body()
                    tmo = BTN_IDLE_NEW
                    if not self.isr_en:
                        tmo = min(tmo, max(2.0, REARM_MS - (self.t - self.isr_off_t) + 1.0))
                    self.btn_next = self.t + tmo
            # 메인 루프 — 진동이면 화면 켜기. 새: 화면 OFF 면 1초 주기 + 진동 판정 시 즉시(kick)
            if self.t >= self.main_next or self.main_kick:
                self.main_kick = False
                self.main_runs += 1
                self.wake()
                recent = self.last_vib is not None and self.t - self.last_vib < RECENT_MS
                event = self.last_vib is not None and self.last_vib != self.main_seen
                if (recent or event) and self.screen_on_t is None:
                    self.screen_on_t = self.t
                self.main_seen = self.last_vib
                slow = (self.v == 'new' and self.screen_on_t is None)
                self.main_next = self.t + (MAIN_SLOW if slow else MAIN_PER)
            self.t += DT
        return self


def tap_level(t0, rng):
    """두드림 — 60ms 안에 HIGH 펄스 2~4개(각 3~15ms). 평상시 LOW."""
    pulses = []
    tt = t0
    for _ in range(rng.randint(2, 4)):
        w = rng.uniform(3.0, 15.0)
        pulses.append((tt, tt + w))
        tt += w + rng.uniform(3.0, 15.0)
    return lambda t: 1 if any(a <= t < b for a, b in pulses) else 0


def main():
    ok = True
    print('== ① 화면 OFF 중 두드림 → 화면 켜짐까지(200회) ==')
    for v in ('old', 'new'):
        lat, miss = [], 0
        for k in range(200):
            rng = random.Random(100 + k)
            t0 = 1000.0 + rng.uniform(0, 1000.0)
            s = Sim(v, tap_level(t0, rng), t0 + 2500.0, seed=k).run()
            if s.screen_on_t is None:
                miss += 1
            else:
                lat.append(s.screen_on_t - t0)
        lat.sort()
        print('  %-4s  켜짐 %d/200  지연 중앙 %.0f / 최대 %.0f ms' % (v, 200 - miss, lat[len(lat) // 2], lat[-1]))
        ok &= miss == 0
        if v == 'new':
            ok &= lat[-1] <= 140.0                            # 메인 루프 100ms + α 안

    print('== ② HIGH 고정 고장 센서(부팅부터) — 고장 판정까지·그동안 깨어남 ==')

    def stuck_level(seed):
        r = random.Random(seed)
        spikes = sorted(r.uniform(0, 60000.0) for _ in range(33 * 60))  # 0.1ms LOW 스파이크 33회/초(ISR 잡음)
        import bisect

        def f(t):
            i = bisect.bisect_right(spikes, t) - 1
            return 0 if (i >= 0 and t - spikes[i] < 0.1) else 1
        return f
    for v in ('old', 'new'):
        s = Sim(v, stuck_level(7), 40000.0, seed=3).run()
        print('  %-4s  고장 판정 %s  (%.1f초)   40초 동안 HP 깨어남 %.0f회/초'
              % (v, s.stuck, (s.stuck_t or float('nan')) / 1000.0, s.hp_wakes / 40.0))
        if v == 'new':
            ok &= s.stuck and s.stuck_t < 3000.0
    print('== ③ 평상시(LOW) 60초 — 오검출 0, 깨어남 ==')
    for v in ('old', 'new'):
        s = Sim(v, lambda t: 0, 60000.0, seed=5).run()
        print('  %-4s  진동 검출 %s   버튼 태스크 %.1f회/초   메인 루프 %.1f회/초   HP 깨어남 %.1f회/초'
              % (v, s.last_vib is not None, s.btn_runs / 60.0, s.main_runs / 60.0, s.hp_wakes / 60.0))
        ok &= s.last_vib is None
    print('== ④ 잡음 센서(0.3ms HIGH 스파이크 33회/초, 평상시 LOW) — 깨어남 상한 ==')

    def noisy(seed):
        r = random.Random(seed)
        spikes = sorted(r.uniform(0, 30000.0) for _ in range(33 * 30))
        import bisect

        def f(t):
            i = bisect.bisect_right(spikes, t) - 1
            return 1 if (i >= 0 and t - spikes[i] < 0.3) else 0
        return f
    for v in ('old', 'new'):
        s = Sim(v, noisy(9), 30000.0, seed=4).run()
        print('  %-4s  HP 깨어남 %.0f회/초   버튼 태스크 실행 %.0f회/초   (진동으로 화면 유지: %s)'
              % (v, s.hp_wakes / 30.0, s.btn_runs / 30.0, s.last_vib is not None))
        if v == 'new':
            ok &= s.btn_runs / 30.0 < 120.0                   # 스파이크당 최대 2번(처리+재활성) 남짓

    print('결과: %s' % ('통과' if ok else '★실패'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
