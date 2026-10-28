# -*- coding: utf-8 -*-
"""얕은잠 바닥 전류 대조군(`lsfloor`) — 상태기계 + 판정 사전 검증.

무엇을 재나 (2026-10-13) — lsfloor_decode.py 머리말 참조.
무선을 켜기 **전에**(esp_matter::start 이전) 순수 얕은잠만 돌리면서 MODEM 도메인을
켬-끔-켬-끔 으로 바꾸고, 표본마다 전압을 NVS 에 남긴다.

    1) `lsfloor <단계분> [정착분] [표본초]` : USB 꽂힌 채 **예약만** 한다.
    2) 사용자가 전원을 끊는다.
    3) 메인 루프가 정착 시간을 기다린 뒤 화면을 끄고 **스스로 재부팅**한다.
    4) 부팅 훅(무선 시작 전)이 4단계를 돌며 표본을 NVS 에 하나씩 적는다.
    5) 끝나면 그대로 정상 부팅을 이어간다 — 기기는 저절로 제어 가능 상태로 돌아온다.

깊은잠 대조군(dsleep)에서 밟은 지뢰를 전부 케이스로 옮겼다:
  · RAM 플래그(부팅 뒤 콘솔 예약이 안 먹던 버그 — 하룻밤 날림)
  · 재부팅 루프(USB 판독이 훅과 메인 루프에서 어긋남 — 718회)
  · 옛 결과 잔존, 도중 리셋, 화면 켜진 채 잠듦
새로 지키는 것:
  · 측정은 **한 부팅에서 한 번만** 돈다. 도중에 리셋되면 다시 시작하지 않는다.
  · 표본은 받는 즉시 NVS 에 남는다 — 중간에 죽어도 그때까지는 살아 있다.
  · 켜짐 단계에서는 ADC 유닛이 살아 있고(=MODEM 켜짐), 꺼짐 단계에서는 없다.
  · 모든 표본을 **새로 만든 유닛**으로 잰다(조건 통일).
  · 끝나면 ADC 유닛 상태를 기본(상주)으로 되돌린다.

★이 시뮬레이터가 **못 하는 것**: "MODEM 을 끄면 전류가 준다"는 물리 가정은 못 잰다.
  그건 실기가 답한다. 여기서는 흐름이 틀려 밤을 날리는 일만 막는다.
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lsfloor_decode import (PHASES, MAX_PER_PHASE, FLAGS_NONE, WAKE_TIMER, BIT_MODEM,   # noqa: E402
                            BIT_TOP, BIT_CPU,
                            RUN_NONE, RUN_ACTIVE, RUN_DONE, RUN_RESET, RUN_USB,
                            phase_wants_pd, pack, unpack, analyse, lp_rates,
                            WAKE_PATS, analyse_wake)

# ── 펌웨어와 공유할 상수 (somfy_app.c 와 일치시킬 것) ────────────────
MAX_TRIES = 3
PHASE_MIN_S, PHASE_MIN_DRY_S, PHASE_MAX_S = 60, 20, 7200
SAMP_MIN_S, SAMP_MAX_S = 5, 600
SETTLE_MIN_S, SETTLE_MAX_S, SETTLE_DEF_S = 60, 14400, 3600
NVS_MARGIN = 12
# ★2026-10-15 실측에 맞췄다. 무선 시작 전 훅에서는 CPU 리텐션이 아직 없어 CPU 도 켜진 채 잔다(0x3C02).
#  TOP 측정(fac=1)은 CPU 리텐션을 먼저 올리므로 평소처럼 CPU 가 꺼진다(0x3C12).
FLAGS_BASE = 0x3C02              # fac=0 (MODEM 측정) 훅 단계 실측
FLAGS_BASE_TOP = 0x3C12          # fac=1 (TOP 측정) — CPU 차단 비트 포함
# TOP 을 끄면 HP 패드가 전원을 잃으므로 측정 내내 붙잡는 핀(펌웨어 _lsf_hold_pins 와 같은 목록)
HOLD_PINS = ('CS', 'SCK', 'MOSI', 'GD0', 'ANT_EN', 'ANT_SEL')
# ★2026-10-16 LP 측정(fac=2) — LP 코어 폴링 속도 실측(10/15 TOP 밤: 전 단계 191회/초)
LP_RATE = 191
# ★2026-10-16 WAKE 측정(fac=3) — 펌웨어 LSF_MAX_SAMPLES / LSF_WAKE_PERIOD_US 와 같게
MAX_SAMPLES = 96
WAKE_RATE = 1e6 / 31450.0          # 31.8회/초
MARC_IDLE, MARC_SLEEP = 0x01, 0x00


def nph_of(fac):
    return len(WAKE_PATS[5]) if fac in (3, 4) else PHASES        # ★2026-10-21 fac=4(ANT) 도 ABACA


def kind_of(fac, p):
    return WAKE_PATS[5][p] if fac in (3, 4) else ('X' if phase_wants_pd(p) else 'A')


class Reset(Exception):
    """측정 도중 전원이 죽거나 리셋됨."""


class Dev:
    def __init__(self, free_entries=400, adc_keep_default=True):
        self.nvs = {}                      # 네임스페이스 "lsfl"
        self.ram_armed = 0                 # ★NVS 와 따로 — 메인 루프는 이것만 본다
        self.ram_settle = SETTLE_DEF_S
        self.dsleep_armed = 0
        self.off_s = 0
        self.restarts = 0                  # 감시가 건 자발 재부팅
        self.dry_restarts = 0
        self.boots = 0
        self.log = []
        self.free = free_entries
        self.adc_keep_default = adc_keep_default
        self.adc_keep = adc_keep_default
        self.adc_unit = adc_keep_default   # ADC 유닛이 살아 있는가 (= MODEM 강제 ON)
        self.adc_created = 0
        self.oled_on = True
        self.sleeps = []                   # [(단계, 절전 중 유닛 생존)]
        self.t = 0
        self.top_blocked = False           # 리텐션 없는 드라이버가 TOP 차단을 막는 경우
        self.holds = set()                 # gpio_hold 로 붙잡힌 핀 — 리셋을 넘어 살아남는다
        self.cpu_ret = False               # CPU 리텐션 초기화 여부
        # ★2026-10-16 LP 코어 — 소프트 재부팅으로는 안 멈추므로 부팅 사이에도 계속 돈다
        self.lp_running = True
        self.lp_cnt = 0                    # LP 공유 RAM 의 poll_cnt
        self.lp_bus_low = False            # 멈출 때마다 I2C 선이 LOW 로 남는 경우(5회 재시도 모두 실패)
        self.lp_run_fail = False           # ulp_lp_core_run 이 실패하는 경우
        # ★2026-10-16 WAKE 측정(fac=3) — CC1101 상태와 고장 주입
        self.cc_state = 'SPWD'             # 'SPWD' / 'IDLE' — ESP 리셋으로는 안 바뀐다(별개 칩)
        self.cc_wake_fail = False          # SIDLE 로 안 깨어남(SPI 고장) → MARCSTATE 0x00
        self.cc_spwd_fail = False          # C 끝의 SPWD 가 안 먹음 → IDLE 로 남는다
        self.wake_rate = WAKE_RATE         # B 단계 실제 깨어남 속도(고장 주입: 0)
        # ★2026-10-21 ANT 측정(fac=4) — XIAO 안테나 핀 상태와 고장 주입
        self.ant = 'prod'                  # 'prod'(깨어 구동·절전 놓음) / 'drive'(옛 동작) / 'release'(놓음)
        self.ant_res = True                # 모듈 안에 R21·R24 가 있다(False = 놓은 단계 읽힘이 반대로 나온다)
        self.ant_drive_fail = False        # 구동이 안 먹음(핀이 다른 기능에 묶임 등) → B 읽힘이 틀린다

    # ── 콘솔 ──────────────────────────────────────────────────────────
    def cmd(self, phase_s, settle_s=SETTLE_DEF_S, samp_s=300, dry=0, fac=0):
        if phase_s == 0:
            self.nvs['armed'] = 0
            self.nvs['tries'] = 0
            self.ram_armed = 0             # ★RAM 플래그도 반드시 내린다
            return 'CANCEL'
        if self.nvs.get('armed'):
            return 'DUP'
        if self.dsleep_armed:
            return 'DSLEEP'
        lo = PHASE_MIN_DRY_S if dry else PHASE_MIN_S
        if not (lo <= phase_s <= PHASE_MAX_S) or not (SAMP_MIN_S <= samp_s <= SAMP_MAX_S):
            return 'RANGE'
        if not 0 <= fac <= 4:
            fac = 0
        nph = nph_of(fac)
        n_per = phase_s // samp_s
        if not (2 <= n_per <= (MAX_SAMPLES - 1) // nph):   # 4단계 23개, 5단계 19개
            return 'NPER'
        settle_s = max(SETTLE_MIN_S, min(SETTLE_MAX_S, settle_s))
        if self.free < nph * n_per + 1 + NVS_MARGIN:
            return 'NOSPACE'               # ★표본을 적을 자리가 없으면 밤을 날린다
        self.nvs = {}                      # nvs_erase_all — 옛 표본·결과를 지운다
        self.nvs.update(armed=1, tries=0, dry=1 if dry else 0, run=RUN_NONE, nph=nph,
                        phase_s=phase_s, samp_s=samp_s, settle_s=settle_s, n=0, fac=fac)
        self.ram_settle = settle_s
        if dry:
            self.dry_restarts += 1         # 모의 실행은 바로 재부팅해 훅으로 들어간다
            return 'RESTART'
        self.ram_armed = 1                 # ★부팅 뒤 예약도 감시가 돌아야 한다
        return 'ARMED'

    # ── 메인 루프 감시 ────────────────────────────────────────────────
    def tick(self, usb, dt=1):
        if not self.ram_armed:
            self.off_s = 0
            return None
        if usb:
            self.off_s = 0
            return None
        self.off_s += dt
        if self.off_s < self.ram_settle:
            return None
        tries = self.nvs.get('tries', 0) + 1
        if tries > MAX_TRIES:
            self.nvs.update(armed=0, tries=0)
            self.ram_armed = 0
            self.log.append('루프 방어 — 예약 해제')
            return None
        self.nvs['tries'] = tries
        self.oled_on = False               # 패널 OFF — SSD1306 은 재부팅 뒤에도 이 상태를 유지
        self.nvs['oled_off'] = 1
        self.restarts += 1
        self.off_s = 0
        return 'RESTART'

    # ── 부팅 (RAM 은 초기화, NVS·OLED 패널 상태는 남는다) ─────────────
    def boot(self, usb, mv_fn=None, events=None, reset_reason=1):
        self.boots += 1
        self.ram_armed = 0
        self.off_s = 0
        self.ram_settle = SETTLE_DEF_S
        self.adc_keep = self.adc_keep_default
        self.adc_unit = self.adc_keep_default
        self.cpu_ret = False
        # ★app_main 이 맨 앞에서 측정이 붙잡았을 수 있는 핀을 **전부** 푼다. 측정 도중 리셋되면
        #  hold 가 리셋을 넘어 남는데, 안 풀면 SPI·RMT 핀이 굳어 RF 가 통째로 죽는다.
        self.holds = set()
        self.ant = 'prod'                  # ★2026-10-21 app_main 맨 앞의 board_rf_ant_apply — 측정 도중 리셋돼도 평소 설정
        # ★app_main Phase 2 의 cc1101_init 이 훅보다 **먼저** 돈다 — 레지스터를 다시 쓰고 SPWD 로 재운다.
        #  그래서 C 단계(IDLE) 도중 리셋돼도 다음 부팅에서 1.7mA 가 남지 않는다.
        self.cc_state = 'SPWD'
        self._hook(usb, mv_fn or (lambda t, p: 4000.0), events or {}, reset_reason)
        self.oled_on = True                # 정상 부팅이 이어져 화면이 켜진다
        # ★btn_handler 의 _lp_core_try_start 가 멈춰 있던 LP 코어도 다시 적재해 돌린다(ulp_lp_core_stop 먼저)
        self.lp_running = True
        return 'RUN'

    # ── LP 코어 켬/끔 (btn_handler_lp_pause / _resume 과 같은 결과) ────────
    def _lp_pause(self):
        if self.lp_bus_low:
            self.lp_running = True         # 5회 모두 선이 LOW → 마지막 동작이 "다시 돌림"이라 계속 돈다
            return False
        self.lp_running = False
        return True

    def _lp_resume(self):
        if self.lp_run_fail:               # ulp_lp_core_run 실패 — 상태는 그대로(돌던 건 돌고, 멈춘 건 멈춘 채)
            return False
        self.lp_running = True
        return True

    def _measure(self, t, p, pd_next, mv_fn):
        """매 표본을 새 유닛으로 재고, 다음 절전에 맞춰 유닛을 남기거나 지운다."""
        self.adc_unit = True
        self.adc_created += 1
        mv = mv_fn(t, p)
        self.adc_keep = not pd_next
        if pd_next:
            self.adc_unit = False
        return int(round(mv * 10))

    def _hook(self, usb, mv_fn, events, reset_reason):
        nv = self.nvs
        if nv.get('run') == RUN_ACTIVE:
            # 지난 부팅이 측정 도중 죽었다 — ★다시 시작하지 않는다
            nv.update(run=RUN_RESET, rst=reset_reason, armed=0)
            return
        if not nv.get('armed'):
            return
        dry = nv.get('dry', 0)
        phase_s, samp_s = nv.get('phase_s', 0), nv.get('samp_s', 0)
        if not phase_s or not samp_s or phase_s // samp_s < 2:
            nv['armed'] = 0
            return
        if not dry and usb:
            self.ram_armed = 1             # ★부팅 시점에 이미 예약돼 있던 경우
            self.ram_settle = nv.get('settle_s') or SETTLE_DEF_S
            return
        nv.update(armed=0, tries=0, run=RUN_ACTIVE, n=0)   # ★예약부터 해제
        self.ram_armed = 0
        fac = nv.get('fac', 0)
        nph = nph_of(fac)
        if fac in (1, 2, 3, 4):            # TOP·LP·WAKE·ANT 측정: CPU 리텐션(평소처럼 CPU 도 꺼지게)
            self.cpu_ret = True
        if fac == 1:                       # TOP 측정만 핀 고정 — LP·WAKE 측정은 TOP 이 켜진 채다
            self.holds = set(HOLD_PINS)
        adc_resident = fac in (2, 3, 4)    # ★LP·WAKE·ANT 는 ADC 유닛을 평소처럼 상주(MODEM·TOP 켜짐)
        lp0 = self.lp_cnt
        n_per = phase_s // samp_s
        idx, stop, t = 0, False, 0
        cc_cal_on = False
        nv['s%02d' % idx] = pack(0, self._measure(0, 0, False if adc_resident else phase_wants_pd(0), mv_fn),
                                 0, 0, False, False, FLAGS_NONE, False)
        idx += 1
        nv['n'] = idx
        for p in range(nph):
            kind = kind_of(fac, p)
            pd = (kind != 'A') if fac in (3, 4) else phase_wants_pd(p)
            if fac == 2:                   # 단계 시작 때 LP 코어를 멈추거나 다시 돌린다(모의 실행도 같다)
                nv['lpk%d' % p] = 1 if (self._lp_pause() if pd else self._lp_resume()) else 0
            if fac == 4:                   # ★2026-10-21 안테나: B 만 구동(옛 동작), A·C 는 놓음 — 읽힘을 an%d 에
                if kind == 'B':
                    self.ant = 'drive'
                    nv['an%d' % p] = 4 | (0 if self.ant_drive_fail else 2)        # EN=0, SEL=1
                else:
                    self.ant = 'release'
                    nv['an%d' % p] = 1 if self.ant_res else 2                     # 반대 풀 걸고: EN=1, SEL=0
            if fac in (3, 4) and kind == 'C':   # 교정 부하: CC1101 SIDLE → IDLE, MARCSTATE 를 값으로 남긴다
                if not self.cc_wake_fail:
                    self.cc_state = 'IDLE'
                nv['cs%d' % p] = MARC_IDLE if self.cc_state == 'IDLE' else MARC_SLEEP
                cc_cal_on = True
            wk = 0
            for i in range(n_per):
                self.sleeps.append((p, self.adc_unit))
                if dry:
                    flags = FLAGS_NONE
                elif fac in (1, 2, 3, 4):
                    # PERIPHERAL_PD 설정에서 ADC 유닛은 MODEM 과 TOP 을 둘 다 붙잡는다
                    pdb = 0 if self.adc_unit else ((1 << BIT_MODEM) |
                                                   (0 if self.top_blocked else (1 << BIT_TOP)))
                    flags = FLAGS_BASE_TOP | pdb
                else:
                    flags = FLAGS_BASE | (0 if self.adc_unit else (1 << BIT_MODEM))
                if fac == 3 and kind == 'B':
                    wk += int(round(self.wake_rate * samp_s))   # 모의 실행도 같은 박자로 돈다(vTaskDelay)
                elif not dry:
                    wk += 1
                t += samp_s
                if self.lp_running:
                    self.lp_cnt += LP_RATE * samp_s
                if events.get('reset_at') == idx:
                    raise Reset()
                usb_now = (not dry) and events.get('usb_at') == idx
                last = (p == nph - 1 and i == n_per - 1)
                next_p = p if i + 1 < n_per else p + 1
                pd_next = False if (last or adc_resident) else phase_wants_pd(next_p)
                mv10 = self._measure(t, p, pd_next, mv_fn)
                nv['s%02d' % idx] = pack(t, mv10, p, 0 if dry else WAKE_TIMER,
                                         usb_now, pd, flags, False)
                idx += 1
                nv['n'] = idx
                if usb_now:
                    stop = True
                if i + 1 == n_per or stop:  # ★펌웨어와 같이 USB 중단 때도 그 단계까지 남긴다
                    nv['lp%d' % p] = self.lp_cnt - lp0   # 단계 끝마다 누적(펌웨어 "lp%u")
                    if fac in (3, 4):
                        nv['wk%d' % p] = wk
                    if cc_cal_on:          # C 끝 — MARCSTATE 확인 후 SPWD
                        nv['ce%d' % p] = MARC_IDLE if self.cc_state == 'IDLE' else MARC_SLEEP
                        if not self.cc_spwd_fail:
                            self.cc_state = 'SPWD'
                        cc_cal_on = False
                if stop:
                    break
            if stop:
                break
        if cc_cal_on and not self.cc_spwd_fail:   # 안전망 — IDLE 로 두고 나가지 않는다
            self.cc_state = 'SPWD'
        if fac == 4:                       # ★2026-10-21 측정 뒤(USB 중단 포함) 안테나를 평소 설정으로
            self.ant = 'prod'
        nv['lp_d'] = self.lp_cnt - lp0
        # 측정 끝 탐침: VERSION 읽기로 깨울 때 이미 깨어 있었으면 cc_awk=1
        nv['cc_awk'] = 1 if self.cc_state == 'IDLE' else 0
        if not self.cc_spwd_fail:
            self.cc_state = 'SPWD'
        self.holds = set()                 # ★CC1101 탐침 전에 반드시 푼다(안 풀면 SPI 가 안 나간다)
        # VERSION 읽기 — CSn 이 붙잡혀 있으면 칩이 무시해 0xFF, 풀렸으면 0x14
        nv['cc_ver'] = 0xFF if 'CS' in self.holds else 0x14
        nv['run'] = RUN_USB if stop else RUN_DONE
        # ADC 유닛을 기본 상태로 되돌린다
        self.adc_keep = self.adc_keep_default
        self.adc_unit = self.adc_keep_default

    def samples(self):
        return [unpack(self.nvs['s%02d' % i]) for i in range(self.nvs.get('n', 0))]


class Cell:
    """전압 모델: OCV 강하 + 부하에 따른 처짐(지수 완화) + 잡음."""

    def __init__(self, currents_ma, k=1.2, sag_mv_per_ma=2.0, tau_s=900.0, noise=0.3,
                 before_ma=4.5, seed=1):
        self.i = currents_ma
        self.k, self.sag, self.tau, self.noise = k, sag_mv_per_ma, tau_s, noise
        self.ocv = 4100.0
        self.off = -sag_mv_per_ma * before_ma      # 직전까지 평소 부하였다
        self.t = 0
        self.rng = random.Random(seed)

    def __call__(self, t, p):
        cur = self.i[p]
        dt = t - self.t
        steps = max(1, int(dt // 10))
        for _ in range(steps if dt else 0):
            h = dt / steps
            self.ocv -= self.k * cur * h / 3600.0
            self.off += (-self.sag * cur - self.off) * (h / self.tau)
        self.t = t
        return self.ocv + self.off + self.rng.gauss(0, self.noise)


def _settle_and_boot(d, usb_at_boot=False, **kw):
    """예약된 기기에서 전원을 끊고 정착을 기다린 뒤 재부팅까지."""
    for _ in range(d.ram_settle + 5):
        if d.tick(False) == 'RESTART':
            break
    d.boot(usb_at_boot, **kw)


def main():
    cases = 0

    # ① 정상: USB 에서 예약 → 끊음 → 정착 → 재부팅 → 4단계 → 정상 부팅 -----------
    d = Dev()
    assert d.cmd(3600, 3600, 300) == 'ARMED'
    for _ in range(30):
        assert d.tick(True) is None
    _settle_and_boot(d, mv_fn=Cell([3.0, 0.3, 3.0, 0.3]))
    s = d.samples()
    assert d.restarts == 1 and d.nvs['run'] == RUN_DONE and d.nvs['armed'] == 0
    assert len(s) == 4 * 12 + 1 and s[0]['flags'] == FLAGS_NONE and s[-1]['t_s'] == 4 * 3600
    assert d.nvs.get('oled_off') == 1, '★화면을 끄지 않고 들어가면 패널 전류가 섞인다'
    assert all(alive == (not phase_wants_pd(p)) for p, alive in d.sleeps), \
        '★켜짐 단계에선 유닛이 살아 있고 꺼짐 단계에선 없어야 한다'
    assert d.adc_created == len(s), '★모든 표본은 새로 만든 유닛으로 잰다'
    assert d.adc_unit == d.adc_keep_default and d.adc_keep == d.adc_keep_default, \
        '★끝난 뒤 ADC 유닛이 기본(상주)으로 돌아와야 한다 — 안 그러면 Thread 가 멈춘다'
    assert d.ram_armed == 0
    a = analyse(s)
    assert a['verdict'] == 'COSTS', a
    assert a['S'] > 2.0, a                                   # 톱니가 뚜렷하다
    assert a['dI_lo'] <= 2.7 <= a['dI_hi'], a                # 참값 2.7mA 가 추정 범위 안
    assert abs((a['M_lo'] + a['M_hi']) / 2 - 1.65) < 0.5, a  # 주기 평균 = (3.0+0.3)/2
    # ★단계별 기울기 차이는 완화 때문에 부풀어 보인다 — 그래서 판정에 쓰지 않는다
    assert (a['s_pd'] - a['s_on']) / 1.2 > 2.7 * 1.3, '완화 모델이 바뀌었으면 이 설명도 다시 볼 것'
    cases += 1

    # ② 예약해 놓고 전원을 안 끊음 → 아무 일도 없다 -----------------------------
    d = Dev()
    d.cmd(3600)
    for _ in range(4000):
        assert d.tick(True) is None
    assert d.restarts == 0
    cases += 1

    # ③ 끊었다가 정착 전에 다시 꽂음 → 발동 안 함 -------------------------------
    d = Dev()
    d.cmd(3600, 600)
    for usb, n in ((False, 500), (True, 5), (False, 500)):
        for _ in range(n):
            assert d.tick(usb) is None, '★off_s 가 리셋 안 되면 여기서 잘못 발동한다'
    cases += 1

    # ④ 취소 ---------------------------------------------------------------------
    d = Dev()
    d.cmd(3600, 60)
    assert d.cmd(0) == 'CANCEL' and d.ram_armed == 0
    for _ in range(200):
        assert d.tick(False) is None
    cases += 1

    # ⑤ ★측정 도중 리셋 → 다시 시작하지 않고, 받은 표본은 남는다 -----------------
    d = Dev()
    d.cmd(3600, 60)
    try:
        _settle_and_boot(d, events={'reset_at': 20})
        raise AssertionError('리셋이 일어나야 한다')
    except Reset:
        pass
    assert d.nvs['run'] == RUN_ACTIVE and d.nvs['n'] == 20
    d.boot(False, reset_reason=9)
    assert d.nvs['run'] == RUN_RESET and d.nvs['rst'] == 9 and d.nvs['armed'] == 0
    assert d.nvs['n'] == 20 and len(d.samples()) == 20, '★받은 표본이 사라지면 안 된다'
    boots = d.boots
    d.boot(False)
    assert d.nvs['run'] == RUN_RESET and d.boots == boots + 1 and d.restarts == 1, \
        '★리셋된 측정을 자동으로 다시 돌리면 재부팅 루프가 된다'
    cases += 1

    # ⑥ 측정 중 USB 를 다시 꽂음 → 다음 표본에서 중단하고 정상 부팅 ---------------
    d = Dev()
    d.cmd(3600, 60)
    _settle_and_boot(d, events={'usb_at': 7})
    s = d.samples()
    assert d.nvs['run'] == RUN_USB and len(s) == 8 and s[-1]['usb'] == 1
    assert d.adc_unit == d.adc_keep_default, '★중단해도 ADC 유닛은 기본 상태로'
    cases += 1

    # ⑦ ★회귀: 부팅한 **뒤** 콘솔로 예약 → 감시가 반드시 돈다 ---------------------
    d = Dev()
    d.boot(True)
    for _ in range(10):
        d.tick(True)
    assert d.cmd(3600, 60) == 'ARMED'
    _settle_and_boot(d)
    assert d.restarts == 1 and d.nvs['run'] == RUN_DONE, \
        '★부팅 후 예약이 발동하지 않는다 — 12시간을 날린 그 버그다'
    cases += 1

    # ⑧ 예약된 채 USB 로 부팅 → RAM 플래그·정착시간 복원 → 끊으면 발동 ------------
    d = Dev()
    d.cmd(3600, 1800)
    d.boot(True)                                   # 재부팅(예: 플래시) 뒤에도 예약 유지
    assert d.ram_armed == 1 and d.ram_settle == 1800 and d.nvs['run'] == RUN_NONE
    for _ in range(1799):
        assert d.tick(False) is None, '★정착 전에 발동하면 완화 잡음이 섞인다'
    assert d.tick(False) == 'RESTART'
    d.boot(False)
    assert d.nvs['run'] == RUN_DONE
    cases += 1

    # ⑨ ★루프 방어: 훅은 계속 "USB 있음"으로 읽는데 메인 루프는 "없음" ------------
    d = Dev()
    d.cmd(3600, 60)
    for _ in range(10):
        for _ in range(70):
            if d.tick(False) == 'RESTART':
                break
        d.boot(True)                               # 훅이 USB 로 오독 → 측정 안 함
    assert d.restarts == MAX_TRIES and d.nvs['armed'] == 0 and d.ram_armed == 0, \
        '★한도를 넘으면 예약을 버려야 한다 (718회 재부팅 사고)'
    cases += 1

    # ⑩ 인자 검증 -----------------------------------------------------------------
    d = Dev()
    assert d.cmd(30) == 'RANGE' and d.cmd(99999) == 'RANGE' and d.cmd(3600, 3600, 2) == 'RANGE'
    assert d.cmd(600, 60, 600) == 'NPER', '단계당 표본 1개로는 기울기를 못 잰다'
    assert d.cmd(7200, 60, 30) == 'NPER', '표본이 96개를 넘으면 NVS 키가 모자란다'
    assert d.cmd(3600, 0)[0] == 'A' and d.nvs['settle_s'] == SETTLE_MIN_S
    assert d.cmd(3600) == 'DUP', '중복 예약이 첫 예약을 덮어쓰면 안 된다'
    d = Dev()
    d.dsleep_armed = 1
    assert d.cmd(3600) == 'DSLEEP', '깊은잠 예약과 겹치면 안 된다'
    cases += 1

    # ⑪ NVS 자리 부족 → 거부 -------------------------------------------------------
    d = Dev(free_entries=40)
    assert d.cmd(3600, 60, 300) == 'NOSPACE' and not d.nvs.get('armed')
    cases += 1

    # ⑫ ★옛 결과가 남은 채 재예약 → 지워져야 한다 ---------------------------------
    d = Dev()
    d.cmd(120, 60, 20)
    _settle_and_boot(d)
    assert d.nvs['n'] == 25
    assert d.cmd(3600, 60) == 'ARMED'
    assert d.nvs['n'] == 0 and 's05' not in d.nvs and d.nvs['run'] == RUN_NONE, \
        '★옛 표본이 남으면 새 측정이 무효일 때 옛 값이 새 결과처럼 보인다'
    cases += 1

    # ⑬ 모의 실행: USB 꽂힌 채 바로 돈다. 절전 플래그 없음, USB 중단 없음 ----------
    d = Dev()
    assert d.cmd(40, 60, 10, dry=1) == 'RESTART' and d.ram_armed == 0
    d.boot(True)
    s = d.samples()
    assert d.nvs['run'] == RUN_DONE and len(s) == 17 and all(x['flags'] == FLAGS_NONE for x in s)
    assert d.restarts == 0 and d.dry_restarts == 1
    assert analyse(s)['verdict'] == 'DRY'
    d.boot(True)
    assert d.nvs['run'] == RUN_DONE and len(d.samples()) == 17, '★모의 실행도 한 번만 돈다'
    cases += 1

    # ⑭ 기본이 '측정할 때만'(adcfix=1)인 보드에서도 끝나면 그 기본으로 돌아간다 ----
    d = Dev(adc_keep_default=False)
    d.cmd(120, 60, 20)
    _settle_and_boot(d)
    assert d.adc_unit is False and d.adc_keep is False
    assert all(alive == (not phase_wants_pd(p)) for p, alive in d.sleeps)
    cases += 1

    # ⑮ 짧은 리허설(2분×4, 20초 표본) — 실제로 쓸 인자 ------------------------------
    d = Dev()
    assert d.cmd(120, 60, 20) == 'ARMED'
    _settle_and_boot(d, mv_fn=Cell([3.0, 0.3, 3.0, 0.3]))
    s = d.samples()
    assert len(s) == 25 and s[-1]['t_s'] == 480
    assert [x['phase'] for x in s[1:]] == [p for p in range(4) for _ in range(6)]
    cases += 1

    # ⑯ 판정: 가설별로 갈리는가 (잡음·완화 포함) ------------------------------------
    def run(currents, seed=1, phase_s=3600, **kw):
        dd = Dev()
        dd.cmd(phase_s, 60, 300)
        _settle_and_boot(dd, mv_fn=Cell(currents, seed=seed, **kw))
        return analyse(dd.samples())
    small, large = [], []
    for seed in range(1, 21):
        assert run([3.0, 0.3, 3.0, 0.3], seed)['verdict'] == 'COSTS', seed
        assert run([3.0, 0.3, 3.0, 0.3], seed, k=0.9)['verdict'] == 'COSTS', seed
        a = run([1.3, 0.3, 1.3, 0.3], seed)            # 1mA 차이도 잡아야 한다
        assert a['verdict'] == 'COSTS' and a['dI_lo'] <= 1.0 <= a['dI_hi'], (seed, a)
        small.append(run([0.3, 0.3, 0.3, 0.3], seed)['verdict'])
        large.append(run([3.0, 3.0, 3.0, 3.0], seed)['verdict'])
    # ★차이가 없는데 "MODEM 이 쓴다"고 하면 안 된다(거짓 양성 0). 잡음이 크면 "모른다"는 허용.
    assert set(small) <= {'FLOOR_SMALL', 'UNCLEAR'} and small.count('FLOOR_SMALL') >= 16, small
    assert set(large) <= {'FLOOR_LARGE_NOT_FACTOR', 'UNCLEAR'} and large.count('FLOOR_LARGE_NOT_FACTOR') >= 16, large
    # 작은 차이(0.4mA): 잡히기도 하고 안 잡히기도 한다 — 잡혔을 때 크기를 부풀리지만 않으면 된다
    for seed in range(1, 21):
        a = run([0.7, 0.3, 0.7, 0.3], seed)
        assert a['verdict'] in ('COSTS', 'FLOOR_SMALL', 'UNCLEAR'), (seed, a['verdict'])
        assert a['dI_hi'] < 2.0, '★0.4mA 차이를 2mA 넘게 부풀려 말하면 안 된다'
    cases += 1

    # ⑰ 플래그가 의도와 다르면 무효 --------------------------------------------------
    d = Dev()
    d.cmd(3600, 60)
    _settle_and_boot(d, mv_fn=Cell([3.0, 0.3, 3.0, 0.3]))
    s = d.samples()
    for x in s:
        if x['phase'] == 1 and x['flags'] != FLAGS_NONE:
            x['flags'] &= ~(1 << BIT_MODEM)        # 끄라고 했는데 실제론 안 꺼짐
    assert analyse(s)['verdict'] == 'INVALID', '★의도대로 안 바뀐 측정으로 결론을 내면 안 된다'
    cases += 1

    # ⑱ 비트 포장 왕복 + 상한 --------------------------------------------------------
    v = unpack(pack(14400, 41234, 3, 4, True, True, 0x3C16, True))
    assert v == {'t_s': 14400, 'mv': 4123.4, 'phase': 3, 'cause': 4, 'usb': 1,
                 'want_pd': 1, 'flags': 0x3C16, 'rej': 1}, v
    v = unpack(pack(9999999, 99999, 2, 99, False, False, 0x12345, False))
    assert v['t_s'] == 0xFFFFF and v['mv'] == 6553.5 and v['cause'] == 15 and v['flags'] == 0x2345
    assert pack(0xFFFFF, 0xFFFF, 7, 15, 1, 1, 0xFFFF, 1) < (1 << 62)
    cases += 1

    # ⑲ ★2026-10-15 TOP 측정(fac=1): CPU 리텐션·핀 고정, 꺼짐 단계에 TOP·MODEM 비트, 끝나면 핀 해제 -----
    d = Dev()
    assert d.cmd(4500, 3600, 300, fac=1) == 'ARMED' and d.nvs['fac'] == 1
    _settle_and_boot(d, mv_fn=Cell([3.0, 0.3, 3.0, 0.3]))
    s = d.samples()
    assert d.cpu_ret, '★CPU 리텐션을 안 올리면 CPU 도 안 꺼지고, 그러면 TOP 도 못 끈다'
    assert d.holds == set(), '★핀을 안 풀고 정상 부팅으로 넘어가면 SPI·RMT 가 굳어 RF 가 죽는다'
    assert d.nvs['cc_ver'] == 0x14, '측정 뒤 VERSION 이 읽혀야 SPI 가 산 것이다'
    for x in s[1:]:
        top = (x['flags'] >> BIT_TOP) & 1
        assert top == (1 if phase_wants_pd(x['phase']) else 0), x
        assert (x['flags'] >> BIT_CPU) & 1, 'TOP 측정에서는 모든 절전에 CPU 차단 비트가 있어야 한다'
    a = analyse(s, fac=1)
    assert a['verdict'] == 'COSTS' and a['factor'] == 'TOP', a['verdict']
    a0 = analyse(s, fac=0)                         # MODEM 비트도 같이 바뀌므로 fac=0 으로 읽어도 유효
    assert a0['verdict'] == 'COSTS'
    cases += 1

    # ⑳ TOP 이 막혀 안 꺼짐(리텐션 없는 드라이버) → 측정 무효로 판정해야 한다 -------------------------
    d = Dev()
    d.top_blocked = True
    d.cmd(4500, 3600, 300, fac=1)
    _settle_and_boot(d, mv_fn=Cell([3.0, 3.0, 3.0, 3.0]))
    a = analyse(d.samples(), fac=1)
    assert a['verdict'] == 'INVALID', '★TOP 이 실제로 안 꺼졌는데 "TOP 무관"으로 결론 내면 안 된다'
    cases += 1

    # ㉑ TOP 측정 도중 리셋 — 다음 부팅에서 핀이 풀리고, 측정은 다시 돌지 않는다 ------------------------
    d = Dev()
    d.cmd(4500, 3600, 300, fac=1)
    try:
        _settle_and_boot(d, events={'reset_at': 9})
        raise AssertionError('리셋이 일어나야 한다')
    except Reset:
        pass
    assert d.holds == set(HOLD_PINS), '리셋 직후에는 hold 가 남아 있다(하드웨어와 같다)'
    d.boot(False, reset_reason=9)
    assert d.holds == set(), '★다음 부팅 맨 앞에서 측정용 hold 를 전부 풀어야 한다'
    assert d.nvs['run'] == RUN_RESET and len(d.samples()) == 9
    cases += 1

    # ㉒ ★2026-10-16 LP 측정(fac=2) 정상: 단계마다 LP 를 멈추고/돌리고, ADC 는 상주, 핀 고정 없음 ----------
    def lp_run(currents, seed=1, **dev_kw):
        dd = Dev()
        for k_, v_ in dev_kw.items():
            setattr(dd, k_, v_)
        assert dd.cmd(4500, 7200, 300, fac=2) == 'ARMED' and dd.nvs['fac'] == 2
        dd.adc_created = 0
        dd.sleeps = []
        _settle_and_boot(dd, mv_fn=Cell(currents, seed=seed))
        ss = dd.samples()
        return dd, ss, analyse(ss, fac=2, lprates=lp_rates(dd.nvs, ss))
    d, s, a = lp_run([3.2, 1.7, 3.2, 1.7])
    assert a['verdict'] == 'COSTS' and a['factor'] == 'LP', a
    assert d.cpu_ret and d.holds == set(), 'LP 측정: CPU 리텐션은 올리고 핀은 붙잡지 않는다'
    assert all(u for _, u in d.sleeps), '★LP 측정 중에는 ADC 유닛이 상주해야 평소와 같은 조건(MODEM·TOP 켜짐)이다'
    assert all(x['flags'] == FLAGS_BASE_TOP for x in s[1:]), [hex(x['flags']) for x in s[1:]]
    assert [d.nvs['lpk%d' % p] for p in range(PHASES)] == [1, 1, 1, 1]
    r = lp_rates(d.nvs, s)
    assert [round(x) for x in r] == [LP_RATE, 0, LP_RATE, 0], r
    assert d.lp_running, '★측정이 끝나면(마지막 단계 = 멈춤) 정상 부팅이 LP 코어를 다시 돌려야 버튼이 산다'
    assert d.nvs['cc_ver'] == 0x14
    a_nolp = analyse(s, fac=2)                    # 단계별 LP 기록이 없으면 검증할 수 없다 → 무효
    assert a_nolp['verdict'] == 'INVALID', '★LP 가 실제로 멈췄는지 모르는 채 결론을 내면 안 된다'
    cases += 1

    # ㉓ LP 가 안 멈춤(멈출 때마다 I2C 선이 LOW → 5회 재시도 끝에 다시 돌림) → 무효 -----------------------
    d, s, a = lp_run([3.2, 3.2, 3.2, 3.2], lp_bus_low=True)
    assert [d.nvs['lpk%d' % p] for p in range(PHASES)] == [1, 0, 1, 0]
    assert a['verdict'] == 'INVALID', '★LP 가 계속 돌았는데 "LP 무관"으로 결론 내면 안 된다'
    cases += 1

    # ㉔ 다시 돌리기 실패 — 단계 1 에서 멈춘 뒤 단계 2 에서 못 살림 → 무효 --------------------------------
    d, s, a = lp_run([3.2, 1.7, 1.7, 1.7], lp_run_fail=True)
    r = lp_rates(d.nvs, s)
    assert [round(x) for x in r] == [LP_RATE, 0, 0, 0], r
    assert a['verdict'] == 'INVALID', a
    cases += 1

    # ㉕ LP 모의 실행(USB) — 절전 없이도 LP 켬/끔은 그대로 돈다 → 흐름과 LP 멈춤을 낮에 확인할 수 있다 -------
    d = Dev()
    assert d.cmd(20, 60, 5, dry=1, fac=2) == 'RESTART'
    d.boot(True)
    s = d.samples()
    a = analyse(s, fac=2, lprates=lp_rates(d.nvs, s))
    assert a['verdict'] == 'DRY' and len(s) == 17, (a['verdict'], len(s))
    assert [round(x) for x in lp_rates(d.nvs, s)] == [LP_RATE, 0, LP_RATE, 0]
    assert d.nvs['run'] == RUN_DONE and d.lp_running
    cases += 1

    # ㉖ LP 측정 판정: 몫이 있으면 잡고(20/20), 없으면 거짓 양성 0 ------------------------------------------
    for seed in range(1, 21):
        assert lp_run([3.2, 1.7, 3.2, 1.7], seed)[2]['verdict'] == 'COSTS', seed
        v = lp_run([3.2, 3.2, 3.2, 3.2], seed)[2]['verdict']
        assert v in ('FLOOR_LARGE_NOT_FACTOR', 'UNCLEAR', 'FLOOR_MID'), (seed, v)
    cases += 1

    # ㉗ LP 측정 도중 리셋(멈춤 단계) — 다음 부팅이 RUN_RESET 으로 끝내고 LP 코어를 다시 돌린다 ----------------
    d = Dev()
    d.cmd(4500, 7200, 300, fac=2)
    try:
        _settle_and_boot(d, events={'reset_at': 20})   # 단계 1(멈춤) 도중
        raise AssertionError('리셋이 일어나야 한다')
    except Reset:
        pass
    assert not d.lp_running
    d.boot(False, reset_reason=9)
    assert d.nvs['run'] == RUN_RESET and d.lp_running, '★리셋 뒤에도 LP 코어가 살아나야 버튼이 동작한다'
    cases += 1

    # ㉘ ★2026-10-16 WAKE 측정(fac=3) 흐름: 5단계 ABACA, B 깨어남·C CC1101 IDLE·끝나면 SPWD -----------
    def wk_run(dB=1.9, floor=1.8, cal=1.7, seed=1, phase_s=3600, samp_s=300, dev_kw=None, **cell_kw):
        dd = Dev()
        for k_, v_ in (dev_kw or {}).items():
            setattr(dd, k_, v_)
        assert dd.cmd(phase_s, 7200, samp_s, fac=3) == 'ARMED' and dd.nvs['nph'] == 5
        dd.sleeps = []
        cur = [floor, floor + dB, floor, floor + cal, floor]
        _settle_and_boot(dd, mv_fn=Cell(cur, seed=seed, **cell_kw))
        ss = dd.samples()
        return dd, ss, analyse_wake(ss, dd.nvs, 5)
    d, s, a = wk_run()
    assert len(s) == 61 and [x['phase'] for x in s[1:]] == [p for p in range(5) for _ in range(12)]
    assert d.cpu_ret and d.holds == set() and all(u for _, u in d.sleeps), 'ADC 상주·CPU 리텐션·핀 고정 없음'
    assert all(x['flags'] == FLAGS_BASE_TOP for x in s[1:]), '평소와 같은 절전 플래그(0x3C12)여야 한다'
    assert d.nvs['cs3'] == MARC_IDLE and d.nvs['ce3'] == MARC_IDLE and d.nvs['cc_awk'] == 0
    assert d.cc_state == 'SPWD', '★C 단계 뒤 CC1101 을 다시 재워야 평소에 1.7mA 가 안 남는다'
    assert abs(d.nvs['wk1'] / 3600.0 - 31.8) < 0.5 and d.nvs['wk0'] == 12 and d.nvs['wk3'] == 12
    assert a['verdict'] == 'WAKE_COST' and abs(a['dB'] - 1.9) < 0.6, a
    cases += 1

    # ㉙ 판정 정확도(20시드): 깨어남 1.9mA / 0.5mA / 0 — 그리고 셀 계수가 3배 달라도 같은 답 -----------------
    for seed in range(1, 21):
        a = wk_run(dB=1.9, seed=seed)[2]
        assert a['verdict'] == 'WAKE_COST' and 1.3 < a['dB'] < 2.5, (seed, a.get('dB'))
        a = wk_run(dB=0.5, seed=seed)[2]
        assert a['verdict'] == 'WAKE_COST' and a['dB'] < 1.0, (seed, a.get('dB'))
        a = wk_run(dB=0.0, seed=seed)[2]
        assert a['verdict'] == 'WAKE_COST' and a['dB_lo'] < 0.3, ('★깨어남이 공짜인데 비싸다고 하면 안 된다', seed)
        for kk in (0.44, 1.3):     # ★현장 교정의 핵심 — 4.03V(평탄부)와 4.06V 의 계수 차가 답을 바꾸면 안 된다
            a = wk_run(dB=1.9, seed=seed, k=kk)[2]
            assert a['verdict'] == 'WAKE_COST' and 1.3 < a['dB'] < 2.5, (seed, kk, a.get('dB'))
    cases += 1

    # ㉚ 교정 부하가 안 켜짐(CC1101 이 SIDLE 에 안 깨어남) → 무효 ---------------------------------------------
    d, s, a = wk_run(cal=0.0, dev_kw={'cc_wake_fail': True})
    assert d.nvs['cs3'] == MARC_SLEEP and a['verdict'] == 'INVALID', a['verdict']
    cases += 1

    # ㉛ C 뒤 SPWD 실패 → 측정 끝에 깨어 있었다(cc_awk=1) → 무효(1.7mA 가 마지막 A 에 섞였다) -------------------
    d, s, a = wk_run(dev_kw={'cc_spwd_fail': True})
    assert d.nvs['cc_awk'] == 1 and a['verdict'] == 'INVALID', a['verdict']
    cases += 1

    # ㉜ B 단계 깨어남이 실제로 안 일어남(속도 0) → 무효 — "깨어남이 공짜"로 잘못 결론 내면 안 된다 -----------
    d, s, a = wk_run(dB=0.0, dev_kw={'wake_rate': 0.0})
    assert a['verdict'] == 'INVALID', a['verdict']
    cases += 1

    # ㉝ 모의 실행(USB): 5단계, B 깨어남 횟수·C 전환·SPWD 복귀까지 낮에 확인 ------------------------------------
    d = Dev()
    assert d.cmd(20, 60, 5, dry=1, fac=3) == 'RESTART'
    d.boot(True)
    s = d.samples()
    a = analyse_wake(s, d.nvs, 5)
    assert a['verdict'] == 'DRY' and len(s) == 21 and d.nvs['run'] == RUN_DONE, (a['verdict'], len(s))
    assert d.nvs['cs3'] == MARC_IDLE and d.nvs['ce3'] == MARC_IDLE and d.cc_state == 'SPWD'
    assert abs(d.nvs['wk1'] / 20.0 - 31.8) < 1.0
    cases += 1

    # ㉞ C 단계 도중 리셋 → 다음 부팅의 cc1101_init(훅보다 먼저)이 SPWD 로 재운다, 측정은 다시 안 돈다 ----------
    d = Dev()
    d.cmd(3600, 7200, 300, fac=3)
    try:
        _settle_and_boot(d, events={'reset_at': 40})    # 단계 3(C) = 표본 37~48
        raise AssertionError('리셋이 일어나야 한다')
    except Reset:
        pass
    assert d.cc_state == 'IDLE', '리셋 직후에는 CC1101 이 IDLE 로 남아 있다(별개 칩)'
    d.boot(False, reset_reason=9)
    assert d.cc_state == 'SPWD' and d.nvs['run'] == RUN_RESET
    cases += 1

    # ㉟ 범위: 5단계라 단계당 19개까지 — 60분/180초(20개)는 거부, 190초(18개)는 받는다 ------------------------
    assert Dev().cmd(3600, 7200, 180, fac=3) == 'NPER'
    assert Dev().cmd(3600, 7200, 190, fac=3) == 'ARMED'
    assert Dev().cmd(3600, 7200, 180, fac=2) == 'ARMED'  # 4단계는 그대로 23개까지
    cases += 1

    # ㊱ ★2026-10-21 ANT 측정(fac=4) 흐름: ABACA, B 만 안테나 구동, C CC1101 IDLE, 끝나면 안테나 평소 설정 ----------
    def ant_run(dI=0.98, floor=1.8, cal=1.7, seed=1, phase_s=3600, samp_s=300, dev_kw=None, **cell_kw):
        dd = Dev()
        for k_, v_ in (dev_kw or {}).items():
            setattr(dd, k_, v_)
        assert dd.cmd(phase_s, 7200, samp_s, fac=4) == 'ARMED' and dd.nvs['nph'] == 5 and dd.nvs['fac'] == 4
        dd.sleeps = []
        cur = [floor, floor + dI, floor, floor + cal, floor]
        _settle_and_boot(dd, mv_fn=Cell(cur, seed=seed, **cell_kw))
        ss = dd.samples()
        return dd, ss, analyse_wake(ss, dd.nvs, 5)
    d, s, a = ant_run()
    assert len(s) == 61 and d.cpu_ret and all(u for _, u in d.sleeps) and d.holds == set()
    assert [d.nvs['an%d' % p] for p in range(5)] == [1, 6, 1, 1, 1], '놓음=0b001(EN=1,SEL=0), 구동=0b110'
    assert d.ant == 'prod', '★측정 뒤 안테나를 평소 설정으로 되돌려야 외장으로 붙는다'
    assert d.nvs['wk1'] == 12 and d.nvs['wk3'] == 12, 'ANT 의 B 단계는 깨어남을 섞지 않는다(표본당 절전 1회)'
    assert a['verdict'] == 'ANT_COST' and abs(a['dB'] - 0.98) < 0.4 and all(a['ant_proof']), a
    cases += 1

    # ㊲ 판정 정확도(20시드): 0.98mA / 0.33mA(내장 모드면 R21 만) / 0 — 셀 계수가 3배 달라도 같은 답 -------------
    for seed in range(1, 21):
        a = ant_run(dI=0.98, seed=seed)[2]
        assert a['verdict'] == 'ANT_COST' and 0.55 < a['dB'] < 1.45, (seed, a.get('dB'))
        a = ant_run(dI=0.33, seed=seed)[2]
        assert a['verdict'] == 'ANT_COST' and a['dB'] < 0.75, (seed, a.get('dB'))
        a = ant_run(dI=0.0, seed=seed)[2]
        assert a['verdict'] == 'ANT_COST' and a['dB_lo'] < 0.3, ('★전류가 없는데 있다고 하면 안 된다', seed)
        for kk in (0.44, 1.3):
            a = ant_run(dI=0.98, seed=seed, k=kk)[2]
            assert a['verdict'] == 'ANT_COST' and 0.55 < a['dB'] < 1.45, (seed, kk, a.get('dB'))
    cases += 1

    # ㊳ 내부 저항이 없는 모듈(놓은 단계 읽힘이 반대) → 판정은 내되 '확인 안 됨'을 밝힌다 -------------------------
    d, s, a = ant_run(dI=0.0, dev_kw={'ant_res': False})
    assert a['verdict'] == 'ANT_COST' and not all(a['ant_proof']) and '확인되지 않았다' in a['why'], a['why']
    cases += 1

    # ㊴ B 단계 구동 실패 → 무효("저항 전류 없음"으로 잘못 결론 내면 안 된다) -------------------------------------
    d, s, a = ant_run(dI=0.0, dev_kw={'ant_drive_fail': True})
    assert a['verdict'] == 'INVALID', a['verdict']
    cases += 1

    # ㊵ 도중 USB 중단·도중 리셋 → 어느 쪽이든 안테나는 평소 설정으로 돌아온다 --------------------------------------
    d = Dev()
    d.cmd(3600, 7200, 300, fac=4)
    _settle_and_boot(d, events={'usb_at': 18})          # 단계 1(B, 구동) 도중
    assert d.nvs['run'] == RUN_USB and d.ant == 'prod'
    d = Dev()
    d.cmd(3600, 7200, 300, fac=4)
    try:
        _settle_and_boot(d, events={'reset_at': 18})
        raise AssertionError('리셋이 일어나야 한다')
    except Reset:
        pass
    assert d.ant == 'drive', '리셋 직후엔 B 단계 구동 상태'
    d.boot(False, reset_reason=9)
    assert d.ant == 'prod' and d.nvs['run'] == RUN_RESET, '★다음 부팅 app_main 이 평소 설정으로'
    cases += 1

    print('lsfloor %d케이스 통과' % cases)
    print('  흐름: 예약→끊음→정착→재부팅→4단계→정상부팅 / 취소 / 도중리셋 / USB중단 / 루프방어')
    print('  판정: 3.0↔0.3·1.3↔0.3mA=COSTS(20/20), 차이 없음=거짓 양성 0 (20시드) / TOP 측정 3케이스')
    print('  LP 측정(fac=2): 3.2↔1.7mA=COSTS(20/20)·거짓 양성 0 / 안 멈춤·못 살림=무효 / 모의·리셋 뒤 LP 재가동')
    print('  ANT 측정(fac=4, 10/21): 0.98mA→0.55~1.45(20시드, K 0.44·1.2·1.3) / 0.33→<0.75 / 0→거짓 양성 0 / '
          '저항 없음=확인 안 됨 표시 / 구동 실패=무효 / USB·리셋 뒤 안테나 평소 설정')
    print('  WAKE 측정(fac=3): 1.9mA→1.3~2.5(20시드, K 0.44·1.2·1.3 모두) / 0.5→<1.0 / 0→거짓 양성 0'
          ' / 교정 안 켜짐·SPWD 실패·깨어남 없음=무효 / 모의·C 도중 리셋 뒤 SPWD')
    print('  ※단계별 기울기 차이는 셀 완화로 45%% 넘게 부풀어 판정에 안 쓴다(톱니 진폭 + 주기 평균)' % ())
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
