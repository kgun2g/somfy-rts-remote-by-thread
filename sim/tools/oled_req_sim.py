#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 감사 보고서 F13 시뮬 — 화면 켜기/끄기를 **콜백이 직접**(옛) vs **UI 태스크에 요청**(새).

옛 구조 (somfy_app.c _exit_sleep / _exit_screensaver):
  버튼 태스크(prio 10)·Matter 콜백(CHIP 태스크)이 메인 화면을 직접 그리고(_render_normal → _fb_flush, 비트뱅 I2C)
  oled_ui_set_display_on(true)(0xAF, 3회 재시도)까지 끝낸 **뒤에야** RF 요청으로 넘어간다.
  끄기(_enter_sleep)는 메인 루프(prio 4)가 0xAE 를 직접 보낸다.
새 구조 (oled_ui_request_display):
  요청은 논리 상태(s_panel_on)·요청 플래그(s_panel_sync_req)만 바꾸고 UI 태스크에 통지 → 즉시 반환.
  UI 태스크(prio 3)는 회전 **처음에** 플래그를 거두고, 그 회전에 현재 화면을 그린 **뒤** 명령을 보낸다.
  회전 도중 새 요청이 들어오면 기다리지 않고 다음 회전으로(continue).

검증 항목
  ① 잔상: 0xAF 가 성공하는 순간 패널 GDDRAM 이 요청 시점의 화면보다 옛것인가(옛 화면이 켜져 보임)
  ② 유실: 조용해진 뒤 물리 상태 ≠ 논리 상태 (재시도 대기 중도 아닌데)
  ③ 켜짐 지연(요청 → 0xAF 성공), 꺼짐 지연(요청 → 0xAE 성공)
  ④ 명령 경로가 OLED 때문에 묶인 시간(버튼·Matter 태스크가 I2C 를 하는 시간)
타이밍 근거
  · 비트뱅 페이지(128B) ≈ 3.8ms: BBO_HALF_US=1 → 비트 2us + GPIO 호출 ≈ 바이트 28us × 135B
  · UI 태스크 50ms(화면 ON) / 통지 대기 5000ms(OFF) — oled_ui.h OLED_TASK_INTERVAL_MS/OFF_MS
  · 재시도 간격 5ms(CONFIG_FREERTOS_HZ=1000 이라 pdMS_TO_TICKS(5)=5틱)
  · 새 구조에서 UI 태스크가 CPU 를 얻기까지: 버튼 = 콜백 0.3~1ms + RF 워커 VCO 안정 5ms 스핀(somfy_rts.c) +
    메인 루프 1회전 0.5~3ms ≈ 6~9ms. Matter = 같은 RF 경로. 진동 = 메인 루프만.
"""
import random
import statistics as st
import sys

PAGE_MS   = 3.8
CMD_MS    = 0.12
RETRY_MS  = 5.0
UI_ON_MS  = 50.0
UI_OFF_MS = 5000.0
IDLE_OFF_MS = 10000.0          # 배터리 CFG_SCREEN_OFF_SEC


class Shared:
    def __init__(self):
        self.logical_on = True     # s_panel_on
        self.sync_req = False      # s_panel_sync_req (새 구조만)
        self.pending = False       # s_panel_cmd_pending (F14)
        self.phys_on = True        # 패널이 실제로 받은 마지막 명령
        self.state_ver = 1         # 화면 내용(상태) 버전 — 상태가 바뀔 때마다 +1
        self.screensaver = False   # OLED_STATE_SCREENSAVER (그리지 않음)
        self.gddram_ver = 1        # 패널 메모리에 마지막으로 전송된 화면의 버전
        self.notify = 0            # 태스크 통지 카운트
        self.fail_cnt = 0          # g_oled_onoff_fail_cnt


class Sim:
    def __init__(self, design, pf, seed):
        self.design = design       # 'old' | 'new'
        self.pf = pf               # I2C 명령 1회 실패 확률
        self.rng = random.Random(seed)
        self.S = Shared()
        self.t = 0.0
        self.ui_next = 0.0         # UI 태스크가 다음에 움직일 시각
        self.ui_phase = 'start'
        self.ui_ctx = {}
        self.ev = []               # (시각, 종류, 인자)
        self.m = dict(stale_on=0, stale_ms=[], on_lat=[], off_lat=[], block=[], mismatch=0,
                      cycles=0, sends=0, last_req_on=None, last_req_off=None)
        self.off_req_t = None
        self.on_req_t = None
        self.on_req_ver = None

    # ── I2C 1바이트 명령(재시도 포함) — 걸린 시간과 성공 여부
    def _send(self, tries):
        dur, ok = 0.0, False
        for i in range(tries):
            if i:
                dur += RETRY_MS
            dur += CMD_MS
            self.m['sends'] += 1
            if self.rng.random() >= self.pf:
                ok = True
                break
        return dur, ok

    def _dirty_pages(self):
        return self.rng.randint(1, 4)          # 꺼진 동안 바뀐 시계·상태·배터리 줄

    def _apply_phys(self, on, t_done):
        S = self.S
        S.phys_on = on
        if on and self.on_req_t is not None:
            self.m['on_lat'].append(t_done - self.on_req_t)
            if S.gddram_ver < self.on_req_ver:
                self.m['stale_on'] += 1
                self.stale_since = t_done
            self.on_req_t = None
        if (not on) and self.off_req_t is not None:
            self.m['off_lat'].append(t_done - self.off_req_t)
            self.off_req_t = None

    # ── 외부 사건(더 높은 우선순위 문맥에서의 순간적 메모리 쓰기)
    def _on_request(self, t, src):
        S = self.S
        if S.logical_on:
            return
        S.screensaver = False
        S.state_ver += 1                       # _exit_sleep: 상태 복원(NORMAL/ACTION…)
        self.on_req_t, self.on_req_ver = t, S.state_ver
        if self.design == 'new':
            S.logical_on = True
            S.sync_req = True
            S.notify += 1
            hp = {'btn': self.rng.uniform(6, 9), 'matter': self.rng.uniform(6, 9),
                  'vibe': self.rng.uniform(0.5, 3)}[src]
            # 통지 대기 중이면 HP 작업이 끝나는 대로 UI 태스크가 돈다
            if self.ui_phase == 'wait_notify':
                self.ui_next = min(self.ui_next, t + hp)
            self.m['block'].append(0.0)
        else:
            # 옛: 요청자가 직접 그린 뒤 켠다(그동안 RF 요청이 묶인다)
            dur = self._dirty_pages() * PAGE_MS
            S.gddram_ver = S.state_ver
            S.logical_on = True
            S.notify += 1
            d2, ok = self._send(3)
            S.pending = not ok
            if not ok:
                S.fail_cnt += 1
            else:
                self._apply_phys(True, t + dur + d2)
            self.m['block'].append(dur + d2)
            if self.ui_phase == 'wait_notify':
                self.ui_next = min(self.ui_next, t + dur + d2 + self.rng.uniform(0.5, 3))

    def _off_request(self, t):
        S = self.S
        if not S.logical_on:
            return
        S.screensaver = True
        S.state_ver += 1
        self.off_req_t = t
        if self.design == 'new':
            S.logical_on = False
            S.sync_req = True
            S.notify += 1
        else:
            S.logical_on = False
            d2, ok = self._send(3)
            S.pending = not ok
            if not ok:
                S.fail_cnt += 1
            else:
                self._apply_phys(False, t + d2)

    def _state_change(self, t):
        self.S.state_ver += 1                  # 켜진 동안 화면 내용 변화(애니메이션 등)

    # ── UI 태스크 한 단계
    def _ui_step(self):
        S, c = self.S, self.ui_ctx
        t = self.ui_next
        if self.ui_phase in ('start', 'wait_notify', 'wait_delay'):
            if self.ui_phase == 'wait_notify':
                S.notify = 0                   # ulTaskNotifyTake(pdTRUE) — 돌아올 때 비운다
            c['sync'] = False
            if self.design == 'new':
                c['sync'] = S.sync_req
                if c['sync']:
                    S.sync_req = False
            if not S.screensaver:
                c['ver'] = S.state_ver
                self.ui_phase = 'render'
                self.ui_next = t + self._dirty_pages() * PAGE_MS * (1 if S.logical_on else 1)
                return
            self.ui_phase = 'send'
            self.ui_next = t
            return
        if self.ui_phase == 'render':
            S.gddram_ver = max(S.gddram_ver, c['ver'])
            if getattr(self, 'stale_since', None) is not None and S.gddram_ver >= (self.on_req_ver or 0):
                self.m['stale_ms'].append(t - self.stale_since)
                self.stale_since = None
            self.ui_phase = 'send'
            self.ui_next = t
            return
        if self.ui_phase == 'send':
            sync = c.get('sync', False)
            if sync or S.pending:
                want = S.logical_on
                dur, ok = self._send(3 if sync else 1)
                if ok:
                    S.pending = False
                    self._apply_phys(want, t + dur)
                elif not S.pending:
                    S.pending = True
                    S.fail_cnt += 1
                t += dur
            # 대기
            if self.design == 'new' and S.sync_req:
                self.ui_phase = 'start'
                self.ui_next = t
                return
            if S.logical_on:
                self.ui_phase = 'wait_delay'
                self.ui_next = t + UI_ON_MS
            else:
                if S.notify > 0:
                    self.ui_phase = 'wait_notify'
                    self.ui_next = t
                else:
                    self.ui_phase = 'wait_notify'
                    self.ui_next = t + UI_OFF_MS
            return

    def run(self, n_cycles):
        rng = self.rng
        t = 0.0
        # 사건열: 켜진 채 유휴 10초 → 끄기 → 0.05~60초 뒤 켜기(출처 무작위) → 켜진 동안 화면 변화 몇 번
        evs = []
        for _ in range(n_cycles):
            t += IDLE_OFF_MS
            evs.append((t, 'off', None))
            r = rng.random()
            gap = rng.uniform(0.0, 0.1) * 1000 if r < 0.15 else rng.uniform(50, 60000)   # 15% 는 끄자마자 켜기
            t += gap
            src = rng.choices(['btn', 'matter', 'vibe'], [0.7, 0.2, 0.1])[0]
            evs.append((t, 'on', src))
            for k in range(rng.randint(0, 3)):
                evs.append((t + rng.uniform(1, 3000), 'chg', None))
            evs.append((t + IDLE_OFF_MS - 1, 'check', None))   # 다음 끄기 직전: 불일치 점검
        evs.sort(key=lambda e: e[0])
        i = 0
        self.ui_next = 0.0
        while i < len(evs):
            et, kind, arg = evs[i]
            if self.ui_next <= et:
                self._ui_step()
                continue
            if kind == 'on':
                self._on_request(et, arg)
            elif kind == 'off':
                self._off_request(et)
            elif kind == 'chg':
                if self.S.logical_on:
                    self._state_change(et)
            elif kind == 'check':
                self.m['cycles'] += 1
                if self.S.phys_on != self.S.logical_on and not self.S.pending:
                    self.m['mismatch'] += 1
            i += 1
        return self.m


def pct(a, q):
    if not a:
        return float('nan')
    a = sorted(a)
    return a[min(len(a) - 1, int(q * len(a)))]


def report(name, m):
    on = m['on_lat']; off = m['off_lat']; bl = m['block']
    print(f"  {name:4s} 켜짐지연 중앙 {st.median(on):5.1f} / 99% {pct(on,.99):5.1f} / 최대 {max(on):6.1f}ms"
          f" | 꺼짐지연 중앙 {st.median(off):5.1f} / 최대 {max(off):6.1f}ms"
          f" | 명령경로 묶임 중앙 {st.median(bl):5.1f} / 최대 {max(bl):5.1f}ms"
          f" | 잔상 {m['stale_on']}회 (최장 {max(m['stale_ms']) if m['stale_ms'] else 0:.0f}ms)"
          f" | 불일치 {m['mismatch']}/{m['cycles']}")


def main():
    n = 20000
    ok = True
    for pf in (0.0, 0.05, 0.3):
        print(f"\n[I2C 명령 실패 확률 {pf:.0%}]  사이클 {n}")
        res = {}
        for d in ('old', 'new'):
            res[d] = Sim(d, pf, seed=1234).run(n)
            report(d, res[d])
        mn = res['new']
        # 판정
        if mn['mismatch'] != 0:
            print("  ✗ 새 구조: 조용해진 뒤 물리≠논리(재시도 대기 아님)"); ok = False
        if pf == 0.0 and mn['stale_on'] != 0:
            print("  ✗ 새 구조: 실패 없는데 잔상"); ok = False
        if max(mn['block']) > 0.0:
            print("  ✗ 새 구조: 명령 경로가 I2C 에 묶였다"); ok = False
        if pf == 0.0 and pct(mn['on_lat'], 0.99) > 40:
            print("  ✗ 새 구조: 켜짐 99% 지연 40ms 초과"); ok = False
        if pf == 0.0 and max(mn['off_lat']) > UI_ON_MS + 4 * PAGE_MS + 1:
            print("  ✗ 새 구조: 꺼짐 지연이 한 회전(50ms+렌더)을 넘었다"); ok = False
    print("\n판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
