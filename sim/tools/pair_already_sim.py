#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 페어링 메뉴 '이미 등록됨' 시뮬 (사용자 결정: "이미 등록됨"을 보여 주고 창을 열지 않기).

옛: 설정 → Matter Pair 를 고르면 등록 여부와 무관하게 _pairing_protect_begin()(SNTP 정지·CPU 최대 고정)
    + matter_blinds_open_commissioning_window()(15분 커미셔닝 창) → 메인 루프가 등록 완료를 보고 곧바로
    SUCCESS → 5초 뒤 메인. 창은 나가도 안 닫힌다.
새: 고를 때 이미 등록(matter_blinds_is_commissioning_complete)이면 보호·창을 건드리지 않고 phase=ALREADY
    ("ALREADY / PAIRED") → SUCCESS 와 같은 5초 뒤 메인. 미등록이면 옛 흐름 그대로.
somfy_app.c 의 메뉴 동작(MA_PAIR)과 메인 루프 페어링 블록을 그대로 옮겼다(100ms 회전).
검증: 등록 기기 = 보호 0회·창 0회·SNTP 정지 0회·ALREADY 표시·5초 뒤 메인 / 짧게 SETUP = 메뉴 복귀
      미등록 기기 = 옛과 똑같은 단계·보호·창(진행 → 완료 → 5초 뒤 메인, 실패 → FAIL 유지)
"""
import sys


class Dev:
    def __init__(self, design, registered):
        self.design = design
        self.cc = registered            # matter_blinds_is_commissioning_complete()
        self.fabric = registered        # matter_blinds_is_commissioned()
        self.failsafe = False           # matter_blinds_is_pairing_in_progress()
        self.screen = 'MENU'
        self.protect = False
        self.protect_on_cnt = 0
        self.window_open_cnt = 0
        self.sntp_stop_cnt = 0
        self.phase = None
        self.phases = []
        self.already = False
        self.done_us = 0
        self.was_active = False
        self.active_since = 0
        self.exit_reason = None
        self.t_us = 0

    # ── 메뉴: Matter Pair 선택 (somfy_app.c MA_PAIR)
    def select_pair(self):
        self.screen = 'PAIR'
        if self.design == 'new' and self.cc:
            self.already = True
            self.phase = 'ALREADY'
            return
        self.protect_begin()
        self.window_open_cnt += 1
        self.phase = 'WAIT'

    def protect_begin(self):
        if self.protect:
            return
        self.protect = True
        self.protect_on_cnt += 1
        self.sntp_stop_cnt += 1          # 등록·부착이면 SNTP 정지(_sntp_apply)

    def protect_end(self):
        self.protect = False

    def setup_short(self):              # 페어링 화면에서 SETUP 짧게 = 메뉴 복귀
        if self.screen == 'PAIR':
            self.screen = 'MENU'

    # ── 메인 루프 페어링 블록 (100ms 회전)
    def loop(self):
        if self.screen != 'PAIR':
            self.was_active = False
            self.done_us = 0
            self.active_since = 0
            self.already = False
            self.protect_end()
            return
        pnow = self.t_us
        if self.design == 'new' and self.already:
            ph = 'ALREADY'
            if self.done_us == 0:
                self.done_us = pnow
            elif pnow - self.done_us >= 5_000_000:
                self.screen = 'MAIN'
                self.exit_reason = '이미 등록됨 5초'
        elif self.cc:
            ph = 'DONE'
            if self.done_us == 0:
                self.done_us = pnow
            elif pnow - self.done_us >= 5_000_000:
                self.screen = 'MAIN'
                self.exit_reason = '페어링 완료 5초'
        elif self.failsafe:
            if self.active_since == 0:
                self.active_since = pnow
            self.was_active = True
            ph = 'FAIL' if pnow - self.active_since >= 90_000_000 else 'ACTIVE'
        elif self.was_active and not self.fabric:
            ph = 'FAIL'
        else:
            ph = 'WAIT'
            self.active_since = 0
        if self.screen == 'PAIR':
            self.phase = ph
            if not self.phases or self.phases[-1] != ph:
                self.phases.append(ph)

    def run_for(self, s, events=None):
        events = sorted(events or [], key=lambda e: e[0])
        end = self.t_us + int(s * 1e6)
        while self.t_us < end:
            while events and events[0][0] * 1e6 <= self.t_us:
                _, fn = events.pop(0)
                fn(self)
            self.loop()
            self.t_us += 100_000


def scenario(design, registered, kind):
    d = Dev(design, registered)
    d.select_pair()
    if kind == 'idle':
        d.run_for(8)
    elif kind == 'back':
        d.run_for(1.0, [(0.5, lambda x: x.setup_short())])
        d.run_for(2)
    elif kind == 'pair_ok':
        d.run_for(40, [(10, lambda x: setattr(x, 'failsafe', True)),
                       (30, lambda x: (setattr(x, 'failsafe', False), setattr(x, 'cc', True), setattr(x, 'fabric', True)))])
    elif kind == 'pair_fail':
        d.run_for(40, [(10, lambda x: setattr(x, 'failsafe', True)),
                       (25, lambda x: setattr(x, 'failsafe', False))])
    return d


def main():
    ok = True
    rows = [
        (True, 'idle'), (True, 'back'),
        (False, 'idle'), (False, 'back'), (False, 'pair_ok'), (False, 'pair_fail'),
    ]
    for reg, kind in rows:
        r = {dsn: scenario(dsn, reg, kind) for dsn in ('old', 'new')}
        for dsn in ('old', 'new'):
            d = r[dsn]
            print(f"  [{'등록' if reg else '미등록'} {kind:9s}] {dsn:3s} 단계 {'→'.join(d.phases) or '-':28s} 보호 {d.protect_on_cnt} 창 {d.window_open_cnt}"
                  f" SNTP정지 {d.sntp_stop_cnt}  끝 화면 {d.screen} ({d.exit_reason or '-'})  보호 남음 {d.protect}")
        n, o = r['new'], r['old']
        if reg:
            if n.protect_on_cnt or n.window_open_cnt or n.sntp_stop_cnt:
                print('  ✗ 새: 등록 기기인데 보호·창·SNTP 를 건드렸다'); ok = False
            if n.phases[:1] != ['ALREADY']:
                print('  ✗ 새: ALREADY 가 안 나왔다'); ok = False
            if kind == 'idle' and n.screen != 'MAIN':
                print('  ✗ 새: 5초 뒤 메인으로 안 갔다'); ok = False
            if kind == 'back' and n.screen != 'MENU':
                print('  ✗ 새: SETUP 짧게로 메뉴 복귀 안 됨'); ok = False
        else:
            same = (n.phases == o.phases and n.protect_on_cnt == o.protect_on_cnt and
                    n.window_open_cnt == o.window_open_cnt and n.screen == o.screen and n.protect == o.protect)
            if not same:
                print('  ✗ 새: 미등록 흐름이 옛과 다르다'); ok = False
        if n.protect and n.screen != 'PAIR':
            print('  ✗ 새: 화면을 나갔는데 보호가 남았다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
