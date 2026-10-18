#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 감사 보고서 F17 시뮬 — Thread 부착 확인과 SNTP 수명.

옛 구조 (somfy_app.c):
  · 메인 루프가 **회전마다** thread_prov_is_attached() (OpenThread 락) — 화면 ON 100ms / OFF 1초
  · _start_app_tasks_once 가 **등록 여부와 무관하게** _sntp_start_once() → 미등록·무선 OFF 에서도 SNTP 가 돈다
  · 부착 전이 → 등록 완료면 _sntp_start_once() → 이미 시작했으면 esp_sntp_restart()
  · 페어링 화면 진입 esp_sntp_stop(), 나올 때 esp_sntp_restart()
    ※IDF sntp_restart 는 **켜져 있을 때만** stop+init 한다(lwip/apps/sntp/sntp.c: if (sntp_enabled())) → 꺼진 뒤엔 무효
새 구조:
  · CHIP kThreadConnectivityChange(부착 상태가 실제로 바뀔 때만) → 요청 플래그 → 다음 회전에 1번 확인
    + 보험: 7초 Matter 상태(화면 ON, 등록 시 — 이미 OT 락을 잡는다)·60초 [PWR] 에서 같은 확인
  · _sntp_apply(): 원하는 상태 = 등록 완료 ∧ 부착 ∧ 페어링 보호 아님. 다를 때만 init/stop(재개도 init)
    호출: 앱 시작·부착 변화·등록 완료 변화(메인 루프)·페어링 보호 시작/끝
lwIP SNTP 모델: init → 0~5초 뒤 요청(CONFIG_LWIP_SNTP_MAXIMUM_STARTUP_DELAY=5000), 성공하면 1시간 뒤,
  도달 불가(미부착)면 15→30→60→120→150초 재시도(SNTP_RETRY_TIMEOUT=15000, ×2, 최대 ×10).
"""
import random
import sys

H = 3600.0


class Lwip:
    """tcpip 태스크가 순서대로 처리하는 SNTP — 요청 시각과 재시도 시한만 모델링."""
    def __init__(self, rng):
        self.rng = rng
        self.enabled = False
        self.next_req = None
        self.retry = 15.0
        self.inits = 0
        self.futile = 0
        self.ok = 0
        self.last_sync = None

    def init(self, t):
        if self.enabled:
            return
        self.enabled = True
        self.inits += 1
        self.retry = 15.0
        self.next_req = t + self.rng.uniform(0, 5)

    def stop(self, t):
        self.enabled = False
        self.next_req = None

    def restart(self, t):            # esp_sntp_restart: 켜져 있을 때만
        if self.enabled:
            self.stop(t)
            self.init(t)
            return True
        return False

    def tick(self, t, reachable):
        if self.enabled and self.next_req is not None and t >= self.next_req:
            if reachable:
                self.ok += 1
                self.last_sync = t
                self.retry = 15.0
                self.next_req = t + H
            else:
                self.futile += 1
                self.next_req = t + self.retry
                self.retry = min(self.retry * 2, 150.0)


class World:
    def __init__(self, seed, registered, attach_times, detach_times, pair_menu, commission_at=None,
                 app_start=17.0, ev_loss=0.0, panel_on_frac=0.02):
        self.rng = random.Random(seed)
        self.registered0 = registered
        self.attach_times = sorted(attach_times)
        self.detach_times = sorted(detach_times)
        self.pair_menu = pair_menu                  # (진입, 나감) 또는 None
        self.commission_at = commission_at          # 페어링 중 등록 완료 시각
        self.app_start = app_start
        self.ev_loss = ev_loss
        self.panel_on_frac = panel_on_frac

    def attached_at(self, t):
        a = [x for x in self.attach_times if x <= t]
        d = [x for x in self.detach_times if x <= t]
        la = a[-1] if a else -1
        ld = d[-1] if d else -1
        return la > ld

    def cc_at(self, t):
        return self.registered0 or (self.commission_at is not None and t >= self.commission_at)


def run(design, w, total):
    rng = random.Random(1)
    L = Lwip(rng)
    m = dict(locks=0, run_unreg=0.0, run_protect=0.0, run_detached=0.0, stale_max=0.0, attach_lat=[])
    # 공통 상태
    protect = False
    t = 0.0
    # 옛
    sntp_started = False
    thr_was = False
    pair_was_on = False
    # 새
    chk_req = True
    thr_seen = False
    sntp_run = False
    cfg = False
    cc_seen = False
    last_7s = -1e9
    last_60s = -1e9
    prev_att = False
    att_change_t = None
    panel_on = False
    panel_toggle_t = 0.0

    def apply_new(why):
        nonlocal sntp_run, cfg
        want = w.cc_at(t) and thr_seen and not protect
        if want == sntp_run:
            return
        if want:
            cfg = True
            L.init(t)
            sntp_run = True
            if protect:
                L.stop(t)
                sntp_run = False
        else:
            L.stop(t)
            sntp_run = False

    def sync_attach(att):
        nonlocal thr_seen
        if att == thr_seen:
            return
        thr_seen = att
        if att_change_t is not None:
            m['attach_lat'].append(t - max(att_change_t, w.app_start))   # 앱 시작 전 변화는 시작부터 잰다
        apply_new('attach')

    app_started = False
    while t < total:
        att = w.attached_at(t)
        if att != prev_att:
            prev_att = att
            att_change_t = t
            if design == 'new' and rng.random() >= w.ev_loss:
                chk_req = True                       # CHIP 이벤트 → 요청 플래그(+메인 루프 깨움)
        # 화면 ON/OFF (사용 패턴: 대부분 OFF)
        if t >= panel_toggle_t:
            panel_on = rng.random() < w.panel_on_frac
            panel_toggle_t = t + (10.0 if panel_on else rng.uniform(60, 3600))
        # 페어링 메뉴
        in_menu = w.pair_menu is not None and w.pair_menu[0] <= t < w.pair_menu[1]
        if in_menu and not protect:
            protect = True
            if design == 'old':
                pair_was_on = L.enabled
                if pair_was_on:
                    L.stop(t)
            else:
                apply_new('protect')
        if (not in_menu) and protect:
            protect = False
            if design == 'old':
                if pair_was_on:
                    L.restart(t)                     # ★무효(꺼져 있으므로)
            else:
                apply_new('protect end')
        # 앱 시작
        if not app_started and t >= w.app_start:
            app_started = True
            if design == 'old':
                if not sntp_started and not L.enabled:
                    L.init(t)
                    sntp_started = True
                else:
                    L.restart(t)
            else:
                apply_new('app start')
        # 메인 루프 본체
        if app_started:
            if design == 'old':
                m['locks'] += 1
                if att != thr_was:
                    thr_was = att
                    if att_change_t is not None:
                        m['attach_lat'].append(t - max(att_change_t, w.app_start))
                    if att and w.cc_at(t):
                        if not sntp_started and not L.enabled:
                            L.init(t); sntp_started = True
                        else:
                            L.restart(t)
            else:
                if chk_req:
                    chk_req = False
                    m['locks'] += 1
                    sync_attach(att)
                cc = w.cc_at(t)
                if cc != cc_seen:
                    cc_seen = cc
                    apply_new('cc')
                if panel_on and cc and t - last_7s >= 7.0:
                    last_7s = t
                    m['locks'] += 1
                    sync_attach(att)
                if t - last_60s >= 60.0:
                    last_60s = t
                    m['locks'] += 1
                    sync_attach(att)
        # SNTP
        L.tick(t, reachable=att)
        dt = 0.1 if (panel_on or in_menu) else 1.0
        if L.enabled:
            if not w.cc_at(t):
                m['run_unreg'] += dt
            if protect:
                m['run_protect'] += dt
            if not att:
                m['run_detached'] += dt
        if w.cc_at(t) and att and t > 120:
            ref = L.last_sync if L.last_sync is not None else w.app_start
            m['stale_max'] = max(m['stale_max'], t - ref)
        t += dt
    m.update(inits=L.inits, futile=L.futile, ok=L.ok, enabled_end=L.enabled)
    return m


def race_check():
    """페어링 보호 시작(버튼 태스크 prio 10)이 메인 루프 _sntp_apply 를 단계마다 선점 — 끝 상태는 항상 '정지'여야."""
    bad = 0
    for cut in range(6):
        L = Lwip(random.Random(0))
        st = dict(protect=False, run=False)
        ops = []                                   # tcpip 콜백 순서

        def apply(stv):
            want = (not stv['protect'])            # 등록·부착은 참으로 둔다
            if want == stv['run']:
                return
            if want:
                ops.append('init'); stv['run'] = True
                if stv['protect']:
                    ops.append('stop'); stv['run'] = False
            else:
                ops.append('stop'); stv['run'] = False

        def btn():
            if st['protect']:
                return
            st['protect'] = True
            apply(st)
        # 메인 루프 _sntp_apply 를 단계로 쪼갠다
        step = 0
        want = None
        def mstep(i):
            nonlocal want
            if i == 0:
                want = (not st['protect'])
            elif i == 1:
                return want != st['run']
            elif i == 2:
                if want: ops.append('init')
            elif i == 3:
                if want: st['run'] = True
            elif i == 4:
                if want and st['protect']:
                    ops.append('stop'); st['run'] = False
            return True
        cont = True
        for i in range(5):
            if i == cut:
                btn()
            r = mstep(i)
            if i == 1 and r is False:
                break
            if i == 0:
                continue
        if cut == 5:
            btn()
        for o in ops:
            (L.init if o == 'init' else L.stop)(0)
        if L.enabled:
            bad += 1
            print(f"  ✗ 경합: 선점 위치 {cut} → 페어링 보호 중 SNTP 가 켜진 채 끝남 (ops={ops})")
    return bad


def main():
    D = 3 * 24 * H
    ok = True
    sc = {
        'A 등록·부팅 전 부착': World(1, True, [8.0], [], None),
        'B 등록·부팅 후 부착': World(2, True, [40.0], [], None),
        'C 미등록(무선 OFF)': World(3, False, [], [], None),
        'D 등록·페어링 메뉴 다녀옴': World(4, True, [8.0], [], (1000.0, 1060.0)),
        'E 등록·6시간마다 분리': World(5, True, [8.0] + [6 * H * k + 300 for k in range(1, 12)],
                                     [6 * H * k for k in range(1, 12)], None),
        'F E + 이벤트 50% 유실': World(5, True, [8.0] + [6 * H * k + 300 for k in range(1, 12)],
                                     [6 * H * k for k in range(1, 12)], None, ev_loss=0.5),
        'G 신규 등록(페어링 화면)': World(6, False, [130.0], [], (100.0, 155.0), commission_at=150.0),
    }
    for name, w in sc.items():
        print(f"\n[{name}]  3일")
        r = {}
        for d in ('old', 'new'):
            r[d] = run(d, w, D)
            m = r[d]
            al = m['attach_lat']
            print(f"  {d:3s} OT락(부착확인) {m['locks']:7d}  SNTP init {m['inits']:2d}  동기화 {m['ok']:3d}  헛시도 {m['futile']:4d}"
                  f"  미등록중실행 {m['run_unreg']/H:6.1f}h  보호중실행 {m['run_protect']:5.1f}s  분리중실행 {m['run_detached']/60:6.1f}분"
                  f"  최장미동기 {m['stale_max']/H:5.1f}h  부착감지 최대 {max(al) if al else 0:5.1f}s")
        n = r['new']
        bad = []
        if n['run_unreg'] > 0: bad.append('미등록인데 SNTP 실행')
        if n['run_protect'] > 0: bad.append('페어링 보호 중 SNTP 실행')
        if n['locks'] > r['old']['locks']: bad.append('OT 락이 옛보다 많다')
        if 'C' not in name and n['stale_max'] > 1.2 * H: bad.append('등록·부착인데 1.2시간 넘게 동기화 없음')
        if n['futile'] > r['old']['futile']: bad.append('헛시도가 옛보다 많다')
        lat_lim = 61.0 if 'F' in name else 1.1
        if n['attach_lat'] and max(n['attach_lat']) > lat_lim: bad.append(f'부착 감지 {lat_lim}s 초과')
        for b in bad:
            print('  ✗ 새: ' + b)
        ok = ok and not bad
    print("\n[선점 경합 — 버튼 태스크의 페어링 보호 시작이 메인 루프 _sntp_apply 를 단계마다 끊음]")
    nb = race_check()
    if nb == 0:
        print("  모든 선점 위치에서 끝 상태 = 정지 ✓")
    ok = ok and nb == 0
    print("\n판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
