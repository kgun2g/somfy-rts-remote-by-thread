#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
★2026-10-18 감사 보고서 F16 시뮬 — USB(VBUS) 판정을 한 곳에서 시간 기준으로 안정화.

옛 구조 (somfy_app.c):
  · PM(_enable_pm_light_sleep)·화면 정책(usb_mode)·메인 루프 주기 = 회전마다 핀 원시값(btn_handler_is_charging)
  · 방전 기록(BATLOG)만 'USB→배터리 2초 연속' 확인(USB_DROP_CONFIRM_MS)
새 구조 (_usb_phys_update — 메인 루프 회전마다 1번):
  · 배터리→USB: 원시값이 USB_PLUG_CONFIRM_MS(200ms) 동안 계속 USB 여야 인정
  · USB→배터리: 원시값이 USB_DROP_CONFIRM_MS(2초, 기존 값) 동안 계속 배터리여야 인정
  · 원시값 ≠ 확정값(확인 중)이면 메인 루프를 100ms 로(느린 1초 주기 금지) — 확인이 빨리 끝나게
  · PM·화면·기록이 모두 확정값을 쓴다. 부팅 첫 판독은 그대로 확정(예전 방전 기록의 '첫 판독 보정'과 같다)
메인 루프 주기: 화면 OFF·배터리·잠듦이면 1000ms, 아니면 100ms (F10). 시뮬은 배터리면 화면 OFF 로 본다(보수적).

시나리오(원시 판독이 어떻게 흔들리는가 — 실제 흔들림 여부는 하드웨어 측정 영역, 감사 F16):
  clean       깨끗한 꽂기/뽑기
  blip_bat    배터리 중 판독 1회가 USB 로 튐(확률/판독)
  blip_usb    USB 중 판독 1회가 배터리로 튐  ← 옛 구조는 그 순간 PM 이 light sleep 을 켠다(USB 콘솔 사망 지뢰)
  slow_unplug 뽑은 뒤 노드가 문턱 근처를 지나며 τ초 동안 흔들림(P(USB)=0.5·e^(-t/τ))
  stuck_mid   문턱에 붙어 영원히 반반(병적인 경우 — 둘 다 나쁘다, 비교용)
측정
  PM 재설정 횟수 / 'USB 인데 light sleep' 진입 횟수(콘솔 위험) / '배터리인데 USB 모드' 시간(낭비) /
  방전 세션 시작 횟수(진짜 뽑기 대비) / 진짜 꽂기·뽑기 감지 지연
"""
import math
import random
import sys

DROP_MS = 2000      # USB_DROP_CONFIRM_MS (기존)
PLUG_MS = 200       # USB_PLUG_CONFIRM_MS (새)
FAST_MS = 100
SLOW_MS = 1000


def gen_truth(rng, total_s, start_usb=True):
    """진짜 전원: (시작초, USB?) 구간들 — USB 60~600초 / 배터리 300~3600초 번갈아."""
    segs, t, usb = [], 0.0, start_usb
    while t < total_s:
        d = rng.uniform(60, 600) if usb else rng.uniform(300, 3600)
        segs.append((t, usb))
        t += d
        usb = not usb
    return segs


def truth_at(segs, t):
    lo, hi = 0, len(segs) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if segs[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return segs[lo][1], segs[lo][0]


def raw_read(rng, scen, usb, t_since, p):
    if scen == 'clean':
        return usb
    if scen == 'blip_bat':
        return True if (not usb and rng.random() < p) else usb
    if scen == 'blip_usb':
        return False if (usb and rng.random() < p) else usb
    if scen == 'slow_unplug':
        if usb:
            return True
        return rng.random() < 0.5 * math.exp(-t_since / p)
    if scen == 'stuck_mid':
        return True if usb else (rng.random() < 0.5)
    raise ValueError(scen)


def simulate(design, scen, p, seed, total_s=3 * 24 * 3600, start_usb=True):
    rng = random.Random(seed)
    segs = gen_truth(rng, total_s, start_usb)
    t = 0.0
    # 옛: 방전 기록 2초 디바운스 상태
    low_since = None
    # 새: 안정화 상태
    init = False
    stable = None
    raw_prev = None
    raw_since = 0.0
    m = dict(pm_cfg=0, usb_ls=0, waste_s=0.0, sess=0, true_unplug=0, plug_lat=[], unplug_lat=[])
    pm_usb_prev = None
    bl_usb_prev = None
    true_prev = None
    true_change_t = 0.0
    pending_plug = pending_unplug = None
    while t < total_s:
        usb, seg_t = truth_at(segs, t)
        if true_prev is not None and usb != true_prev:
            true_change_t = t
            if usb:
                pending_plug = t
            else:
                pending_unplug = t
                m['true_unplug'] += 1
        true_prev = usb
        r = raw_read(rng, scen, usb, t - seg_t, p)
        # ── 소비자 입력 결정
        if design == 'old':
            pm_usb = r
            ui_usb = r
            if ui_usb:
                low_since = None
                bl_usb = True
            else:
                if low_since is None:
                    low_since = t if bl_usb_prev is not None else t - DROP_MS / 1000  # 부팅 첫 판독 보정
                bl_usb = (t - low_since) < DROP_MS / 1000
            settling = False
        else:
            if not init:
                init, stable, raw_prev, raw_since = True, r, r, t
            if r != raw_prev:
                raw_prev, raw_since = r, t
            if r != stable:
                need = (PLUG_MS if r else DROP_MS) / 1000
                if t - raw_since >= need:
                    stable = r
            pm_usb = ui_usb = bl_usb = stable
            settling = (raw_prev != stable)
        # ── PM
        if pm_usb_prev is not None and pm_usb != pm_usb_prev:
            m['pm_cfg'] += 1
            if not pm_usb and usb:
                m['usb_ls'] += 1                     # 진짜 USB 인데 light sleep 을 켰다
        if pm_usb and pending_plug is not None:
            m['plug_lat'].append(t - pending_plug)
            pending_plug = None
        if (not pm_usb) and pending_unplug is not None:
            m['unplug_lat'].append(t - pending_unplug)
            pending_unplug = None
        pm_usb_prev = pm_usb
        # ── 방전 세션(기록): USB→배터리 전환 = 새 세션
        if bl_usb_prev is not None and bl_usb_prev and not bl_usb:
            m['sess'] += 1
        bl_usb_prev = bl_usb
        # ── 다음 회전까지
        slow = (not ui_usb) and (not settling)
        dt = (SLOW_MS if slow else FAST_MS) / 1000
        if pm_usb and not usb:
            m['waste_s'] += dt                       # 배터리인데 USB 모드(절전 없음·전속)
        t += dt
    return m


def fmt(m):
    pl = sorted(m['plug_lat']); ul = sorted(m['unplug_lat'])
    def mx(a): return max(a) if a else float('nan')
    def md(a): return a[len(a) // 2] if a else float('nan')
    return (f"PM재설정 {m['pm_cfg']:6d}  USB인데light sleep {m['usb_ls']:5d}  배터리인데USB모드 {m['waste_s']:8.1f}s  "
            f"세션 {m['sess']:4d}/{m['true_unplug']:3d}  꽂기감지 {md(pl):4.2f}/{mx(pl):4.2f}s  "
            f"뽑기감지 {md(ul):4.2f}/{mx(ul):4.2f}s")


def main():
    cases = [('clean', 0, True), ('clean', 0, False), ('blip_bat', 1e-4, True), ('blip_bat', 1e-2, True),
             ('blip_usb', 1e-4, True), ('blip_usb', 1e-2, True),
             ('slow_unplug', 1.0, True), ('slow_unplug', 30.0, True), ('stuck_mid', 0, True)]
    ok = True
    for scen, p, su in cases:
        print()
        print(f"[{scen} p={p} 부팅={'USB' if su else '배터리'}]  (3일)")
        res = {}
        for d in ('old', 'new'):
            res[d] = simulate(d, scen, p, seed=7, start_usb=su)
            print(f"  {d:3s} " + fmt(res[d]))
        n, o = res['new'], res['old']
        bad = []
        if scen != 'stuck_mid':
            # 반드시 지킬 것
            if n['usb_ls'] != 0: bad.append('진짜 USB 인데 light sleep')
            if n['pm_cfg'] > o['pm_cfg']: bad.append('PM 재설정이 옛보다 많다')
            if n['sess'] > o['sess']: bad.append('세션이 옛보다 많다')
            if scen in ('clean', 'blip_bat', 'blip_usb') and n['sess'] != n['true_unplug']:
                bad.append('세션 수 != 진짜 뽑기 수')
            if scen == 'clean':
                if n['plug_lat'] and max(n['plug_lat']) > 0.31: bad.append('꽂기 감지 0.3초 초과')
                if n['unplug_lat'] and max(n['unplug_lat']) > 2.11: bad.append('뽑기 감지 2.1초 초과')
        for b in bad:
            print('  ✗ 새: ' + b)
        ok = ok and not bad
    print()
    print("판정:", "통과" if ok else "실패")
    print("※stuck_mid(문턱에 붙어 영원히 반반)는 하드웨어 고장 — 옛=PM·세션 폭주, 새=USB 모드 고정(절전 없음). 비교용.")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
