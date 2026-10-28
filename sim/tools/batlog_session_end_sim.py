# -*- coding: utf-8 -*-
"""★2026-10-29 끝난 방전 세션을 배터리 재부팅이 '이어받는' 버그 — somfy_app.c 방전 세션 상태기계 재현.

실기(10/23~29, COM7): 10/22 에 USB 를 꽂아 세션 #70 이 끝났는데, 10/23 분리 순간 배터리 재부팅이 나자
#70 을 **이어받았다**. batlog 기준점이 10/22 값(4094mV/98%)으로, 경과가 그 뒤로 이어 붙어 기록이 틀렸다.
원인: USB 연결 분기가 `_batlog_flush_if_due(force)` 로 저장한 **뒤에** s_dis_t0_us = 0 을 한다 → NVS 의 dis_on 은 1
로 남는다(그 뒤로는 dirty 가 없어 다시 저장되지 않는다). 부팅 때 _batlog_try_resume 은 "배터리 + dis_on=1" 이면
이어받는다.

수정(new):
  ① 세션 종료(배터리→USB) 때 dis_on=0 을 **곧바로 저장**한다. 단 dis_el 은 세션의 마지막 경과로 **보존** —
     0 으로 덮으면 디코더가 끝난 세션의 절전 통계(회/초)를 못 낸다(batlog_decode 는 dis_el 로 나눈다).
  ② USB 로 부팅했는데 NVS 에 dis_on=1 이 남아 있으면(옛 펌웨어가 남긴 것) 그 자리에서 끝난 것으로 적는다.

불변식(무작위 사건열 2,000개 × 각 400단계)
  · 잘못 이어받기 0 : 배터리 부팅이 이어받은 세션은 재부팅 직전까지 **진행 중**이던 세션이어야 한다
  · 이어받기 누락 0 : 진행 중이던 세션에서 배터리 재부팅이 나면 이어받아야 한다
  · 경과 보존     : 세션이 끝난 뒤 NVS dis_el = 그 세션의 마지막 경과(±저장 간격)
"""
import random
import sys

CONVERGE_S = 25       # USB 분리 뒤 기준점 수렴(BAT_FLOOR_MIN_N 5회 × 5초)
SAVE_IV_S = 5         # 방전 기록 저장(초반 5초, 실제로는 이후 120초 — 불변식엔 영향 없음)


class Nvs(object):
    def __init__(self):
        self.dis_on = 0
        self.dis_el = 0
        self.sess = 0
        self.n = 1            # 기록이 있다(_batlog_try_resume 조건)


class Fw(object):
    def __init__(self, nv, design, on_battery, now):
        self.nv, self.design = nv, design
        self.t0 = None        # s_dis_t0_us (None = 세션 없음)
        self.pending = False  # dis_pending
        self.pend_at = 0
        self.was_usb = not on_battery
        self.end_el = 0       # ★new: 끝난 세션의 마지막 경과(_batlog_save 가 쓴다)
        self.resumed = None   # 이번 부팅에서 이어받은 세션 번호
        self._first(on_battery, now)

    # _batlog_save: dis_on = t0 ? 1 : 0, dis_el = t0 ? 경과 : (new: end_el / old: 0)
    def save(self, now):
        nv = self.nv
        nv.dis_on = 1 if self.t0 is not None else 0
        if self.t0 is not None:
            nv.dis_el = now - self.t0
        else:
            nv.dis_el = self.end_el if self.design == 'new' else 0

    def _first(self, on_battery, now):
        nv = self.nv
        if on_battery:
            if nv.dis_on and nv.n:                       # _batlog_try_resume
                self.t0 = now - nv.dis_el
                self.resumed = nv.sess
            else:
                self.pending, self.pend_at = True, now
        elif self.design == 'new' and nv.dis_on:         # ② USB 부팅인데 진행 중 표시가 남아 있다
            self.end_el = nv.dis_el
            self.save(now)

    def step(self, now, on_battery):
        nv = self.nv
        if self.was_usb and on_battery:                  # USB 분리 감지
            self.pending, self.pend_at, self.t0 = True, now, None
        elif not self.was_usb and not on_battery:        # 방전 종료(USB 연결)
            if self.t0 is not None:
                self.save(now)                           # _batlog_flush_if_due(force) — 남은 것 저장
            if self.design == 'new' and self.t0 is not None:
                self.end_el = now - self.t0              # ① 마지막 경과 보존 후
            self.pending, self.t0 = False, None
            if self.design == 'new':
                self.save(now)                           # ① dis_on=0 을 곧바로 저장
        self.was_usb = not on_battery
        if self.pending and on_battery and now - self.pend_at >= CONVERGE_S:
            self.pending = False
            nv.sess += 1                                 # _batlog_new_session
            self.t0 = now
            self.save(now)
        elif self.t0 is not None and on_battery and (now - self.t0) % SAVE_IV_S == 0:
            self.save(now)


def run(design, seed):
    rng = random.Random(seed)
    nv = Nvs()
    now = 0
    on_bat = rng.random() < 0.5
    fw = Fw(nv, design, on_bat, now)
    bad = dict(wrong=0, missed=0, el_lost=0, resumes=0)
    for _ in range(400):
        r = rng.random()
        if r < 0.08:                                     # 전원 전환
            on_bat = not on_bat
        elif r < 0.14:                                   # 재부팅(배터리 글리치·덤프 리셋 등)
            active = fw.t0 is not None and on_bat        # 재부팅 직전 진행 중이던 세션
            active_sess = nv.sess if active else None
            # ★펌웨어는 세션 중 SAVE_IV_S 마다 저장한다 — 재부팅 직전 저장 이후분은 잃어도 된다(경과만 덜 이어짐)
            fw = Fw(nv, design, on_bat, now)
            if fw.resumed is not None:
                bad['resumes'] += 1
                if fw.resumed != active_sess:
                    bad['wrong'] += 1
            elif on_bat and active:
                bad['missed'] += 1
        dt = rng.choice([1, 3, 5, 10, 30, 60])
        for _k in range(dt):
            now += 1
            was_active = fw.t0 is not None
            last_el = now - 1 - fw.t0 if was_active else None
            fw.step(now, on_bat)
            if was_active and fw.t0 is None and not on_bat:     # 방금 USB 로 세션이 끝났다
                if nv.dis_el < last_el - SAVE_IV_S:             # 끝난 세션의 경과가 NVS 에 남아야 한다
                    bad['el_lost'] += 1
    return bad


def main():
    ok = True
    for design in ('old', 'new'):
        tot = dict(wrong=0, missed=0, el_lost=0, resumes=0)
        for s in range(2000):
            r = run(design, s)
            for k in tot:
                tot[k] += r[k]
        print('[%s] 이어받기 %5d회  잘못 이어받기 %5d  누락 %3d  끝난 세션 경과 소실 %5d'
              % (design, tot['resumes'], tot['wrong'], tot['missed'], tot['el_lost']))
        if design == 'new' and (tot['wrong'] or tot['missed'] or tot['el_lost']):
            ok = False
        if design == 'old' and tot['wrong'] == 0:
            print('  ※옛 방식에서 잘못 이어받기가 재현되지 않았다 — 모델이 실기 사고를 못 담는다')
            ok = False
    # 실기 사고 그대로: USB 로 세션 종료 → (덤프 리셋, USB 부팅) → 분리 순간 배터리 재부팅
    for design in ('old', 'new'):
        nv = Nvs()
        fw = Fw(nv, design, True, 0)                     # 배터리 부팅, 세션 시작
        for t in range(1, 1000):
            fw.step(t, True)
        sess_before = nv.sess
        for t in range(1000, 1100):
            fw.step(t, False)                            # USB 연결 = 세션 끝
        fw = Fw(nv, design, False, 1100)                 # 덤프 리셋 → USB 부팅
        for t in range(1101, 1200):
            fw.step(t, False)
        fw = Fw(nv, design, True, 1200)                  # 분리 순간 재부팅 → 배터리 부팅
        res = '세션 #%d 을 이어받음(★끝난 세션)' % fw.resumed if fw.resumed is not None else '새 세션 대기(정상)'
        print('[실기 재현 %s] USB 로 끝난 세션 #%d → 덤프 리셋 → 분리 순간 재부팅: %s  (NVS dis_el=%d)'
              % (design, sess_before, res, nv.dis_el))
        if design == 'new' and fw.resumed is not None:
            ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
