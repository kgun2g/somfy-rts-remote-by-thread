# -*- coding: utf-8 -*-
"""★2026-10-29 PCF 정지 상태 기록(batlog "pcf_rest") — 수정 전 가상검증 (사용자 규칙 "빌드 전 가상검증").

왜: 절전 후보 2 — 하프스텝 로터리(EC05 계열)는 디텐트가 A/B 열림(11)과 닫힘(00)을 번갈아 간다. 닫힌 채 멈추면
PCF8575 의 입력 전류원(TI 표 5.5: 30~300µA/핀)이 접점을 통해 계속 흐른다. 실제로 어느 위치에 얼마나 멈춰 있는지부터 잰다.

펌웨어(somfy_app.c _pcfrest_tick): 방전 세션 중 메인 루프 회전마다 PCF 원시값을 읽어, **직전 값이 유지된 시간**을
  [0] 로터리 닫힌 접점 0개 / [1] 1개 / [2] 2개 / [3] 그 밖 LOW 핀 × 시간(핀·초) 칸에 더한다. 저장은 batlog 와 함께
  "pcf_rest" = 이전 부팅 누적(초) + 이번 부팅 누적. 재부팅하면 이어받고, 새 세션이면 0 부터. 값을 모르면(-1) 그 구간은 버린다.

불변식(무작위 3일 × 시드 40)
  · 칸별 기록 = 진짜 체류 시간 ± (재부팅 수 × 저장 간격 + 상태 전환 수 × 루프 간격)
  · 새 세션은 0 부터(이전 세션 시간이 섞이지 않는다), USB 중에는 늘지 않는다
  · 그 밖 LOW 핀은 누름 시간만큼만(정지 중 0)
"""
import random
import sys

ROT_A, ROT_B = 0, 1
NBITS = 16
SAVE_IV = 120          # batlog 정상 저장 간격(초)
CONVERGE = 25          # 분리 뒤 세션 시작까지
LOOP_OFF = 1.0         # 화면 OFF 메인 루프 간격(초)


def bins_of(st):
    low = ~st & ((1 << NBITS) - 1)
    rot = (1 << ROT_A) | (1 << ROT_B)
    return bin(low & rot).count('1'), bin(low & ~rot).count('1')


class Fw(object):
    def __init__(self, nvs, on_bat, resume):
        self.nvs = nvs
        self.ms = [0.0] * 4
        self.base = list(nvs.get('pcf_rest', [0, 0, 0, 0])[:4]) if resume else [0, 0, 0, 0]
        self.last = -1
        self.prev = None

    def session_begin(self, resume):
        self.ms = [0.0] * 4
        self.base = list(self.nvs.get('pcf_rest', [0, 0, 0, 0])[:4]) if resume else [0, 0, 0, 0]
        self.prev = None

    def tick(self, t, active, st):
        if not active:
            self.prev = None
            return
        if self.prev is not None and self.last >= 0:
            dt = t - self.prev
            r, o = bins_of(self.last)
            self.ms[r] += dt
            self.ms[3] += dt * o
        self.last = st
        self.prev = t

    def save(self):
        self.nvs['pcf_rest'] = [self.base[i] + int(self.ms[i]) for i in range(4)] + [self.last & 0xFFFFFFFF]


def run(seed):
    rng = random.Random(seed)
    nvs = {}
    dur = 3 * 86400
    # 사용자 입력: 로터리 디텐트(11↔00 번갈아), 가끔 중간(01/10)에 멈춤, 버튼 누름
    rot_events = []
    t = 0
    while t < dur:
        t += rng.randint(600, 20000)
        rot_events.append(t)
    presses = []
    t = 0
    while t < dur:
        t += rng.randint(300, 9000)
        presses.append((t, t + rng.uniform(0.1, 0.6), rng.choice((4, 5, 6, 8, 9))))
    usb = []                       # USB 구간
    t = rng.randint(3600, 20000)
    while t < dur:
        a = t + rng.randint(20000, 120000)
        usb.append((a, a + rng.randint(600, 7200)))
        t = usb[-1][1]
    reboots = sorted(rng.randrange(100, dur) for _ in range(rng.randint(0, 4)))

    def rot_state(t):
        n = sum(1 for e in rot_events if e <= t)
        mid = (n % 7 == 3)          # 가끔 디텐트 사이에 멈춤(01)
        if mid:
            return 0b01
        return 0b11 if n % 2 == 0 else 0b00

    def pcf(t):
        st = 0xFFFF
        ab = rot_state(t)
        st &= ~((1 << ROT_A) | (1 << ROT_B))
        st |= ((ab >> 1) & 1) << ROT_A | (ab & 1) << ROT_B
        for a, b, k in presses:
            if a <= t < b:
                st &= ~(1 << k)
        return st

    def on_usb(t):
        return any(a <= t < b for a, b in usb)

    fw = Fw(nvs, True, False)
    sess_t0 = None
    pend = 0.0
    was_usb = True
    truth = [0.0] * 4
    truth_sess = None
    last_save = 0
    ri = 0
    loop = 0.0
    sessions = 0
    while loop < dur:
        tnow = loop
        u = on_usb(tnow)
        if ri < len(reboots) and tnow >= reboots[ri]:
            ri += 1
            resume = (not u) and sess_t0 is not None
            fw = Fw(nvs, not u, resume)
            fw.session_begin(resume) if resume else None
            if not resume:
                sess_t0 = None
                pend = tnow if not u else None
            loop += 3.0                # 부팅 시간 — 이 사이는 아무도 못 센다
            continue
        if was_usb and not u:
            pend = tnow
            sess_t0 = None
        if not was_usb and u:
            if sess_t0 is not None:
                fw.tick(tnow, True, pcf(tnow))
                fw.save()
            sess_t0 = None
        was_usb = u
        if pend is not None and not u and sess_t0 is None and tnow - pend >= CONVERGE:
            sess_t0 = tnow
            pend = None
            fw.session_begin(False)
            truth = [0.0] * 4
            sessions += 1
        active = sess_t0 is not None and not u
        st = pcf(tnow)
        fw.tick(tnow, active, st)
        if active and tnow - last_save >= SAVE_IV:
            fw.save()
            last_save = tnow
        # 진짜 체류(다음 루프까지 이 상태가 유지된다고 보고 1/10초 해상도로 적분)
        nxt = tnow + LOOP_OFF
        if active:
            k = tnow
            while k < nxt:
                r, o = bins_of(pcf(k))
                truth[r] += 0.1
                truth[3] += 0.1 * o
                k += 0.1
        loop = nxt
    fw.save()
    return nvs.get('pcf_rest', [0] * 5), truth, len(reboots), sessions, rot_events, presses


def main():
    ok = True
    worst = [0.0] * 4
    for seed in range(40):
        rec, tr, nrb, ns, rots, pr = run(seed)
        tol_rot = nrb * (SAVE_IV + 3) + 2 * len(rots) * LOOP_OFF + 5
        tol_oth = nrb * (SAVE_IV + 3) * 2 + 2 * len(pr) * LOOP_OFF + 5
        for i in range(4):
            err = rec[i] - tr[i]
            worst[i] = max(worst[i], abs(err))
            tol = tol_rot if i < 3 else tol_oth
            if abs(err) > tol:
                print('   ✗ 시드 %d 칸 %d: 기록 %d 참 %.0f (허용 ±%.0f, 재부팅 %d)' % (seed, i, rec[i], tr[i], tol, nrb))
                ok = False
        if ns == 0:
            print('   (시드 %d: 세션 없음)' % seed)
    print('[칸별 최대 오차(초)] 닫힘0 %.0f / 닫힘1 %.0f / 닫힘2 %.0f / 그 밖 LOW 핀·초 %.0f' % tuple(worst))
    # 새 세션은 0 부터 — 마지막 세션 길이보다 기록이 클 수 없다
    rec, tr, nrb, ns, _, _ = run(3)
    print('[예] 시드 3: 기록 %s  참 %s  (재부팅 %d, 세션 %d)' % (rec[:4], [int(x) for x in tr], nrb, ns))
    tot = sum(rec[:3])
    frac2 = rec[2] / float(tot) if tot else 0
    print('   닫힌 접점 2개로 멈춰 있던 비율 %.0f%% → 핀당 30~300µA 면 평균 %.0f~%.0f µA'
          % (frac2 * 100, frac2 * 2 * 30 + rec[1] / float(tot or 1) * 30, frac2 * 2 * 300 + rec[1] / float(tot or 1) * 300))
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
