# -*- coding: utf-8 -*-
"""로터리 핀 주차(★2026-10-31) — LP 코어 폴링 + PCF8575 준양방향 핀 + 접점 바운스 시뮬레이터.

왜 (2026-10-31 사용자 "로터리 대책도 적용해보자")
──────────────────────────────────────────────
하프스텝 로터리는 디텐트가 11 과 00 을 번갈아 쓴다. 00(두 접점 닫힘)에 멈추면 PCF8575 의 '1 쓰기 = 약한 전류원'
(핀당 30~300µA)이 접점으로 계속 흐른다. 10/30 lsfloor rot 실측 0.13±0.36mA(접점당 66µA).
대책 = **주차**: 폴 끝에 LOW 로 읽힌 로터리 핀을 PCF 출력 0(강한 LOW)으로 몰아 둔다 — 닫힌 접점과 같은 GND 라 전류 0.
       다음 폴은 **놓고 읽는다**(write 0xFFFF → 반복 START → read, 한 트랜잭션). 읽는 시점은 늘 놓인 뒤다.

이 시뮬레이터가 지키는 것
  ① 회전 디코딩이 주차 전과 같다 — 같은 회전·바운스 시나리오에서 디텐트 오차가 늘지 않는다(느림~아주 빠름).
  ② **몰아 둔 LOW 를 접점으로 오독하지 않는다** — 매 폴의 rx 가 그 순간 접점 상태와 같다(불일치 0).
  ③ I2C 실패(주소 NACK / 놓기 뒤 읽기 실패 / 주차 쓰기 반쯤 적용) · LP 재기동(LP RAM 초기화) · HP 인계에서도 ②가 지켜진다.
  ④ 음성 대조: 재기동 때 '안 몰려 있다'고 가정하거나 HP 인계 때 놓지 않으면 ②가 깨진다 — 시뮬레이터가 그걸 잡는다.
  ⑤ 전류: 00 정지 중 전류원이 접점으로 흐르는 시간 비율(옛 100% → 놓고-읽고-다시 모는 창만큼), I2C 추가 비트.
  ⑥ park_en=0(lsfloor park 의 B 단계) 이면 다음 폴에 놓고 옛 동작으로 돌아간다.
"""
import random
import sys

POLL_US = 5000                  # pcf_lp_config.h LP_POLL_US — 바꾸지 않는다
BIT_US = 2.5                    # LP_I2C 400kHz (LP_CORE_I2C_DEFAULT_CONFIG = FAST_MODE)
BOOT_US = 120                   # halt → 기상 → I2C 시작까지(실측 '폴 1회 ~0.25ms' 와 맞춘 가정)
PARK_IDLE_POLLS = 40            # pcf_lp_config.h LP_ROT_PARK_IDLE_POLLS — 로터리 비트가 이만큼(200ms) 그대로일 때만 주차
ROT_A, ROT_B = 0, 1
ROT_MASK = (1 << ROT_A) | (1 << ROT_B)
KQUAD = [0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0]   # pcf_poll.c kQuad 와 같은 표


class Encoder:
    """하프스텝 로터리 접점. closed(pin,t) = 접점이 닫혔나(GND). 디텐트는 11 과 00 을 번갈아."""

    def __init__(self, rng, start_ab=0b00):
        self.rng = rng
        self.events = []        # (t, pin, closed) — 시간순
        self.ab = start_ab      # 논리 위치(1=열림)
        self.truth = 0          # 실제로 돌린 디텐트(+CW)

    def turn(self, t0, n, dt_us, bounce_us):
        """t0 부터 n 디텐트(부호=방향)를 디텐트당 dt_us 로 돈다. 전이마다 bounce_us 안에서 접점이 튄다."""
        cw = n > 0
        t = t0
        for _ in range(abs(n)):
            for _half in range(2):
                # CW: 00→01→11→10→00 (kQuad +1) — 바꿀 비트: 00·11 에서는 B, 01·10 에서는 A
                a, b = (self.ab >> 1) & 1, self.ab & 1
                if cw:
                    pin = ROT_B if a == b else ROT_A
                else:
                    pin = ROT_A if a == b else ROT_B
                bitpos = 0 if pin == ROT_B else 1
                newv = ((self.ab >> bitpos) & 1) ^ 1
                tt = t + dt_us * 0.5 * self.rng.uniform(0.3, 0.7)
                k = self.rng.randint(0, 4) if bounce_us > 0 else 0
                for j in range(k):     # 바운스: 새 값과 옛 값을 오간다
                    tb = tt + self.rng.uniform(0, bounce_us)
                    self.events.append((tb, pin, (j % 2 == 0) == (newv == 0)))
                self.events.append((tt + bounce_us + 1, pin, newv == 0))
                self.ab ^= (1 << bitpos)
                t += dt_us * 0.5
            self.truth += 1 if cw else -1
        return t

    def finalize(self):
        self.events.sort()
        self.idx = 0
        self.state = {ROT_A: False, ROT_B: False}   # 시작 위치(아래 run 에서 설정)

    def closed(self, pin, t):
        while self.idx < len(self.events) and self.events[self.idx][0] <= t:
            _, p, c = self.events[self.idx]
            self.state[p] = c
            self.idx += 1
        return self.state[pin]


class Pcf:
    """PCF8575 준양방향 핀 — latch 1 = 약한 전류원 입력, 0 = 강한 LOW. 읽힘 = 0 if (latch 0 or 접점 닫힘)."""

    def __init__(self, enc):
        self.enc = enc
        self.latch = 0xFFFF
        self.t_last = 0.0
        self.src_on_us = {ROT_A: 0.0, ROT_B: 0.0}   # 전류원이 닫힌 접점으로 흐른 시간
        self.rng = None

    def _accum(self, t):
        # 직전 시각부터 t 까지 latch 1 + 접점 닫힘이면 전류원이 흐른다(구간 끝 상태로 근사 — 1µs 격자)
        step = 20.0
        tt = self.t_last
        while tt < t:
            h = min(step, t - tt)
            for p in (ROT_A, ROT_B):
                if (self.latch >> p) & 1 and self.enc.closed(p, tt + h):
                    self.src_on_us[p] += h
            tt += h
        self.t_last = t

    def write(self, t, v):
        self._accum(t)
        self.latch = v & 0xFFFF

    def read(self, t):
        self._accum(t)
        v = 0xFFFF
        for p in (ROT_A, ROT_B):
            if not (self.latch >> p) & 1 or self.enc.closed(p, t):
                v &= ~(1 << p)
        return v

    def truth(self, t):
        v = 0xFFFF
        for p in (ROT_A, ROT_B):
            if self.enc.closed(p, t):
                v &= ~(1 << p)
        return v


class Lp:
    """pcf_poll.c main() 1회 = poll(). park=True 가 새 동작. 고장 주입은 fail(kind) 로."""

    def __init__(self, park, assume_parked_on_boot=True, idle_polls=PARK_IDLE_POLLS):
        self.park_en = 1 if park else 0
        self.idle_polls = idle_polls
        self.assume = assume_parked_on_boot
        self.reset()
        self.rot_delta = 0
        self.mismatch = 0          # rx 의 로터리 비트가 그 순간 접점과 다름(= 몰아 둔 LOW 를 오독)
        self.bits = 0              # I2C 비트 수(추가 비용)
        self.park_polls = 0
        self.polls = 0
        self.err = 0

    def reset(self):
        """LP RAM 재적재 — 파일 스코프 static 이 초깃값으로."""
        self.prev_ab = 0x03
        self.accum = 0
        self.parked = ROT_MASK if self.assume else 0
        self.last_rot = 0xFFFFFFFF        # 정지 판정용 직전 로터리 비트(아직 모름)
        self.still = 0

    def poll(self, t, pcf, fail=None):
        """t = 기상 시각. 반환 = 이번 폴이 끝난 시각."""
        t += BOOT_US
        if self.parked:
            # write_read: START+주소(9)+2바이트(18) → 놓기 적용 → Sr+주소(9) → 표본 → 2바이트(18)+STOP
            if fail == 'addr':
                self.bits += 10; self.err += 1; self.parked = ROT_MASK
                return t + 10 * BIT_US
            pcf.write(t + 27 * BIT_US, 0xFFFF)
            if fail == 'rd_after_rel':
                self.bits += 37; self.err += 1; self.parked = ROT_MASK
                return t + 37 * BIT_US
            rx = pcf.read(t + 36 * BIT_US)
            tr = pcf.truth(t + 36 * BIT_US)
            self.bits += 56
            t_end = t + 56 * BIT_US
        else:
            if fail == 'addr':
                self.bits += 10; self.err += 1; self.parked = ROT_MASK
                return t + 10 * BIT_US
            rx = pcf.read(t + 9 * BIT_US)
            tr = pcf.truth(t + 9 * BIT_US)
            self.bits += 28
            t_end = t + 28 * BIT_US
        self.polls += 1
        if (rx ^ tr) & ROT_MASK:
            self.mismatch += 1
        ab = (((rx >> ROT_A) & 1) << 1) | ((rx >> ROT_B) & 1)
        if ab != self.prev_ab:
            self.accum += KQUAD[((self.prev_ab & 3) << 2) | (ab & 3)]
            self.prev_ab = ab
            if self.accum >= 2:
                self.rot_delta += 1; self.accum = 0
            elif self.accum <= -2:
                self.rot_delta -= 1; self.accum = 0
        t_end += 15                       # 디코딩·래치·HP 깨움(ulp_lp_core_wakeup_main_processor)
        rot = rx & ROT_MASK
        if rot == self.last_rot:
            if self.still < self.idle_polls:
                self.still += 1
        else:
            self.last_rot, self.still = rot, 0
        want = (~rx & ROT_MASK) if (self.park_en and self.still >= self.idle_polls) else 0
        if want:
            if fail == 'park_half':       # 첫 바이트(P0x)는 적용됐는데 둘째에서 NACK → 오류 반환
                pcf.write(t_end + 18 * BIT_US, 0xFFFF & ~want)
                self.bits += 19; self.err += 1; self.parked = ROT_MASK
                return t_end + 19 * BIT_US
            if fail == 'park_addr':
                self.bits += 10; self.err += 1; self.parked = ROT_MASK
                return t_end + 10 * BIT_US
            pcf.write(t_end + 18 * BIT_US, 0xFFFF & ~want)
            self.bits += 28
            self.parked = want
            self.park_polls += 1
            return t_end + 28 * BIT_US
        self.parked = 0
        return t_end


def scenario(seed, speed_dt_us, n_bursts=40, bounce_us=1500, start_ab=0b00):
    rng = random.Random(seed)
    enc = Encoder(rng, start_ab)
    t = 50_000.0
    rests = []
    for _ in range(n_bursts):
        n = rng.choice([1, 1, 2, 3, 5, 8, -1, -1, -2, -3, -5, -8])
        t = enc.turn(t, n, speed_dt_us * rng.uniform(0.8, 1.25), bounce_us)
        rest = rng.uniform(0.15, 2.0) * 1e6
        rests.append((t + 20_000, t + rest))
        t += rest
    enc.finalize()
    enc.state = {ROT_A: not ((start_ab >> 1) & 1), ROT_B: not (start_ab & 1)}
    return enc, t, rests


def run(seed, speed_dt_us, park, fail_p=0.0, restart_p=0.0, assume=True, bounce_us=1500, start_ab=0b00,
        park_toggle=None, n_bursts=40, fair=False):
    """fair=True: 실패는 늘 '주소 NACK'(= 그 폴의 읽기를 잃음) — 옛·새 방식의 읽기 손실 확률을 같게 둔다."""
    enc, t_end, rests = scenario(seed, speed_dt_us, n_bursts=n_bursts, bounce_us=bounce_us, start_ab=start_ab)
    pcf = Pcf(enc)
    if not park:
        assume = False
    lp = Lp(park, assume)
    rng = random.Random(seed * 7 + 3)
    t = 0.0
    while t < t_end:
        if park_toggle is not None:
            lp.park_en = 1 if park_toggle(t) else 0
        if restart_p and rng.random() < restart_p:
            lp.reset()                    # LP 재기동 — 공유 RAM rot_delta 는 HP 가 이미 소비했다고 본다(누적 비교만)
        f = None
        if fail_p and rng.random() < fail_p:
            f = 'addr' if fair else rng.choice(['addr', 'rd_after_rel', 'park_half', 'park_addr'])
        done = lp.poll(t, pcf, f)
        t = done + POLL_US                # LP 타이머는 halt 뒤부터 센다
    pcf._accum(t_end)
    rest_us = sum(b - a for a, b in rests)
    return {'enc': enc, 'lp': lp, 'pcf': pcf, 'err_det': abs(lp.rot_delta - enc.truth), 'truth': enc.truth,
            'mismatch': lp.mismatch, 'bits_per_s': lp.bits / (t_end / 1e6), 'src_us': pcf.src_on_us, 't_end': t_end,
            'rest_us': rest_us}


def hp_handoff(park_release):
    """LP 가 00 에 주차한 채 멈춤 → HP 인계 → 사용자가 노브를 11 로 돌림 → HP 가 읽는다."""
    rng = random.Random(5)
    enc = Encoder(rng, 0b00)
    enc.turn(600_000, 1, 40_000, 0)       # 0.6초에 00 → 11 (한 디텐트)
    enc.finalize()
    enc.state = {ROT_A: True, ROT_B: True}
    pcf = Pcf(enc)
    lp = Lp(True)
    t = 0.0
    while t < 400_000:                    # 0.4초 동안 LP 가 돌며 주차(정지 200ms 뒤부터)
        t = lp.poll(t, pcf) + POLL_US
    assert lp.parked == ROT_MASK
    # ── 인계: LP 정지(_lp_pins_to_hp). 새 동작은 여기서 0xFFFF 를 쓴다 ──
    if park_release:
        pcf.write(t + 100, 0xFFFF)
    rx = pcf.read(800_000)                # 노브는 이미 11(열림)
    return (rx & ROT_MASK) == ROT_MASK


def main():
    cases = 0
    # ① 회전 디코딩 동치 — 느림~아주 빠름, 바운스 1.5ms, 20시드. 주차가 오차를 늘리지 않는다.
    #   (8ms/디텐트 = 125디텐트/초 는 원래 펌웨어도 놓친다 — 2026-08-27 power_lever_sim: 5ms 폴은 전이 간격 ≥ ~5ms 까지)
    for dt in (200_000, 80_000, 40_000, 20_000, 12_000):
        e_old = e_new = 0
        for seed in range(1, 21):
            o = run(seed, dt, park=False)
            n = run(seed, dt, park=True)
            e_old += o['err_det']
            e_new += n['err_det']
            assert n['mismatch'] == 0, ('★몰아 둔 LOW 를 오독', dt, seed, n['mismatch'])
        # 20ms 이하는 전이 간격이 ~5ms(폴 주기)까지 좁아지는 한계 영역 — 옛 방식도 놓친다. 주차는 **정지 200ms 뒤에만** 하므로
        #  돌리는 동안의 폴은 옛 방식과 같고, 다른 것은 정지에서 벗어나는 첫 전이의 표본 시각(+0.07ms)·그 폴의 길이뿐이다.
        #  (처음엔 늘 주차했더니 주차 중 폴이 0.17ms 길어져 실효 주기가 3% 늘고 12ms/디텐트에서 57→68 로 나빠졌다.)
        #  40ms 이상은 둘 다 0 이어야 한다.
        assert e_new <= e_old + 2, ('★주차가 디텐트 오차를 늘렸다', dt, e_old, e_new)
        if dt >= 40_000:
            assert e_old == 0 and e_new == 0, (dt, e_old, e_new)
        print('  ① 디텐트당 %5.1fms: 오차 옛 %d / 새 %d (20시드 × 40묶음)' % (dt / 1000, e_old, e_new))
    cases += 1

    # ② 아주 빠른 회전(디텐트 8ms) — 둘 다 놓칠 수 있는 영역에서도 새 쪽이 더 나쁘지 않다(같은 표본 시각 ±0.07ms)
    e_old = sum(run(s, 8_000, park=False)['err_det'] for s in range(1, 21))
    e_new = sum(run(s, 8_000, park=True)['err_det'] for s in range(1, 21))
    assert e_new <= e_old + 2, (e_old, e_new)
    print('  ② 디텐트당 8ms(한계 밖): 오차 옛 %d / 새 %d' % (e_old, e_new))
    cases += 1

    # ③ I2C 실패 5%(네 가지 모양) + LP 재기동 0.2% — **오독 0**(안전성). 실패가 읽기를 잃게 하면 디텐트를 놓치는 건
    #   옛 방식도 같다. 새 방식은 주차 중 트랜잭션이 길어(놓고 읽기 56비트 vs 읽기 28비트) 같은 바이트당 실패율이면
    #   읽기 손실이 더 잦다 — 그래서 (a) 안전성은 네 모양 전부로, (b) 오차는 **읽기 손실 확률을 같게** 두고 비교한다.
    #   (실기 i2c_err 는 며칠 단위로 0 근처 — 5% 는 일부러 가혹하게 잡은 값)
    e_old = e_new = mm = 0
    for seed in range(1, 21):
        n = run(seed, 40_000, park=True, fail_p=0.05, restart_p=0.002)
        mm += n['mismatch']
    assert mm == 0, ('★실패·재기동 뒤 몰아 둔 LOW 를 오독', mm)
    for seed in range(1, 21):
        o = run(seed, 40_000, park=False, fail_p=0.0125, restart_p=0.002, fair=True)   # 'addr' 만 = 읽기 손실 1.25%
        n = run(seed, 40_000, park=True, fail_p=0.0125, restart_p=0.002, fair=True)
        e_old += o['err_det']; e_new += n['err_det']
    assert e_new <= e_old + 3, (e_old, e_new)
    print('  ③ I2C 실패 5%%(4모양)·LP 재기동 0.2%%: 오독 %d / 읽기 손실 같게(1.25%%): 오차 옛 %d · 새 %d' % (mm, e_old, e_new))
    cases += 1

    # ④ 음성 대조 — 재기동 때 '안 몰려 있다'고 가정하면 오독이 생긴다(그래서 parked 초깃값 = ROT_MASK)
    mm = sum(run(s, 40_000, park=True, restart_p=0.01, assume=False)['mismatch'] for s in range(1, 21))
    assert mm > 0, '음성 대조가 오독을 못 만들었다 — 시뮬레이터가 이 위험을 못 본다'
    print('  ④ 음성 대조(재기동 때 놓지 않음): 오독 %d회 — 초깃값 ROT_MASK 가 필요하다' % mm)
    cases += 1

    # ⑤ HP 인계 — 놓고 넘기면 HP 가 실제 위치(11)를 본다 / 안 놓으면 00 으로 굳는다(음성 대조)
    assert hp_handoff(True), '★인계 때 놓았는데 HP 가 실제 위치를 못 본다'
    assert not hp_handoff(False), '음성 대조 실패'
    print('  ⑤ HP 인계: 0xFFFF 쓰고 넘김 → 실제 위치 / 안 쓰면 00 고착(음성 대조) — _lp_pins_to_hp 에 쓰기 필요')
    cases += 1

    # ⑥ 전류: 00 에 오래 멈춤(회전 1묶음 뒤 정지) — 전류원이 닫힌 접점으로 흐르는 시간 비율
    def rest_case(park):
        r = run(11, 40_000, park=park, n_bursts=1)
        return r, (r['src_us'][ROT_A] + r['src_us'][ROT_B]) / 2 / r['t_end']
    ro, fo = rest_case(False)
    rn, fn = rest_case(True)
    assert ro['enc'].ab == 0b00 or ro['enc'].ab == 0b11
    print('  ⑥ 정지 위치 %s: 전류원 흐름 비율 옛 %.1f%% → 새 %.2f%%  (핀당 66µA 면 접점 2개 %.0f → %.1f µA)'
          % ('00(닫힘)' if ro['enc'].ab == 0 else '11(열림)', fo * 100, fn * 100, 2 * 66 * fo, 2 * 66 * fn))
    # 00 정지 강제 확인 — 시작 00 에서 회전 없이
    def still(park, ab):
        enc = Encoder(random.Random(1), ab)
        enc.finalize()
        enc.state = {ROT_A: not ((ab >> 1) & 1), ROT_B: not (ab & 1)}
        pcf = Pcf(enc)
        lp = Lp(park, park)
        t = 0.0
        while t < 20e6:                   # 20초 — 주차 전 200ms 는 1% 로 묻힌다
            t = lp.poll(t, pcf) + POLL_US
        pcf._accum(t)
        return (pcf.src_on_us[ROT_A] + pcf.src_on_us[ROT_B]) / 2 / t, lp.bits / (t / 1e6), lp
    f00o, b00o, _ = still(False, 0b00)
    f00n, b00n, l00 = still(True, 0b00)
    f11o, b11o, _ = still(False, 0b11)
    f11n, b11n, l11 = still(True, 0b11)
    assert f00o > 0.99 and f00n < 0.05, (f00o, f00n)
    assert f11o == 0 and f11n == 0 and b11n <= b11o * 1.01 + 1, (b11o, b11n)
    extra_us = (b00n - b00o) * BIT_US       # 초당 추가 I2C 시간
    print('  ⑥ 00 정지: 전류원 흐름 %.1f%% → %.2f%% / I2C %.0f → %.0f 비트/초 (LP·버스 활성 +%.0fµs/초 = +%.2f%%)'
          % (f00o * 100, f00n * 100, b00o, b00n, extra_us, extra_us / 1e4))
    print('     11 정지: 전류 0 그대로, I2C %.0f → %.0f 비트/초(추가 없음) · 주차 폴 비율 00=%.0f%% 11=%.0f%%'
          % (b11o, b11n, 100.0 * l00.park_polls / l00.polls, 100.0 * l11.park_polls / max(1, l11.polls)))
    cases += 1

    # ⑦ park_en=0(lsfloor park 의 B 단계) — 다음 폴에 놓고 옛 동작(전류원 흐름 100%)으로
    #   (lsfloor 단계는 1시간이라 첫 200ms 는 무시할 만하다. 여기선 3초 구간으로 본다 — 첫 구간만 200ms 주차 전)
    def toggle(t):
        return t < 3e6 or t >= 6e6
    enc = Encoder(random.Random(1), 0b00)
    enc.finalize()
    enc.state = {ROT_A: True, ROT_B: True}
    pcf = Pcf(enc)
    lp = Lp(True)
    t, seg = 0.0, {}
    while t < 9e6:
        lp.park_en = 1 if toggle(t) else 0
        k = int(t // 3e6)
        before = pcf.src_on_us[ROT_A]
        t2 = lp.poll(t, pcf) + POLL_US
        pcf._accum(t2)
        seg[k] = seg.get(k, 0) + pcf.src_on_us[ROT_A] - before
        t = t2
    assert seg[0] < 0.12 * 3e6 and seg[1] > 0.99 * 3e6 and seg[2] < 0.05 * 3e6, seg
    assert lp.mismatch == 0
    print('  ⑦ park_en 1→0→1: 구간별 전류원 흐름 %.1f%% / %.1f%% / %.1f%% — 끄면 옛 동작, 다시 켜면 곧바로 주차(정지 그대로)'
          % (seg[0] / 3e4, seg[1] / 3e4, seg[2] / 3e4))
    cases += 1

    print('rot_park_sim %d케이스 통과' % cases)
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
