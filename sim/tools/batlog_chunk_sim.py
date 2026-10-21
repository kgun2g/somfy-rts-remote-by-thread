# -*- coding: utf-8 -*-
"""★2026-10-20 batlog 조각 저장 검증 (GPT 재감사 N08) — somfy_app.c _batlog_save/_batlog_load/_batlog_add_ev 재현.

바뀐 것: 링 전체 blob "buf"(300×11=3,300B)를 매번 쓰던 것을 20표본 조각 "b0".."b14" 로 나눠 **바뀐 조각만** 쓴다.
쓰는 순서도 n/head → buf  에서  조각 → n/head  로 바꿨다.

검사
  1. 쓰기량: 저장 1회당 쓰는 batlog 표본 바이트(옛 3,300B 고정 대비)
  2. 아무 곳에서나 전원이 끊긴 뒤 다시 읽었을 때
       · 보이는 구간(n 개)에 **추가된 적 없는 칸**(0 으로 빈 칸)이 끼지 않는다
       · 순서가 어긋난 표본(시간 역행) 수 — 옛 방식과 비교
       · 잃은 표본은 **가장 최근 것**뿐이다(마지막 저장 이후분 + 끊긴 저장분)
  3. 옛 "buf" 형식 → 조각 이전: 이전 도중 끊겨도 기록이 그대로이고, 끝나면 "buf" 가 지워진다
  4. 디코더(batlog_decode.py)의 조각 이어 붙이기 = 펌웨어 링과 같은 바이트
"""
import random
import struct
import sys

MAXN = 300
CHUNK = 20
NCH = (MAXN + CHUNK - 1) // CHUNK
ALL = (1 << NCH) - 1
SZ = 11
FAST_S, FAST_IV, SLOW_IV = 120, 5, 120


def sample(t):
    return struct.pack('<IHBBBBb', t, 4000, 80, 0, 0, 90, -60)


class Cut(Exception):
    pass


class Nvs(object):
    """키 → bytes/int. 쓰기 1건마다 예산을 깎아 0 이 되면 전원 차단(Cut)."""

    def __init__(self):
        self.kv = {}
        self.budget = None
        self.bytes = 0

    def set(self, k, v):
        if self.budget is not None:
            if self.budget <= 0:
                raise Cut()
            self.budget -= 1
        self.kv[k] = v
        if isinstance(v, (bytes, bytearray)):
            self.bytes += len(v)

    def erase(self, k):
        if self.budget is not None:
            if self.budget <= 0:
                raise Cut()
            self.budget -= 1
        self.kv.pop(k, None)


class Fw(object):
    def __init__(self, nvs, design):
        self.nvs, self.design = nvs, design
        self.buf = bytearray(SZ * MAXN)
        self.n = self.head = 0
        self.dirty = 0
        self.legacy = False

    # ── _batlog_add_ev ──
    def add(self, t):
        self.buf[self.head * SZ:(self.head + 1) * SZ] = sample(t)
        self.dirty |= 1 << (self.head // CHUNK)
        self.head = (self.head + 1) % MAXN
        if self.n < MAXN:
            self.n += 1

    # ── _batlog_save ──
    def save(self):
        nv = self.nvs
        if self.design == 'old':
            nv.set('n', self.n); nv.set('head', self.head)
            nv.set('buf', bytes(self.buf))
            return
        for c in range(NCH):
            if not (self.dirty >> c) & 1:
                continue
            a = c * CHUNK
            cnt = min(CHUNK, MAXN - a)
            nv.set('b%d' % c, bytes(self.buf[a * SZ:(a + cnt) * SZ]))
            self.dirty &= ~(1 << c)
        # ★n·head 를 **한 항목**(u32 "nh")으로 — 둘이 따로면 그 사이 전원 차단에 n=298·head=297 처럼 어긋나
        #  쓴 적 없는 칸(0)이 보였다(첫 시뮬에서 잡힘, 옛 방식도 같은 구멍). NVS 항목 1개 쓰기는 원자적이다.
        nv.set('nh', (self.n << 16) | self.head)
        if self.legacy and self.dirty == 0:
            nv.erase('buf')
            self.legacy = False

    # ── _batlog_load ──
    def load(self):
        kv = self.nvs.kv
        ok = False
        if self.design == 'new' and 'b0' in kv:
            if len(kv['b0']) == SZ * CHUNK:
                ok = True
                # ★이전 도중 끊겼으면 옛 "buf" 가 아직 있다 → 그걸 바탕으로 깔고 써진 조각만 덮는다
                #  (첫 시뮬: 바탕 없이 빈 조각을 0 으로 채워 옛 기록 300건이 비어 보였다)
                base = kv.get('buf') if len(kv.get('buf', b'')) == SZ * MAXN else None
                for c in range(NCH):
                    a = c * CHUNK
                    cnt = min(CHUNK, MAXN - a)
                    v = kv.get('b%d' % c)
                    if v is None or len(v) != SZ * cnt:
                        self.buf[a * SZ:(a + cnt) * SZ] = (base[a * SZ:(a + cnt) * SZ] if base
                                                            else b'\x00' * (SZ * cnt))
                        self.dirty |= 1 << c
                    else:
                        self.buf[a * SZ:(a + cnt) * SZ] = v
                if 'buf' in kv:
                    self.legacy = True
        elif 'buf' in kv and len(kv['buf']) == SZ * MAXN:
            ok = True
            self.buf[:] = kv['buf']
            if self.design == 'new':
                self.legacy = True
                self.dirty = ALL
        if not ok:
            self.buf[:] = b'\x00' * (SZ * MAXN); self.n = self.head = 0
        elif self.design == 'new' and 'nh' in kv:
            self.n, self.head = kv['nh'] >> 16, kv['nh'] & 0xFFFF
        else:
            self.n = kv.get('n', 0); self.head = kv.get('head', 0)

    def visible(self):
        start = 0 if self.n < MAXN else (self.head + MAXN - self.n) % MAXN
        out = []
        for i in range(self.n):
            j = (start + i) % MAXN
            out.append(struct.unpack_from('<I', self.buf, j * SZ)[0])
        return out


def decode_chunks(kv):
    """batlog_decode.py 와 같은 이어 붙이기"""
    have = [i for i in range(32) if ('b%d' % i) in kv]
    full = len(kv['b0'])
    base = kv.get('buf')
    out = b''
    for i in range(NCH if base else max(have) + 1):
        if ('b%d' % i) in kv:
            out += kv['b%d' % i]
        elif base:
            out += base[len(out):len(out) + min(full, len(base) - len(out))]
        else:
            out += b'\x00' * full
    return out


def scenario(design, seed, legacy_start=False):
    rng = random.Random(seed)
    nv = Nvs()
    if legacy_start:                   # 옛 펌웨어가 남긴 기록: 300건 꽉 참(t = 1..300)
        old = Fw(nv, 'old')
        for t in range(1, 301):
            old.add(t)
        old.save()
    fw = Fw(nv, design); fw.load()
    added = list(fw.visible())         # 지금까지 추가된 t (보이는 순서)
    t = 1000
    res = dict(saves=0, bytes=0, zero_in_view=0, order_bad=0, lost_not_suffix=0, cuts=0)
    for step in range(rng.randrange(200, 2000)):
        t += FAST_IV if rng.random() < 0.1 else SLOW_IV
        fw.add(t); added.append(t)
        b0 = nv.bytes
        if rng.random() < 0.02:        # 저장 도중 전원 차단 → 재부팅
            nv.budget = rng.randrange(0, 5)
            try:
                fw.save()
            except Cut:
                pass
            nv.budget = None
            res['cuts'] += 1
            fw = Fw(nv, design); fw.load()
            v = fw.visible()
            if 0 in v:
                res['zero_in_view'] += 1
            res['order_bad'] += sum(1 for a, b in zip(v, v[1:]) if b < a)
            # 잃은 것이 최근 것뿐인가: 보이는 마지막 t 이후로만 빠져야 한다
            if v:
                tail = [x for x in added if x <= v[-1]][-len(v):]
                if tail != v and sorted(v) == v:
                    res['lost_not_suffix'] += 1
            added = list(v)
        else:
            fw.save()
        res['saves'] += 1
        res['bytes'] += nv.bytes - b0
    res['legacy_left'] = ('buf' in nv.kv) if design == 'new' else None
    res['chunks_ok'] = (decode_chunks(nv.kv) == bytes(fw.buf)) if design == 'new' and 'b0' in nv.kv else None
    res['final'] = fw
    return res


def main():
    ok = True
    agg = {}
    for design in ('old', 'new'):
        a = dict(saves=0, bytes=0, zero_in_view=0, order_bad=0, lost_not_suffix=0, cuts=0)
        for s in range(60):
            r = scenario(design, s)
            for k in a:
                a[k] += r[k]
        agg[design] = a
        print('[%s] 저장 %d회, 회당 표본 바이트 %.0fB, 전원 차단 %d회 → 빈 칸 노출 %d, 순서 역행 %d, 최근 아닌 손실 %d'
              % (design, a['saves'], a['bytes'] / float(a['saves']), a['cuts'], a['zero_in_view'],
                 a['order_bad'], a['lost_not_suffix']))
    cut = 100.0 * (1 - agg['new']['bytes'] / float(agg['old']['bytes']))
    print('  → 쓰기량 %.1f%% 감소' % cut)
    if cut < 85:
        print('  ✗ 감소가 85%% 미만'); ok = False
    if agg['new']['zero_in_view']:
        print('  ✗ 빈 칸이 보이는 구간에 끼었다'); ok = False
    if agg['new']['order_bad'] > agg['old']['order_bad']:
        print('  ✗ 순서 역행이 옛 방식보다 많다'); ok = False
    if agg['new']['lost_not_suffix']:
        print('  ✗ 최근 것이 아닌 표본을 잃었다'); ok = False

    print('[이전] 옛 "buf" 300건에서 시작 — 이전 도중 전원 차단 포함, 시드 60개')
    bad = left = 0
    for s in range(60):
        r = scenario('new', s, legacy_start=True)
        v = r['final'].visible()
        if 0 in v:
            bad += 1
        if r['legacy_left']:
            left += 1
        if r['chunks_ok'] is False:
            bad += 1
    print('  빈 칸·디코더 불일치 %d건, 끝나고도 "buf" 가 남은 경우 %d건' % (bad, left))
    if bad or left:
        ok = False
    # 이전 직후(첫 저장 전에 끊김) 옛 기록이 그대로 읽히는가
    nv = Nvs(); o = Fw(nv, 'old')
    for t in range(1, 301):
        o.add(t)
    o.save()
    f = Fw(nv, 'new'); f.load(); f.add(999)
    nv.budget = 3
    try:
        f.save()
    except Cut:
        pass
    nv.budget = None
    g = Fw(nv, 'new'); g.load()
    vv = g.visible()
    print('  첫 이전 저장이 조각 3개 쓰고 끊긴 경우: 보이는 %d건, 빈 칸 %s, 옛 buf %s'
          % (len(vv), '있음' if 0 in vv else '없음', '남음(다음 저장 때 지움)' if 'buf' in nv.kv else '없음'))
    if 0 in vv:
        ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
