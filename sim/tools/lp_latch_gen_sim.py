# -*- coding: utf-8 -*-
"""★2026-10-20 LP 눌림 래치 경합 (GPT 재감사 N06) — 옛 '공유 래치 RMW' vs 새 '버튼별 4비트 세대 카운터'.

옛: LP  press_latch |= low & prev_low        (5ms 마다, LP 의 RMW)
    HP  lat = press_latch; ...; press_latch &= ~lat   (HP 의 RMW)
    → 두 RMW 가 겹치면 (a) HP 의 쓰기가 LP 가 막 OR 한 **다른 버튼** 비트를 지운다(누름 손실)
                       (b) LP 의 쓰기가 HP 가 막 지운 비트를 되살린다(이미 처리한 누름이 한 번 더 = 헛누름)
새: LP  만 쓴다 — 버튼마다 4비트 카운터, '2연속 눌림이 새로 시작될 때' +1.
        버튼 0~7 은 press_gen0, 8~15 는 press_gen1 (한 버튼의 칸은 반드시 한 워드 안 → 워드별 32비트 저장 1번이면 원자적)
    HP  는 읽기만 — 자기 사본(seen)과 칸별로 다르면 그 버튼을 '그동안 눌렸음'으로 본다
    → 쓰는 쪽이 하나라 경합 자체가 없다.
    ★첫 판은 2비트(32비트 한 워드)였다 — HP 가 한 번 읽는 사이 같은 버튼이 4번 새로 눌리면 한 바퀴 돌아 '안 눌림'이
      됐다(깨움 없는 6.5초 공백에서 실제로 걸림). 4비트면 16번이다. 실제로는 LP 가 상태 변화마다 HP 를 깨워
      수 ms 안에 읽으므로 두 번 이상 쌓일 일도 드물다(아래 '깨움' 모드).

검사(같은 무작위 입력으로 두 방식)
  · 놓친 누름 : LP 가 2연속 표본으로 인정한 누름인데 HP 가 그 누름이 살아 있던 창 안에서 한 번도 '눌림'으로 못 본 것
  · 헛누름   : HP 가 래치로 '눌림'을 봤는데(지금은 안 눌림) 직전 읽기 이후 그 버튼이 한 번도 안 눌렸던 것
  · 로터리 A/B 비트는 래치에 절대 안 나타난다
  · 모드: 현실(경합 없음) / 경합을 일부러 만듦(HP 읽기마다 (a) 또는 (b) 순서가 끼어듦)
  · HP 읽기: 자주(평균 3ms) / LP 깨움(상태 변화 0.2~3ms 뒤 + 1초 주기, 실기와 같음) / 깨움 없는 1초(극단)
"""
import random
import sys

LP_US = 5000
NBTN = 16
BITS = 4
PER_WORD = 32 // BITS
ROT_A, ROT_B = 0, 1          # 로터리 비트(예시) — 래치에서 빠져야 한다
MASK = 0xFFFF & ~((1 << ROT_A) | (1 << ROT_B))
FMASK = (1 << BITS) - 1


def gen_bump(g, starts):
    """pcf_poll.c 와 같은 계산: 새로 시작된 버튼 칸만 +1. g = [word0, word1]"""
    g = list(g)
    for i in range(NBTN):
        if (starts >> i) & 1:
            w, sh = i // PER_WORD, (i % PER_WORD) * BITS
            c = ((g[w] >> sh) + 1) & FMASK
            g[w] = (g[w] & ~(FMASK << sh) & 0xFFFFFFFF) | (c << sh)
    return g


def gen_diff(g, seen):
    """button_handler.c 와 같은 계산: 칸이 다르면 그 버튼 비트"""
    lat = 0
    for i in range(NBTN):
        w, sh = i // PER_WORD, (i % PER_WORD) * BITS
        if ((g[w] ^ seen[w]) >> sh) & FMASK:
            lat |= 1 << i
    return lat


def make_input(rng, dur_us, n_press):
    """같은 버튼의 누름은 겹치지 않고 사이가 30ms 이상(LP 5ms 표본으로는 원래 못 가르는 간격은 빼고 본다)."""
    ev = []
    busy = {}
    while len(ev) < n_press:
        b = rng.choice([i for i in range(NBTN) if (MASK >> i) & 1])
        t0 = rng.randrange(0, dur_us - 500000)
        d = rng.choice([rng.randrange(8000, 40000), rng.randrange(80000, 400000)])
        if any(not (t0 + d + 30000 <= a or z + 30000 <= t0) for a, z in busy.get(b, [])):
            continue
        busy.setdefault(b, []).append((t0, t0 + d))
        ev.append((b, t0, t0 + d))
    return ev


def run(design, seed, race, hp_mode):
    rng = random.Random(seed)
    dur = 60 * 1000000
    ev = make_input(rng, dur, 300)

    def level(t):
        low = 0
        for b, a, z in ev:
            if a <= t < z:
                low |= 1 << b
        rot = (((t // 7000) & 1) << ROT_A) | (((t // 11000) & 1) << ROT_B)
        return low | rot          # 1 = 눌림(= ~rx 의 비트)

    S = dict(prev_low=0, held=0, latch=0, gen=[0, 0], last_raw=None)
    wake_req = []

    def lp_poll(t, stale_latch=None, wake=True):
        raw = level(t)
        low = raw & MASK
        now_held = low & S['prev_low']
        starts = now_held & ~S['held']
        S['held'] = now_held
        base = S['latch'] if stale_latch is None else stale_latch
        S['latch'] = base | now_held
        S['gen'] = gen_bump(S['gen'], starts)
        S['prev_low'] = low
        if raw != S['last_raw']:                 # rx != last → HP 깨움
            S['last_raw'] = raw
            if wake:   # ★경합 모드에서 끼워 넣은 폴은 깨움을 만들지 않는다(첫 판: 그 깨움이 시간 역순 읽기를 만들어 헛누름으로 셌다)
                wake_req.append(t + rng.randrange(200, 3000))

    # HP 읽기 일정
    if hp_mode == 'often':
        hp_t, t = [], 0
        while t < dur:
            t += max(200, int(rng.expovariate(1.0 / 3000)))
            hp_t.append(t)
    elif hp_mode == 'idle_nowake':
        hp_t, t = [], 0
        while t < dur:
            t += max(200, int(rng.expovariate(1.0 / 1000000)))
            hp_t.append(t)
    else:
        hp_t = None                               # 'wake': LP 깨움 + 1초 주기, 진행 중에 만든다
    seen_press = set()
    seen_copy = [0, 0]
    last_hp = 0
    next_periodic = 1000000
    phantom = rot_leak = 0
    hi = 0

    def hp_read(th):
        nonlocal seen_copy, last_hp, phantom, rot_leak
        if design == 'old':
            if race and rng.random() < 0.5:
                r = S['latch']; lat = r & MASK        # (a) HP 읽음
                lp_poll(th, wake=False)                #     LP 가 OR
                S['latch'] = r & ~lat                  #     HP 가 옛 값 기준으로 씀 → LP 의 OR 소실
            elif race:
                lat = S['latch'] & MASK
                lr = S['latch']                        # (b) LP 가 먼저 읽음
                S['latch'] &= ~lat                     #     HP 가 지움
                lp_poll(th, stale_latch=lr, wake=False)  # LP 가 옛 값 | 새 것 을 씀 → 지운 비트 부활
            else:
                lat = S['latch'] & MASK
                S['latch'] &= ~lat
        else:
            if race:
                g0 = S['gen'][0]                       # 워드 0 을 읽고
                lp_poll(th, wake=False)                # LP 가 끼어들어도
                g = [g0, S['gen'][1]]                  # 워드 1 은 그 뒤 값 — 버튼 칸이 워드를 안 넘으니 무해
            else:
                g = list(S['gen'])
            lat = gen_diff(g, seen_copy) & MASK
            seen_copy = g
        state = level(th) & MASK
        if lat & ~MASK:
            rot_leak += 1
        for b in range(NBTN):
            if (lat >> b) & 1 and not (state >> b) & 1:
                if not any(bb == b and a < th and z > last_hp for bb, a, z in ev):
                    phantom += 1
        seen_now = state | lat
        for k, (b, a, z) in enumerate(ev):
            if (seen_now >> b) & 1 and a <= th and z > last_hp:
                seen_press.add(k)
        last_hp = th

    for tl in range(0, dur, LP_US):
        if hp_t is not None:
            while hi < len(hp_t) and hp_t[hi] < tl:
                hp_read(hp_t[hi]); hi += 1
        else:
            pend = sorted(x for x in wake_req if x < tl)
            wake_req[:] = [x for x in wake_req if x >= tl]
            while next_periodic < tl:
                pend.append(next_periodic); next_periodic += 1000000
            for th in sorted(pend):
                if th > last_hp:          # 시간 순서대로만(같은 시각 깨움 여러 개는 한 번 읽기)
                    hp_read(th)
        lp_poll(tl)
    lp_saw = set()
    for k, (b, a, z) in enumerate(ev):
        if sum(1 for _ in range((a // LP_US + 1) * LP_US, z, LP_US)) >= 2:
            lp_saw.add(k)
    missed = sum(1 for k in lp_saw if k not in seen_press)
    return dict(missed=missed, total=len(lp_saw), phantom=phantom, rot_leak=rot_leak)


def main():
    ok = True
    modes = (('often', 'HP 자주(평균 3ms)'), ('wake', 'LP 깨움 + 1초(실기)'), ('idle_nowake', '깨움 없는 1초(극단)'))
    for race, rname in ((False, '현실(경합 없음)'), (True, '경합을 일부러 만듦')):
        print('[%s]' % rname)
        for hp_mode, lab in modes:
            tot = {d: dict(missed=0, total=0, phantom=0, rot_leak=0) for d in ('old', 'new')}
            for s in range(6):
                for d in ('old', 'new'):
                    r = run(d, s, race, hp_mode)
                    for k in tot[d]:
                        tot[d][k] += r[k]
            for d in ('old', 'new'):
                x = tot[d]
                print('   %-22s %-3s 인정된 누름 %5d  놓침 %4d  헛누름 %4d  로터리 샘 %d'
                      % (lab, d, x['total'], x['missed'], x['phantom'], x['rot_leak']))
            n, o = tot['new'], tot['old']
            if n['rot_leak'] or n['phantom']:
                print('   ✗ 새 방식: 헛누름/로터리 샘'); ok = False
            if n['missed'] > o['missed']:
                print('   ✗ 새 방식이 옛 방식보다 더 놓친다'); ok = False
            if hp_mode != 'idle_nowake' and n['missed'] > 0:
                print('   ✗ 새 방식이 실기 조건에서 놓쳤다'); ok = False
    print('판정:', '통과' if ok else '실패')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
