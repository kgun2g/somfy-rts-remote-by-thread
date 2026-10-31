# -*- coding: utf-8 -*-
"""얕은잠 바닥 전류 대조군(`lsfloor`) 결과 해석기.

왜 필요한가 (2026-10-13)
────────────────────────
깊은잠 대조군(9/29)으로 "기판은 ~0.3mA, 범인은 칩"까지 좁혔다. 평소 4.56mA 중
깨어남·무선 몫을 빼면 **얕은잠 상태 자체에 약 3mA** 가 남는데 정체를 모른다.
데이터시트의 얕은잠 수치(0.18mA)는 "무선 모듈 전원 차단"이 전제다. 그런데 이 기기는
ADC 유닛 때문에 MODEM 도메인이 절전 중에도 **항상 켜져 있다**. 그게 3mA 일 수 있다.

같은 날 MODEM 을 꺼 봤더니(adcfix=1) 28초 뒤 Thread 가 멈췄다. 그래서 **무선을
켜기 전**(esp_matter::start 이전)에 순수 얕은잠만 돌리며 MODEM 을 켰다 껐다 한다.
무선이 아직 없으니 멈출 것도 없다.

    단계 0  MODEM 켜짐 (평소와 같은 상태 — ADC 유닛 상주)
    단계 1  MODEM 꺼짐 (유닛 삭제)
    단계 2  MODEM 켜짐   ← 같은 조건을 한 번 더 — 표류·완화와 구분하려고
    단계 3  MODEM 꺼짐

읽는 법
  · 꺼짐 단계 끝이 켜짐 단계 끝보다 번갈아 위에 있다(톱니) → MODEM 도메인이 전류를 쓴다.
    그러면 "MODEM 을 꺼도 Thread 가 사는 방법"을 찾는 것이 15일로 가는 길이다.
  · 톱니가 없고 전체가 평평 → 무선 없는 얕은잠은 문제없다. 3mA 는 무선 스택이 뜬 뒤에 생긴다.
  · 톱니가 없고 전체가 가파름 → MODEM 무관. TOP 도메인 등 다른 곳을 봐야 한다.

★2026-10-16 `lsfloor wake` (fac=3) — 깨어남 비용을 CC1101 IDLE(1.7mA) 로 현장 교정해 mA 로 잰다.
  단계 A(바닥)-B(+깨어남)-A-C(+CC1101)-A. 아래 analyse_wake() 머리말 참조.

★전압 기울기를 mA 로 바꾸는 계수는 셀·전압대마다 다르다(0.9~1.3 mV/h per mA).
  mA 는 **범위**로만 말한다. 측정이 끝난 뒤 기기는 평소대로 돌아가므로, 그 뒤 몇 시간의
  평소 방전 기울기(batlog)와 **비율**로 비교하는 것이 가장 믿을 만하다.

    python sim/tools/nvs_dump_safe.py COM7 --tag lsfloor
    python sim/tools/lsfloor_decode.py logs/nvs_COM7_..._lsfloor.bin
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PHASES = 4                      # 기본 단계 수 (펌웨어 LSF_PHASES)
MAX_SAMPLES = 96
MAX_PER_PHASE = 23              # 4*23+1 = 93 <= 96
WAKE_TIMER = 4                  # ESP_SLEEP_WAKEUP_TIMER
FLAGS_NONE = 0xFFFF             # 절전 없이 찍은 표본(첫 표본, 모의 실행)

BIT_TOP, BIT_VDDSDIO, BIT_MODEM, BIT_HP_PERIPH, BIT_CPU = 0, 1, 2, 3, 4
# ★2026-10-15 무엇을 켰다 껐다 하는가(NVS lsfl/fac). 0=MODEM(첫 본측정), 1=TOP(주변장치 도메인).
#  TOP 측정도 같은 손잡이(ADC 유닛 상주/삭제)를 쓴다 — CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
#  에서는 ADC 유닛이 MODEM 과 TOP 을 **둘 다** 붙잡기 때문이다. MODEM 몫은 0 으로 이미 쟀으니
#  남는 차이는 TOP 몫이다. 판정은 그 요인의 전원차단 비트로 검증한다.
FACTORS = {0: ('MODEM', BIT_MODEM), 1: ('TOP', BIT_TOP), 2: ('LP', None)}
# ★2026-10-16 fac=2(LP 폴링 켬/끔)는 전원 도메인이 아니라 플래그 비트로 검증할 수 없다.
#  대신 단계별 LP 폴링 수(NVS lp0~lp3)로 검증한다: 끈 단계는 ~0회/초, 켠 단계는 평소(~190회/초).
LP_ON_MIN, LP_OFF_MAX = 100.0, 1.0


def lp_rates(meta, samples, nph=PHASES, prefix='lp'):
    """NVS lp0..lp(n-1)(측정 시작부터의 누적 LP 폴링 수) → 단계별 초당 폴링. 기록이 없으면 None.
    ★2026-10-31 prefix='pp' 면 같은 방식으로 단계별 주차 폴(fac 9 PARK)."""
    vals = [meta.get('%s%d' % (prefix, p)) for p in range(nph)]
    if not samples or all(v is None for v in vals):
        return None
    prev_c, prev_t, rates = 0, 0, []
    for p in range(nph):
        v = vals[p]
        ends = [s['t_s'] for s in samples if s['phase'] == p and s['t_s'] > 0]
        if v is None or not ends:
            break
        rates.append((v - prev_c) / max(1, ends[-1] - prev_t))
        prev_c, prev_t = v, ends[-1]
    return rates

RUN_NONE, RUN_ACTIVE, RUN_DONE, RUN_RESET, RUN_USB = 0, 1, 2, 3, 4
RUN_NAMES = {RUN_NONE: '아직 시작 안 함', RUN_ACTIVE: '★측정 중이거나 도중에 전원이 죽음',
             RUN_DONE: '완료', RUN_RESET: '★도중에 리셋됨(받은 표본까지만 유효)',
             RUN_USB: 'USB 가 꽂혀 중단됨(받은 표본까지만 유효)'}

# 기울기 → 전류 환산 계수 범위 (mV/h per mA).
#   0.915 : 2026-09-11 COM8 평소 2.79mV/h = 3.05mA
#   1.3   : 2026-09-29 COM7 4.04~4.12V 대, 평소 -6.47mV/h ≈ 5mA
K_LO, K_HI = 0.9, 1.3
THR_SMALL = 1.0                 # 주기 평균 기울기가 이보다 완만하면 "평평"
THR_LARGE = 1.5                 # 이보다 가파르면 "가파름"
S_MIN = 0.8                     # 톱니 진폭(mV)이 이보다 커야 "MODEM 이 전류를 쓴다" — 약 1mA
END_N = 3                       # 단계 끝점 = 마지막 3표본 평균
IPD_LO, IPD_HI = 0.3, 0.6       # MODEM 을 끈 얕은잠 바닥의 **가정** 범위(mA): 기판 0.3 + 칩 얕은잠
DI_BOUND = 1.5                  # 상한(95%)이 이보다 작으면 "큰 몫은 없다"로 판정을 내린다(2026-10-15)


def phase_wants_pd(p):
    """홀수 단계에서 MODEM 을 끈다 (켬-끔-켬-끔)."""
    return (p % 2) == 1


def pack(t_s, mv10, phase, cause, usb, want_pd, flags16, rej):
    """somfy_app.c _lsf_pack 과 비트 단위로 같아야 한다."""
    t_s = min(int(t_s), 0xFFFFF)
    mv10 = min(int(mv10), 0xFFFF)
    cause = min(int(cause), 15)
    return (t_s
            | (mv10 << 20)
            | ((phase & 7) << 36)
            | (cause << 39)
            | ((1 if usb else 0) << 43)
            | ((1 if want_pd else 0) << 44)
            | ((flags16 & 0xFFFF) << 45)
            | ((1 if rej else 0) << 61))


def unpack(v):
    return {'t_s': v & 0xFFFFF,
            'mv': ((v >> 20) & 0xFFFF) / 10.0,
            'phase': (v >> 36) & 7,
            'cause': (v >> 39) & 15,
            'usb': (v >> 43) & 1,
            'want_pd': (v >> 44) & 1,
            'flags': (v >> 45) & 0xFFFF,
            'rej': (v >> 61) & 1}


def lsq_slope(points):
    """[(t_s, mV)] → mV/시간. 점이 2개 미만이면 None."""
    n = len(points)
    if n < 2:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    den = sum((p[0] - mx) ** 2 for p in points)
    if den == 0:
        return None
    return sum((p[0] - mx) * (p[1] - my) for p in points) / den * 3600.0


def _mean(xs):
    return sum(xs) / len(xs)


def analyse(samples, nph=PHASES, fac=0, lprates=None):
    """표본 목록 → {'phases': [...], 'verdict': 코드, 'why': 설명, ...}

    ★★왜 "단계별 기울기 차이"로 판정하지 않는가 (2026-10-13 시뮬레이터가 잡아냄)
      부하가 바뀌면 셀 단자전압이 수 분~수십 분에 걸쳐 **완화**한다(부하가 줄면 회복,
      늘면 처짐). 60분 단계의 기울기에는 이 완화가 그대로 섞여, 3.0↔0.3mA 를 넣은
      모델에서 차이가 2.7mA 가 아니라 3.6~5.2mA 로 **45% 넘게 부풀었다**.
      그래서 완화에 휘둘리지 않는 두 가지를 쓴다:
        ① 톱니 진폭 S — 단계 끝점이 이웃 두 끝점을 이은 선에서 얼마나 벗어나는가.
           MODEM 이 전류를 쓰면 꺼짐 끝은 위로, 켜짐 끝은 아래로 **번갈아** 벗어난다.
           완화도 같은 방향으로 보태므로(부하가 줄면 회복) 존재 판정에는 오히려 유리하고,
           직선 표류·완만한 곡률은 상쇄된다.
        ② 주기 평균 기울기 — **같은 종류 끝점끼리**(꺼짐 끝 ↔ 다음 꺼짐 끝) 비교.
           완화 상태가 같으므로 순수한 방전분만 남는다 = (켜짐+꺼짐) 평균 소비.
      단계별 기울기는 참고용으로만 찍는다."""
    fname, fbit = FACTORS.get(fac, FACTORS[0])
    out = {'phases': [], 'verdict': 'EMPTY', 'why': '표본이 없다', 'factor': fname}
    if not samples:
        return out
    dry = all(s['flags'] == FLAGS_NONE for s in samples)
    bad_flags = 0
    cpu_bad = 0
    resid = []
    for p in range(nph):
        # 단계 p 의 표본 = 그 단계 설정으로 잔 **뒤에** 잰 것. 구간의 시작점은 앞 표본이다.
        mine = [s for s in samples if s['phase'] == p and s['t_s'] > 0]
        if not mine:
            break                                       # 단계는 순서대로만 온다
        prev = [s for s in samples if s['t_s'] < mine[0]['t_s']]
        pts = ([(prev[-1]['t_s'], prev[-1]['mv'])] if prev else []) + [(s['t_s'], s['mv']) for s in mine]
        tail = pts[len(pts) // 3:]                      # 앞 1/3 은 완화가 가장 큰 구간
        st = lsq_slope(tail)
        if st is not None and len(tail) >= 4:
            mx, my = _mean([q[0] for q in tail]), _mean([q[1] for q in tail])
            resid += [(q[1] - my) - st / 3600.0 * (q[0] - mx) for q in tail]
        modem_pd = sum(1 for s in mine if (s['flags'] >> BIT_MODEM) & 1)
        want = phase_wants_pd(p)
        if fbit is not None:
            fac_pd = sum(1 for s in mine if (s['flags'] >> fbit) & 1)
        else:                 # LP 측정: 단계별 폴링 수로 "실제로 멈췄나/돌았나"를 판정한다
            r = lprates[p] if (lprates and p < len(lprates)) else None
            ok = r is not None and ((r <= LP_OFF_MAX) if want else (r >= LP_ON_MIN))
            fac_pd = (len(mine) if ok else 0) if want else (0 if ok else len(mine))
        if not dry:
            bad_flags += (len(mine) - fac_pd) if want else fac_pd
            if fac in (1, 2):  # TOP·LP 측정은 CPU 리텐션을 먼저 올린다 — 평소처럼 CPU 도 꺼져야 한다
                cpu_bad += sum(1 for s in mine if not (s['flags'] >> BIT_CPU) & 1)
        endp = mine[-END_N:]
        out['phases'].append({
            'p': p, 'want_pd': want, 'n': len(mine),
            't0': pts[0][0], 't1': pts[-1][0], 'mv0': pts[0][1], 'mv1': pts[-1][1],
            'slope': lsq_slope(pts), 'slope_tail': st,
            'E': _mean([s['mv'] for s in endp]), 'TE': _mean([s['t_s'] for s in endp]),
            'modem_pd': modem_pd,
            'top_pd': sum(1 for s in mine if (s['flags'] >> BIT_TOP) & 1),
            'not_timer': sum(1 for s in mine if s['cause'] != WAKE_TIMER),
            'usb': sum(1 for s in mine if s['usb']),
            'rej': sum(1 for s in mine if s['rej'])})
    ph = out['phases']
    if dry:
        out.update(verdict='DRY', why='모의 실행(절전 없음) — 흐름만 확인한 것이고 전압은 의미 없다')
        return out
    if bad_flags:
        what = ('LP 폴링이 의도대로 멈추거나 돌지 않았다(단계별 초당 %s)'
                % ', '.join('%.0f' % r for r in (lprates or [])) if fbit is None
                else '%s 도메인이 의도대로 바뀌지 않은 표본이 %d개' % (fname, bad_flags))
        out.update(verdict='INVALID', why=what + ' — 측정 무효')
        return out
    warns = []
    if any(x['not_timer'] or x['usb'] or x['rej'] for x in ph):
        warns.append('타이머가 아닌 기상·USB·절전 거부가 섞여 있다 — 해당 단계는 의심할 것')
    if cpu_bad:
        warns.append('CPU 도메인이 안 꺼진 표본 %d개 — 평소 절전과 조건이 다르다' % cpu_bad)
    lpp = sum(1 for s in samples if s['flags'] != FLAGS_NONE and (s['flags'] >> 14) & 1)
    if fac in (1, 2) and lpp:
        warns.append('LP 주변장치(LP_I2C)가 꺼진 표본 %d개 — LP 코어가 멈췄을 수 있어 평소와 조건이 다르다' % lpp)
    if fac == 2:
        dom = sum(1 for s in samples if s['flags'] != FLAGS_NONE and s['flags'] & ((1 << BIT_TOP) | (1 << BIT_MODEM)))
        if dom:
            warns.append('LP 측정인데 TOP/MODEM 이 꺼진 표본 %d개 — 평소(둘 다 켜짐)와 조건이 다르다' % dom)
    if warns:
        out['warn'] = ' / '.join(warns)
    if len(ph) < 3 or any(x['n'] < 2 for x in ph):
        out.update(verdict='SHORT', why='단계가 3개 미만이라 톱니를 볼 수 없다(켜짐-꺼짐-켜짐이 최소)')
        return out
    # ① 톱니 진폭
    terms = []
    for j in range(1, len(ph) - 1):
        a, b, c = ph[j - 1], ph[j], ph[j + 1]
        line = a['E'] + (c['E'] - a['E']) * (b['TE'] - a['TE']) / (c['TE'] - a['TE'])
        d = b['E'] - line
        terms.append(d if b['want_pd'] else -d)
    S = _mean(terms)
    sigma = (sum(r * r for r in resid) / len(resid)) ** 0.5 if len(resid) >= 6 else 0.5
    sS = sigma / (min(END_N, min(x['n'] for x in ph)) ** 0.5) * (1.5 ** 0.5)
    thr = max(S_MIN, 3.0 * sS)
    # ② 주기 평균 기울기 — 같은 종류 끝점끼리. 꺼짐 끝 쌍을 우선(단계 0 은 시작 전 이력이 다르다)
    cyc = [((ph[j + 2]['E'] - ph[j]['E']) / (ph[j + 2]['TE'] - ph[j]['TE']) * 3600.0, ph[j]['want_pd'])
           for j in range(len(ph) - 2)]
    pref = [v for v, is_pd in cyc if is_pd] or [v for v, _ in cyc]
    Ms = _mean(pref)
    on = [x['slope_tail'] for x in ph if not x['want_pd'] and x['slope_tail'] is not None]
    pd = [x['slope_tail'] for x in ph if x['want_pd'] and x['slope_tail'] is not None]
    out.update(S=S, sS=sS, thr=thr, sigma=sigma, Ms=Ms, cyc=[v for v, _ in cyc],
               s_on=_mean(on) if on else None, s_pd=_mean(pd) if pd else None,
               M_lo=max(0.0, -Ms / K_HI), M_hi=max(0.0, -Ms / K_LO))
    out['dI_lo'] = max(0.0, 2 * out['M_lo'] - 2 * IPD_HI)
    out['dI_hi'] = max(0.0, 2 * out['M_hi'] - 2 * IPD_LO)
    # ★2026-10-15 상한 — "톱니가 없다"가 "잡음이 커서 모른다"와 같은 말이 아니게.
    #  첫 본측정(75분×4)에서 톱니 -0.09mV 인데 3σ 가 판정선(0.8mV)을 넘어 UNCLEAR 로만
    #  나왔다. 그러나 단계별 기울기 차이의 신뢰구간으로 "MODEM 몫은 있어도 X mA 미만"을
    #  말할 수 있다. 셀 완화는 꺼짐 단계를 **유리하게**(덜 가파르게) 보이게 하므로 이 상한은
    #  보수적이다(실제 몫은 더 작다). 가장 작은 환산계수(K_LO)로 나눠 크게 잡는다.
    if out['s_on'] is not None and out['s_pd'] is not None:
        sxx = []
        for x in ph:
            n_t = len([1 for s in samples if s['phase'] == x['p'] and s['t_s'] > 0]) + 1
            n_t -= n_t // 3
            dt = (x['t1'] - x['t0']) / max(1, (n_t - 1)) / 3600.0
            sxx.append(dt * dt * sum((i - (n_t - 1) / 2.0) ** 2 for i in range(n_t)))
        se_one = sigma / (min(sxx) ** 0.5) if min(sxx) > 0 else float('inf')
        n_on = len([x for x in ph if not x['want_pd']])
        n_pd = len([x for x in ph if x['want_pd']])
        se_diff = se_one * ((1.0 / n_on + 1.0 / n_pd) ** 0.5)
        out['dslope'] = out['s_pd'] - out['s_on']
        out['dslope_se'] = se_diff
        out['dI_95'] = max(0.0, (out['dslope'] + 2.0 * se_diff) / K_LO)
    # ★2026-10-15 판정 코드를 요인 중립으로 바꿨다(MODEM_COSTS → COSTS, FLOOR_LARGE_NOT_MODEM →
    #  FLOOR_LARGE_NOT_FACTOR). 설명 문장에 요인 이름(fname)이 들어간다.
    if S >= thr:
        out.update(verdict='COSTS',
                   why='%s 을 끈 단계 끝이 켠 단계 끝보다 %.1fmV 위로 번갈아 벗어난다 — '
                       '%s 도메인이 켜져 있는 것만으로 전류를 쓴다 (대략 %.1f~%.1f mA)'
                       % (fname, S, fname, out['dI_lo'], out['dI_hi']))
    elif S <= -thr:
        out.update(verdict='ODD',
                   why='%s 을 끈 단계가 오히려 더 낮다(%.1fmV) — 예상과 반대. 원자료를 볼 것' % (fname, S))
    elif 3.0 * sS > S_MIN and not (out.get('dI_95') is not None and out['dI_95'] < DI_BOUND):
        out.update(verdict='UNCLEAR',
                   why='잡음이 커서(3σ=%.1fmV) 1mA 급 차이를 가를 수 없다 — 더 길게 재야 한다' % (3 * sS))
    elif abs(Ms) < THR_SMALL:
        out.update(verdict='FLOOR_SMALL',
                   why='%s 을 켜든 끄든 차이가 없고 전체가 평평하다 — 무선이 없는 얕은잠은 '
                       '문제가 없다. 3mA 는 무선 스택이 뜬 뒤에 생긴다' % fname)
    elif Ms <= -THR_LARGE:
        out.update(verdict='FLOOR_LARGE_NOT_FACTOR',
                   why='%s 을 켜든 끄든 차이가 없는데 전체가 가파르다 — %s 과 무관하다. '
                       '다른 곳을 봐야 한다 (단, 충전 직후 표류일 수도 있다)' % (fname, fname))
    else:
        out.update(verdict='FLOOR_MID',
                   why='%s 을 켜든 끄든 차이가 없고 바닥은 1mA 안팎이다' % fname)
    return out


# ══════════════════════════════════════════════════════════════════════════
#  ★★2026-10-16 깨어남 비용 + 현장 교정 (fac=3, `lsfloor wake`)
# ══════════════════════════════════════════════════════════════════════════
#  왜: MODEM·TOP·LP 세 밤이 전부 "차이 없음"이었고, 10/16 LP 밤에서 바닥이 같은 보드 직후 평소의
#  ~40% 로 나왔다. 그렇다면 나머지 ~60% 는 잠든 상태가 아니라 **깨어남**(초당 31.8회 × 전환 2.52ms)과
#  무선이다. 그런데 mV/h → mA 계수가 4.03V 와 4.06V 에서 3배쯤 달라 어느 숫자도 mA 로 확정할 수 없었다.
#  그래서 **크기를 아는 부하**를 같이 건다: CC1101 IDLE = 1.7mA(데이터시트 전형값, 크리스털만 돎).
#
#    단계  A  바닥 (무선 없음, ADC 상주 = 평소와 같은 TOP·MODEM 켜짐)
#          B  바닥 + 평소처럼 깨어남(초당 ~31.8회, 회당 0.7ms 일) — 평소 절전 플래그 0x3C12 와 같다
#          C  바닥 + CC1101 IDLE(1.7mA) — 교정용
#
#  톱니 진폭 S(단계 끝점이 이웃 A 끝점을 이은 선보다 얼마나 아래인가)는 셀 계수 K 와 완화(sag)에
#  **둘 다 비례**한다: S = (K·T/2 + sag)·ΔI. B 와 C 가 같은 길이·같은 모양이면 그 비는 ΔB/ΔC 그 자체다
#  — K 도 완화도 약분된다. 그래서 ΔB = 1.7mA × S_B/S_C. 여기에 계수 오차(전압대)가 없다.
#  ★남는 오차: CC1101 IDLE 의 개체차·전원 효율(±20% 가정), 그리고 충전 직후 표류의 **곡률**
#    (직선 보간이 못 지우는 부분 — B 와 C 의 위치가 다르면 서로 다르게 걸린다. 시뮬레이터로 잰다).
WAKE_PATS = {5: 'ABACA', 7: 'ABACABA'}   # 펌웨어 LSF_PAT_* 와 같아야 한다(nph 로 고른다)
CAL_MA = 1.7                              # CC1101 IDLE 전형값 (SWRS061 Table 4, 3.0V 25°C)
CAL_SYS = 0.20                            # 교정 부하 계통 오차(개체차·전원 효율) — 가정
WAKE_RATE_LO, WAKE_RATE_HI = 25.0, 40.0   # B 단계 실제 깨어남(회/초) 허용 범위
MARC_IDLE = 0x01                          # CC1101 MARCSTATE IDLE
FACTORS[3] = ('WAKE', None)
# ★★2026-10-21 fac=4 `lsfloor ant` — XIAO 내부 안테나 바이어스 저항 전류(doc/HARDWARE_POWER_AUDIT_V6_2026-10-21.md §3).
#  같은 ABACA: A = 핀을 놓음(전류 0), B = 옛 동작(EN LOW·SEL HIGH 를 절전 중에도 구동 → R21 10k·R24 5.1k 로 회로상 0.98mA),
#  C = CC1101 IDLE 1.7mA 교정. ΔB = 1.7mA × S_B/S_C — 둘 다 3.3V 레일 전류라 회로 계산값과 바로 비교된다.
#  펌웨어는 단계마다 an%u 에 핀 읽힘을 남긴다: bit0=EN, bit1=SEL, bit2=구동 단계. 놓은 단계는 내부 풀을 반대로 걸고
#  읽으므로 EN=1·SEL=0(an=0b001) 이면 모듈 안에 R21·R24 가 정말 있다는 증거다.
FACTORS[4] = ('ANT', None)
ANT_CIRCUIT_MA = 3.3 / 10000 * 1000 + 3.3 / 5100 * 1000   # 0.977mA — Seeed v1.0 회로도 R21 10k + R24 5.1k
# ★★2026-10-29 절전 남은 후보 셋을 같은 ABACA + CC1101 교정으로 (somfy_app.c _lsf_abaca 위 주석).
#  fac=5 OLED : A = 화면 OFF + 차지펌프 끔(새 동작) / B = 차지펌프 켬(옛 동작, AE 만). 단계마다 ol%u = 패널 ACK.
#  fac=6 ADC  : A = ADC 유닛 삭제(TOP·MODEM 얕은잠 중 꺼짐) / B = 상주(평소). 플래그로 검증 — A·C 는 TOP·MODEM 꺼짐 비트가 서야,
#               B 는 둘 다 켜져야 한다. 핀 고정·CPU 리텐션은 fac 1 과 같은 전제.
#  fac=7 LPC  : A = LP 폴링 멈춤 / B = 돌림. lpk%u(멈춤·돌림 성공)와 단계별 LP 폴링 속도(lp%u)로 검증.
FACTORS[5] = ('OLED', None)
FACTORS[6] = ('ADC', None)
FACTORS[7] = ('LPC', None)
# ★2026-10-29 fac=8 ROT : A = 로터리 두 핀 PCF 출력 LOW(닫힌 접점에도 전류 0) / B = 입력(평소). LP 는 측정 내내 멈춤(전 단계 같다).
#  단계마다 놓고 읽은 노브 원시값 rs%u(닫힌 접점 수가 전 단계 같아야 — 아니면 측정 중 노브가 움직였다), 쓰기 성공 ro%u, 인계 rtk.
FACTORS[8] = ('ROT', None)
ROT_MASK = 0x0003                                   # P0·P1(-Rot ba 여도 두 비트 묶음은 같다)
# ★2026-10-31 fac=9 PARK : 로터리 주차(main/lp_core/pcf_lp_config.h)의 순이득. LP 는 전 단계 돈다.
#  A = 주차 켬(새 동작) / B = 주차 끔(옛 동작) → B−A = 닫힌 접점 전류 − 주차 I2C 비용. 양수면 주차가 아낀다.
#  단계마다 pk%u(설정 성공), rs%u(LP 가 놓고 읽은 노브 — 전 단계 같고 닫힘 > 0 이어야), pp%u(누적 주차 폴 — A·C 는 LP 폴과 같이, B 는 0).
FACTORS[9] = ('PARK', None)
PARK_ON_MIN, PARK_OFF_MAX = 100.0, 1.0              # 단계별 주차 폴(회/초) — LP 폴(~191/s) 기준
ABACA_FACS = (3, 4, 5, 6, 7, 8, 9)
B_LABEL = {3: '+깨어남', 4: '+안테나', 5: '+차지펌프', 6: '+ADC상주', 7: '+LP폴링', 8: '+접점전류', 9: '+주차끔'}
B_WHAT = {3: '깨어남', 4: '안테나 바이어스', 5: 'OLED 차지펌프(화면 OFF 중)', 6: 'ADC 상주(TOP·MODEM 켜짐)', 7: 'LP 코어 5ms 폴링',
          8: '로터리 닫힌 접점 전류', 9: '로터리 주차 순이득(주차 끔 − 켬)'}
VERDICT_OF = {3: 'WAKE_COST', 4: 'ANT_COST', 5: 'OLED_COST', 6: 'ADC_COST', 7: 'LP_COST', 8: 'ROT_COST', 9: 'PARK_NET'}


def rot_closed(rs):
    """노브 원시값 → 닫힌 접점 수(0~2). 기록이 없거나 읽기 실패면 None."""
    if rs is None or rs == 0xFFFFFFFF or rs < 0:
        return None
    return bin(~rs & ROT_MASK).count('1')


def is_abaca(fac):
    return fac in ABACA_FACS


def phase_kind(fac, p, nph=PHASES):
    """단계의 종류 — fac=3·4 는 'A'/'B'/'C', 그 밖은 켬 'A' / 끔 'X'."""
    if is_abaca(fac):
        pat = WAKE_PATS.get(nph, '')
        return pat[p] if p < len(pat) else '?'
    return 'X' if phase_wants_pd(p) else 'A'


def _phase_ends(samples, nph):
    """단계별 끝점(마지막 END_N 표본 평균)과 꼬리 잔차(잡음 추정용)."""
    ph, resid = [], []
    for p in range(nph):
        mine = [s for s in samples if s['phase'] == p and s['t_s'] > 0]
        if not mine:
            break
        prev = [s for s in samples if s['t_s'] < mine[0]['t_s']]
        pts = ([(prev[-1]['t_s'], prev[-1]['mv'])] if prev else []) + [(s['t_s'], s['mv']) for s in mine]
        tail = pts[len(pts) // 3:]
        st = lsq_slope(tail)
        if st is not None and len(tail) >= 4:
            mx, my = _mean([q[0] for q in tail]), _mean([q[1] for q in tail])
            resid += [(q[1] - my) - st / 3600.0 * (q[0] - mx) for q in tail]
        endp = mine[-END_N:]
        ph.append({'p': p, 'n': len(mine), 't0': pts[0][0], 't1': pts[-1][0],
                   'mv0': pts[0][1], 'mv1': pts[-1][1], 'slope': lsq_slope(pts), 'slope_tail': st,
                   'E': _mean([s['mv'] for s in endp]), 'TE': _mean([s['t_s'] for s in endp]),
                   'not_timer': sum(1 for s in mine if s['cause'] != WAKE_TIMER),
                   'usb': sum(1 for s in mine if s['usb']), 'rej': sum(1 for s in mine if s['rej']),
                   'flags': [s['flags'] for s in mine]})
    sigma = (sum(r * r for r in resid) / len(resid)) ** 0.5 if len(resid) >= 6 else 0.5
    return ph, sigma


def analyse_wake(samples, meta, nph=5):
    """fac=3·4 판정. 반환 {'verdict', 'why', 'S_B', 'S_C', 'R', 'dB', 'dB_se', 'per_wake_uc', ...}
    ★2026-10-21 fac=4(ANT) 도 여기서 — B 의 뜻만 다르다(깨어남 → 안테나 바이어스). 유효성 검사가 다르다."""
    fac = meta.get('fac', 3)
    kinds = WAKE_PATS.get(nph, '')
    out = {'verdict': 'EMPTY', 'why': '표본이 없다', 'factor': FACTORS.get(fac, ('WAKE', None))[0],
           'kinds': kinds, 'phases': []}
    if not samples or not kinds:
        return out
    dry = all(s['flags'] == FLAGS_NONE for s in samples)
    ph, sigma = _phase_ends(samples, nph)
    out['phases'] = ph
    for x in ph:
        x['kind'] = kinds[x['p']]
    # ── 단계별 실제 깨어남 (NVS wk%u = 그 단계의 절전 횟수) ──
    rates = {}
    for x in ph:
        wk = meta.get('wk%d' % x['p'])
        dur = max(1, x['t1'] - x['t0'])
        rates[x['p']] = (wk / dur) if wk is not None else None
        x['wake_rate'] = rates[x['p']]
    out['rates'] = rates
    if dry:
        out.update(verdict='DRY', why='모의 실행(절전 없음) — 흐름·CC1101 전환·깨어남 횟수만 확인한 것이고 전압은 의미 없다')
        return out
    # ── 유효성: 의도대로 됐는가 ──
    bad = []
    ant_proof = []                 # ★fac=4: 놓은 단계에서 내부 저항이 확인됐나(단계별)
    lpr = lp_rates(meta, samples, nph) if fac in (7, 9) else None
    ppr = lp_rates(meta, samples, nph, prefix='pp') if fac == 9 else None
    for x in ph:
        k, r = x['kind'], x['wake_rate']
        if fac == 3 and k == 'B' and (r is None or not (WAKE_RATE_LO <= r <= WAKE_RATE_HI)):
            bad.append('단계%d(B) 깨어남 %s회/초 — 허용 %.0f~%.0f' % (x['p'], '?' if r is None else '%.1f' % r,
                                                              WAKE_RATE_LO, WAKE_RATE_HI))
        if (k in 'AC' or fac != 3) and r is not None and r > 1.0:      # ★10/29 fac 3 만 B 에 깨어남이 있다
            bad.append('단계%d(%s) 에 깨어남이 섞였다(%.1f회/초)' % (x['p'], k, r))
        if fac == 5:                   # ★2026-10-29 단계마다 패널이 차지펌프 명령을 받았나
            ol = meta.get('ol%d' % x['p'])
            if ol != 1:
                bad.append('단계%d(%s) OLED 명령 %s — 패널이 차지펌프 설정을 못 받았다'
                           % (x['p'], k, '기록 없음' if ol is None else '미수신(ol=0)'))
        if fac == 8:                   # ★2026-10-29 단계마다 PCF 쓰기·노브 읽기 성공
            if meta.get('ro%d' % x['p']) != 1:
                bad.append('단계%d(%s) PCF 쓰기/읽기 실패(ro=%s)' % (x['p'], k, meta.get('ro%d' % x['p'])))
        if fac == 9:                   # ★2026-10-31 주차 설정·LP 폴링·주차 폴이 단계 종류대로였나
            if meta.get('pk%d' % x['p']) != 1:
                bad.append('단계%d(%s) 주차 설정 실패(pk=%s)' % (x['p'], k, meta.get('pk%d' % x['p'])))
            if lpr is None or x['p'] >= len(lpr) or lpr[x['p']] < LP_ON_MIN:
                bad.append('단계%d(%s) LP 폴링 %s회/초 — 전 단계 돌아야 한다' % (
                    x['p'], k, '?' if lpr is None or x['p'] >= len(lpr) else '%.1f' % lpr[x['p']]))
            opened = rot_closed(meta.get('rs%d' % x['p'])) == 0   # 열린 노브엔 몰 핀이 없다 — 아래 PARK_OPEN 으로
            if ppr is None or x['p'] >= len(ppr):
                bad.append('단계%d(%s) 주차 폴 기록(pp%d)이 없다' % (x['p'], k, x['p']))
            elif (k == 'B' and ppr[x['p']] > PARK_OFF_MAX) or (k != 'B' and not opened and ppr[x['p']] < PARK_ON_MIN):
                bad.append('단계%d(%s) 주차 폴 %.1f회/초 — %s' % (x['p'], k, ppr[x['p']],
                           '껐는데 주차했다' if k == 'B' else '켰는데 주차를 안 했다(노브가 열렸거나 주차 실패)'))
        if fac == 7:                   # ★2026-10-29 멈춤·돌림이 의도대로였나(fac 2 와 같은 기준)
            lk = meta.get('lpk%d' % x['p'])
            if lk != 1:
                bad.append('단계%d(%s) LP %s 실패(lpk=%s)' % (x['p'], k, '돌림' if k == 'B' else '멈춤', lk))
            if lpr and x['p'] < len(lpr):
                rr = lpr[x['p']]
                if (k == 'B' and rr < LP_ON_MIN) or (k != 'B' and rr > LP_OFF_MAX):
                    bad.append('단계%d(%s) LP 폴링 %.1f회/초 — %s' % (x['p'], k, rr,
                               '돌아야 하는데 멈췄다' if k == 'B' else '멈춰야 하는데 돌았다'))
        if fac == 4:
            an = meta.get('an%d' % x['p'])
            if an is None:
                bad.append('단계%d 안테나 핀 기록(an%d)이 없다' % (x['p'], x['p']))
            elif k == 'B':
                if an & 7 != (4 | 2):          # 구동 단계: EN=0 · SEL=1 로 읽혀야 한다
                    bad.append('단계%d(B) 안테나 핀이 의도대로 구동되지 않았다(an=0x%X, EN=0·SEL=1 이어야)' % (x['p'], an))
            else:
                if an & 4:
                    bad.append('단계%d(%s) 안테나 핀이 구동 상태다 — 놓았어야 한다' % (x['p'], k))
                ant_proof.append((an & 3) == 1)
        if k == 'C':
            for key in ('cs%d' % x['p'], 'ce%d' % x['p']):
                v = meta.get(key)
                if v is None or (v & 0x1F) != MARC_IDLE:
                    bad.append('단계%d(C) CC1101 상태 %s=%s — IDLE(0x01) 이 아니다' %
                               (x['p'], key, '없음' if v is None else '0x%02X' % v))
        TM = (1 << BIT_TOP) | (1 << BIT_MODEM)
        if fac == 6 and k != 'B':      # ★2026-10-29 ADC 측정 바닥·교정은 TOP·MODEM 이 **꺼져야** 한다(CPU 도)
            off = sum(1 for f in x['flags'] if f != FLAGS_NONE and (f & TM) != TM)
            nocpu = sum(1 for f in x['flags'] if f != FLAGS_NONE and not (f >> BIT_CPU) & 1)
            if off:
                bad.append('단계%d(%s) 에 TOP·MODEM 이 다 꺼지지 않은 절전 %d회 — 삭제가 안 먹었거나 리텐션 없는 드라이버가 막았다'
                           % (x['p'], k, off))
            if nocpu:
                bad.append('단계%d(%s) 에 CPU 가 안 꺼진 절전 %d회 — TOP 차단 전제(CPU 리텐션) 실패' % (x['p'], k, nocpu))
        else:
            dom = sum(1 for f in x['flags'] if f != FLAGS_NONE and f & TM)
            if dom:
                bad.append('단계%d 에 TOP/MODEM 이 꺼진 절전 %d회 — 평소(둘 다 켜짐)와 다르다' % (x['p'], dom))
    out['ant_proof'] = ant_proof
    out['park_rates'] = ppr
    if fac in (8, 9):                  # ★2026-10-29 노브가 측정 내내 같은 위치였나 (★10/31 PARK 도)
        if fac == 8 and meta.get('rtk') != 1:
            bad.append('PCF 선을 HP 로 못 넘겼다(rtk=%s)' % meta.get('rtk'))
        cl = [rot_closed(meta.get('rs%d' % x['p'])) for x in ph]
        out['rot_closed'] = cl
        if any(c is None for c in cl):
            bad.append('노브 위치를 못 읽은 단계가 있다(rs)')
        elif len(set(cl)) > 1:
            bad.append('측정 중 노브가 움직였다(단계별 닫힌 접점 %s)' % cl)
    if meta.get('cc_awk') == 1:
        bad.append('측정 끝에 CC1101 이 깨어 있었다 — C 단계 뒤 SPWD 로 못 돌아갔다')
    if bad:
        out.update(verdict='INVALID', why=' / '.join(bad) + ' — 측정 무효')
        return out
    if len(ph) < nph:
        out.update(verdict='SHORT', why='단계가 다 오지 않았다(%d/%d) — USB 중단이나 리셋' % (len(ph), nph))
        return out
    # ── 톱니: B·C 단계 끝점이 이웃 A 끝점을 이은 선에서 얼마나 아래인가(양수 = 더 먹는다) ──
    Ss = {'B': [], 'C': []}
    for j in range(1, len(ph) - 1):
        k = ph[j]['kind']
        if k not in 'BC' or ph[j - 1]['kind'] != 'A' or ph[j + 1]['kind'] != 'A':
            continue
        a, b, c = ph[j - 1], ph[j], ph[j + 1]
        line = a['E'] + (c['E'] - a['E']) * (b['TE'] - a['TE']) / (c['TE'] - a['TE'])
        ph[j]['S'] = line - b['E']
        Ss[k].append(line - b['E'])
    sS = sigma / (min(END_N, min(x['n'] for x in ph)) ** 0.5) * (1.5 ** 0.5)
    S_B, S_C = _mean(Ss['B']), _mean(Ss['C'])
    sB, sC = sS / len(Ss['B']) ** 0.5, sS / len(Ss['C']) ** 0.5
    out.update(S_B=S_B, S_C=S_C, sB=sB, sC=sC, sigma=sigma)
    rate_b = _mean([x['wake_rate'] for x in ph if x['kind'] == 'B'])
    out['rate_B'] = rate_b
    # 바닥 기울기(참고): A 끝점끼리의 평균 기울기 — B·C 의 추가 방전이 섞여 있다(아래 설명)
    a_ends = [(x['TE'], x['E']) for x in ph if x['kind'] == 'A']
    out['slope_AA'] = lsq_slope(a_ends)
    if S_C < 3.0 * sC:
        out.update(verdict='CAL_WEAK',
                   why='교정 부하(CC1101 IDLE 1.7mA)의 톱니가 %.2fmV 로 잡음(3σ=%.2fmV)에 묻혔다 — '
                       'mA 로 환산할 수 없다. B 톱니 %.2fmV 는 원값으로만 본다' % (S_C, 3 * sC, S_B))
        return out
    R = S_B / S_C
    R_se = (sB ** 2 + (R * sC) ** 2) ** 0.5 / S_C
    dB = CAL_MA * R
    dB_se = CAL_MA * R_se
    lo = (R - 2 * R_se) * CAL_MA
    out.update(R=R, R_se=R_se, dB=dB, dB_se=dB_se,
               # ★2026-10-31 PARK(순이득)은 음수일 수 있다(주차 비용 > 접점 전류) — 0 에서 자르지 않는다
               dB_lo=(lo * ((1 - CAL_SYS) if lo > 0 else (1 + CAL_SYS))) if fac == 9 else max(0.0, lo * (1 - CAL_SYS)),
               dB_hi=(R + 2 * R_se) * CAL_MA * (1 + CAL_SYS),
               per_wake_uc=(dB / rate_b * 1000.0) if (rate_b and fac == 3) else None)
    if fac == 4:
        proof = ant_proof and all(ant_proof)
        out.update(verdict='ANT_COST',
                   why='옛 동작(절전 중에도 안테나 핀 구동)의 추가 전류 %.2f ± %.2f mA (95%%·교정 계통오차 포함 %.2f~%.2f mA) — '
                       '회로 계산 %.2f mA(R21 10k + R24 5.1k)%s'
                       % (dB, dB_se, out['dB_lo'], out['dB_hi'], ANT_CIRCUIT_MA,
                          '. 놓은 단계 핀 읽힘으로 내부 저항 확인됨' if proof
                          else '. ★놓은 단계에서 내부 저항이 확인되지 않았다(반대 풀을 못 이김) — 모듈이 회로도와 다를 수 있다'))
        return out
    if fac == 8:                       # ★2026-10-29 접점 수로 나눠 핀당 전류
        n = (out.get('rot_closed') or [0])[0]
        if not n:
            out.update(verdict='ROT_OPEN',
                       why='노브가 열린 위치(닫힌 접점 0개)라 잴 몫이 없다 — 결과 %.2f ± %.2f mA 는 0 근처여야 정상(아니면 다른 핀이 새는 것)'
                           % (dB, dB_se))
            return out
        out.update(verdict='ROT_COST', per_pin=dB / n,
                   why='닫힌 접점 %d개의 전류 %.2f ± %.2f mA (95%%·교정 계통오차 포함 %.2f~%.2f mA) = 접점당 %.0f µA '
                       '(TI 표 30~300µA) — 노브가 이 위치에 멈춰 있는 동안 계속 흐른다'
                       % (n, dB, dB_se, out['dB_lo'], out['dB_hi'], dB / n * 1000.0))
        return out
    if fac == 9:                       # ★2026-10-31 로터리 주차 순이득
        n = (out.get('rot_closed') or [0])[0]
        if not n:
            out.update(verdict='PARK_OPEN',
                       why='노브가 열린 위치(닫힌 접점 0개)라 주차할 핀이 없었다 — 결과 %.2f ± %.2f mA 는 0 근처여야 정상'
                           % (dB, dB_se))
            return out
        out.update(verdict='PARK_NET',
                   why='닫힌 접점 %d개에서 주차가 아낀 순전류 %.2f ± %.2f mA (95%%·교정 계통오차 포함 %.2f~%.2f mA) — '
                       '접점 전류에서 주차 I2C 비용을 뺀 값. 양수면 주차가 이득, 0 이하면 주차를 끈다(LP_ROT_PARK=0)'
                       % (n, dB, dB_se, out['dB_lo'], out['dB_hi']))
        return out
    if fac in (5, 6, 7):               # ★2026-10-29
        out.update(verdict=VERDICT_OF[fac],
                   why='%s 몫 %.2f ± %.2f mA (95%%·교정 계통오차 포함 %.2f~%.2f mA)%s'
                       % (B_WHAT[fac], dB, dB_se, out['dB_lo'], out['dB_hi'],
                          ' — 0 에 가까우면 끄는 이득이 없다(데이터시트: 차지펌프는 화면 ON 동안만 돈다)' if fac == 5 else
                          ' — 이만큼이 "ADC 를 지우면(=MODEM·TOP 을 놓으면)" 얻는 상한. Thread 를 살리는 방법이 따로 필요하다' if fac == 6 else
                          ' — 폴링 주기를 바꾸면 이 비율로 준다(주기 변경은 사용자 승인 사항)'))
        return out
    out.update(verdict='WAKE_COST',
               why='평소처럼 깨어나는 것(초당 %.1f회)만으로 %.2f ± %.2f mA (95%%·교정 계통오차 포함 %.2f~%.2f mA) — '
                   '깨어남 1회당 약 %.0f µC'
                   % (rate_b, dB, dB_se, out['dB_lo'], out['dB_hi'], out['per_wake_uc'] or 0))
    return out


def load(path):
    from nvs_parse import parse
    _, items = parse(open(path, 'rb').read())
    meta, raw = {}, {}
    for it in items:
        if it['ns_name'] != 'lsfl':
            continue
        k, v = it['key'], it['value']
        if len(k) == 3 and k[0] == 's' and k[1:].isdigit() and isinstance(v, int):
            raw[int(k[1:])] = v
        elif isinstance(v, int):
            meta[k] = v
    n = meta.get('n', len(raw))
    samples = [unpack(raw[i]) for i in sorted(raw) if i < n]
    return meta, samples


KIND_NAMES = {'A': '바닥', 'B': '+깨어남', 'C': '+CC1101', '?': '?'}
KIND_NAMES_ANT = {'A': '핀놓음', 'B': '+안테나', 'C': '+CC1101', '?': '?'}   # ★2026-10-21 fac=4


def report_wake(meta, samples, nph):
    """fac=3·4 출력 — 표본 표, 단계 표, 교정 결과."""
    kinds = WAKE_PATS.get(nph, '?' * nph)
    fac = meta.get('fac', 3)
    KIND_NAMES = KIND_NAMES_ANT if fac == 4 else globals()['KIND_NAMES']   # noqa: N806 — 아래 표기만 바꾼다
    if fac in (5, 6, 7, 8, 9):         # ★2026-10-29 (★10/31 9)
        KIND_NAMES = {'A': {5: '펌프끔', 6: 'ADC삭제', 7: 'LP멈춤', 8: '핀LOW', 9: '주차켬'}[fac], 'B': B_LABEL[fac],  # noqa: N806
                      'C': '+CC1101', '?': '?'}
    print()
    print('   #    +분   단계         mV       기상  플래그   비고')
    for i, s in enumerate(samples):
        note = []
        k = kinds[s['phase']] if s['phase'] < len(kinds) else '?'
        fl = '  -   ' if s['flags'] == FLAGS_NONE else '0x%04X' % s['flags']
        if s['flags'] != FLAGS_NONE:
            if s['flags'] & ((1 << BIT_TOP) | (1 << BIT_MODEM)):
                note.append('★TOP/MODEM꺼짐')
            if not (s['flags'] >> BIT_CPU) & 1:
                note.append('CPU안꺼짐')
            if s['cause'] != WAKE_TIMER:
                note.append('기상원인 %d' % s['cause'])
        if s['usb']:
            note.append('USB')
        if s['rej']:
            note.append('절전거부')
        lab = '-' if s['t_s'] == 0 else '%s %s' % (k, KIND_NAMES.get(k, ''))
        print('  %2d  %6.1f   %d %-9s %7.1f   %2d   %s  %s'
              % (i, s['t_s'] / 60.0, s['phase'], lab, s['mv'], s['cause'], fl, ' '.join(note)))
    a = analyse_wake(samples, meta, nph)
    print()
    print('  단계  종류       구간(분)        끝점(mV)  기울기 뒤2/3   깨어남(회/초)   톱니 S(mV)')
    for x in a['phases']:
        r = x.get('wake_rate')
        print('   %d    %s %-7s %6.1f~%6.1f   %8.2f     %+6.2f        %s          %s'
              % (x['p'], x['kind'], KIND_NAMES.get(x['kind'], ''), x['t0'] / 60.0, x['t1'] / 60.0,
                 x['E'], x['slope_tail'] or 0.0, '   -  ' if r is None else '%6.2f' % r,
                 '%+.2f' % x['S'] if 'S' in x else '  -'))
    for p in range(nph):
        if kinds[p] == 'C':
            cs, ce = meta.get('cs%d' % p), meta.get('ce%d' % p)
            print('  CC1101 단계%d: 시작 MARCSTATE=%s  끝 MARCSTATE=%s  (IDLE=0x01)'
                  % (p, '-' if cs is None else '0x%02X' % cs, '-' if ce is None else '0x%02X' % ce))
    if fac == 4:      # ★2026-10-21 단계별 안테나 핀 읽힘
        for p in range(nph):
            an = meta.get('an%d' % p)
            if an is None:
                continue
            if an & 4:
                print('  안테나 단계%d(%s): 구동 — 읽힘 EN=%d SEL=%d (EN=0·SEL=1 이면 옛 동작대로)'
                      % (p, kinds[p], an & 1, (an >> 1) & 1))
            else:
                print('  안테나 단계%d(%s): 놓음 — 반대 풀 걸고 읽힘 EN=%d SEL=%d  %s'
                      % (p, kinds[p], an & 1, (an >> 1) & 1,
                         '← 모듈 내부 R21 풀업·R24 풀다운 확인' if (an & 3) == 1 else '★내부 저항이 확인되지 않음'))
    if fac == 5:      # ★2026-10-29 단계별 OLED 차지펌프 명령 ACK
        print('  OLED 명령 ACK(단계별): ' + '  '.join('%d(%s):%s' % (p, kinds[p], {1: 'OK', 0: '★미수신'}.get(meta.get('ol%d' % p), '-'))
                                                 for p in range(nph)))
    if fac == 7:      # ★2026-10-29 단계별 LP 멈춤·돌림 + 폴링 속도
        rr = lp_rates(meta, samples, nph) or []
        print('  LP 멈춤/돌림(단계별): ' + '  '.join('%d(%s):%s %s' % (p, kinds[p], {1: 'OK', 0: '★실패'}.get(meta.get('lpk%d' % p), '-'),
                                                                   ('%.0f회/초' % rr[p]) if p < len(rr) else '')
                                              for p in range(nph)))
    if fac == 8:      # ★2026-10-29 단계별 노브 위치·PCF 쓰기
        print('  노브(단계별, 놓고 읽음): ' + '  '.join(
            '%d(%s):%s' % (p, kinds[p], '-' if meta.get('rs%d' % p) is None else
                           '0x%04X 닫힘%s%s' % (meta['rs%d' % p] & 0xFFFF, rot_closed(meta['rs%d' % p]),
                                                '' if meta.get('ro%d' % p) == 1 else ' ★쓰기실패'))
            for p in range(nph)))
    if fac == 9:      # ★2026-10-31 단계별 노브·주차 설정·주차 폴 속도·LP 폴 속도
        pr = lp_rates(meta, samples, nph, prefix='pp') or []
        lr = lp_rates(meta, samples, nph) or []
        print('  주차(단계별): ' + '  '.join(
            '%d(%s):%s 노브%s 주차폴%s LP%s' % (
                p, kinds[p], {1: '설정OK', 0: '★설정실패'}.get(meta.get('pk%d' % p), '-'),
                '-' if meta.get('rs%d' % p) is None else '0x%04X(닫힘%s)' % (meta['rs%d' % p] & 0xFFFF, rot_closed(meta['rs%d' % p])),
                ('%.0f/s' % pr[p]) if p < len(pr) else '-', ('%.0f/s' % lr[p]) if p < len(lr) else '-')
            for p in range(nph)))
    print()
    blab = B_LABEL.get(fac, '+깨어남')
    if 'S_B' in a:
        print('  ① 톱니: B(%s) %+.2f ± %.2f mV   C(+CC1101 1.7mA) %+.2f ± %.2f mV   (표본 잡음 σ=%.2f mV)'
              % (blab, a['S_B'], a['sB'], a['S_C'], a['sC'], a['sigma']))
        if a.get('slope_AA') is not None:
            print('     (참고) 바닥 A 끝점끼리 기울기 %+.2f mV/h — B·C 의 추가 방전이 섞여 있다' % a['slope_AA'])
    if 'R' in a:
        print('  ② 비 S_B/S_C = %.3f ± %.3f  → %s 몫 = 1.7mA × 비 = **%.2f ± %.2f mA**%s'
              % (a['R'], a['R_se'], B_WHAT.get(fac, '깨어남'), a['dB'], a['dB_se'],
                 ('   (회로 계산 %.2f mA)' % ANT_CIRCUIT_MA) if fac == 4 else ''))
        print('     교정 부하 계통오차(CC1101 개체차·전원 효율 ±%d%%)까지 넣은 95%% 범위: %.2f ~ %.2f mA'
              % (int(CAL_SYS * 100), a['dB_lo'], a['dB_hi']))
        if a.get('per_wake_uc'):
            pw = a['per_wake_uc']
            print('  ③ 깨어남 1회당 %.0f µC (B 단계 실측 %.1f회/초) → 초당 10회 줄이면 약 %.2f mA, 31.8→5회/초면 약 %.2f mA 절약'
                  % (pw, a['rate_B'], pw * 10 / 1000.0, pw * (31.8 - 5) / 1000.0))
    print('  판정[%s]: %s' % (a['verdict'], a['why']))
    return 0


def report(meta, samples):
    run = meta.get('run', RUN_NONE)
    nph = meta.get('nph', PHASES)
    fac = meta.get('fac', 0)
    fname, fbit = FACTORS.get(fac, FACTORS[0])
    print('=== 얕은잠 바닥 전류 대조군 (lsfloor) — %s ===' %
          ('깨어남 비용 + CC1101 IDLE 교정 (%s)' % WAKE_PATS.get(nph, '?') if fac == 3 else
           'XIAO 안테나 바이어스 저항 + CC1101 IDLE 교정 (%s)' % WAKE_PATS.get(nph, '?') if fac == 4 else
           '%s + CC1101 IDLE 교정 (%s)' % (B_WHAT[fac], WAKE_PATS.get(nph, '?')) if fac in (5, 6, 7, 8, 9) else
           '켰다 끈 것: ' + fname))
    # ★2026-10-18 oled_off 는 이제 실제 결과다: 1 = 패널이 OFF 명령을 받음, 0 = 못 받음(패널 미검출 포함)
    _oo = meta.get('oled_off')
    print('  상태: %s   예약=%s  모의=%s  화면끔확인=%s'
          % (RUN_NAMES.get(run, run), meta.get('armed', 0), meta.get('dry', 0),
             '-' if _oo is None else ('1(받음)' if _oo == 1 else '★0(명령 미확인 — 패널 전류가 섞였을 수 있다)')))
    print('  설정: 단계 %s초 × %d  표본 %s초마다  정착 %s초   받은 표본 %d개'
          % (meta.get('phase_s', '-'), nph, meta.get('samp_s', '-'),
             meta.get('settle_s', '-'), len(samples)))
    if 'rst' in meta:
        print('  ★도중 리셋 사유 코드: %s' % meta['rst'])
    if meta.get('ng_n'):
        # ★2026-10-31 배터리로 부팅했는데 정착 경로의 재시작이 아니어서 측정 대신 정착부터 다시 한 횟수(somfy_app.c 훅 주석)
        print('  ★정착 없이 배터리로 부팅해 정착부터 다시 함: %d회 (마지막 리셋 사유 %s — 1=전원투입 3=SW 4=패닉 9=브라운아웃)'
              % (meta['ng_n'], meta.get('ng_rr', '?')))
    if 'anr' in meta:
        # ★2026-10-31 핀 고정 측정(fac 1·6)은 고정 전에 안테나 핀을 놓는다 — 반대 풀 걸고 읽힘 EN=1·SEL=0 이면 놓였다
        an = meta['anr']
        print('  안테나 핀: 고정 전에 놓음 — 읽힘 EN=%d SEL=%d %s' % (an & 1, (an >> 1) & 1,
              '(평소 절전과 같다)' if (an & 3) == 1 else '★놓였는지 확인 안 됨(옛 동작 ~0.98mA 가 전 단계에 섞였을 수 있다)'))
    if 'cc_awk' in meta:
        print('  CC1101: 측정 끝에 %s' % ('★깨어 있었다 — 1.7mA 가 섞였을 수 있다' if meta['cc_awk']
                                       else '자고 있었다(SPWD 유지)'))
    if 'cc_ver' in meta:
        # ★2026-10-15 SNOP 탐침은 CSn 이 붙잡혀 있어도 0xFF 로 읽혀 "잤다"가 된다 — VERSION 값으로 SPI 를 확인
        ok = meta['cc_ver'] in (0x14, 0x04, 0x17)
        print('  CC1101 VERSION=0x%02X — %s' % (meta['cc_ver'], '측정 뒤 SPI 정상(핀 고정 해제됨)' if ok
              else '★SPI 가 칩에 안 닿는다 — 핀 고정이 안 풀렸으면 RF 송신이 죽는다'))
    if 'lp_d' in meta and samples:
        dur = max(1, samples[-1]['t_s'])
        # ★2026-10-16 LP 측정(fac=2)은 일부러 반을 멈추므로 총합으로 판단하지 않는다 — 아래 단계별을 본다
        print('  LP 코어: 측정 중 폴링 %d회 (초당 %.1f) — %s'
              % (meta['lp_d'], meta['lp_d'] / dur,
                 '켬/끔 측정이라 총합은 판단에 안 쓴다(아래 단계별)' if fac in (2, 7)
                 else '로터리 측정은 LP 를 일부러 멈춘다(전 단계 같다)' if fac == 8
                 else ('평소처럼 돌고 있었다' if meta['lp_d'] / dur > 5 else '★멈춰 있었다(평소 절전과 조건이 다르다)')))
    # ★2026-10-15 단계별 LP 폴링 — 첫 TOP 리허설에서 LP 가 첫 꺼짐 단계에서 멈춘 것을 총합만으로는
    #  늦게 알았다(초당 47.6). 단계마다 속도를 보면 바로 보인다. (2026-10-16 lp_rates() 로 뺐다)
    rates = lp_rates(meta, samples, nph)
    if rates and fac not in (7, 8):   # ★2026-10-29 LPC 는 아래 report_wake 가 단계 종류로, ROT 는 일부러 멈춘다
        if fac == 2:      # LP 측정: 끈 단계는 멈춰야, 켠 단계는 돌아야 의도대로
            bad = lambda i, r: (r > LP_OFF_MAX) if phase_wants_pd(i) else (r < LP_ON_MIN)
        else:             # 그 밖의 측정: LP 는 평소처럼 계속 돌아야 한다
            bad = lambda i, r: r < 20
        print('  LP 폴링 단계별(초당): ' + '  '.join('%d:%.0f' % (i, r) for i, r in enumerate(rates)) +
              ('   ★의도와 다른 단계가 있다' if any(bad(i, r) for i, r in enumerate(rates)) else '   (전 단계 의도대로)'))
    lpk = [meta.get('lpk%d' % p) for p in range(nph)]
    if any(v is not None for v in lpk):
        print('  LP 멈춤/돌림 성공 여부(단계별): ' + '  '.join('%d:%s' % (i, '-' if v is None else ('OK' if v else '★실패'))
                                              for i, v in enumerate(lpk)))
    if 'rt_i' in meta or 'rt_c' in meta:
        # 측정 시작 때의 절전 리텐션 모듈 — TOP 이 안 꺼졌다면 여기서 막은 모듈 이름이 보인다
        print('  리텐션 모듈(측정 시작): inited=0x%08X created=0x%08X'
              % (meta.get('rt_i', 0), meta.get('rt_c', 0)))
        try:
            from pd_flags import rt_report
            rt_report(meta.get('rt_i', 0), meta.get('rt_c', 0))
        except Exception as ex:                      # 해석기 하나가 없다고 보고 전체를 멈추지 않는다
            print('   (리텐션 해석 생략: %s)' % ex)
    if 'cpu_ok' in meta:
        print('  CPU 리텐션 초기화: %s' % ('성공' if meta['cpu_ok'] else '★실패 — CPU 가 안 꺼졌을 것'))
    if not samples:
        print('  (표본 없음)')
        return 1
    if is_abaca(fac):                                # ★2026-10-16 깨어남 비용 + 교정 — 표·판정이 다르다(★10/21 ANT 도)
        return report_wake(meta, samples, nph)
    print()
    print('   #    +분   단계  %-5s   mV       기상  플래그   비고' % fname)
    for i, s in enumerate(samples):
        note = []
        if s['flags'] == FLAGS_NONE:
            fl, md = '  -   ', '  -  '
        else:
            fl = '0x%04X' % s['flags']
            if fbit is None:       # LP 측정 — 표본마다 표시할 비트가 없다, 단계의 의도를 보인다
                md = ' 꺼짐 ' if s['want_pd'] else ' 켜짐 '
            else:
                md = ' 꺼짐 ' if (s['flags'] >> fbit) & 1 else ' 켜짐 '
                if bool((s['flags'] >> fbit) & 1) != bool(s['want_pd']):
                    note.append('★의도와 다름')
            if fac in (1, 2) and not (s['flags'] >> BIT_CPU) & 1:
                note.append('CPU안꺼짐')
        if s['usb']:
            note.append('USB')
        if s['rej']:
            note.append('절전거부')
        if s['flags'] != FLAGS_NONE and s['cause'] != WAKE_TIMER:
            note.append('기상원인 %d' % s['cause'])
        print('  %2d  %6.1f   %d   %s  %7.1f   %2d   %s  %s'
              % (i, s['t_s'] / 60.0, s['phase'], md, s['mv'], s['cause'], fl, ' '.join(note)))
    a = analyse(samples, nph, fac, lp_rates(meta, samples, nph))
    print()
    print('  단계  %-5s   구간(분)        전압(mV)          끝점(mV)   기울기 mV/h (전체 / 뒤 2/3 — 참고용)' % fname)
    for x in a['phases']:
        print('   %d    %s  %6.1f~%6.1f   %7.1f → %7.1f   %8.2f   %+7.2f / %+7.2f'
              % (x['p'], '꺼짐' if x['want_pd'] else '켜짐', x['t0'] / 60.0, x['t1'] / 60.0,
                 x['mv0'], x['mv1'], x['E'], x['slope'] or 0.0, x['slope_tail'] or 0.0))
    print()
    if 'S' in a:
        print('  ① 톱니 진폭 %+.2f mV  (판정선 %.2f mV, 표본 잡음 σ=%.2f mV)'
              % (a['S'], a['thr'], a['sigma']))
        print('     = %s 을 끈 단계 끝이 이웃 켠 단계 끝을 이은 선보다 얼마나 위에 있는가' % fname)
        print('  ② 주기 평균 기울기 %+.2f mV/h  → (켜짐+꺼짐) 평균 소비 약 %.1f~%.1f mA'
              % (a['Ms'], a['M_lo'], a['M_hi']))
        print('     참고: 깊은잠 -0.33 mV/h(~0.3mA), 평소 -6.47 mV/h (2026-09-29 COM7, 4.04~4.12V)')
        if a['verdict'] == 'COSTS':
            print('  → %s 몫 대략 %.1f~%.1f mA  ※%s 을 끈 바닥을 %.1f~%.1f mA 로 **가정**한 값.'
                  % (fname, a['dI_lo'], a['dI_hi'], fname, IPD_LO, IPD_HI))
            print('     환산계수(%.1f~%.1f mV/h per mA)도 셀·전압대에 따라 달라 ±30%% 는 봐야 한다.' % (K_LO, K_HI))
        if a.get('s_on') is not None and a.get('s_pd') is not None:
            print('  (참고) 단계별 기울기: 켜짐 %+.2f / 꺼짐 %+.2f mV/h — 셀 완화가 섞여 차이가 부풀어 보인다'
                  % (a['s_on'], a['s_pd']))
        if a.get('dI_95') is not None:
            print('  ③ 상한: 기울기 차이 %+.2f ± %.2f mV/h → 끄는 쪽이 아끼는 전류는 있어도 **%.1f mA 미만**(95%%, 보수적)'
                  % (a['dslope'], a['dslope_se'], a['dI_95']))
    if a.get('warn'):
        print('  [주의] ' + a['warn'])
    print('  판정[%s]: %s' % (a['verdict'], a['why']))
    return 0


if __name__ == '__main__':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(report(*load(sys.argv[1])))
