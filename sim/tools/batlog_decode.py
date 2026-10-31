# -*- coding: utf-8 -*-
"""NVS 덤프에서 배터리 방전 로그를 읽어 콘솔 `bl` 과 같은 표로 출력한다.

기기를 켜지도, 리셋하지도 않는다. H2 처럼 콘솔을 못 쓰는 보드에서 유일한 방법이고,
C6 에서도 **측정 데이터를 지킬 때** 이쪽이 안전하다.

    esptool --chip esp32h2 -p COM3 --before default_reset --after no_reset \
            read_flash 0x10000 0xC000 nvs.bin
    python sim/tools/batlog_decode.py nvs.bin

main/somfy_app.c 의 bat_sample_t (packed, 8B) 와 짝을 이룬다:
    uint32 t_s / uint16 mv / uint8 pct / uint8 flags / uint8 sp / uint8 ls / int8 rssi
    flags: bit0=무선ON bit1=화면ON bit2~3=PM상태 bit4~7=이벤트코드
"""
import struct
import sys

sys.path.insert(0, __file__.rsplit('\\', 1)[0].rsplit('/', 1)[0])
from nvs_parse import parse   # noqa: E402

EVN = ["주기", "LEFT", "RIGHT", "SEL", "UP", "DOWN", "ROT", "PROG", "기타",
       "세션시작", "권외-", "미등록X", "?", "?", "?"]
SAMPLE = struct.Struct('<IHBBBBb')   # ★2026-08-17 t_s uint32(18.2h 포화 해소) → 11B


def _batlog_ns(path):
    """NVS 덤프에서 batlog 네임스페이스만 dict 로 꺼낸다(--vs 비교 기준용)."""
    _, items = parse(open(path, "rb").read())
    ns = {}
    for it in items:
        if it["ns_name"] != "batlog":
            continue
        if isinstance(it["value"], (bytes, bytearray)) or it["key"] not in ns:
            ns[it["key"]] = it["value"]
    return ns


def decode(path, div_top=100, div_bot=100, capacity_mah=700, base_ns=None):
    _, items = parse(open(path, 'rb').read())
    ns = {}
    for it in items:
        if it['ns_name'] != 'batlog':
            continue
        # 같은 키로 BLOB_IDX(dict) 와 합쳐진 BLOB_DATA(bytes) 가 둘 다 나온다.
        # 실제 내용인 bytes 를 우선한다.
        if isinstance(it['value'], (bytes, bytearray)) or it['key'] not in ns:
            ns[it['key']] = it['value']
    if not ns:
        print('batlog 네임스페이스가 없다'); return 1
    buf = ns.get('buf')
    # ★2026-10-20 (GPT 재감사 N08) 새 판은 링을 20표본 조각 "b0".."bN" 으로 나눠 바뀐 조각만 쓴다.
    #  조각이 있으면 그걸 이어 붙인다(옛 "buf" 가 함께 남아 있으면 이전 도중 — 펌웨어가 다음 저장 때 지운다).
    if isinstance(ns.get('b0'), (bytes, bytearray)):
        have = [i for i in range(32) if isinstance(ns.get('b%d' % i), (bytes, bytearray))]
        full = len(ns['b0'])                       # 첫 조각 = 꽉 찬 조각 크기(20표본)
        base = ns.get('buf') if isinstance(ns.get('buf'), (bytes, bytearray)) else None
        nch = max(max(have) + 1, (len(base) + full - 1) // full if base else 0)
        parts = []
        for i in range(nch):                       # 빠진 조각은 펌웨어와 같이 옛 buf(이전 도중) 또는 0
            if i in have:
                parts.append(ns['b%d' % i])
            elif base:
                parts.append(base[i * full:(i + 1) * full])
            else:
                parts.append(b'\x00' * full)
        buf = b''.join(parts)
        gaps = [i for i in range(nch) if i not in have]
        print('(조각 형식: %d조각%s%s)' % (len(parts),
              (', ★빠진 조각 %s 는 %s 으로 채움(다음 저장 때 펌웨어가 다시 쓴다)' % (gaps, '옛 buf' if base else '0'))
              if gaps else '',
              ', 옛 buf 도 남아 있음 — 이전 중' if base else ''))
    n = ns.get('n', 0)
    head = ns.get('head', 0)
    if isinstance(ns.get('nh'), int):              # ★2026-10-20 새 판: n·head 한 항목(원자적)
        n, head = ns['nh'] >> 16, ns['nh'] & 0xFFFF
    sess = ns.get('sess', 0)
    if not isinstance(buf, (bytes, bytearray)):
        print('buf blob 을 못 찾았다 (키: %s)' % ', '.join(ns)); return 1
    maxn = len(buf) // SAMPLE.size
    print('=== 방전 기록: 세션 #%s, %s건 / 링 %d칸 (버퍼 %dB) ===' %
          (sess, n, maxn, len(buf)))
    print('세션상태: dis_on=%s  경과=%ss  기준=%smV %s%%' %
          (ns.get('dis_on'), ns.get('dis_el'), ns.get('dis_mv0'),
           ns.get('dis_pc0')))
    # ★2026-10-20 (GPT 재감사 N03) 이 세션을 만든 펌웨어 — 판을 모르고 비교하면 안 된다.
    def _s(v):
        return v.split(b'\x00')[0].decode('utf-8', 'replace') if isinstance(v, (bytes, bytearray)) else v
    fw0, fw = _s(ns.get('fw0')), _s(ns.get('fw'))
    if fw0 or fw:
        print('펌웨어: 세션 시작 %s / 마지막 부팅 %s%s'
              % (fw0 or '(기록 없음 — 10/20 이전 판이 시작한 세션)', fw or '?',
                 '' if (not fw0 or fw0 == fw) else '   ★★세션 도중 판이 바뀌었다(배터리 중 OTA?) — 앞뒤를 섞어 보지 말 것'))
    else:
        print('펌웨어: 기록 없음 (10/20 이전 판 — 어느 빌드의 세션인지 이 덤프로는 모른다)')
    if ns.get('ol_fail') is not None:
        print('OLED ON/OFF 명령 실패(세션): %s회  (정상 0 — 0 이 아니면 화면이 실제로는 안 꺼졌을 수 있다)'
              % ns.get('ol_fail'))
    # ★2026-10-29 PCF 정지 상태(somfy_app.c _pcfrest_tick) — 로터리가 접점 닫힌 위치에 멈춰 있던 시간
    pr = ns.get('pcf_rest')
    if isinstance(pr, (bytes, bytearray)) and len(pr) >= 20:
        c0, c1, c2, oth, last = struct.unpack('<5I', bytes(pr[:20]))
        tot = c0 + c1 + c2
        if tot:
            # PCF8575 입력 전류원(TI 표 5.5): 외부에서 LOW 로 잡힌 핀마다 30~300µA — 닫힌 접점 수 × 체류 비율
            avg_lo = (c1 * 1 + c2 * 2) * 30.0 / tot
            avg_hi = (c1 * 1 + c2 * 2) * 300.0 / tot
            print('PCF 정지 상태(세션 %.1f시간): 로터리 닫힌 접점 0개 %.1f%% / 1개 %.1f%% / 2개 %.1f%%  → 접점 전류 평균 %.0f~%.0f µA'
                  ' (핀당 30~300µA 가정)' % (tot / 3600.0, 100.0 * c0 / tot, 100.0 * c1 / tot, 100.0 * c2 / tot, avg_lo, avg_hi))
            print('   그 밖 LOW 핀 %d 핀·초(누름 시간 — 정지 중 0 이어야)   마지막 원시값 %s'
                  % (oth, '모름' if last == 0xFFFFFFFF else '0x%04X' % last))
        else:
            print('PCF 정지 상태: 아직 기록 없음(세션이 짧거나 PCF 를 못 읽었다)')
    if ns.get('lp_ho') is not None:
        print('LP→HP 인계(부팅 후): %s회  (정상 0 — 0 이 아니면 LP 코어가 멈춰 HP 폴링으로 넘어간 적이 있다, N07)' % ns.get('lp_ho'))
    # ★2026-10-31 로터리 주차(main/lp_core/pcf_lp_config.h) — LP 적재 뒤 [주차한 채 끝난 폴, 전체 폴]
    rk = ns.get('rot_park')
    if isinstance(rk, (bytes, bytearray)) and len(rk) >= 8:
        pk, pn = struct.unpack('<2I', bytes(rk[:8]))
        if pn:
            # 주차한 폴은 놓고-읽고-다시 모는 창(~2.5%)만 전류원이 흐른다(sim/tools/rot_park_sim.py ⑥)
            print('로터리 주차(LP 적재 뒤): 폴 %d회 중 %d회(%.1f%%)가 주차한 채 끝났다 — 위 "닫힌 접점" 시간 중 이만큼은 전류가 ~2.5%%만 흐른다'
                  % (pn, pk, 100.0 * pk / pn))
    if not n:
        print('(비어 있음)'); return 0

    start = 0 if n < maxn else (head + maxn - n) % maxn
    rows = []
    for i in range(n):
        off = ((start + i) % maxn) * SAMPLE.size
        rows.append(SAMPLE.unpack_from(buf, off))

    print('%4s %8s %8s %5s %8s %6s %6s %5s %8s %6s'
          % ('#', '+초', 'mV', '%', '평균mA', '무선', '화면', 'pm', 'sleep%', 'rssi'))
    p0 = t0 = None
    for i, (t_s, mv, pct, flags, sp, ls, rssi) in enumerate(rows):
        ev = (flags >> 4) & 0x0F
        if ev == 9 or p0 is None:          # 세션 경계에서 기준 재설정
            p0, t0 = pct, t_s
            if ev == 9:
                print('     ────── 새 방전 세션 시작 ──────')
        dt = t_s - t0
        ma = ((p0 - pct) * capacity_mah * 36) // dt if dt > 0 else 0
        sp_mv = sp * 3100 // div_bot * (div_top + div_bot) // 4095
        print('%4d %8d %8d %5d %8d %6d %6d %5d %8d %6s   산포%d(%dmV) %s'
              % (i, t_s, mv, pct, ma, flags & 1, (flags >> 1) & 1,
                 (flags >> 2) & 3, ls, ('-' if rssi == 127 else rssi),
                 sp, sp_mv, EVN[ev]))

    v = [r[1] for r in rows]
    t = [r[0] for r in rows]
    print()
    print('요약: %d건  +%ds~+%ds (%.2f시간)  %dmV → %dmV (%+dmV)'
          % (len(rows), t[0], t[-1], (t[-1] - t[0]) / 3600.0, v[0], v[-1],
             v[-1] - v[0]))
    print('      최저 %dmV / 최고 %dmV   sleep 평균 %.1f%%'
          % (min(v), max(v), sum(r[5] for r in rows) / len(rows)))
    rs = [r[6] for r in rows if r[6] != 127]
    if rs:
        print('      RSSI  최저 %ddBm / 최고 %ddBm / 평균 %.1fdBm  (표본 %d)'
              % (min(rs), max(rs), sum(rs) / float(len(rs)), len(rs)))
    else:
        print('      RSSI  기록 없음(127=측정불가 이거나 옛 형식)')

    # ── light sleep 진입 계측 (2026-08-16 추가) ────────────────────────────
    #  `rt` 실측에서 전 태스크 작업 합이 1.02%(somfy_app 회당 703µs) 로 나왔다.
    #  그런데 sleep 체류는 85% → 남는 14%p 가 '일'이 아니라 **깨어남 1회당
    #  고정비용**인지 여기서 가른다.
    cnt = ns.get('ls_cnt')
    us = ns.get('ls_us')
    el = ns.get('dis_el')
    if cnt and us and el:
        el_us = el * 1000000.0
        awake = el_us - us
        print()
        print('── light sleep 계측 ──')
        # ★2026-10-20 (GPT 재감사 N04) ls_rjs/ls_rjh 가 있으면 새 판 — ls_cnt 는 **실제로 잔 것만**이다.
        #  없으면 옛 판 — IDF 가 거부된 시도에도 종료 콜백을 불러 ls_cnt·ls_us 에 거부가 섞여 있다
        #  (10/18 COM7: 정지 13%, 들고 움직인 55초 66%). 옛 판 값을 '깨어남 횟수'로 읽으면 부풀려진다.
        rjs, rjh = ns.get('ls_rjs'), ns.get('ls_rjh')
        if rjs is not None and rjh is not None:
            att = cnt + rjs + rjh
            print('  잔 횟수(성공) %d회 / 거부 %d회 = 짧음 %d + 하드웨어 %d  (시도의 %.1f%%)'
                  % (cnt, rjs + rjh, rjs, rjh, 100.0 * (rjs + rjh) / att if att else 0))
        else:
            print('  ★옛 판 기록 — 아래 "진입"에는 **거부된 시도가 섞여 있다**(성공 횟수 아님, N04)')
        print('  세션 경과 %.1f초 / 잔 시간 %.1f초 (%.1f%%) / 진입 %d회'
              % (el, us / 1e6, 100.0 * us / el_us, cnt))
        print('  깨어남 1회당: 잠 %.1fms  +  깨어있음 %.2fms'
              % (us / cnt / 1000.0, awake / cnt / 1000.0))
        print('  깨어남 빈도 %.2f회/초' % (cnt / float(el)))
        WORK_US = 703.0        # rt 실측: somfy_app 루프 1회 작업량
        print('  ★회당 오버헤드 ≈ %.2fms (깨어있음 %.2fms − 실작업 %.2fms)'
              % ((awake / cnt - WORK_US) / 1000.0, awake / cnt / 1000.0,
                 WORK_US / 1000.0))
        print('  판정: %s'
              % ('오버헤드 지배 — 깨어남 횟수를 줄이는 것이 유효하다'
                 if (awake / cnt - WORK_US) > 3000 else
                 '오버헤드 작음 — 루프 주기를 늘려도 이득이 작다'))
    else:
        print('  (light sleep 계측 없음 — ls_cnt/ls_us 를 남기는 펌웨어가 아니다)')

    # ── 수면 길이 히스토그램 (2026-08-20) ─────────────────────────────────
    #  태스크 주기 전수조사로는 유휴 54회/초가 나오는데 실측이 93.8회/초였다.
    #  분포의 봉우리가 어느 주기인지 보면 지배 요인이 그대로 드러난다.
    hb = ns.get('ls_hist')
    if isinstance(hb, (bytes, bytearray)) and len(hb) >= 32:
        import struct as _st
        h8 = _st.unpack_from('<8I', hb)
        lbl = ['<2ms', '2~5ms', '5~8ms', '8~12ms', '12~20ms',
               '20~30ms', '30~60ms', '>=60ms']
        tot = sum(h8) or 1
        print()
        _bh = base_ns.get('ls_hist') if base_ns else None
        if isinstance(_bh, (bytes, bytearray)) and len(_bh) >= 32:
            b8 = _st.unpack_from('<8I', _bh)
            h8 = tuple(max(0, x - y) for x, y in zip(h8, b8))
            tot = sum(h8) or 1
            print('── 수면 길이 분포 — **이번 구간 차분** ──')
        else:
            print('── 수면 길이 분포 ──  ★★이 값은 **평생 누적**이다(세션 아님).')
            print('   펌웨어가 ls_hist 를 세션 기준으로 차감하지 않는다'
                  ' (somfy_app.c _batlog_save 주석).')
            print('   과거 다른 설정의 세션이 섞여 있으니 **모양을 믿지 말 것**.'
                  ' --vs <이전덤프.bin> 로 차분을 볼 것.')
        print('  LP 코어=%s  버튼 폴 주기=%sms'
              % ('ON' if ns.get('lp_on') else 'OFF', ns.get('poll_ms')))
        for l, v in zip(lbl, h8):
            bar = '#' * int(40.0 * v / tot)
            print('  %-9s %9d  %5.1f%%  %s' % (l, v, 100.0 * v / tot, bar))
        top = lbl[h8.index(max(h8))]
        if isinstance(_bh, (bytes, bytearray)) and len(_bh) >= 32:
            print('  ★지배 구간 %s — 버튼 폴 주기(%sms)와 %s'
                  % (top, ns.get('poll_ms'),
                     '일치' if str(ns.get('poll_ms')) in top else '**불일치**'))
        else:
            print('  (지배 구간 판정은 누적값으로 하면 틀린다 — 생략)')

    # ── 깨어난 원인 + 태스크별 실제 반복 (2026-08-20) ─────────────────────
    import struct as _st
    wcb = ns.get('wc_hist')
    wib = ns.get('wi_cnt')
    el2 = ns.get('dis_el')
    if isinstance(wcb, (bytes, bytearray)) and len(wcb) >= 36:
        wc = _st.unpack_from('<9I', wcb)
        nm = ['TIMER', 'GPIO', 'ULP', 'UART', 'BT', 'WIFI(802.15.4)',
              'EXT1', 'UNDEFINED', '표에 없음']
        tot = sum(wc) or 1
        print()
        _bw = base_ns.get('wc_hist') if base_ns else None
        if isinstance(_bw, (bytes, bytearray)) and len(_bw) >= 36:
            wb = _st.unpack_from('<9I', _bw)
            wc = tuple(max(0, x - y) for x, y in zip(wc, wb))
            tot = sum(wc) or 1
            print('── 깨어난 원인 — **이번 구간 차분** ──')
        else:
            print('── 깨어난 원인 ──  ★★이 값도 **평생 누적**이다(세션 아님).')
            print('   2026-09-11 실례: 건강한 세션(100%% TIMER)인데도 누적 탓에'
                  " 'GPIO 81.8%%' 로 찍혔다. --vs 로 차분을 볼 것.")
        for n, v in zip(nm, wc):
            if v:
                print('  %-16s %9d  %5.1f%%' % (n, v, 100.0 * v / tot))
    if isinstance(wib, (bytes, bytearray)) and len(wib) >= 16 and el2:
        it = _st.unpack_from('<4I', wib)
        nm = ['btn_handler', 'somfy_app 메인', 'oled_ui', 'hold_repeat']
        # ★2026-10-18 (감사 F18) btn 의 '예상 40' 은 옛 25ms 폴링 시절 값이었다. LP 경로 유휴는 100ms
        #  주기라 루프 10회/초가 맞다(10/18 이전 펌웨어는 110ms 라 9.1). 화면 ON 중·눌림 중엔 더 많다.
        # ★같은 날 3차 펌웨어(버튼 유휴 1000ms 를 poll_ms 로 알린다)는 버튼·메인 1초, OLED·hold 5초 안전망 →
        #  유휴 기대 1/1/0.2/0.2. 화면이 켜져 있던 시간(OLED 20fps)이 섞이면 oled 가 더 높다.
        exp = [1.0, 1.0, 0.2, 0.2] if (ns.get('poll_ms') or 0) >= 1000 else [10.0, 10.0, 2.0, 2.0]
        print()
        print('── 태스크별 실제 반복 (부팅 후 누적 / 세션 %ds) ──' % el2)
        tot = 0.0
        for n, v, e in zip(nm, it, exp):
            r = v / float(el2)
            tot += r
            print('  %-16s %9d  = %6.2f회/초   (예상 %.1f)' % (n, v, r, e))
        print('  %-16s %9s  = %6.2f회/초' % ('우리 태스크 합', '', tot))
        cnt = ns.get('ls_cnt')
        if cnt:
            w = cnt / float(el2)
            print('  %-16s %9s  = %6.2f회/초' % ('실제 깨어남', '', w))
            # ★2026-10-18 정정(감사 F18): 이 차이를 "스택 몫"이라 불렀지만, 10/18 이전 펌웨어에서는
            #  대부분이 **우리 버튼 태스크의 이중 대기**(루프 1회에 통지 대기 + 공통 10ms 지연 = 깨어남 2회)였다.
            #  위 카운터는 루프 1회에 1 씩만 세므로 두 번째 깨어남이 여기로 빠졌다. 고친 펌웨어에서는 작아져야 한다.
            print('  ★나머지(루프 밖 대기·SDK 몫) %6.2f회/초  = 전체의 %.0f%%'
                  '   ※10/18 이전 펌웨어는 대부분 버튼 태스크 이중 대기'
                  % (w - tot, 100.0 * (w - tot) / w if w else 0))
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    _base = None
    if '--vs' in sys.argv:
        _i = sys.argv.index('--vs')
        _base = _batlog_ns(sys.argv[_i + 1])
        del sys.argv[_i:_i + 2]
    sys.exit(decode(sys.argv[1], base_ns=_base))
