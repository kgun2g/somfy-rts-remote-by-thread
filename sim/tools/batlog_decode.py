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


def decode(path, div_top=100, div_bot=100, capacity_mah=700):
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
    n = ns.get('n', 0)
    head = ns.get('head', 0)
    sess = ns.get('sess', 0)
    if not isinstance(buf, (bytes, bytearray)):
        print('buf blob 을 못 찾았다 (키: %s)' % ', '.join(ns)); return 1
    maxn = len(buf) // SAMPLE.size
    print('=== 방전 기록: 세션 #%s, %s건 / 링 %d칸 (버퍼 %dB) ===' %
          (sess, n, maxn, len(buf)))
    print('세션상태: dis_on=%s  경과=%ss  기준=%smV %s%%' %
          (ns.get('dis_on'), ns.get('dis_el'), ns.get('dis_mv0'),
           ns.get('dis_pc0')))
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
        print('── 수면 길이 분포 (깨어남 출처) ──')
        print('  LP 코어=%s  버튼 폴 주기=%sms'
              % ('ON' if ns.get('lp_on') else 'OFF', ns.get('poll_ms')))
        for l, v in zip(lbl, h8):
            bar = '#' * int(40.0 * v / tot)
            print('  %-9s %9d  %5.1f%%  %s' % (l, v, 100.0 * v / tot, bar))
        top = lbl[h8.index(max(h8))]
        print('  ★지배 구간 %s — 버튼 폴 주기(%sms)와 %s'
              % (top, ns.get('poll_ms'),
                 '일치' if str(ns.get('poll_ms')) in top else '**불일치**'))

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
        print('── 깨어난 원인 (esp_sleep_get_wakeup_cause) ──')
        for n, v in zip(nm, wc):
            if v:
                print('  %-16s %9d  %5.1f%%' % (n, v, 100.0 * v / tot))
    if isinstance(wib, (bytes, bytearray)) and len(wib) >= 16 and el2:
        it = _st.unpack_from('<4I', wib)
        nm = ['btn_handler', 'somfy_app 메인', 'oled_ui', 'hold_repeat']
        exp = [40.0, 10.0, 2.0, 2.0]
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
            print('  ★남의 몫(스택 등) %6.2f회/초  = 전체의 %.0f%%'
                  % (w - tot, 100.0 * (w - tot) / w if w else 0))
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    sys.exit(decode(sys.argv[1]))
