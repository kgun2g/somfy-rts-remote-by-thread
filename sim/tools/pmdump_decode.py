# -*- coding: utf-8 -*-
"""NVS 덤프에서 `esp_pm_dump_locks()` 출력을 꺼내 **배터리 구간만** 계산해 보여준다.

왜 필요한가 (2026-08-24): H2 가 배터리 13.9시간 동안 light sleep 을 **0.0%** 밖에
안 했다. batlog 의 pm 열은 3(= light sleep ON + 배터리)이라 **펌웨어 설정은 정상**
이다. 즉 누군가 ESP_PM_NO_LIGHT_SLEEP 락을 계속 쥐고 있다.

esp_pm_dump_locks 는 stdout 으로만 출력하는데, 배터리 구동 중엔 USB 콘솔이 죽는다
(기록된 지뢰). 그래서 펌웨어가 open_memstream 으로 받아 NVS 에 남기고 여기서 읽는다.

★★2026-09-10 차분 계산 추가 — 이게 이 도구의 핵심이 됐다.
esp_pm 카운터는 **부팅 기준 누적**이고 `Time since boot up` 에는 플래시 직후의 USB
구간이 통째로 섞인다. USB 중에는 light sleep 이 아예 꺼져 있으므로
`light_sleep_counts / boot_s` 는 **항상 과소평가**된다. 이 오염을 못 보고 집계해
두 번 헛짚었다. → 펌웨어가 배터리 진입 첫 스냅샷을 `pmdump/locks0` 에,
60초 갱신분을 `pmdump/locks` 에 남긴다. 여기서 **둘의 차이**를 쓴다.

★함께 기억할 것: `sleep%` 자체는 지표로 쓰면 안 된다. C6 는 절전 1회당 약 2.5ms 를
고정으로 쓰므로(REGDMA 382+480 · 발진기 재교정 194 · LP/HP 기상대기 1796 · 도메인
동기 150 µs), 평균 절전 길이가 이보다 짧으면 **자는 시간보다 자러 가는 비용이 크다**.
봐야 할 것은 **초당 절전 횟수**다.

    esptool ... read_flash 0x10000 0xC000 nvs.bin
    python sim/tools/pmdump_decode.py nvs.bin
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nvs_parse import parse   # noqa: E402

# 절전 1회당 고정 하드웨어 비용(µs). ESP-IDF C6 상수에서 산출 — 위 머리말 참조.
SLEEP_FIXED_COST_US = 2520.0


def _get(items, key):
    for it in items:
        if it['ns_name'] == 'pmdump' and it['key'] == key:
            v = it['value']
            if isinstance(v, (bytes, bytearray)):
                return v.split(b'\x00')[0].decode('utf-8', 'replace')
            if isinstance(v, str):
                return v
    return None


def _get_u32(items, key):
    """pmdump 네임스페이스의 정수 키 (없으면 None)."""
    for it in items:
        if it['ns_name'] == 'pmdump' and it['key'] == key:
            v = it['value']
            if isinstance(v, int):
                return v
    return None


def _ints(line):
    return [int(x) for x in re.findall(r'\d+', line)]


def _snap(txt):
    """덤프 텍스트 → {boot_us, sleep_us, ls_cnt, ls_rej, locks{name:(count,us)}}"""
    d = {'boot_us': 0, 'sleep_us': 0, 'ls_cnt': 0, 'ls_rej': 0, 'locks': {}}
    m = re.search(r'Time since boot up:\s*(\d+)\s*us', txt)
    if m:
        d['boot_us'] = int(m.group(1))
    m = re.search(r'light_sleep_counts:(\d+)\s+light_sleep_reject_counts:(\d+)', txt)
    if m:
        d['ls_cnt'], d['ls_rej'] = int(m.group(1)), int(m.group(2))
    sect = None
    for line in txt.splitlines():
        s = line.strip()
        if s.startswith('Lock stats'):
            sect = 'lock'; continue
        if s.startswith('Mode stats'):
            sect = 'mode'; continue
        if not s or s.startswith('Name') or s.startswith('Mode'):
            continue
        nums = _ints(s)
        if sect == 'mode' and s.startswith('SLEEP') and len(nums) >= 2:
            d['sleep_us'] = nums[-2]
        elif sect == 'lock' and len(nums) >= 3:
            # Name Type Arg Active Total_count Time(us) Time(%)
            d['locks'][s.split()[0]] = (nums[-3], nums[-2])
    return d


def _report(a, b, real_delta=True):
    """a=배터리 진입 시점(locks0), b=마지막(locks). 차분을 사람 말로 푼다."""
    wall = (b['boot_us'] - a['boot_us']) / 1e6
    if wall <= 0:
        print('※ locks0 와 locks 의 시각이 같다 — 배터리 구간이 60초 미만이다.')
        return
    cnt = b['ls_cnt'] - a['ls_cnt']
    rej = b['ls_rej'] - a['ls_rej']
    slp = (b['sleep_us'] - a['sleep_us']) / 1e6
    if real_delta:
        print('=== 배터리 구간만 (locks0 -> locks 차분) ===')
    else:
        print('=== [주의] 부팅 전체 기준 (locks0 없음 - 배터리 구간 아님) ===')
    print('  창 길이        : %.1f분 (%.2f시간)' % (wall / 60, wall / 3600))
    if cnt <= 0:
        print('  ※ 절전 횟수 증가 0 — light sleep 이 걸리지 않았다.')
        return
    per = slp / cnt * 1000.0                      # 회당 평균 절전(ms)
    rate = cnt / wall
    overhead = rate * SLEEP_FIXED_COST_US / 1e6   # 전환비용이 차지하는 벽시계 비율
    print('  ★초당 절전 횟수: %.1f회/초   (총 %d회, 거부 %d회)' % (rate, cnt, rej))
    print('  sleep 체류율    : %.1f%%   깨어있음 %.1f%%' % (100 * slp / wall,
                                                            100 * (1 - slp / wall)))
    print('  회당 평균 절전  : %.2f ms' % per)
    print('  회당 고정비용   : %.2f ms  (C6 하드웨어 전환 — 위 머리말)' % (SLEEP_FIXED_COST_US / 1000))
    print('  → 전환비용이 벽시계의 %.0f%% 를 먹는다' % (100 * overhead))
    if per < SLEEP_FIXED_COST_US / 1000:
        print('  ★★자는 시간(%.2fms)보다 자러 가는 비용(%.2fms)이 크다 — 절약이 사실상 0.'
              % (per, SLEEP_FIXED_COST_US / 1000))
    print()
    print('  락별 증가(횟수 / 보유 µs):')
    for k in sorted(set(a['locks']) | set(b['locks'])):
        c0, t0 = a['locks'].get(k, (0, 0))
        c1, t1 = b['locks'].get(k, (0, 0))
        dc, dt = c1 - c0, t1 - t0
        if dc or dt:
            print('    %-12s %8d회 (%.1f/초)  %10dµs (%.1f%%)'
                  % (k, dc, dc / wall, dt, 100.0 * dt / (wall * 1e6)))


def main(path):
    _, items = parse(open(path, 'rb').read())
    txt = _get(items, 'locks')
    txt0 = _get(items, 'locks0')
    if not txt:
        print('pmdump/locks 가 없다 — 배터리 구동 중 60초 주기로 저장된다.')
        print('USB 를 뺀 채 최소 1분 이상 두었다가 다시 연결해 덤프할 것.')
        return 1
    if txt0:
        _report(_snap(txt0), _snap(txt))
        print()
    else:
        # 옛 펌웨어(locks0 없음): 0 을 기준점으로 삼아 그래도 계산한다.
        # 부팅 기준 누적이라 USB 구간이 섞이지만, 배터리 세션이 몇 시간이면
        # 오염은 수 % 수준이라 초당 절전 횟수의 자릿수 판단에는 충분하다.
        print('*** pmdump/locks0 이 없다. 뜻은 둘 중 하나다:')
        print('    (a) 2026-09-10 이전 펌웨어 - 아래 값은 부팅 기준이라 USB 구간이 섞임')
        print('    (b) 새 펌웨어인데 **배터리 모드에 한 번도 안 들어갔다**')
        print('        → 이 경우 아래 locks 는 **이전 세션의 낡은 값**이다.')
        print('        (플래시는 NVS 0x10000 을 지우지 않는다. 새 펌웨어는 locks0 와')
        print('         locks 를 같은 호출에서 함께 쓰므로, locks0 가 있으면 둘 다 최신이다.)')
        print()
        _report({'boot_us':0,'sleep_us':0,'ls_cnt':0,'ls_rej':0,'locks':{}}, _snap(txt),
                real_delta=False)
        print()
    print('=== esp_pm_dump_locks (마지막 저장분, 부팅 기준 누적) ===')
    print(txt)
    print('=== 끝 ===')
    print()
    print('읽는 법: NO_LIGHT_SLEEP 류 락을 잡고 있는 항목이 범인이다.')
    print('        rtos0 은 결과지 원인이 아니다(깨어 있는 틱마다 잡힌다).')
    # ★2026-10-07 얕은잠 중 전원 도메인 (펌웨어 CONFIG_ESP_SLEEP_DEBUG 계측)
    print()
    import pd_flags
    pd_flags.report(_get_u32(items, 'pdand'), _get_u32(items, 'pdor'),
                    _get_u32(items, 'pdlast'), _get_u32(items, 'pdcnt'),
                    _get_u32(items, 'pdrej'))
    last = _get_u32(items, 'pdlast')
    if last:
        for b, nm in pd_flags.mode_bits(last):
            print('  (모드 비트 %d: %s)' % (b, nm))
    print()
    pd_flags.cc_report(_get_u32(items, 'ccp_slp'), _get_u32(items, 'ccp_awk'),
                       _get_u32(items, 'ccp_last'))
    print()
    pd_flags.rt_report(_get_u32(items, 'rt_init'), _get_u32(items, 'rt_crt'))
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main(sys.argv[1]) if len(sys.argv) > 1 else (print(__doc__) or 1))
