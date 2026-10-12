# -*- coding: utf-8 -*-
"""얕은잠 중 PMU 전원 차단 플래그 해석 — 어느 도메인이 켜진 채 자는가.

왜 필요한가 (2026-10-07)
────────────────────────
깊은잠 대조군(9/29)에서 기판 전체가 평소 소비의 약 5%(~0.3mA)로 나왔다. 나머지
95% 는 칩이고, 깨어있는 시간 몫을 빼면 **얕은잠 상태 자체가 ~3mA** 다. 제대로 설정된
C6 얕은잠은 0.1~0.3mA 이므로, 꺼져야 할 도메인이 켜진 채 자고 있다는 뜻이다.

펌웨어(somfy_app.c)는 CONFIG_ESP_SLEEP_DEBUG 로 IDF 가 넘겨주는 sleep_flags
(= "Enter sleep" 직전 하드웨어에 넘긴 PMU_SLEEP_PD_* 최종값)를 절전마다 누적해
NVS pmdump 네임스페이스에 남긴다:
    pdand  모든 절전에서 꺼진 도메인 (AND)
    pdor   한 번이라도 꺼진 도메인   (OR)
    pdlast 마지막 절전의 값
    pdcnt  누적한 절전 횟수
    pdrej  절전 요청이 거부된 횟수
비트 1 = 그 도메인을 **끔**. OR 에 없는 비트 = 한 번도 안 꺼진 도메인.

비트 정의 출처: esp-idf components/esp_hw_support/include/esp_private/esp_pmu.h
(C6: SOC_PM_SUPPORT_HP_AON_PD=1 → BIT5 유효, SOC_PM_SUPPORT_CNNT_PD 없음 → BIT15 무효)

    python sim/tools/pd_flags.py --selftest
"""
import sys

# (비트, 이름, 설명, 켜져 있으면 의심하는가)
#  의심=False 인 것은 얕은잠에서 켜져 있는 게 **정상일 수 있는** 도메인이다:
#   - MEM_G*  : SRAM 은 얕은잠에서 내용을 유지해야 한다
#   - HP_AON  : HP always-on
#   - RC_FAST : LP 코어 클럭(17.5MHz). LP 코어가 5ms 마다 PCF 를 읽으니 켜짐이 정상
#   - LP_PERIPH: LP_I2C 등. 같은 이유로 켜짐이 정상
PD_BITS = [
    (0,  'TOP',       'HP 주변장치 상위 도메인 (I2C·SPI·UART·GPIO 매트릭스)', True),
    (1,  'VDDSDIO',   '외부 플래시 전원 (절전이 충분히 길 때만 끔)',          True),
    (2,  'MODEM',     '무선 모뎀 (802.15.4 / BLE)',                         True),
    (3,  'HP_PERIPH', 'HP 주변장치',                                        True),
    (4,  'CPU',       'HP CPU',                                             True),
    (5,  'HP_AON',    'HP always-on',                                       False),
    (6,  'MEM_G0',    'SRAM 그룹0 (유지가 정상)',                            False),
    (7,  'MEM_G1',    'SRAM 그룹1 (유지가 정상)',                            False),
    (8,  'MEM_G2',    'SRAM 그룹2 (유지가 정상)',                            False),
    (9,  'MEM_G3',    'SRAM 그룹3 (유지가 정상)',                            False),
    (10, 'XTAL',      '40MHz 주 수정발진기',                                 True),
    (11, 'RC_FAST',   '17.5MHz 내부 RC — LP 코어 클럭 (켜짐이 정상일 수 있음)', False),
    (12, 'XTAL32K',   '32kHz 외부 수정 (미사용)',                            True),
    (13, 'RC32K',     '32kHz 내부 RC (미사용 — RTC 는 RC_SLOW 150k)',        True),
    (14, 'LP_PERIPH', 'LP 주변장치 — LP_I2C (켜짐이 정상일 수 있음)',         False),
]


def classify(pd_and, pd_or):
    """비트별 상태: 'always'(매번 끔) / 'some'(가끔 끔) / 'never'(한 번도 안 끔)."""
    out = {}
    for bit, name, _, _ in PD_BITS:
        m = 1 << bit
        if pd_and & m:
            out[name] = 'always'
        elif pd_or & m:
            out[name] = 'some'
        else:
            out[name] = 'never'
    return out


def suspects(pd_and, pd_or):
    """켜져 있으면 의심스러운 도메인 중 매번 꺼지지 않은 것."""
    st = classify(pd_and, pd_or)
    return [(name, st[name], desc) for _, name, desc, sus in PD_BITS
            if sus and st[name] != 'always']


def report(pd_and, pd_or, pd_last, pd_cnt, pd_rej):
    print('=== 얕은잠 중 전원 도메인 (CONFIG_ESP_SLEEP_DEBUG) ===')
    if pd_cnt is None:
        print('  pmdump/pd* 가 없다 — 이 계측이 들어간 펌웨어로 배터리 구동한 적이 없다.')
        return
    if pd_cnt == 0:
        print('  절전 횟수 0 — 배터리 모드에서 얕은잠이 한 번도 안 걸렸다.')
        return
    print('  누적 절전 %d회  (요청 거부 %d회)   마지막 플래그 0x%04X'
          % (pd_cnt, pd_rej or 0, pd_last or 0))
    print()
    lab = {'always': '매번 끔', 'some': '★가끔만 끔', 'never': '★★한 번도 안 끔'}
    st = classify(pd_and, pd_or)
    print('  %-10s %-14s %s' % ('도메인', '절전 중', '설명'))
    print('  ' + '-' * 72)
    for _, name, desc, sus in PD_BITS:
        s = st[name]
        mark = lab[s] if (sus or s == 'always') else ('켜짐(정상 가능)' if s == 'never'
                                                      else lab[s])
        print('  %-10s %-14s %s' % (name, mark, desc))
    sus = suspects(pd_and, pd_or)
    print()
    if not sus:
        print('  → 의심 도메인 없음. 얕은잠 ~3mA 의 원인은 도메인 밖(깨어있는 시간·주변장치)이다.')
    else:
        print('  → ★용의자 (꺼져야 하는데 매번 꺼지지 않음):')
        for name, s, desc in sus:
            print('      %-10s %s   %s' % (name, lab[s], desc))


# sleep_flags 에 섞여 오는 **도메인이 아닌** 모드 비트 (esp_pmu.h)
MODE_BITS = {
    26: 'RTC_SLEEP_POWER_BY_VBAT',
    28: 'RTC_SLEEP_USE_ADC_TESEN_MONITOR',
    29: 'RTC_SLEEP_NO_ULTRA_LOW (깊은잠용 모드 비트 — 얕은잠 전류와 무관)',
}


def mode_bits(flags):
    """도메인 표에 없는 비트를 이름과 함께 돌려준다 — 모르는 비트를 조용히 버리지 않는다."""
    known = {b for b, _, _, _ in PD_BITS}
    out = []
    for b in range(32):
        if (flags >> b) & 1 and b not in known:
            out.append((b, MODE_BITS.get(b, '★모르는 비트 — esp_pmu.h 확인 필요')))
    return out


def cc_classify(status):
    """CC1101 SNOP 상태 바이트. bit7=CHIP_RDYn: 1=크리스털 정지(SPWD 로 자고 있었음)."""
    return 'asleep' if (status & 0x80) else 'awake'


def cc_report(slp, awk, last, fix=None):
    print('=== CC1101 수면 탐침 (60초마다 + 송신 때마다) ===')
    if slp is None and awk is None:
        print('  ccp_* 가 없다 — 탐침이 들어간 펌웨어로 배터리 구동한 적이 없다.')
        return
    slp, awk = slp or 0, awk or 0
    n = slp + awk
    if n == 0:
        print('  탐침 0회 — 배터리 구간이 60초보다 짧았다.')
        return
    print('  자고 있었음 %d회 / ★깨어 있었음 %d회  (깨어 있던 비율 %.0f%%)   마지막 상태 0x%02X'
          % (slp, awk, 100.0 * awk / n, last if last is not None else 0xFF))
    if awk == 0:
        print('  → CC1101 은 매번 SPWD 로 자고 있었다. CSn 이 칩을 깨우지 않는다.')
    elif awk >= n * 0.5:
        print('  → ★대부분 깨어 있었다 = 얕은잠 중 떠 있는 CSn 이 SPWD 를 깨운다.')
        print('    CC1101 IDLE 은 약 1.7mA — 얕은잠 중 미설명 전류의 큰 몫이다.')
    else:
        print('  → 가끔 깨어 있었다 — CSn 이 간헐적으로 LOW 로 흐른다.')


def _selftest():
    TOP, VDDSDIO, MODEM, CPU, XTAL, RC_FAST = 1, 2, 4, 16, 1 << 10, 1 << 11
    # ① 플래시는 가끔만 끄고(짧은 절전), 모뎀·XTAL 은 한 번도 안 끔
    a = TOP | CPU
    o = TOP | CPU | VDDSDIO
    st = classify(a, o)
    assert st['TOP'] == 'always' and st['CPU'] == 'always'
    assert st['VDDSDIO'] == 'some'
    assert st['MODEM'] == 'never' and st['XTAL'] == 'never'
    names = [n for n, _, _ in suspects(a, o)]
    assert 'MODEM' in names and 'XTAL' in names and 'VDDSDIO' in names
    assert 'TOP' not in names and 'CPU' not in names
    # ② RC_FAST 가 켜져 있어도 용의자로 몰지 않는다(LP 코어 클럭)
    assert 'RC_FAST' not in [n for n, _, _ in suspects(a, o)]
    # ③ 이상적인 경우 — 의심 도메인이 전부 매번 꺼짐
    ideal = TOP | VDDSDIO | MODEM | (1 << 3) | CPU | XTAL | (1 << 12) | (1 << 13)
    assert suspects(ideal, ideal) == [], suspects(ideal, ideal)
    # ④ 초기값(AND=전부 1, 횟수 0)은 해석하지 않는다 — report 가 막는다
    report(0xFFFFFFFF, 0, 0, 0, 0)
    print()
    report(a, o, a, 1234, 5)
    # ⑤ 실측 마지막 플래그 0x20003C12 의 비트 29 를 모르는 비트로 버리지 않는다
    mb = mode_bits(0x20003C12)
    assert mb == [(29, MODE_BITS[29])], mb
    # ⑥ CC1101 상태 바이트 — bit7 만 본다
    assert cc_classify(0xFF) == 'asleep'     # 크리스털 정지, SO 가 HIGH 로 남음
    assert cc_classify(0x8F) == 'asleep'
    assert cc_classify(0x0F) == 'awake'      # IDLE(000), FIFO 15
    assert cc_classify(0x00) == 'awake'
    assert cc_classify(0x1F) == 'awake'      # RX 상태여도 깨어 있음
    print()
    cc_report(0, 47, 0x0F)                   # 가설이 맞을 때의 모양
    cc_report(52, 0, 0xFF)                   # 고친 뒤의 모양
    print('\nselftest 통과')


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if '--selftest' in sys.argv:
        _selftest()
