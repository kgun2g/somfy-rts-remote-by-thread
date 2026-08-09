#!/usr/bin/env python3
"""oled_i2c_fault_sim.py — OLED I2C 버스 고장모드 시뮬레이터 (2026-07-23)

목적: 실기 없이(USB 미연결) OLED 미점등 원인을 좁힌다.
방법: KiCad 회로도에서 추출한 실제 넷리스트를 저항 네트워크로 모델링해
      각 고장 가설이 만들어낼 **측정 신호**(핀 float/내부PD/내부PU 레벨,
      bit-bang 스캔 응답)를 계산하고, 실기에서 실측한 값과 대조한다.

회로 근거(kicad/somfy_blinds_{h4,v4}.kicad_sch 파서 추출):
  h4(COM7 신PCB): R8,R9 = 4.7K 풀업(+3V3 → I2C_SCL/I2C_SDA) **PCB 에 존재**
                  SDA1/SCL1 = SPDT 스위치  C=I2C(22/23) ─ B=PCF8575 / A=LP_I2C(6/7)
                  U2 OLED: pin1 GND, pin2 VCC(+3V3), pin3 SCL, pin4 SDA
  v4(COM4 구PCB): I2C(22/23)에 **PCB 풀업 없음**, 스위치 없음, PCF8574 는 LP(6/7) 직결

전기 모델: 각 넷의 3V3 쪽 저항(병렬)과 GND 쪽 저항(병렬)로 분압 → ESP32 입력임계와 비교.
  ESP32-C6 입력임계(3.3V): VIL_max ≈ 0.25*VDD = 0.83V, VIH_min ≈ 0.75*VDD = 2.48V
  내부 풀업/풀다운 ≈ 45kΩ, 모듈/보드 풀업 4.7kΩ.

사용: python sim/tools/oled_i2c_fault_sim.py
"""

VDD = 3.3
R_INT = 45_000.0      # ESP32 내부 풀업/풀다운
R_PU_PCB = 4_700.0    # h4 R8/R9
R_PU_MOD = 4_700.0    # OLED 모듈 온보드 풀업(대표값)
R_PU_LP = 4_700.0     # h4 R1/R2 (LP_I2C 풀업)
R_SHORT = 10.0        # 하드 단락(솔더브릿지) 등가저항

VIL = 0.25 * VDD
VIH = 0.75 * VDD


def par(rs):
    """병렬 합성. 빈 리스트면 무한대(연결 없음)."""
    rs = [r for r in rs if r is not None and r > 0]
    if not rs:
        return float('inf')
    inv = sum(1.0 / r for r in rs)
    return 1.0 / inv if inv > 0 else float('inf')


def level(r_up, r_dn):
    """3V3쪽 r_up, GND쪽 r_dn 분압 → (전압, 논리레벨 '1'/'0'/'?')"""
    if r_up == float('inf') and r_dn == float('inf'):
        return None, '?'                      # 완전 플로팅
    if r_dn == float('inf'):
        v = VDD
    elif r_up == float('inf'):
        v = 0.0
    else:
        v = VDD * r_dn / (r_up + r_dn)
    return v, ('1' if v >= VIH else '0' if v <= VIL else '?')


class Net:
    """한 I2C 라인(SDA 또는 SCL)의 3V3/GND 쪽 저항 목록."""
    def __init__(self):
        self.up = []      # 3V3 쪽 저항들
        self.dn = []      # GND 쪽 저항들

    def measure(self, esp_mode):
        """esp_mode: 'float' | 'pd' | 'pu'  → 측정 논리레벨"""
        up = list(self.up)
        dn = list(self.dn)
        if esp_mode == 'pd':
            dn.append(R_INT)
        elif esp_mode == 'pu':
            up.append(R_INT)
        return level(par(up), par(dn))[1]


def build(board, fault, line):
    """board: 'h4'|'v4', fault: 고장모드, line: 'SDA'|'SCL' → Net"""
    n = Net()
    # 1) PCB 풀업 (h4 에만 존재)
    if board == 'h4':
        n.up.append(R_PU_PCB)
    # 2) OLED 모듈 (온보드 풀업 + 응답)
    if fault in ('normal', 'sda_scl_swap', 'dead_chip', 'sda_short'):
        n.up.append(R_PU_MOD)                 # 모듈 정상 전원 → 풀업이 3V3 로
    elif fault == 'vcc_gnd_swap':
        n.dn.append(R_PU_MOD)                 # ★역전원: 모듈 풀업이 GND(=모듈VCC핀) 로 향함
    elif fault == 'absent':
        pass                                   # 모듈 없음 → 기여 없음
    # 3) SDA 하드 단락 고장
    if fault == 'sda_short' and line == 'SDA':
        n.dn.append(R_SHORT)
    # 4) h4 스위치 A위치: LP 버스(자체 풀업 4.7K)와 단락
    if board == 'h4' and fault == 'switch_A':
        n.up.append(R_PU_LP)
    return n


def responds(fault):
    """이 고장모드에서 I2C 주소 ACK 가 나오는가"""
    return fault == 'normal' or fault == 'switch_A'


def scan_result(board, fault):
    """bit-bang 스캔이 볼 결과. SDA 가 항상 LOW 면 전 주소 거짓 ACK."""
    sda = build(board, fault, 'SDA')
    if sda.measure('pu') == '0':
        return '전 주소 거짓ACK'
    return '0x3C 응답' if responds(fault) else '(없음)'


FAULTS = [
    ('normal',        '정상 (모듈 연결·전원·배선 모두 정상)'),
    ('switch_A',      'h4 스위치 A위치 (GPIO22↔6, 23↔7 단락)'),
    ('absent',        '모듈 미연결/미장착'),
    ('vcc_gnd_swap',  '모듈 VCC↔GND 뒤바뀜(역전원)'),
    ('sda_scl_swap',  '모듈 SDA↔SCL 뒤바뀜'),
    ('dead_chip',     '모듈 전원 정상인데 칩 무응답(사망)'),
    ('sda_short',     'SDA 라인 GND 단락'),
]

# 실기 실측값 (이 세션에서 캡처)
ACTUAL = {
    'h4': {'SDA': ('1', '1', '1'), 'SCL': ('1', '1', '1'), 'scan': '(없음)'},
    'v4': {'SDA': ('0', '0', '0'), 'SCL': ('1', '1', '1'), 'scan': '전 주소 거짓ACK'},
}
BOARD_NAME = {'h4': 'h4 = COM7 (신PCB, PCB풀업 O, 스위치 O)',
              'v4': 'v4 = COM4 (구PCB, PCB풀업 X, 스위치 X)'}


def main():
    # Windows 콘솔(cp949)에서 한글/기호가 깨지지 않도록 UTF-8 로 재설정
    try:
        import sys as _s
        _s.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    for board in ('h4', 'v4'):
        print("=" * 78)
        print(BOARD_NAME[board])
        act = ACTUAL[board]
        print("  [실측]  SDA float/PD/PU = %s  |  SCL = %s  |  스캔 = %s"
              % ('/'.join(act['SDA']), '/'.join(act['SCL']), act['scan']))
        print("-" * 78)
        print("  %-34s %-12s %-12s %-14s %s" % ("고장모드", "SDA(f/PD/PU)", "SCL(f/PD/PU)", "스캔", "실측일치"))
        for key, desc in FAULTS:
            if board == 'v4' and key == 'switch_A':
                continue                       # v4 엔 스위치가 없다
            sda = build(board, key, 'SDA')
            scl = build(board, key, 'SCL')
            s = tuple(sda.measure(m) for m in ('float', 'pd', 'pu'))
            c = tuple(scl.measure(m) for m in ('float', 'pd', 'pu'))
            sc = scan_result(board, key)
            match = (s == act['SDA'] and c == act['SCL'] and sc == act['scan'])
            print("  %-34s %-12s %-12s %-14s %s"
                  % (desc[:34], '/'.join(s), '/'.join(c), sc, "★일치" if match else "불일치"))
        print()

    print("=" * 78)
    print("결론 — 실측과 일치하는 고장모드만 남긴다:")
    for board in ('h4', 'v4'):
        act = ACTUAL[board]
        hits = []
        for key, desc in FAULTS:
            if board == 'v4' and key == 'switch_A':
                continue
            sda = build(board, key, 'SDA'); scl = build(board, key, 'SCL')
            s = tuple(sda.measure(m) for m in ('float', 'pd', 'pu'))
            c = tuple(scl.measure(m) for m in ('float', 'pd', 'pu'))
            if s == act['SDA'] and c == act['SCL'] and scan_result(board, key) == act['scan']:
                hits.append(desc)
        print("  [%s] %s" % (board, " / ".join(hits) if hits else "(일치하는 모드 없음 — 모델 재검토 필요)"))

    print()
    print("=" * 78)
    print("★ h4 스위치 위치가 정상동작에 미치는 영향 (펌웨어 상호작용):")
    print("  - 스위치 B: PCF8575 가 I2C(22/23) 공유버스에 붙음 → OLED 와 공존(정상 설계).")
    print("  - 스위치 A: GPIO22↔GPIO6 / GPIO23↔GPIO7 물리 단락 + PCF8575 는 어디에도 미연결.")
    print("      → 펌웨어의 LP 비트뱅(GPIO6/7 을 OUTPUT LOW 로 구동)이 OLED 버스를 직접 끌어내림.")
    print("      → OLED 통신 파괴. 또한 PCF 는 LP 에서도 응답 못 함(스위치가 B 로만 연결하므로).")
    print("  ⇒ 부팅 로그에서 'PCF8574/8575 미응답' 이면 스위치가 A 위치일 가능성이 높다.")


if __name__ == '__main__':
    main()
