#!/usr/bin/env python3
"""i2c_rise_time_sim.py — I2C 라인 상승시간으로 '모듈 전원 인가' 판별 가능한지 검증 (2026-07-23)

문제: h4(COM7)는 PCB 에 4.7K 풀업(R8/R9)이 있어, OLED 모듈이 무전원/미연결이어도
      버스가 HIGH 로 보인다. 디지털 읽기로는 모듈 유무를 구분할 수 없다.
아이디어: 모듈이 살아 있으면 **모듈 온보드 풀업(≈4.7K)이 병렬**로 붙어 합성 저항이
      절반(2.35K)이 되고, 라인 상승시간(RC)도 절반이 된다.
      v4(COM4)는 PCB 풀업이 없어 **모듈 풀업 단독** → 기준값 역할.

이 스크립트는 (a) 각 시나리오의 실제 상승시간을 계산하고
             (b) ESP32-C6 폴링 루프가 그 차이를 **분해할 수 있는지**(구분력) 확인한다.

사용: python sim/tools/i2c_rise_time_sim.py
"""
import math

VDD = 3.3
VIH = 0.75 * VDD          # ESP32 입력 HIGH 판정 임계 ≈ 2.475V
R_PU = 4700.0             # PCB(R8/R9) 및 모듈 온보드 풀업 대표값

# 버스 정전용량: 짧은 배선 + 모듈 입력 + 패드. I2C 규격 상한 400pF.
C_CASES = [(50e-12, "짧은배선(50pF)"), (100e-12, "보통(100pF)"), (200e-12, "긴배선(200pF)")]

# ESP32-C6 폴링 1회 소요시간(보수적 추정):
#   gpio_get_level() + 루프 오버헤드 ≈ 12~25ns @160MHz (레지스터 직접 읽기 기준)
TICK_NS_CASES = [(12e-9, "빠름 12ns/회"), (25e-9, "보통 25ns/회")]


def rise_time(r_ohm, c_farad):
    """0V 에서 시작해 VIH 에 도달하는 시간 t = -RC·ln(1 - VIH/VDD)"""
    return -r_ohm * c_farad * math.log(1.0 - VIH / VDD)


def par(*rs):
    return 1.0 / sum(1.0 / r for r in rs)


SCENARIOS = [
    ("COM4(v4) 모듈 전원정상 : 모듈풀업 단독",        par(R_PU)),
    ("COM4(v4) 모듈 무전원   : 풀업 없음(플로팅)",     None),
    ("COM7(h4) 모듈 무전원/미연결 : PCB풀업 단독",     par(R_PU)),
    ("COM7(h4) 모듈 전원정상 : PCB+모듈 병렬",         par(R_PU, R_PU)),
]


def main():
    try:
        import sys; sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    print("=" * 76)
    print("I2C 상승시간 기반 '모듈 전원 인가' 판별 — 구분력 검증")
    print("VIH=%.2fV, 풀업 %.0fΩ" % (VIH, R_PU))
    print("=" * 76)

    for c, cname in C_CASES:
        print("\n■ 버스 용량 %s" % cname)
        print("   %-42s %-12s %s" % ("시나리오", "상승시간", "폴링횟수(12ns / 25ns)"))
        counts = {}
        for name, r in SCENARIOS:
            if r is None:
                print("   %-42s %-12s %s" % (name, "무한(안 올라감)", "타임아웃"))
                continue
            t = rise_time(r, c)
            n12 = t / TICK_NS_CASES[0][0]
            n25 = t / TICK_NS_CASES[1][0]
            counts[name] = (n12, n25)
            print("   %-42s %-12s %.0f / %.0f" % (name, "%.0f ns" % (t * 1e9), n12, n25))
        # 구분력: COM7 두 시나리오의 폴링횟수 차이
        a = counts.get("COM7(h4) 모듈 무전원/미연결 : PCB풀업 단독")
        b = counts.get("COM7(h4) 모듈 전원정상 : PCB+모듈 병렬")
        if a and b:
            for i, (_, tname) in enumerate(TICK_NS_CASES):
                diff = a[i] - b[i]
                verdict = ("구분 가능" if diff >= 3 else
                           "경계(±1~2회, 반복측정 필요)" if diff >= 1 else "구분 불가")
                print("     → %s 기준 차이 %.0f회 : %s" % (tname, diff, verdict))

    print("\n" + "=" * 76)
    print("결론:")
    print("  · 상승시간은 풀업 개수에 반비례 → 모듈 풀업이 붙으면 시간이 정확히 절반.")
    print("  · 폴링 분해능(12~25ns)이 상승시간(수백 ns)보다 훨씬 작아 **차이를 셀 수 있다**.")
    print("  · 단, 절대값은 배선/온도/컴파일러에 따라 변하므로 **보드 간·핀 간 상대비교**로 판독.")
    print("  · 판독 규칙:")
    print("      COM7 폴링수 ≈ COM4 폴링수      → 풀업 1개  = 모듈이 풀업 기여 없음")
    print("                                        = 모듈 무전원/미연결 (VCC·GND 결선 확인)")
    print("      COM7 폴링수 ≈ COM4 의 절반     → 풀업 2개  = 모듈 전원 정상 (칩 무응답 쪽)")


if __name__ == '__main__':
    main()
