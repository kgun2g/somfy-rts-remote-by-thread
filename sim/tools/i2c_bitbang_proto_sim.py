#!/usr/bin/env python3
"""i2c_bitbang_proto_sim.py — bit-bang I2C 프로브 로직 검증 (2026-07-23)

목적: 실기 플래시 **전에**, oled_only_test.c 의 bit-bang 프로브가
      (a) 정상 배선에서 ACK 를 제대로 받고
      (b) SDA/SCL 이 교차 연결된 모듈에서는 정방향 프로브가 무응답이며
      (c) 그 교차 모듈을 **핀을 맞바꾼 프로브로는 검출**하는지
      를 프로토콜 수준에서 확인한다. (사용자 규칙: 빌드/플래시 전 파이썬 검증)

모델:
  - 와이어 2개(W22=GPIO22, W23=GPIO23). open-drain + 풀업 → 아무도 안 당기면 1.
  - 마스터는 test/somfy_cases/oled_only_test.c 의 _bb_probe_pins() 와 **동일한 순서**로
    START → 주소8비트(MSB first) → ACK 읽기 → STOP 을 만든다.
  - 슬레이브(SSD1306 @0x3C)는 자기 SDA/SCL 핀이 어느 와이어에 붙었는지에 따라 동작.
    START(자기SCL=H 중 자기SDA 하강) 검출 → 자기SCL 상승엣지마다 비트 샘플 →
    8비트가 주소와 일치하면 9번째 클럭 동안 자기SDA 를 LOW 로 당김(ACK).

사용: python sim/tools/i2c_bitbang_proto_sim.py
"""


class Bus:
    """와이어 2개. 각 장치가 당기는(0) 여부를 모아 wired-AND."""
    def __init__(self):
        self.drivers = {}          # (dev, wire) -> 0(당김) / 1(릴리즈)

    def drive(self, dev, wire, val):
        self.drivers[(dev, wire)] = val

    def level(self, wire):
        for (d, w), v in self.drivers.items():
            if w == wire and v == 0:
                return 0           # 하나라도 당기면 LOW
        return 1                   # 풀업


class Slave:
    """SSD1306 @addr. sda_wire/scl_wire 로 배선방향을 표현(교차 시 서로 바뀜)."""
    def __init__(self, bus, addr, sda_wire, scl_wire, name="OLED"):
        self.bus, self.addr = bus, addr
        self.sda_w, self.scl_w = sda_wire, scl_wire
        self.name = name
        self.prev_sda = 1
        self.prev_scl = 1
        self.started = False
        self.bits = []
        self.acking = False
        self.ack_clocks = 0
        self.saw_start = False

    def step(self):
        """현재 버스 레벨을 보고 상태 갱신(엣지 검출).
        ★실제 I2C 슬레이브 타이밍: 8비트 수신 후 **8번째 클럭 하강**에서 SDA 를 LOW 로
          당겨 ACK 를 세우고, **9번째 클럭 하강**에서 놓는다. (놓지 않으면 버스가 계속
          LOW 로 물려 이후 모든 주소가 거짓 ACK — 실제로 COM4 에서 관측된 현상과 동일) """
        sda = self.bus.level(self.sda_w)
        scl = self.bus.level(self.scl_w)
        # START: SCL 이 HIGH 인 동안 SDA 하강
        if scl == 1 and self.prev_sda == 1 and sda == 0:
            self.started = True; self.saw_start = True
            self.bits = []; self.acking = False; self.ack_clocks = 0
            self.bus.drive(self.name, self.sda_w, 1)
        # STOP: SCL HIGH 인 동안 SDA 상승
        elif scl == 1 and self.prev_sda == 0 and sda == 1:
            self.started = False; self.bits = []; self.acking = False; self.ack_clocks = 0
            self.bus.drive(self.name, self.sda_w, 1)
        # SCL 상승엣지 → 데이터 비트 샘플 (8비트까지)
        elif self.prev_scl == 0 and scl == 1 and self.started:
            if len(self.bits) < 8:
                self.bits.append(sda)
            elif self.acking:
                self.ack_clocks += 1          # 9번째(ACK) 클럭의 상승
        # SCL 하강엣지 → ACK 세우기/놓기
        elif self.prev_scl == 1 and scl == 0 and self.started:
            if len(self.bits) == 8 and not self.acking and self.ack_clocks == 0:
                val = 0
                for b in self.bits: val = (val << 1) | b
                if (val >> 1) == self.addr:   # 주소+R/W 8비트
                    self.acking = True
                    self.bus.drive(self.name, self.sda_w, 0)   # ACK 어서트
                else:
                    self.started = False       # 내 주소 아님 → 무시
            elif self.acking and self.ack_clocks >= 1:
                self.bus.drive(self.name, self.sda_w, 1)       # 9번째 클럭 후 릴리즈
                self.acking = False; self.started = False
        self.prev_sda, self.prev_scl = sda, scl


class Master:
    """oled_only_test.c 의 _bb_probe_pins() 와 동일한 파형 생성."""
    def __init__(self, bus, slaves, sda_wire, scl_wire):
        self.bus, self.slaves = bus, slaves
        self.sda_w, self.scl_w = sda_wire, scl_wire

    def _set(self, wire, v):
        self.bus.drive("MASTER", wire, v)
        for s in self.slaves:
            s.step()

    def probe(self, addr7):
        # 초기: 둘 다 릴리즈
        self._set(self.sda_w, 1); self._set(self.scl_w, 1)
        # START: SDA↓ (SCL=H)
        self._set(self.sda_w, 0)
        self._set(self.scl_w, 0)
        byte = (addr7 << 1) & 0xFF                  # write
        for i in range(8):
            bit = 1 if (byte & 0x80) else 0
            byte = (byte << 1) & 0xFF
            self._set(self.sda_w, bit)
            self._set(self.scl_w, 1)                # 상승엣지 → 슬레이브 샘플
            self._set(self.scl_w, 0)
        # ACK 읽기: SDA 릴리즈 후 SCL 상승, 그때 SDA 읽기
        self._set(self.sda_w, 1)
        self._set(self.scl_w, 1)
        ack = self.bus.level(self.sda_w)
        self._set(self.scl_w, 0)
        # STOP
        self._set(self.sda_w, 0)
        self._set(self.scl_w, 1)
        self._set(self.sda_w, 1)
        return ack == 0

    def scan(self):
        return [a for a in range(0x08, 0x78) if self.probe(a)]


def run(case, slave_sda, slave_scl):
    """slave_sda/scl: 슬레이브 핀이 붙은 와이어 이름"""
    results = {}
    for label, (m_sda, m_scl) in (("정방향프로브(SDA=22,SCL=23)", ("W22", "W23")),
                                  ("교환프로브 (SDA=23,SCL=22)", ("W23", "W22"))):
        bus = Bus()
        sl = Slave(bus, 0x3C, slave_sda, slave_scl)
        m = Master(bus, [sl], m_sda, m_scl)
        hits = m.scan()
        results[label] = hits
    print("■ %s" % case)
    for k, v in results.items():
        txt = ", ".join("0x%02X" % a for a in v) if v else "(응답 없음)"
        print("    %-28s → %s" % (k, txt))
    return results


def main():
    try:
        import sys; sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    print("=" * 72)
    print("bit-bang I2C 프로브 로직 검증 (oled_only_test.c 와 동일 시퀀스)")
    print("=" * 72)

    a = run("정상 배선: 모듈 SDA→GPIO22, SCL→GPIO23", "W22", "W23")
    print()
    b = run("교차 배선: 모듈 SDA→GPIO23, SCL→GPIO22 (모듈 핀순서 변종)", "W23", "W22")
    print()

    print("=" * 72)
    print("판정:")
    ok1 = a["정방향프로브(SDA=22,SCL=23)"] == [0x3C]
    ok2 = b["정방향프로브(SDA=22,SCL=23)"] == []
    ok3 = b["교환프로브 (SDA=23,SCL=22)"] == [0x3C]
    print("  1) 정상배선에서 정방향프로브가 0x3C 검출          : %s" % ("OK" if ok1 else "실패"))
    print("  2) 교차배선에서 정방향프로브는 무응답(실측과 동일) : %s" % ("OK" if ok2 else "실패"))
    print("  3) 교차배선을 교환프로브가 0x3C 로 검출            : %s" % ("OK" if ok3 else "실패"))
    print()
    if ok1 and ok2 and ok3:
        print("  ⇒ 펌웨어의 [SWAP] 자동판별 로직이 교차배선을 정확히 잡아낸다. 플래시 가능.")
    else:
        print("  ⇒ 로직에 문제 있음 — 플래시 전 수정 필요.")


if __name__ == '__main__':
    main()
