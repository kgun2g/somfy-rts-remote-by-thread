# -*- coding: utf-8 -*-
"""deep sleep 대조군(`dsleep`) — 상태기계 + 측정 분해능 사전 검증.

왜 필요한가 (2026-09-11)
────────────────────────
CHG_STAT 웨이크를 잡은 뒤 C6 는 3.05mA(약 5~6일)다. 목표 15일 = 1.94mA.
그런데 지금 **96.3% 가 수면**이라 깨어남을 0으로 만들어도 목표에 못 간다.
남은 것은 수면 전류인데 그 값을 **한 번도 잰 적이 없다**. 전류계 직렬 측정은
이 기판에서 불가능하다(배터리 선을 못 끊고, +/- 직결은 셀 단락).

그래서 **deep sleep 을 대조군으로 쓴다.** 깊은잠은 칩이 ~10uA 만 먹으므로:
    깊은잠 강하 ~= 얕은잠 강하  ->  범인은 **기판**(LED/LDO/누설). 펌웨어 무력.
    깊은잠 강하 ~= 0           ->  범인은 **칩**(라디오 등). 펌웨어가 듣는다.

★설계상 딜레마와 해법 (이거 때문에 설계를 한 번 갈아엎었다)
────────────────────────────────────────────────────────────
처음엔 "USB 꽂힌 상태면 명령 거부"로 만들었다. 그런데 **콘솔이 USB 로만 된다.**
즉 사용자가 이 명령을 영원히 못 쓴다. 실기에 올려서 `dsleep 720` 을 쳐 보고서야
드러났다(거부 메시지만 나옴).

해법 = **예약과 실행을 분리한다.**
    1) `dsleep <분>`  : USB 꽂힌 채로도 **예약만** 한다(armed=1, phase=0).
    2) 사용자가 USB 를 뽑는다.
    3) 메인 루프가 USB 없어짐을 보고 SETTLE 초 기다린 뒤 **스스로 재부팅**한다.
    4) 부팅 훅이 mv0 를 재고 깊은잠에 든다(phase=1).
    5) 타이머로 깨어나 같은 부팅 훅에서 mv1 을 잰다.

★왜 굳이 재부팅을 끼우나: 두 전압을 **같은 부하 지점**에서 재야 뺄셈이 의미를
갖는다. 자기 직전을 풀로드(라디오 ON)에서, 깬 뒤를 부팅 중(저부하)에서 재면
단자전압 회복분이 섞여 강하를 **과소평가**한다(= 거짓으로 "칩이 범인" 결론).

이 시뮬레이터가 확인하는 것
───────────────────────────
 ① 어떤 사고에도 **영구 잠듦 / 무한 재부팅 / 거짓 결과**가 안 난다
 ② USB 를 다시 꽂으면 예약이 발동하지 않는다
 ③ ★몇 시간을 재워야 두 가설이 갈리는가 (ADC 잡음 대비)
"""
import sys

# ── 펌웨어와 공유할 상수 (somfy_app.c 와 일치시킬 것) ────────────────
DUR_MIN_M, DUR_MAX_M = 1, 720          # 1분 ~ 12시간
SETTLE_S = 60                          # 기본 정착(초). 2026-09-14 부터 런타임 인자.
SETTLE_MAX_M = 240
# ★★셀 완화(relaxation): 2분 리허설에서 COM7 +18mV / COM8 -26mV 가 나왔다.
#   부하가 사라지면 단자전압이 OCV 로 회복하고, 방금 충전을 끝냈으면 표면전하가 빠진다.
#   그 +-20mV 가 12시간 신호(A -18mV / B -0.5mV)보다 **크다** -> mv0 전에 쉬게 해야 한다.
WAKE_TIMER, WAKE_OTHER = 'TIMER', 'OTHER'


class Dev:
    """NVS 4값(armed/phase/dur_s/mv0) 상태기계."""

    def __init__(self):
        self.nvs = {'armed': 0, 'phase': 0, 'dur_s': 0, 'mv0': 0, 'settle_s': SETTLE_S}
        # ★★RAM 플래그를 NVS 와 분리해서 모델링한다.
        #   처음 시뮬은 tick() 이 self.nvs['armed'] 를 **직접** 읽어서, 펌웨어의
        #   "부팅당 1회만 NVS 를 읽는 캐시"를 재현하지 못했다. 그래서 실기에서
        #   "부팅 뒤 콘솔로 예약하면 감시가 영영 안 돈다"는 버그를 못 잡고
        #   12시간을 날렸다. 이제 tick() 은 ram_armed 만 본다.
        self.ram_armed = 0
        self.log = []
        self.result = None
        self.boots = 0
        self.restarts = 0
        self.off_s = 0          # USB 빠진 뒤 경과(메인 루프가 셈)

    # ── 콘솔 `dsleep <분>` : 0 이면 취소 ──
    def cmd(self, minutes, settle_min=1):
        if minutes == 0:
            self.nvs.update(armed=0, phase=0)
            self.ram_armed = 0
            self.log.append('예약 취소')
            return
        if not (DUR_MIN_M <= minutes <= DUR_MAX_M):
            self.log.append('거부: %d분은 범위(%d~%d) 밖' % (minutes, DUR_MIN_M, DUR_MAX_M))
            return
        if self.nvs['armed']:
            self.log.append('거부: 이미 예약됨 (취소는 dsleep 0)')
            return
        # ★USB 꽂힌 채로도 받는다 — 콘솔이 USB 로만 되기 때문이다.
        settle_min = max(1, min(SETTLE_MAX_M, settle_min))
        self.nvs.update(armed=1, phase=0, dur_s=minutes * 60,
                        settle_s=settle_min * 60)
        self.ram_armed = 1                  # ★이 한 줄이 실기에서 빠져 있었다
        self.log.append('예약 %d분 — 이제 USB 를 뽑으면 %d초 뒤 자동 시작' % (minutes, SETTLE_S))

    # ── 메인 루프(100ms) — USB 없어짐 감시 ──
    def tick(self, usb, dt_s=1):
        if not self.ram_armed:              # ★NVS 가 아니라 RAM 플래그를 본다
            self.off_s = 0
            return None
        if usb:
            self.off_s = 0                      # 다시 꽂히면 리셋
            return None
        self.off_s += dt_s
        if self.off_s >= self.nvs['settle_s']:
            self.restarts += 1
            self.log.append('USB 빠짐 %ds 확인 — 재부팅해 측정 시작' % self.off_s)
            self.off_s = 0
            return 'RESTART'
        return None

    # ── 부팅 초기 훅 (배터리 mV 를 막 읽은 지점) ──
    def on_boot(self, mv, wake_cause, usb):
        self.boots += 1
        if not self.nvs['armed']:
            return 'RUN'
        if self.nvs['phase'] == 0:
            if usb:
                # 예약은 살아 있으나 아직 USB — 정상 부팅하고 메인 루프가 지켜본다
                self.ram_armed = 1          # ★부팅 시점에 이미 예약돼 있던 경우
                self.log.append('부팅: 예약 있음, 아직 USB — 대기')
                return 'RUN'
            self.nvs['mv0'] = mv
            self.nvs['phase'] = 1              # ★커밋 뒤에 잠들어야 한다
            self.log.append('phase0: mv0=%d 기록, %ds 깊은잠' % (mv, self.nvs['dur_s']))
            return 'DEEPSLEEP'
        # phase 1 — 깨어났다. 무슨 일이 있어도 예약부터 해제한다.
        self.nvs['armed'] = 0
        self.ram_armed = 0
        if wake_cause != WAKE_TIMER:
            self.log.append('phase1: 기상원인 %s — **무효**(타이머 아님). 해제만 하고 진행'
                            % wake_cause)
            return 'RUN'
        dmv = mv - self.nvs['mv0']
        hrs = self.nvs['dur_s'] / 3600.0
        self.result = (dmv, hrs)
        self.log.append('phase1: mv1=%d  d=%+dmV / %.2fh = %+.3f mV/h'
                        % (mv, dmv, hrs, dmv / hrs))
        return 'RUN'


def _run(name, script):
    """script: ('CMD', n) / ('TICK', usb, secs) / ('BOOT', mv, wake, usb)"""
    d = Dev()
    print('\n' + '=' * 74)
    print('케이스: %s' % name)
    print('=' * 74)
    for step in script:
        if step[0] == 'CMD':
            d.cmd(*step[1:])
        elif step[0] == 'TICK':
            _, usb, secs = step
            for _ in range(secs):
                if d.tick(usb) == 'RESTART':
                    break
        else:
            _, mv, wc, usb = step
            d.on_boot(mv, wc, usb)
    for l in d.log:
        print('   ' + l)
    print('   → 부팅 %d회, 자발재부팅 %d회, armed=%d, 결과=%s'
          % (d.boots, d.restarts, d.nvs['armed'], d.result))
    assert d.restarts <= 1, '★자발 재부팅이 2회 이상 = 무한 재부팅 위험'
    return d


def main():
    # ① 정상 — USB 에서 예약 → 뽑음 → 자동 시작 -------------------------
    d = _run('정상: USB 에서 예약 → 뽑음 → 12시간',
             [('CMD', 720), ('TICK', True, 30), ('TICK', False, 120),
              ('BOOT', 4000, WAKE_OTHER, False), ('BOOT', 3982, WAKE_TIMER, False)])
    assert d.result == (-18, 12.0) and d.restarts == 1

    # ② 예약해 놓고 USB 를 안 뽑음 → 아무 일도 없어야 한다 ---------------
    d = _run('예약 후 계속 USB 연결', [('CMD', 720), ('TICK', True, 600)])
    assert d.restarts == 0 and d.result is None

    # ③ 뽑았다가 SETTLE 전에 다시 꽂음 → 발동 안 함 ---------------------
    d = _run('뽑았다 다시 꽂음(SETTLE 이내)',
             [('CMD', 720), ('TICK', False, 30), ('TICK', True, 5), ('TICK', False, 30)])
    assert d.restarts == 0, '★off_s 가 리셋 안 되면 여기서 잘못 발동한다'

    # ④ 취소 ------------------------------------------------------------
    d = _run('예약 후 취소', [('CMD', 720), ('CMD', 0), ('TICK', False, 200)])
    assert d.restarts == 0 and d.nvs['armed'] == 0

    # ⑤ 깊은잠 중 브라운아웃 → 거짓 결과 금지 ---------------------------
    d = _run('깊은잠 중 브라운아웃',
             [('CMD', 720), ('TICK', False, 120),
              ('BOOT', 4000, WAKE_OTHER, False), ('BOOT', 3400, WAKE_OTHER, False)])
    assert d.result is None, '★기상원인 검사가 없으면 거짓 mV/h 가 나온다'
    assert d.nvs['armed'] == 0

    # ⑥ 범위 밖 / 중복 --------------------------------------------------
    d = _run('오타 방어 9999분 + 중복 예약',
             [('CMD', 9999), ('CMD', 720), ('CMD', 60)])
    assert d.nvs['dur_s'] == 720 * 60, '중복 예약이 첫 예약을 덮어쓰면 안 된다'

    # ⑦ ★회귀: 부팅한 **뒤** 콘솔로 예약 → 감시가 반드시 돌아야 한다.
    #    2026-09-11 실기 실패 그대로: 부팅(예약 없음) → dsleep 720 → USB 뽑음.
    #    RAM 플래그를 안 세우면 여기서 restarts==0 이 되어 12시간을 날린다.
    d = _run('★회귀: 부팅 후 콘솔 예약 → 뽑음',
             [('BOOT', 4100, WAKE_OTHER, True),   # 평범한 부팅, 아직 예약 없음
              ('TICK', True, 10), ('CMD', 720),   # 부팅 뒤에 예약
              ('TICK', False, 120),
              ('BOOT', 4000, WAKE_OTHER, False), ('BOOT', 3982, WAKE_TIMER, False)])
    assert d.restarts == 1, '★부팅 후 예약이 발동하지 않는다 — 12시간을 날린 그 버그다'
    assert d.result == (-18, 12.0)

    # ⑧ ★정착 1시간(COM7용) — 59분엔 안 되고 60분엔 되어야 한다
    d = _run('정착 60분: 59분 경과 → 발동 안 함',
             [('CMD', 720, 60), ('TICK', False, 59 * 60)])
    assert d.restarts == 0, '★정착 전에 발동하면 완화 잡음이 그대로 섞인다'
    d = _run('정착 60분: 60분 경과 → 발동',
             [('CMD', 720, 60), ('TICK', False, 61 * 60),
              ('BOOT', 3900, WAKE_OTHER, False), ('BOOT', 3882, WAKE_TIMER, False)])
    assert d.restarts == 1 and d.result == (-18, 12.0)

    # ⑨ 정착 인자 클램프 (0분·9999분)
    d = _run('정착 클램프', [('CMD', 720, 0)])
    assert d.nvs['settle_s'] == 60, '0분은 1분으로 올려야 한다'
    d = _run('정착 클램프 상한', [('CMD', 720, 9999)])
    assert d.nvs['settle_s'] == SETTLE_MAX_M * 60

    # ── ③ 분해능 ────────────────────────────────────────────────────────
    print('\n' + '=' * 74)
    print('★몇 시간을 재워야 결론이 나는가 (ADC 잡음 대비)')
    print('=' * 74)
    LS_MVH, LS_MA = 2.79, 3.05          # 얕은잠 실측(COM8 2026-09-11)
    k = LS_MVH / LS_MA                  # mV/h per mA
    A_MA, B_MA = 1.60, 0.05             # 가설 A(기판) / B(칩)
    NOISE_MV = 3.0                      # 8표본 평균 x 2회, 보수적
    print('  기준선 얕은잠 %.2f mV/h = %.2f mA  ->  1mA ~= %.3f mV/h' % (LS_MVH, LS_MA, k))
    print('  차분 잡음 가정 %.1f mV,  판정선 = 3-sigma' % NOISE_MV)
    print()
    print('  %6s | %9s | %9s | %9s | 판정' % ('시간', 'A 강하', 'B 강하', '간격'))
    print('  ' + '-' * 60)
    need = None
    for h in (1, 2, 4, 6, 8, 10, 12):
        a, b = A_MA * k * h, B_MA * k * h
        ok = (a - b) >= 3 * NOISE_MV
        if ok and need is None:
            need = h
        print('  %5dh | %8.1fmV | %8.2fmV | %8.1fmV | %s'
              % (h, -a, -b, a - b, '**갈림**' if ok else '부족'))
    print()
    print('  → 최소 %dh. 여유 있게 **12시간**(기본값)을 쓴다.' % need)
    print('    12h 면 A 는 -%.0fmV, B 는 -%.1fmV — 눈으로도 구분된다.'
          % (A_MA * k * 12, B_MA * k * 12))
    print('\n모든 케이스 통과.')
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
