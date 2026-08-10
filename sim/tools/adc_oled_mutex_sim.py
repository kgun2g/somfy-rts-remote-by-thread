#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADC / 화면제어 <-> OLED 비트뱅 직렬화 시뮬레이터
================================================

목적
----
충전률 측정(`_read_bat_mv`)을 되살렸을 때 OLED 비트뱅 I2C 전송이 깨지는지를
**플래시 전에** 판정한다. 프로젝트 규칙(빌드/플래시 전 파이썬 검증)에 따른다.

모델링한 것
-----------
ESP32-C6 는 **싱글코어**다. 따라서 "동시 실행"이 아니라 **선점(preemption)** 이
파괴 기전이다. 실제 태스크 우선순위(펌웨어에서 확인):

    btn_handler  prio 10   PCF 비트뱅  (oled_ui_i2c_lock 사용 -> 보호됨)
    somfy_app    prio  4   ADC 8회 읽기 + oled_ui_set_display_on()
    oled_ui      prio  3   OLED 비트뱅 flush

somfy_app(4) > oled_ui(3) 이므로 **somfy_app 이 OLED 전송 도중에 끼어든다.**
비트뱅은 CPU 가 곧 클럭이라, 선점당한 전송은 SCL/SDA 가 중간 상태로 멈춘 채
수백 us 늘어난다 -> SSD1306 이 프레임을 잘못 받거나 상태머신이 고착된다.
(`adc_oneshot_read` 의 portENTER_CRITICAL 은 이 구간을 인터럽트로도 되돌릴 수
없게 만들어 악화시키는 요소다. 근본은 선점 그 자체다.)

손상 판정
---------
전송(`_bbo_write`) 한 건이 진행 중일 때 **다른 태스크가 실제로 CPU 를 쓴 시간**을
누적하고, STRETCH_LIMIT_US 를 넘으면 손상으로 센다.
락에 막혀 즉시 블록되는 것은 CPU 를 안 쓰므로 손상이 아니다(우선순위 상속으로
보유자가 곧바로 이어서 끝낸다) -- 이것이 두 구성을 가르는 지점이다.

비교 구성
---------
  A. 현재 (flush 만 락)        : oled_ui_i2c_lock 이 `_fb_flush` 만 감쌈
  B. 수정 (_bbo_write 가 락)   : 전송 함수 자체가 재귀 뮤텍스를 잡음
  C. 뮤텍스 없음 (참고)        : 보호가 전혀 없을 때의 상한

실행: python sim/tools/adc_oled_mutex_sim.py
"""

import sys

MS = 1000
SEC = 1000 * 1000

# ── 실측/계산 기반 타이밍 ────────────────────────────────────────────────────
BBO_HALF_US = 1                      # oled_ui.c 의 BBO_HALF_US
BYTE_US = 9 * 2 * BBO_HALF_US        # 8bit + ACK, 비트당 2xHALF = 18us/byte
START_STOP_US = 6


def xfer_us(payload_bytes):
    """_bbo_write(addr7, buf, len) 한 건의 소요 시간(us). addr 1바이트 포함."""
    return (payload_bytes + 1) * BYTE_US + START_STOP_US


XFER_CMD = xfer_us(4)        # 컬럼/페이지 주소 4바이트            -> ~96us
XFER_DATA = xfer_us(33)      # 0x40 + 32바이트 청크                -> ~618us
XFER_CMDS = xfer_us(2)       # _oled_send_cmds(0xAE/0xAF)          -> ~60us
XFER_PROBE = xfer_us(0)      # _bbo_probe (주소 바이트만)          -> ~24us
PROBE_PERIOD = 5 * SEC       # 미검출 시 재검출 주기(oled_ui.c:1109)

ADC_ONE_US = 50              # adc_oneshot_read 1회(크리티컬 포함) 근사
ADC_N = 8                    # _read_bat_mv 의 평균 횟수

OLED_PERIOD = 50 * MS        # _ui_task flush 주기
DIRTY_PAGES = 2              # dirty-page 도입 후 평균 갱신 페이지(실측 86% skip)
BAT_PERIOD = 5 * SEC         # 배터리 측정 주기
SCREEN_PERIOD = 10 * SEC     # 화면 자동 OFF/ON (CFG_SCREEN_OFF_SEC=10)
BTN_PERIOD = 10 * MS         # btn_handler 폴링
BTN_CPU_US = 300             # PCF 읽기 1회

STRETCH_LIMIT_US = 20        # 이 이상 늘어나면 프레임 손상으로 간주
SIM_TIME = 600 * SEC         # 10분

# ★지터 -- 이게 없으면 결과가 거짓이 된다.
#   주기가 5s / 50ms / 10ms 로 전부 정수배라, 지터 0 이면 somfy_app 의 기상 시점이
#   oled_ui 프레임에 대해 **항상 같은 위상**에 떨어져 충돌이 구조적으로 회피된다
#   (첫 시뮬에서 구성 A 손상 0 이 나온 원인 -- 모델 인공물이었다).
#   실제 FreeRTOS 는 틱 정렬·인터럽트·가변 처리시간으로 위상이 계속 흐른다.
JITTER_US = 3 * MS           # 주기마다 +-1.5ms 범위로 위상 흔들기


class Rng(object):
    """재현 가능한 선형합동 난수(외부 의존성 없이 매번 같은 결과)."""

    def __init__(self, seed=12345):
        self.s = seed

    def next(self, n):
        self.s = (self.s * 1103515245 + 12345) & 0x7FFFFFFF
        return self.s % n if n else 0

    def jitter(self, span):
        return self.next(span) - span // 2


# ── 초소형 선점형 스케줄러 ───────────────────────────────────────────────────
class Task(object):
    def __init__(self, name, prio, gen_fn, sim):
        self.name = name
        self.base_prio = prio
        self.prio = prio
        self.gen = gen_fn(self)
        self.op = None          # ('cpu', us, tag) / ('delay', us) / ('lock', to) / ('unlock',)
        self.remain = 0
        self.wake_at = None
        self.blocked = False
        self.stretch = 0        # 현재 cpu 구간이 남에게 뺏긴 시간(us)
        self.started = False    # 현재 cpu 구간을 한 번이라도 실행했는가
        self.sim = sim


class Sim(object):
    def __init__(self, label, mutex_enabled=True, bbo_locks=False, seed=20260811):
        self.label = label
        self.seed = seed
        self.mutex_enabled = mutex_enabled   # False = 보호 전무(구성 C)
        self.bbo_locks = bbo_locks           # True  = _bbo_write 가 직접 락(구성 B)
        self.now = 0
        self.owner = None
        self.depth = 0
        self.waiters = []
        self.tasks = []
        self.rng = Rng(seed)          # 구성별로 동일 시드 -> 공정 비교
        self.present = True           # SSD1306 응답 여부(고착되면 False)
        self.st = {"xfer": 0, "corrupt": 0, "adc_ok": 0, "adc_skip": 0,
                   "worst_stretch": 0, "wedge": 0, "recover": 0,
                   "first_wedge_us": None, "wedged_us": 0}

    # ---- 뮤텍스 (재귀 획득 지원) ----
    def _try_lock(self, t):
        if not self.mutex_enabled:
            return True
        if self.owner is None:
            self.owner, self.depth = t, 1
            return True
        if self.owner is t:              # 재귀 획득 -- flush 안에서 _bbo_write 가 부를 때
            self.depth += 1
            return True
        return False

    def _unlock(self, t):
        if not self.mutex_enabled:
            return
        if self.owner is not t:
            return
        self.depth -= 1
        if self.depth:
            return
        self.owner = None
        t.prio = t.base_prio             # 우선순위 상속 해제
        if self.waiters:
            w = max(self.waiters, key=lambda x: x.base_prio)
            self.waiters.remove(w)
            w.blocked = False
            w.wake_at = None
            self.owner, self.depth = w, 1
            self._advance(w, send=True)

    # ---- 오퍼레이션 진행 ----
    def _advance(self, t, send=None):
        while True:
            try:
                op = t.gen.send(send) if send is not None else next(t.gen)
            except StopIteration:
                t.op, t.wake_at, t.remain = None, None, 0
                return
            send = None
            kind = op[0]
            if kind == "cpu":
                t.op, t.remain = op, op[1]
                t.stretch, t.started = 0, False
                t.wake_at = None
                return
            if kind == "delay":
                # 위상 고정을 깨는 지터(위 JITTER_US 주석 참조)
                t.op, t.remain = op, 0
                t.wake_at = self.now + op[1] + self.rng.jitter(JITTER_US)
                return
            if kind == "lock":
                if self._try_lock(t):
                    send = True
                    continue
                if op[1] == 0:
                    send = False
                    continue
                t.blocked = True
                self.waiters.append(t)
                if self.owner is not None and t.base_prio > self.owner.prio:
                    self.owner.prio = t.base_prio     # 우선순위 상속
                t.op, t.wake_at = op, self.now + op[1]  # 타임아웃
                return
            if kind == "unlock":
                self._unlock(t)
                continue

    def add(self, name, prio, gen_fn):
        t = Task(name, prio, gen_fn, self)
        self.tasks.append(t)
        self._advance(t)
        return t

    def _finish_cpu(self, t):
        tag = t.op[2]
        if tag in ("xfer", "probe"):
            self.st["xfer"] += 1
            if t.stretch > self.st["worst_stretch"]:
                self.st["worst_stretch"] = t.stretch
            broken = t.stretch >= STRETCH_LIMIT_US
            if broken:
                self.st["corrupt"] += 1
            # ★래치 모델 -- 이것이 "몇 분 내 멈춤 후 영영 안 돌아옴"의 정체다.
            #   늘어난 전송 1건이 SSD1306 상태머신을 고착시키면 s_oled_present=false 가
            #   되고, 그 뒤 flush 는 바깥 락을 잡기 전에 return 해 **락 없이** 재검출
            #   probe 만 5초마다 돈다. 그 probe 마저 계속 깨지면 영구 정지.
            if tag == "xfer" and broken and self.present:
                self.present = False
                self._wedge_t0 = self.now
                self.st["wedge"] += 1
                if self.st["first_wedge_us"] is None:
                    self.st["first_wedge_us"] = self.now
            elif tag == "probe" and not broken and not self.present:
                self.present = True          # 깨끗한 probe 1회면 재검출 성공
                self.st["recover"] += 1
                self.st["wedged_us"] += self.now - self._wedge_t0
        t.remain = 0
        self._advance(t)

    def run(self):
        while self.now < SIM_TIME:
            # 1) 만료된 타이머 처리 (delay 종료 / lock 타임아웃)
            progressed = True
            # ★같은 시각에 여러 태스크가 깨어나면 **우선순위 높은 쪽이 먼저** 락을
            #   시도해야 실제 FreeRTOS 와 같다(낮은 쪽이 먼저 채가면 거짓 기아 발생).
            order = sorted(self.tasks, key=lambda x: -x.base_prio)
            while progressed:
                progressed = False
                for t in order:
                    if t.op is None or t.wake_at is None or t.wake_at > self.now:
                        continue
                    if t.blocked:                     # 락 타임아웃
                        self.waiters.remove(t)
                        t.blocked = False
                        t.wake_at = None
                        self._advance(t, send=False)
                    else:                             # delay 종료
                        self._advance(t)
                    progressed = True

            # 2) 실행 가능한 CPU 태스크 중 최고 우선순위 선택
            runnable = [t for t in self.tasks
                        if t.op is not None and t.op[0] == "cpu" and not t.blocked]
            if not runnable:
                nxt = [t.wake_at for t in self.tasks
                       if t.op is not None and t.wake_at is not None
                       and t.wake_at > self.now]
                if not nxt:
                    break
                self.now = min(nxt)
                continue
            cur = max(runnable, key=lambda x: x.prio)

            # 3) 다음 타이머 이벤트까지만 실행(그 시점에 스케줄러가 다시 판단)
            evts = [t.wake_at for t in self.tasks
                    if t.op is not None and t.wake_at is not None
                    and t.wake_at > self.now]
            limit = min(evts) - self.now if evts else cur.remain
            ran = min(cur.remain, limit)
            if ran <= 0:
                ran = cur.remain          # 이벤트가 지금이면 그냥 진행(무한루프 방지)

            # 4) ★핵심: 이 CPU 실행이 "이미 시작된 남의 전송"을 늘린다
            for t in self.tasks:
                if t is cur or t.op is None or t.op[0] != "cpu":
                    continue
                if t.started and t.remain > 0:
                    t.stretch += ran

            cur.started = True
            cur.remain -= ran
            self.now += ran
            if cur.remain <= 0:
                self._finish_cpu(cur)
        if not self.present:                 # 끝까지 고착이면 남은 시간도 계상
            self.st["wedged_us"] += self.now - self._wedge_t0
        return self.st


# ── 태스크 본체 ──────────────────────────────────────────────────────────────
def build(sim):
    LOCK_WAIT = 100 * MS          # _bbo_write 의 바운디드 대기

    def bbo_write(us, tag="xfer"):
        """비트뱅 전송 1건. 구성 B 에서는 여기서 락을 잡는다.

        ※호출부는 반드시 `yield from` 을 쓸 것. `for op in gen: yield op` 로 하면
          send() 값이 전달되지 않아 `ok` 가 None 이 되고 **unlock 이 실행되지 않는다**
          (뮤텍스가 영구 점유되어 배터리 측정이 100% 실패하는 가짜 결과가 나왔었다).
        """
        if sim.bbo_locks:
            ok = yield ("lock", LOCK_WAIT)
            yield ("cpu", us, tag)
            if ok:
                yield ("unlock",)
        else:
            yield ("cpu", us, tag)

    # oled_ui (prio 3) -- 50ms 주기 flush
    def oled_ui(t):
        last_probe = [-PROBE_PERIOD]
        while True:
            yield ("delay", OLED_PERIOD)

            if not sim.present:
                # ★_fb_flush 의 early-return 경로(oled_ui.c:1105).
                #   바깥 락을 **잡기 전에** return 하므로 재검출 probe 는 무방비다.
                #   구성 B 는 _bbo_write 안에서 락을 잡아 여기까지 보호된다.
                if sim.now - last_probe[0] >= PROBE_PERIOD:
                    last_probe[0] = sim.now
                    for _ in range(4):                 # _oled_try_detect 의 4회 재시도
                        yield from bbo_write(XFER_PROBE, "probe")
                        if sim.present:
                            break
                        yield from bbo_write(XFER_PROBE, "probe")   # 0x3D 도 시도
                        if sim.present:
                            break
                        yield ("delay", 10 * MS)
                continue

            yield ("lock", 0xFFFFFFF)          # _fb_flush 바깥 락(A/B 모두 유지)
            for _ in range(DIRTY_PAGES):
                yield from bbo_write(XFER_CMD)
                for _ in range(4):             # 128바이트를 32씩 4청크
                    yield from bbo_write(XFER_DATA)
            yield ("unlock",)

    # somfy_app (prio 4) -- 5초마다 ADC, 10초마다 화면 OFF/ON
    def somfy_app(t):
        n = 0
        while True:
            yield ("delay", BAT_PERIOD)
            n += 1
            # _read_bat_mv(): A/B 모두 trylock(30ms) 후 ADC 8회
            got = yield ("lock", 30 * MS)
            if got:
                yield ("cpu", ADC_N * ADC_ONE_US, "adc")
                yield ("unlock",)
                sim.st["adc_ok"] += 1
            else:
                sim.st["adc_skip"] += 1
            # oled_ui_set_display_on() -> _oled_send_cmds : ★구성 A 에서 무방비
            if n % (SCREEN_PERIOD // BAT_PERIOD) == 0:
                yield from bbo_write(XFER_CMDS)

    # btn_handler (prio 10) -- 10ms 폴링, 이미 락을 잡는다
    def btn(t):
        while True:
            yield ("delay", BTN_PERIOD)
            yield ("lock", 0xFFFFFFF)
            yield ("cpu", BTN_CPU_US, "btn")
            yield ("unlock",)

    sim.add("oled_ui", 3, oled_ui)
    sim.add("somfy_app", 4, somfy_app)
    sim.add("btn_handler", 10, btn)


def main():
    # Windows 콘솔(CP949)에서 죽지 않게
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print("=" * 76)
    print(" ADC / 화면제어 <-> OLED 비트뱅 직렬화 (싱글코어 선점 모델, 10분)")
    print("=" * 76)
    print("  전송 시간 : cmd %dus / data %dus / send_cmds %dus"
          % (XFER_CMD, XFER_DATA, XFER_CMDS))
    print("  ADC       : %d회 x %dus = %dus, %d초 주기"
          % (ADC_N, ADC_ONE_US, ADC_N * ADC_ONE_US, BAT_PERIOD // SEC))
    print("  우선순위  : btn 10 > somfy_app 4 > oled_ui 3   <- 선점 발생 조건")
    print("  손상 기준 : 전송이 %dus 이상 늘어나면 손상" % STRETCH_LIMIT_US)
    print()

    configs = [
        ("A. 현재 (flush 만 락)", dict(mutex_enabled=True, bbo_locks=False)),
        ("B. 수정 (_bbo_write 가 락)", dict(mutex_enabled=True, bbo_locks=True)),
        ("C. 보호 없음 (참고 상한)", dict(mutex_enabled=False, bbo_locks=False)),
    ]
    # ★시드 1개 결과로 결론내지 않는다. 위상 관계가 시드마다 달라 손상 건수가
    #   크게 흔들리므로, 여러 시드를 돌려 합산한다(총 SEEDS x 10분).
    SEEDS = [20260811, 777, 4242, 98765, 31337, 1, 555555, 20250520]
    rows = []
    for label, kw in configs:
        agg = {"xfer": 0, "corrupt": 0, "adc_ok": 0, "adc_skip": 0,
               "worst_stretch": 0, "wedge": 0, "recover": 0,
               "first_wedge_us": None, "wedged_us": 0, "runs_wedged": 0}
        for sd in SEEDS:
            sim = Sim(label, seed=sd, **kw)
            build(sim)
            st = sim.run()
            for k in ("xfer", "corrupt", "adc_ok", "adc_skip", "wedge",
                      "recover", "wedged_us"):
                agg[k] += st[k]
            agg["worst_stretch"] = max(agg["worst_stretch"], st["worst_stretch"])
            if st["first_wedge_us"] is not None:
                agg["runs_wedged"] += 1
                if agg["first_wedge_us"] is None:
                    agg["first_wedge_us"] = st["first_wedge_us"]
                else:
                    agg["first_wedge_us"] = min(agg["first_wedge_us"],
                                                st["first_wedge_us"])
        agg["_runs"] = len(SEEDS)
        rows.append((label, agg))

    print("%-30s%10s%7s%9s%10s%9s"
          % ("구성", "전송", "손상", "고착", "고착된회차", "최빠른고착"))
    print("-" * 76)
    for label, st in rows:
        fw = ("%.0f초" % (st["first_wedge_us"] / 1e6)) if st["first_wedge_us"] else "-"
        print("%-30s%10s%7s%9s%7d/%-3d%9s"
              % (label, "{:,}".format(st["xfer"]), "{:,}".format(st["corrupt"]),
                 "{:,}".format(st["wedge"]), st["runs_wedged"], st["_runs"], fw))
    print()
    print("  %d회 x 10분 = 총 %d분 합산.  '고착' = 화면이 멈춘 횟수." %
          (rows[0][1]["_runs"], rows[0][1]["_runs"] * 10))
    print()

    a, b, c = rows[0][1], rows[1][1], rows[2][1]
    print("판정")
    print("-" * 76)
    if a["corrupt"] > 0 and b["corrupt"] == 0:
        print("  [OK] 수정안 B 가 손상 %d건 -> 0건, 고착 %d회 -> 0회."
              % (a["corrupt"], a["wedge"]))
        print("       구성 A 의 손상 경로 = somfy_app(prio 4)의")
        print("         oled_ui_set_display_on() -> _oled_send_cmds() -> _bbo_write()")
        print("       가 락을 안 잡아, flush 중이던 oled_ui(prio 3)의 전송을 잘라먹는다.")
        print("       (화면 자동 OFF/ON 이 %d초마다 -> 상시 노출)" % (SCREEN_PERIOD // SEC))
        print("       B 는 _bbo_write 가 락을 잡으므로 즉시 블록되고, 우선순위 상속으로")
        print("       oled_ui 가 전송을 끝낸 뒤 넘겨받는다 -> 늘어남 0.")
        print()
        print("       ※이 모델은 고착 후 '깨끗한 probe 1회면 복구'로 낙관 가정했다.")
        print("         실기의 SSD1306 은 9클럭 복구를 반복해야 풀리거나 아예 안 풀린다")
        print("         -> 실제 체감은 모델보다 나쁘다(= '몇 분 뒤 멈춰서 안 돌아옴').")
    elif a["corrupt"] == 0 and b["corrupt"] == 0:
        print("  [?] 두 구성 모두 손상 0 -- 이 파라미터로는 재현되지 않음.")
    else:
        print("  [X] 수정안에도 손상 %d건 -- 설계 재검토 필요." % b["corrupt"])
    print()
    print("  참고) 보호가 전혀 없으면(C) 손상 %s건 = %.1f%%"
          % ("{:,}".format(c["corrupt"]),
             c["corrupt"] / c["xfer"] * 100.0 if c["xfer"] else 0))
    print("        -> 뮤텍스가 NULL 인 구간(oled_ui_init 이전)이 있으면 이 상태가 된다.")
    print()

    print("  배터리 측정 성공률 : A %d/%d  ·  B %d/%d"
          % (a["adc_ok"], a["adc_ok"] + a["adc_skip"],
             b["adc_ok"], b["adc_ok"] + b["adc_skip"]))
    if b["adc_skip"] > b["adc_ok"] * 0.2:
        print("  [!] B 에서 배터리 측정 건너뜀이 잦다 -- trylock 타임아웃을 늘릴 것.")
    else:
        print("  [OK] 측정 주기(5초)/표본 수(8회)를 바꾸지 않아도 기아 없음")
        print("       -> _nobat_track 의 '5분 창 / 반쪽당 30표본' 가정을 건드리지 않는다.")
        print("          (주기를 30초로 늘리면 반쪽당 5표본 -> 노이즈가 문턱 4mV 에 근접해")
        print("           배터리 미연결 오판이 생긴다. 그래서 주기는 그대로 둔다.)")

    print()
    print("재귀 뮤텍스가 필수인 이유")
    print("-" * 76)
    print("  _fb_flush() 가 바깥 락을 쥔 채 _bbo_write() 를 부른다.")
    print("  일반 뮤텍스면 자기 자신을 기다려 즉시 데드락(화면 영구 정지)이다.")
    print("  -> xSemaphoreCreateRecursiveMutex + Take/GiveRecursive 로 바꿔야 한다.")
    dead = Sim("x", mutex_enabled=True, bbo_locks=True)

    class T(object):
        pass
    holder = T()
    dead.owner, dead.depth = holder, 1
    print("  · 재귀 모델에서 중첩 획득 : %s (depth %d)"
          % ("통과" if dead._try_lock(holder) else "데드락", dead.depth))
    other = T()
    print("  · 다른 태스크의 획득 시도 : %s (정상)"
          % ("통과" if dead._try_lock(other) else "블록"))


if __name__ == "__main__":
    main()
