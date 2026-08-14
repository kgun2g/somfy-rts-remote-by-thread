# -*- coding: utf-8 -*-
"""FreeRTOS tick 주기가 Somfy RF 송신에 미치는 영향 시뮬레이터.

배경 (2026-08-14): ①(tick 100→1000Hz)이 절전에 도움이 안 되고 오히려 소폭
악화로 측정돼(세션#22 47.4mA → #32 52.7mA, 동일 전압구간) 되돌릴지 검토 중.
되돌리기 전에 **RF 송신이 깨지지 않는지** 먼저 확인한다.

모델링 (main/somfy_rts.c 실제 코드):
  _transmit_frame():
     rmt_transmit(...)                       ← 비동기, 큐 깊이 4 (가득차면 블록)
     ms = (us+999)/1000 + 4
     vTaskDelay(pdMS_TO_TICKS(ms))
  us = hw_sync*(2520+2520) + (4752+632) + 80*2*632 + 5448
     첫 프레임 hw_sync=12 → 172,432us / 이후 hw_sync=6 → 142,192us
  ※ inter-frame gap(5448us)은 RMT 심볼 안에 포함 → 프레임끼리 붙여 내보내면
    전파상 '연속'이다. 문제는 소프트가 하드웨어보다 앞서거나 뒤처질 때다.

★ pdMS_TO_TICKS 는 **정수 나눗셈으로 내림**한다:
     pdMS_TO_TICKS(147) @1000Hz = 147 tick = 147ms
     pdMS_TO_TICKS(147) @ 100Hz =  14 tick = 140ms   ← 프레임(142.192ms)보다 짧다
  vTaskDelay 는 tick 격자에서 깨어나므로 격자 위상까지 모델링한다.
"""
import sys

T_SYMBOL = 644            # 2026-08-14 정품 실측 갱신
T_HWSYNC = 2520 + 2520
T_SWSYNC = 4840 + 644     # 2026-08-14 정품 실측 갱신
T_INTERF = 3916           # 2026-08-14 정품 실측 갱신
QUEUE_DEPTH = 4          # rmt_tx_channel_config_t.trans_queue_depth
MIN_LOOPS = 2            # SOMFY_REPEAT_COUNT (2026-08-14 3→2, 정품 실측 2프레임)


def frame_us(hw_sync):
    return hw_sync * T_HWSYNC + T_SWSYNC + 80 * 2 * T_SYMBOL + T_INTERF


def wait_ms(hw_sync):
    # ★2026-08-14 `+999` 올림과 `+4` 여유 제거 → 내림.
    #  대기 < 프레임 이라 RMT 가 프레임을 붙여 내보낸다(프레임간 여분 0).
    return frame_us(hw_sync) // 1000


class Sim:
    def __init__(self, hz):
        self.hz = hz
        self.tick_us = 1000000 // hz
        self.t = 0                  # 현재시각 us
        self.hw_free = 0            # 하드웨어가 큐를 다 비우는 시각
        self.pending = []           # 각 전송의 (start, end)
        self.air = []               # 실제 전파 구간

    def delay_ms(self, ms):
        """vTaskDelay(pdMS_TO_TICKS(ms)) — 내림 + tick 격자 기상."""
        ticks = ms * self.hz // 1000
        if ticks == 0:
            return 0                # vTaskDelay(0) = yield, 대기 없음 ★
        cur = self.t // self.tick_us
        wake = (cur + ticks) * self.tick_us
        d = wake - self.t
        self.t = wake
        return d

    def rmt_transmit(self, hw_sync):
        # 큐가 가득 차 있으면 한 자리 날 때까지 블록
        self.pending = [p for p in self.pending if p[1] > self.t]
        if len(self.pending) >= QUEUE_DEPTH:
            self.t = self.pending[0][1]
            self.pending = [p for p in self.pending if p[1] > self.t]
        start = max(self.t, self.hw_free)
        end = start + frame_us(hw_sync)
        self.hw_free = end
        self.pending.append((start, end))
        self.air.append((start, end))

    def run(self, hold_ms, release_at_ms):
        """hold_ms = _send_command_ex 에 넘긴 값. release_at_ms 에 abort=true."""
        min_loops = MIN_LOOPS
        max_loops = min_loops + hold_ms // 143
        settle = self.delay_ms(5)        # ★ VCO/PA settle (somfy_rts.c:339)
        loops = 0
        for i in range(max_loops):
            aborted = (self.t >= release_at_ms * 1000)
            if i >= min_loops and aborted:
                break
            hw = 12 if i == 0 else 6
            self.rmt_transmit(hw)
            loops += 1
            self.delay_ms(wait_ms(hw))
        return settle, loops


def report(hz, hold_ms, release_ms):
    s = Sim(hz)
    settle, loops = s.run(hold_ms, release_ms)
    air = s.air
    gaps = [air[i + 1][0] - air[i][1] for i in range(len(air) - 1)]
    tail = (s.hw_free - release_ms * 1000) / 1000.0
    maxq = 0
    for st, en in air:
        q = sum(1 for a, b in air if a <= st < b)
        maxq = max(maxq, q)
    print('  %4dHz  settle %2dms  프레임 %3d개  프레임간 틈 최대 %5.2fms  '
          '뗀 뒤 전파 꼬리 %6.1fms  큐최대 %d'
          % (hz, settle // 1000, loops,
             max(gaps) / 1000.0 if gaps else 0.0, tail, maxq))
    return settle, max(gaps) if gaps else 0, tail


CASES = [(15000, 1000), (15000, 3000), (15000, 5000), (15000, 10000), (15000, 15000)]

if __name__ == '__main__':
    print('프레임 실소요: 첫 %0.3fms (hw_sync=12) / 이후 %0.3fms (hw_sync=6)'
          % (frame_us(12) / 1000.0, frame_us(6) / 1000.0))
    print('요청 대기값  : 첫 %dms / 이후 %dms'
          % (wait_ms(12), wait_ms(6)))
    for hz in (1000, 100):
        t = wait_ms(6) * hz // 1000
        print('  @%4dHz → pdMS_TO_TICKS(%d) = %d tick = %dms  %s'
              % (hz, wait_ms(6), t, t * 1000 // hz,
                 '(프레임보다 짧다 ★)' if t * 1000 // hz < frame_us(6) / 1000.0 else ''))
    print()
    bad = 0
    for hold, rel in CASES:
        print('■ 누름 %.1f초 (hold_ms=%d)' % (rel / 1000.0, hold))
        for hz in (1000, 100):
            settle, gap, tail = report(hz, hold, rel)
            if hz == 100:
                if settle == 0:
                    bad += 1        # settle 소멸 = 첫 프레임 깨짐
        print()
    print('=== 100Hz 에서 settle 이 사라진 케이스: %d / %d ===' % (bad, len(CASES)))
    sys.exit(1 if bad else 0)
