# -*- coding: utf-8 -*-
"""태스크 통합 후 호출 주기가 원래와 같은지 검증.

2026-08-20 사용자 요청("task 가 너무 많지 않어?")으로 `time_update`(2048B)와
`time_persist`(3072B)를 메인 루프(100ms)로 흡수했다. 흡수하면 **호출 시점이
100ms 격자에 맞춰 정렬**되므로, 원래 주기와 어긋나지 않는지 확인한다.

원래:
  time_update  : ulTaskNotifyTake 타임아웃 = 화면 ON 1000ms / OFF 300000ms
                 (+ 화면이 켜질 때 통지로 즉시 깨움)
  time_persist : vTaskDelay(300000ms)
흡수 후:
  메인 루프(100ms)가 매 틱마다 경과를 검사해 같은 간격에 호출.
  통지 대신 **화면이 켜지면 간격이 1000ms 로 바뀌어 즉시 조건 성립** — 지연은
  최대 메인 루프 1틱(100ms).
"""
MAIN_MS = 100


def run(dur_ms, panel_on_at=None, panel_off_at=None):
    """흡수 후 호출 시각 목록을 만든다."""
    tick_last = 0
    pers_last = 0
    tick_calls, pers_calls = [], []
    t = MAIN_MS
    while t <= dur_ms:
        on = (panel_on_at is not None and t >= panel_on_at and
              (panel_off_at is None or t < panel_off_at))
        iv = 1000 if on else 300000
        if tick_last == 0 or (t - tick_last) >= iv:
            tick_last = t
            tick_calls.append(t)
        if pers_last == 0:
            pers_last = t
        elif (t - pers_last) >= 300000:
            pers_last = t
            pers_calls.append(t)
        t += MAIN_MS
    return tick_calls, pers_calls


def gaps(xs):
    return [b - a for a, b in zip(xs, xs[1:])]


if __name__ == '__main__':
    print('① 화면 ON 30초 — 원래 1000ms 주기여야 한다')
    tc, _ = run(30000, panel_on_at=0)
    g = gaps(tc)
    print('   호출 %d회, 간격 최소 %d / 최대 %dms  → %s'
          % (len(tc), min(g), max(g),
             'OK' if min(g) == max(g) == 1000 else '★어긋남'))

    print('② 화면 OFF 20분 — 원래 300000ms(5분) 주기여야 한다')
    tc, _ = run(1200000)
    g = gaps(tc)
    print('   호출 %d회, 간격 %s  → %s'
          % (len(tc), set(g), 'OK' if g and set(g) == {300000} else '★어긋남'))

    print('③ 화면 OFF 로 10분 있다가 켜짐 — 통지 대신 몇 ms 만에 갱신되나')
    tc, _ = run(700000, panel_on_at=600000)
    late = [x for x in tc if x >= 600000]
    d = (late[0] - 600000) if late else -1
    print('   켜진 뒤 첫 호출까지 %dms  → %s (요구: 메인 루프 1틱 이내 = 100ms)'
          % (d, 'OK' if 0 <= d <= MAIN_MS else '★지연 과다'))

    print('④ time_persist — 5분 주기, 부팅 직후엔 저장하지 않아야 한다')
    _, pc = run(1500000)
    g = gaps(pc)
    print('   저장 %d회, 첫 저장 t=%ds, 간격 %s  → %s'
          % (len(pc), pc[0] // 1000 if pc else -1, set(g) if g else '-',
             'OK' if pc and pc[0] >= 300000 and (not g or set(g) == {300000})
             else '★어긋남'))

    print()
    print('회수: RAM 5,120B (2048+3072), 깨어남 ~1회/초 (메인 루프가 이미 돌고 있어')
    print('      추가 깨어남 0). 합치지 않은 것: hold_repeat(→abort 가 송신 중에도')
    print('      돌아야 함), oled_ui(prio 3 격리가 버그 수정으로 확정된 값).')
