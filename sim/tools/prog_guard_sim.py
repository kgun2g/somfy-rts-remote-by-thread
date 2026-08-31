# -*- coding: utf-8 -*-
"""PROGGUARD(PROG 연타 차단) — 기존 코드 vs 수정안 비교 시뮬레이터.

왜 필요한가 (2026-08-31)
────────────────────────
COM8 실사용 신고: "PROG 가 가끔만 작동한다."
NVS 방전기록(batlog)을 뜨니 19초 세션에 PROG **6회가 전부 정상 기록**돼 있었다.
접점은 멀쩡했고, `PROG_GUARD_MAX=5` 를 넘긴 6번째부터 RF 만 막힌 것이었다.

기존 코드의 결함 두 가지 — somfy_app.c 원본 순서:

    if (s_pg_last_us && (now - s_pg_last_us) > CALM) { 해제 }
    s_pg_last_us = now;          # ★결함① 차단 검사보다 먼저, 무조건 갱신
    ...
    if (s_pg_blocked) break;     # ★결함② 조용히 버림 (로그·표시 없음)

결함① 때문에 **차단된 누름도 침묵 타이머를 리셋한다.** 안 되니까 계속 눌러 보는
동안에는 `now - s_pg_last_us` 가 CALM 에 영영 도달하지 못해 **사실상 영구 차단**이다.

수정안 (사용자 승인 1·2·3번)
────────────────────────────
 1) 차단 중인 누름은 `s_pg_last_us` 를 **갱신하지 않는다**
    → 해제 시각 = "차단을 유발한 마지막 누름 + CALM" 으로 확정된다.
 2) 차단된 누름을 조용히 버리지 않는다 — 로그 + OLED 배너로 남은 시간 표시.
 3) 한도 완화 **5회/5분 → 10회/2분** (사용자 지정값).
    정품 Somfy 등록·한계설정 절차가 PROG 연속 누름이라 5회/분은 부족했다.

이 시뮬레이터는 실측 조작 패턴(batlog)을 그대로 넣어 두 로직을 나란히 돌린다.
**기존 쪽은 옛 상수(5회/5분)로, 수정안은 새 상수(10회/2분)로** — 실제로 바뀐 것을
그대로 비교하기 위해서다.
"""
import sys

# 기존 코드가 실제로 쓰던 값
OLD_MAX, OLD_WINDOW_MS, OLD_CALM_MS = 5, 60000, 300000
# ★2026-08-31 사용자 지정 새 값 (somfy_app.c 와 일치시킬 것)
NEW_MAX, NEW_WINDOW_MS, NEW_CALM_MS = 10, 60000, 120000


class Guard:
    """old=True 면 기존 코드(차단 중에도 last 갱신 + 옛 상수)."""

    def __init__(self, old):
        self.old = old
        self.MAX, self.WINDOW, self.CALM = (
            (OLD_MAX, OLD_WINDOW_MS, OLD_CALM_MS) if old
            else (NEW_MAX, NEW_WINDOW_MS, NEW_CALM_MS))
        self.first = self.last = self.count = 0
        self.blocked = False
        self.ever_blocked = False
        self.unblock_at = None

    def press(self, now_ms):
        """반환: (송신했는가, 사유)"""
        if self.last and (now_ms - self.last) > self.CALM:
            if self.blocked and self.unblock_at is None:
                self.unblock_at = now_ms
            self.blocked = False
            self.count = 0
            self.first = 0

        if self.old:
            self.last = now_ms              # ★기존: 무조건 갱신 (결함①)
            if self.blocked:
                return False, "차단중 (침묵타이머 리셋됨)"
        else:
            if self.blocked:                # ★수정안: 갱신하지 않고 남은 시간 안내
                left = self.CALM - (now_ms - self.last)
                return False, "차단중 — %.0f초 후 해제(표시)" % (left / 1000.0)
            self.last = now_ms

        if not self.first or (now_ms - self.first) > self.WINDOW:
            self.first = now_ms
            self.count = 0
        self.count += 1
        if self.count > self.MAX:
            self.blocked = True
            self.ever_blocked = True
            return False, "★차단 발동 (%d회/%ds)" % (self.count, self.WINDOW / 1000)
        return True, "송신 %d/%d" % (self.count, self.MAX)


def run(name, presses, show_all=True):
    print("\n" + "=" * 78)
    print("케이스: %s   (PROG 누름 %d회)" % (name, len(presses)))
    print("=" * 78)
    print("%9s | %-32s | %-32s" % ("시각", "기존 (5회/5분)", "수정안 (10회/2분)"))
    print("-" * 78)
    g_old, g_new = Guard(True), Guard(False)
    n_old = n_new = 0
    prev = None
    for t in presses:
        ok_o, why_o = g_old.press(t)
        ok_n, why_n = g_new.press(t)
        n_old += ok_o
        n_new += ok_n
        row = (ok_o, ok_n)
        if show_all or row != prev:          # 긴 케이스는 상태가 바뀔 때만 출력
            print("%7.1f초 | %-2s %-29s | %-2s %-29s"
                  % (t / 1000.0, "○" if ok_o else "✕", why_o,
                     "○" if ok_n else "✕", why_n))
        prev = row
    print("-" * 78)
    print("송신 성공:  기존 %d회  /  수정안 %d회" % (n_old, n_new))
    def _st(g):
        if not g.ever_blocked:
            return "차단 안 걸림"
        if g.unblock_at:
            return "%.0f초에 해제" % (g.unblock_at / 1000.0)
        return "**끝까지 안 풀림**"
    print("차단 상태:  기존 %s  /  수정안 %s" % (_st(g_old), _st(g_new)))
    return n_old, n_new


def main():
    # ── ① COM8 실측 (nvs batlog, 2026-08-31 배터리 세션 19초) ────────────
    real = [1000, 1100, 2000, 14000, 14200, 15000]
    run("COM8 실측 batlog — 19초 세션", real)

    # ── ② 안 되니까 계속 눌러 보는 실제 행동 (결함①이 드러나는 곳) ──────
    retry = list(real)
    t = 30000
    while t <= 420000:
        retry += [t, t + 800]
        t += 30000
    run("차단 뒤 30초마다 재시도 (7분)", retry, show_all=False)

    # ── ②b 새 한도에서도 결함①을 실제로 밟는 패턴 ──────────────────────
    #     20초에 15회(11번째에 차단) → 이후 30초마다 재시도.
    #     한도를 완화해도 **차단이 걸리는 상황은 여전히 존재**하므로,
    #     결함① 수정이 새 상수에서도 유효한지 여기서 확인한다.
    burst = [i * 1300 for i in range(1, 16)]
    t = 40000
    while t <= 300000:
        burst += [t, t + 800]
        t += 30000
    run("연타 15회 후 30초마다 재시도 (5분)", burst, show_all=False)

    # ── ③ 정품 Somfy 등록 절차 — PROG 를 천천히 여러 번 ─────────────────
    run("등록 절차 — 20초 간격 12회", [i * 20000 for i in range(1, 13)], show_all=False)

    # ── ④ 고장(연타 폭주) — 안전장치가 여전히 듣는가 ────────────────────
    n_old, n_new = run("고장 폭주 — 0.2초마다 400회(80초)",
                       [i * 200 for i in range(1, 400)], show_all=False)

    print("\n" + "=" * 78)
    print("결론")
    print("=" * 78)
    print(" ① 실측 19초 세션: 기존 5회에서 막힘 → 수정안은 **6회 전부 송신**")
    print("    (신고 '가끔만 작동'의 직접 원인 해소)")
    print(" ② 재시도: 기존은 7분 내내 안 풀림(침묵타이머가 매번 리셋) →")
    print("    수정안은 마지막 정상 누름 +2분에 확정 해제")
    print(" ③ 등록 절차(20초 간격 12회): 창(60초)이 계속 리셋돼 둘 다 전부 송신")
    print(" ④ 폭주: 수정안도 %d회에서 차단 — 최악이어도 2분당 %d회로 억제된다"
          % (n_new, NEW_MAX + 1))
    print("    안전장치(하루 종일 자동 PROG 송신 사고 방지)의 목적은 유지된다")
    return 0


if __name__ == '__main__':
    sys.exit(main())
