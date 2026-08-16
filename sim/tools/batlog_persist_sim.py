# -*- coding: utf-8 -*-
"""방전 로그가 재부팅에서 살아남는지 검증하는 시뮬레이터.

배경 (2026-08-16): 사용자 방전 기록이 반복해서 사라졌다. 원인은 두 겹이었다.

  ① **첫 판독 보정 누락** (진짜 원인)
     somfy_app.c 의 USB 판정은 2초 디바운스를 쓴다:
         now_usb = usb_mode or (now - usb_low_since_us) < 2000ms
     그런데 부팅 첫 루프에서 usb_low_since_us 가 방금 now 로 채워지므로 경과가
     0 이라 **배터리인데도 now_usb 가 true**. 그래서
       (1) s_bl_first 가 "USB 부팅"으로 오인 → _batlog_try_resume 을 안 부르고
       (2) 2초 뒤 배터리로 뒤집히면 was_usb_pwr && !now_usb 가 성립 →
           "USB 분리"로 오인 → 새 세션
     즉 **배터리로 부팅할 때마다** 가짜 분리가 만들어졌다.

  ② **새 세션이 링을 통째로 비웠다**
     세션 시작에서 `_batlog_reset()` 을 불렀다. ①과 겹치면 부팅 한 번에 과거
     기록이 전부 소멸한다.

수정: ① 배터리 부팅이면 디바운스 창을 이미 지난 것으로 놓는다.
      ② `_batlog_reset()` 제거 — 세션 번호만 올리고 경계 표식(BLEV_SESS)을 남긴다.
         전체 삭제는 콘솔 `blclear` 로만.

이 시뮬레이터는 두 수정을 각각 끄고 켜며 **기록이 남는지**를 센다.
"""
import sys

USB_DROP_CONFIRM_MS = 2000
BAT_SAMPLE_MS       = 5000    # 배터리 측정 주기
BAT_FLOOR_MIN_N     = 5       # 기준점 확정에 필요한 실측 횟수
LOOP_MS             = 100     # somfy_app 메인 루프 주기
BATLOG_MAX          = 128     # H2


class Batlog:
    """NVS 에 남는 링버퍼. 부팅해도 내용이 유지된다."""

    def __init__(self):
        self.buf = []          # (t_s, ev) 목록 — 링
        self.sess = 0
        self.saved_on = 0      # dis_on
        self.saved_el = 0      # dis_el (초)

    def add(self, t_s, ev='주기'):
        self.buf.append((t_s, ev))
        if len(self.buf) > BATLOG_MAX:
            self.buf.pop(0)    # 가장 오래된 것이 밀려난다(삭제 아님)

    def wipe(self):
        self.buf = []
        self.sess += 1


def boot(nv, usb_mode_at, run_ms, fix_first_read, keep_on_new_session,
         resume_ok=True):
    """부팅 한 번을 돌린다. usb_mode_at(t_ms) -> bool 이 실제 전원 상태."""
    first        = True
    was_usb      = True
    dis_pending  = False
    dis_seq0     = 0
    usb_low_since = 0
    sm_seq       = 0        # s_bat_sm_seq
    hist_n       = 0        # s_bat_hist_n
    dis_t0       = None     # 세션 시작 시각(ms). None = 세션 없음
    events       = []

    t = 0
    while t < run_ms:
        # ── 배터리 측정(5초 주기) ────────────────────────────────────────
        if t % BAT_SAMPLE_MS == 0:
            sm_seq += 1
            hist_n = min(hist_n + 1, 9)

        usb_mode = usb_mode_at(t)

        # ── ★수정① 첫 판독 보정 ────────────────────────────────────────
        if fix_first_read and first and not usb_mode and usb_low_since == 0:
            usb_low_since = t - USB_DROP_CONFIRM_MS

        if usb_mode:
            usb_low_since = 0
        elif usb_low_since == 0:
            usb_low_since = t
        now_usb = usb_mode or (t - usb_low_since) < USB_DROP_CONFIRM_MS

        if first:
            first = False
            was_usb = now_usb
            if not now_usb:
                if resume_ok and nv.saved_on and nv.buf:
                    dis_t0 = t - nv.saved_el * 1000
                    events.append('이어받기')
                    nv.add(nv.saved_el, '기타')      # 재부팅 지점
                else:
                    dis_pending = True
                    dis_seq0 = sm_seq
                    events.append('이어받을 세션 없음')
        elif was_usb and not now_usb:
            dis_pending = True
            dis_seq0 = sm_seq
            dis_t0 = None
            events.append('USB 분리 감지')
        elif not was_usb and now_usb:
            dis_t0 = None
            dis_pending = False
            events.append('방전 종료(USB 연결)')
        was_usb = now_usb

        # ── 기준점 확정 = 새 세션 시작 ──────────────────────────────────
        if (dis_pending and not now_usb and sm_seq > dis_seq0
                and hist_n >= BAT_FLOOR_MIN_N):
            dis_pending = False
            dis_t0 = t
            events.append('★방전 시작')
            if keep_on_new_session:
                nv.sess += 1
                nv.add(0, '세션시작')      # 경계만 표시, 지우지 않는다
            else:
                nv.wipe()                  # ← 옛 코드: 통째로 삭제
                nv.add(0)
        elif dis_t0 is not None and t % 60000 == 0 and t > 0:
            nv.add((t - dis_t0) // 1000)   # 주기 기록

        # 세션 상태 저장(NVS)
        nv.saved_on = 1 if dis_t0 is not None else 0
        nv.saved_el = (t - dis_t0) // 1000 if dis_t0 is not None else 0

        t += LOOP_MS
    return events


def scenario(name, usb_seq, fix_first_read, keep_on_new_session):
    """usb_seq = 부팅마다 (usb_mode 함수, 지속시간ms)"""
    nv = Batlog()
    # 1회차: 배터리로 오래 돌아 기록을 쌓아둔다(USB 분리 상황)
    boot(nv, lambda t: t < 10000, 30 * 60000, fix_first_read, keep_on_new_session)
    before = len(nv.buf)
    # 2회차: 배터리 구동 중 재부팅(글리치/포트열기) — usb_mode 계속 false
    ev = boot(nv, lambda t: False, 3 * 60000, fix_first_read, keep_on_new_session)
    after = len(nv.buf)
    kept = after >= before
    print('  %-22s 재부팅 전 %3d건 → 후 %3d건   %s'
          % (name, before, after,
             '★기록 보존' if kept else '기록 %d건 소멸 ✗' % (before - after)))
    print('     부팅 이벤트: %s' % ' / '.join(ev[:4]))
    return kept


if __name__ == '__main__':
    print('■ 배터리 구동 중 재부팅 — 직전 기록이 남는가?\n')
    print(' [옛 코드] 첫판독 보정 없음 + 새 세션이 링을 비움')
    old = scenario('옛 코드', None, False, False)
    print()
    print(' [수정①만] 첫 판독 보정')
    only1 = scenario('수정① 첫판독', None, True, False)
    print()
    print(' [수정②만] 새 세션이 비우지 않음')
    only2 = scenario('수정② 비우지 않음', None, False, True)
    print()
    print(' [수정①+②] 실제 적용본')
    both = scenario('수정①+②', None, True, True)
    print()
    print('=== 판정 ===')
    print('  옛 코드      : %s' % ('보존' if old else '★소멸 — 재현됨'))
    print('  수정①만     : %s' % ('보존' if only1 else '소멸'))
    print('  수정②만     : %s' % ('보존' if only2 else '소멸'))
    print('  수정①+②    : %s' % ('★보존' if both else '소멸 ✗'))
    ok = (not old) and both
    print('\n%s' % ('통과 — 버그 재현 + 수정 확인' if ok else '실패 — 모델 재검토 필요'))
    sys.exit(0 if ok else 1)
