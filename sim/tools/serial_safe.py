# -*- coding: utf-8 -*-
"""시리얼 콘솔 대화 — **절대 세션을 막지 않는** 버전.

왜 필요한가 (2026-10-13)
────────────────────────
01:26:03 에 시작한 직렬 스크립트(`timeout 120 python ...`)가 **제한 시간을 넘겨서도**
돌아오지 않았다. 사용자가 01:32:25 에 중단을 누르고 01:32:45 에 앱을 종료할 때까지
(6분 40초) 그 도구 호출 하나가 세션 전체를 붙잡고 있었다.

※원인은 **확정하지 못했다.** "굳은 USB CDC 포트에서 WriteFile/CloseHandle 이 커널 안에서
  막힌다"는 것은 가설이다. 같은 시각 다른 프로젝트 세션의, 직렬 포트와 무관한 Bash
  호출도 13분 멈춰 있었으므로(01:23:59~01:37:33) 시스템 공통 원인일 수 있다.
  (★정정: 처음엔 이 자리에 "28분간", "커널 안에서 막혀서"라고 단정해 썼다. 28분은 앱
   종료와 PC 전원 재투입 2회를 포함한 숫자였고, 원인은 재지 않은 추정이었다.)
어느 쪽이 원인이든 아래 구조는 유효하다 — 도구 호출이 자식의 종료를 기다리지 않는다.

해법 = **기다리는 쪽과 포트를 만지는 쪽을 분리한다.**
  · 자식(분리 프로세스)만 포트를 연다. 출력은 파일로 쓴다.
  · 부모는 그 파일만 지켜보다가 제한 시간이 되면 **자식을 기다리지 않고** 돌아온다.
    자식이 커널에서 막혀 있어도 도구 호출은 반드시 끝난다.
  · 자식도 스스로 os._exit 하는 감시 타이머를 건다(close 를 거치지 않는다).

사용:
    python sim/tools/serial_safe.py COM7 "rtmod" --keys RTMOD --wait 8
    python sim/tools/serial_safe.py COM7 "adcfix 0" "reboot" --keys ADCFIX,"ADC 유닛" --wait 30
    python sim/tools/serial_safe.py COM7 --listen 20 --keys PWR      # 쓰지 않고 듣기만
반환코드: 0 정상 / 3 제한 시간 초과(자식이 막힘 — 포트가 굳었을 가능성)
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _child(port, cmds, wait, keys, listen, out_path, deadline):
    # 감시 타이머: close()/write() 가 커널에서 막혀도 프로세스를 끝낸다.
    threading.Timer(deadline, lambda: os._exit(124)).start()
    sys.path.insert(0, HERE)
    out = open(out_path, 'w', encoding='utf-8', buffering=1)

    def emit(s):
        out.write(s + '\n')

    try:
        from serial_noreset import open_noreset
        sp = open_noreset(port, timeout=0.4, write_timeout=4)
    except Exception as e:
        emit('!! 포트 열기 실패: %s' % e)
        os._exit(2)

    def pump(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                ln = sp.readline()
            except Exception as e:
                emit('!! 읽기 실패: %s' % e)
                return
            if not ln:
                continue
            s = ln.decode('utf-8', 'replace').rstrip()
            if not keys or any(k in s for k in keys):
                emit(s)

    try:
        if listen:
            pump(listen)
        else:
            time.sleep(1.0)                     # 열자마자 쓰면 타임아웃이 난다
            for c in cmds:
                try:
                    sp.write(b'\n')
                    time.sleep(0.3)
                    sp.write(c.encode('utf-8') + b'\n')
                except Exception as e:
                    emit('!! 쓰기 실패(%s): %s — USB 콘솔이 굳었을 수 있다' % (c, e))
                    break
                pump(wait)
    finally:
        emit('##END##')
        out.flush()
        os._exit(0)                             # ★close 를 부르지 않는다 — 거기서 막힌다


def run(port, cmds, wait=8.0, keys=None, listen=0.0, deadline=None):
    """부모: 자식을 띄우고 출력 파일만 본다. 제한 시간이 되면 **기다리지 않고** 돌아온다."""
    total = (listen if listen else (1.5 + len(cmds) * (wait + 0.5))) + 6
    deadline = deadline or total
    fd, out_path = tempfile.mkstemp(prefix='ser_', suffix='.txt')
    os.close(fd)
    args = [sys.executable, os.path.abspath(__file__), '--child', port,
            json.dumps({'cmds': cmds, 'wait': wait, 'keys': keys or [],
                        'listen': listen, 'out': out_path, 'deadline': deadline})]
    flags = 0
    if os.name == 'nt':
        flags = 0x00000008 | 0x00000200         # DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP
    p = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, creationflags=flags, close_fds=True)
    t0 = time.time()
    done = False
    text = ''
    while time.time() - t0 < deadline + 2:
        time.sleep(0.4)
        try:
            text = open(out_path, encoding='utf-8', errors='replace').read()
        except Exception:
            text = ''
        if '##END##' in text:
            done = True
            break
        if p.poll() is not None:
            time.sleep(0.3)
            try:
                text = open(out_path, encoding='utf-8', errors='replace').read()
            except Exception:
                pass
            done = '##END##' in text
            break
    lines = [l for l in text.splitlines() if l != '##END##']
    if not done:
        # 자식이 막혔다. 죽여 보되 **결과를 기다리지 않는다**.
        if os.name == 'nt':
            subprocess.Popen(['taskkill', '/F', '/T', '/PID', str(p.pid)],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        lines.append('!! 제한 시간 %.0f초 초과 — %s 가 응답하지 않는다(USB 콘솔이 굳었을 수 있다). '
                     'esptool 리셋이나 케이블 재연결이 필요하다.' % (deadline, port))
    return (0 if done else 3), lines


def main():
    if len(sys.argv) >= 4 and sys.argv[1] == '--child':
        a = json.loads(sys.argv[3])
        _child(sys.argv[2], a['cmds'], a['wait'], a['keys'], a['listen'], a['out'], a['deadline'])
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('cmds', nargs='*')
    ap.add_argument('--wait', type=float, default=8.0, help='명령마다 응답을 모으는 시간(초)')
    ap.add_argument('--keys', default='', help='쉼표로 구분한 필터 문자열(비우면 전부)')
    ap.add_argument('--listen', type=float, default=0.0, help='쓰지 않고 이 시간만큼 듣기만')
    ap.add_argument('--deadline', type=float, default=0.0, help='전체 제한 시간(초)')
    a = ap.parse_args()
    keys = [k for k in a.keys.split(',') if k]
    rc, lines = run(a.port, a.cmds, a.wait, keys, a.listen, a.deadline or None)
    for l in lines:
        print(l)
    return rc


if __name__ == '__main__':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    sys.exit(main())
