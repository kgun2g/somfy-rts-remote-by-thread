# -*- coding: utf-8 -*-
"""NVS 파티션 덤프 — 여러 보드를 동시에, **절대 세션을 막지 않고**.

왜 따로 있나 (2026-10-13)
────────────────────────
lp_ab_collect.py 의 dump() 는 subprocess.run 에 제한 시간이 없어서 esptool 이 멈추면
도구 호출도 같이 멈춘다(그날 새벽 도구 호출 하나가 세션을 6분 40초 붙잡았다).
serial_safe.py 와 같은 구조로 만든다:
  · esptool 은 **분리 프로세스**로 띄우고 출력은 파일로 받는다(파이프를 물려주지 않는다).
  · 부모는 제한 시간이 되면 **자식을 기다리지 않고** 돌아온다.
  · 기존 덤프 파일은 덮어쓰지 않는다 — 측정 자료는 지워지면 안 된다.
  · 덤프 뒤 hard_reset 으로 펌웨어를 다시 띄운다(부트로더에 남기면 블라인드 제어 불가).

    python sim/tools/nvs_dump_safe.py COM7 COM8 --tag adcab
결과: logs/nvs_<포트>_<MMDD_HHMM>_<tag>.bin
반환코드: 0 전부 성공 / 3 하나라도 실패·시간 초과
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
ESPTOOL = os.path.expanduser(
    r"~/.espressif/python_env/idf5.5_py3.14_env/Scripts/esptool.exe")
NVS_OFF, NVS_LEN = "0x10000", "0xC000"
DEVNULL = subprocess.DEVNULL


def _start(job, chip):
    flags = (0x00000008 | 0x00000200) if os.name == 'nt' else 0   # DETACHED | NEW_GROUP
    f = open(job['log'], 'a', encoding='utf-8', errors='replace')
    job['p'] = subprocess.Popen(
        [ESPTOOL, "--chip", chip, "-p", job['port'], "--before", "default_reset",
         "--after", "hard_reset", "read_flash", NVS_OFF, NVS_LEN, job['path']],
        stdin=DEVNULL, stdout=f, stderr=subprocess.STDOUT,
        creationflags=flags, close_fds=True)
    f.close()
    job['tries'] += 1


def _good(job):
    return (job['p'].poll() == 0 and os.path.exists(job['path'])
            and os.path.getsize(job['path']) == int(NVS_LEN, 16))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ports', nargs='+')
    ap.add_argument('--tag', default='dump')
    ap.add_argument('--chip', default='esp32c6')
    ap.add_argument('--deadline', type=float, default=75.0, help='전체 제한 시간(초)')
    ap.add_argument('--tries', type=int, default=3)
    a = ap.parse_args()

    outdir = os.path.join(ROOT, 'logs')
    os.makedirs(outdir, exist_ok=True)
    stamp = time.strftime('%m%d_%H%M')
    jobs = []
    for port in a.ports:
        path = os.path.join(outdir, 'nvs_%s_%s_%s.bin' % (port, stamp, a.tag))
        n = 1
        while os.path.exists(path):                 # ★덮어쓰지 않는다
            n += 1
            path = os.path.join(outdir, 'nvs_%s_%s_%s_%d.bin' % (port, stamp, a.tag, n))
        fd, log = tempfile.mkstemp(prefix='esptool_%s_' % port, suffix='.txt')
        os.close(fd)
        job = {'port': port, 'path': path, 'log': log, 'tries': 0, 'p': None}
        _start(job, a.chip)
        jobs.append(job)

    t0 = time.time()
    while time.time() - t0 < a.deadline:
        busy = False
        for j in jobs:
            code = j['p'].poll()
            if code is None:
                busy = True
            elif not _good(j) and j['tries'] < a.tries:
                time.sleep(2.0)                     # 포트가 다시 열릴 틈
                _start(j, a.chip)
                busy = True
        if not busy:
            break
        time.sleep(0.3)

    rc = 0
    for j in jobs:
        code = j['p'].poll()
        if code is None:                            # 멈췄다 — 죽여 보되 기다리지 않는다
            subprocess.Popen(['taskkill', '/F', '/T', '/PID', str(j['p'].pid)],
                             stdin=DEVNULL, stdout=DEVNULL, stderr=DEVNULL)
        if _good(j):
            print('[%s] OK   %s  (시도 %d회)' % (j['port'], os.path.relpath(j['path'], ROOT), j['tries']))
            try:
                os.remove(j['log'])
            except OSError:
                pass
            continue
        rc = 3
        why = ('시간 초과 %.0f초 - esptool 이 멈췄다' % a.deadline) if code is None else ('종료코드 %s' % code)
        tail = ''
        try:
            tail = open(j['log'], encoding='utf-8', errors='replace').read()
        except OSError:
            pass
        print('[%s] 실패 (%s, 시도 %d회)' % (j['port'], why, j['tries']))
        print('      ' + ' '.join(tail.split())[-320:])
    return rc


if __name__ == '__main__':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    sys.exit(main())
