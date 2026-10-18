# -*- coding: utf-8 -*-
"""sleep_probe — **USB 를 꽂아 둔 채** 절전 동작을 주기적으로 재는 무인 하네스.

왜 필요한가 (2026-09-07)
────────────────────────
절전 문제를 확인하려면 지금까지 **케이블을 뽑고 몇 시간을 기다려야** 했다.
한 번 재는 데 반나절이 걸려 가설 하나 검증에 하루가 갔다.

`usbsim off` 는 원래 앱 동작만 배터리인 척했고 **PM(light sleep)은 물리 VBUS 기준**
이라 절전이 안 걸렸다. 2026-09-07 에 `on_usb` 판정이 이 플래그를 함께 보도록 고쳐서,
이제 USB 를 꽂은 채로도 **진짜 light sleep 이 걸린다**.

  → 이 스크립트가 그것을 주기적으로 반복하고 NVS 를 떠서 표로 뽑는다.

측정할 수 있는 것 / 없는 것
───────────────────────────
  ○ SLEEP 체류율, 깨어남 횟수, `rtos0`, light_sleep_counts/reject
  ○ 유령 입력(버튼 이벤트)이 유휴 중에 나오는지 — **시각까지** 본다
  ✕ 전압·방전속도 — USB 가 물려 있어 **충전 중**이다. 의미 없다.
    그건 물리 분리로만 잴 수 있다.

주의
────
  · light sleep 이 걸리면 **USB-JTAG 콘솔이 죽는다**(기록된 지뢰).
    그래서 매 회차 끝에 esptool 로 리셋하며 NVS 를 뜬다 — 콘솔과 무관하게 동작한다.
  · 리셋하면 PM 카운터가 0 부터 다시 시작하므로 **회차마다 깨끗한 창**이 된다.
  · USB 에 꽂힌 채 light sleep 이 돌면 과거 테스트보드에서 I2C 고착이 관측된 적이
    있다(somfy_app.c 주석). 이상하면 회차를 짧게 잡을 것.

사용:
    python sim/tools/sleep_probe.py COM7 COM8 --minutes 60 --rounds 4
    python sim/tools/sleep_probe.py COM7 --minutes 20 --rounds 1 --outdir caps/probe
"""
import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from serial_noreset import open_noreset            # noqa: E402

ESPTOOL = os.path.expanduser(
    r"~/.espressif/python_env/idf5.5_py3.14_env/Scripts/esptool.exe")
CHIP = "esp32c6"
NVS_OFF, NVS_LEN = "0x10000", "0xC000"


def _console(port, cmd, wait=3.0):
    """콘솔 한 줄 전송. light sleep 으로 콘솔이 죽어 있으면 조용히 실패한다."""
    try:
        sp = open_noreset(port, timeout=0.3, write_timeout=2)
    except Exception as e:
        print("   [%s] 콘솔 열기 실패: %s" % (port, e))
        return False
    try:
        sp.write(b"\n")
        sp.write(cmd.encode() + b"\n")
        t0 = time.time()
        while time.time() - t0 < wait:
            ln = sp.readline()
            if ln and (b"OK" in ln or b"USBSIM" in ln):
                print("   [%s] %s" % (port, ln.decode("utf-8", "replace").rstrip()))
                return True
    except Exception as e:
        print("   [%s] 콘솔 쓰기 실패: %s" % (port, e))
    finally:
        try:
            sp.close()
        except Exception:
            pass
    return False


def _dump(port, path):
    """esptool 로 NVS 를 뜬다. 콘솔이 죽어 있어도 되고, 끝나면 하드리셋된다."""
    # ★인코딩을 명시하지 않으면 Windows 기본(cp949)으로 디코딩하다 터진다.
    #  esptool 은 ASCII 지만 리더 스레드가 예외를 내면 결과를 못 받는다.
    for attempt in range(3):
        r = subprocess.run([ESPTOOL, "--chip", CHIP, "-p", port,
                            "--before", "default_reset", "--after", "hard_reset",
                            "read_flash", NVS_OFF, NVS_LEN, path],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        if r.returncode == 0 and os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(3)      # 포트가 아직 잡혀 있을 수 있다(콘솔 닫힘 직후)
    print("   [%s] esptool: %s" % (port, (r.stdout or "")[-200:].strip()))
    return False


def _decode(path):
    """PM 덤프 + batlog 에서 판정에 쓰는 값만 뽑는다."""
    out = {}
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    pm = subprocess.run([sys.executable, os.path.join(HERE, "pmdump_decode.py"), path],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", env=env).stdout
    m = re.search(r"Time since boot up:\s*(\d+)\s*us", pm)
    out["boot_s"] = int(m.group(1)) / 1e6 if m else 0
    for key, pat in (("sleep_pct", r"SLEEP\s+\S+\s+\d+\s+(\d+)\s*%"),
                     ("ls_cnt",    r"light_sleep_counts:(\d+)"),
                     ("ls_rej",    r"light_sleep_reject_counts:(\d+)")):
        m = re.search(pat, pm)
        out[key] = int(m.group(1)) if m else 0
    m = re.search(r"rtos0\s+\S+\s+\d+\s+\d+\s+(\d+)", pm)
    out["rtos0"] = int(m.group(1)) if m else 0

    bl = subprocess.run([sys.executable, os.path.join(HERE, "batlog_decode.py"), path],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", env=env).stdout
    evs = []
    for line in bl.splitlines():
        m = re.match(r"\s*\d+\s+(\d+)\s+\d+\s", line)
        if m:
            tag = line.rstrip().split()[-1]
            if tag not in ("주기",):
                evs.append((int(m.group(1)), tag))
    out["events"] = evs
    return out


def probe(ports, minutes, rounds, outdir):
    os.makedirs(outdir, exist_ok=True)
    for rnd in range(1, rounds + 1):
        print("\n═══ %d/%d 회차 — 배터리 모드 시뮬 %d분 ═══" % (rnd, rounds, minutes))
        for p in ports:
            _console(p, "usbsim off")
        # light sleep 이 걸리면 콘솔이 죽으므로 여기서부터는 기다리기만 한다.
        deadline = time.time() + minutes * 60
        while time.time() < deadline:
            time.sleep(min(30, max(1, deadline - time.time())))
        print("   → NVS 수집 (리셋됨, 다음 회차는 새 창)")
        for p in ports:
            path = os.path.join(outdir, "probe_%s_r%d.bin" % (p, rnd))
            if not _dump(p, path):
                print("   [%s] 덤프 실패" % p)
                continue
            d = _decode(path)
            idle = [e for e in d["events"]]
            print("   [%-5s] 창 %5.1f분 | SLEEP **%2d%%** | 깨어남 %6.1f회/초 "
                  "| reject %7d | 이벤트 %d개"
                  % (p, d["boot_s"] / 60,
                     d["sleep_pct"],
                     d["ls_cnt"] / d["boot_s"] if d["boot_s"] else 0,
                     d["ls_rej"], len(idle)))
            if idle:
                # 유령 입력 판별: 시각이 몰려 있으면 사람, 흩어져 있으면 유령
                ts = [t for t, _ in idle]
                span = (max(ts) - min(ts)) if len(ts) > 1 else 0
                print("            이벤트 시각 폭 %ds — %s"
                      % (span, "몰림(사람 조작 의심)" if span < 300 else
                               "★흩어짐(유령 입력 의심)"))
                for t, tag in idle[:12]:
                    print("              t=%6d초  %s" % (t, tag))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ports", nargs="+", help="예: COM7 COM8")
    ap.add_argument("--minutes", type=int, default=60, help="회차당 배터리 모드 유지(분)")
    ap.add_argument("--rounds", type=int, default=1)
    ap.add_argument("--outdir", default=os.path.join(HERE, "caps", "probe"))
    a = ap.parse_args()
    probe(a.ports, a.minutes, a.rounds, a.outdir)
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.exit(main())
